/*
   Este arquivo faz parte da JCFLIGHT.

   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar
   sob os termos da GNU General Public License conforme publicada por
   a Free Software Foundation, seja a versão 3 da Licença, ou
   (à sua escolha) qualquer versão posterior.

  JCFLIGHT é distribuído na esperança de ser útil,
  mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de
  COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o
  GNU General Public License para mais detalhes.

   Você deve ter recebido uma cópia da Licença Pública Geral GNU
  junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>.
*/

#include "GPSUBLOX.h"
#include "FastSerial/FASTSERIAL.h"
#include "GPSNavigation/NAVIGATION.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "ProgMem/PROGMEM.h"
#include "Common/ENUM.h"
#include "GPS/GPSSTATES.h"
#include "DJINAZAGPS.h"
#include "Build/BOARDDEFS.h"

//COM OS GPS-M8N É POSSIVEL ATIGIR MAIS DE 30 SATELITES
#define UBLOX_BUFFER_SIZE 464

struct Ublox_Navigation_PosLLH
{
  uint32_t Time;
  int32_t Longitude;
  int32_t Latitude;
  int32_t Altitude_Ellipsoid;
  int32_t Altitude_MSL;
  uint32_t Horizontal_Accuracy;
  uint32_t Vertical_Accuracy;
};

struct Ublox_Navigation_Solution
{
  uint32_t Time;
  int32_t Time_NSec;
  int16_t Week;
  uint8_t Fix_Type;
  uint8_t Fix_Status;
  int32_t ECEF_X;
  int32_t ECEF_Y;
  int32_t ECEF_Z;
  uint32_t Position_Accuracy_3D;
  int32_t ECEF_X_Velocity;
  int32_t ECEF_Y_Velocity;
  int32_t ECEF_Z_Velocity;
  uint32_t Speed_Accuracy;
  uint16_t Position_DOP;
  uint8_t Res;
  uint8_t Satellites;
  uint32_t Res2;
};

struct Ublox_Navigation_VelNED
{
  uint32_t Time;
  int32_t North;
  int32_t East;
  int32_t Down;
  uint32_t Speed_3D;
  uint32_t Speed_2D;
  int32_t Heading_2D;
  uint32_t Speed_Accuracy;
  uint32_t Heading_Accuracy;
};

static union
{
  Ublox_Navigation_PosLLH PositionLLH;
  Ublox_Navigation_Solution Solution;
  Ublox_Navigation_VelNED VelocityNED;
  uint8_t Bytes_Array[464];
} Buffer;

//VERIFICAÇÃO DOS PACOTES DE DADOS
static uint8_t Check_Packet_A;
static uint8_t Check_Packet_B;

//ESTADO DE MAQUINA
static uint8_t Step_Counter;
static uint8_t Get_GPS_Message_ID;
static uint16_t Payload_Length;
static uint16_t Payload_Counter;

const uint8_t Ublox_Set_Configuration[] FLASH_MEMORY_ATTRIBUTE = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x00, 0x3B, 0xA2,
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28,
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0xC9,
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

static void SerialSendConfigToGPS(const char *STR)
{
#ifdef __AVR_ATmega2560__
  char ProgramMemory;
  while (STR && (ProgramMemory = ProgMemReadByte(STR++)))
  {
    FASTSERIAL.Write(UART_NUMB_1, ProgramMemory);
    SCHEDULERTIME.Sleep(5);
  }
#elif defined __arm__ || defined ESP32

#endif
}

void GPS_SerialInit(uint32_t Get_BaudRate)
{
  if (Get_GPS_Type(GPS_UBLOX))
  {
    //GPS AUTO BAUD-RATE
    static uint8_t Parse_Baud_Rate = 0;
    FASTSERIAL.Begin(UART_NUMB_1, Get_BaudRate);
    SCHEDULERTIME.Sleep(1000);
    if (Parse_Baud_Rate == 0)
    {
      FASTSERIAL.Begin(UART_NUMB_1, 9600);
      if (Get_BaudRate == 19200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
      }
      else if (Get_BaudRate == 38400)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
      }
      else if (Get_BaudRate == 57600)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
      }
      else if (Get_BaudRate == 115200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
      }
      while (!FASTSERIAL.TXFree(UART_NUMB_1))
      {
        SCHEDULERTIME.Sleep(50);
      }
      Parse_Baud_Rate = 1;
    }
    else if (Parse_Baud_Rate == 1)
    {
      FASTSERIAL.Begin(UART_NUMB_1, 19200);
      if (Get_BaudRate == 19200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
      }
      else if (Get_BaudRate == 38400)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
      }
      else if (Get_BaudRate == 57600)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
      }
      else if (Get_BaudRate == 115200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
      }
      while (!FASTSERIAL.TXFree(UART_NUMB_1))
      {
        SCHEDULERTIME.Sleep(50);
      }
      Parse_Baud_Rate = 2;
    }
    else if (Parse_Baud_Rate == 2)
    {
      FASTSERIAL.Begin(UART_NUMB_1, 38400);
      if (Get_BaudRate == 19200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
      }
      else if (Get_BaudRate == 38400)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
      }
      else if (Get_BaudRate == 57600)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
      }
      else if (Get_BaudRate == 115200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
      }
      while (!FASTSERIAL.TXFree(UART_NUMB_1))
      {
        SCHEDULERTIME.Sleep(50);
      }
      Parse_Baud_Rate = 3;
    }
    else if (Parse_Baud_Rate == 3)
    {
      FASTSERIAL.Begin(UART_NUMB_1, 57600);
      if (Get_BaudRate == 19200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
      }
      else if (Get_BaudRate == 38400)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
      }
      else if (Get_BaudRate == 57600)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
      }
      else if (Get_BaudRate == 115200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
      }
      while (!FASTSERIAL.TXFree(UART_NUMB_1))
      {
        SCHEDULERTIME.Sleep(50);
      }
      Parse_Baud_Rate = 4;
    }
    else if (Parse_Baud_Rate == 4)
    {
      FASTSERIAL.Begin(UART_NUMB_1, 115200);
      if (Get_BaudRate == 19200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
      }
      else if (Get_BaudRate == 38400)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
      }
      else if (Get_BaudRate == 57600)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
      }
      else if (Get_BaudRate == 115200)
      {
        SerialSendConfigToGPS(ProgramMemoryString("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
      }
      while (!FASTSERIAL.TXFree(UART_NUMB_1))
      {
        SCHEDULERTIME.Sleep(50);
      }
    }
    SCHEDULERTIME.Sleep(200);
    FASTSERIAL.Begin(UART_NUMB_1, Get_BaudRate);
    for (uint8_t SizeOfCount = 0; SizeOfCount < sizeof(Ublox_Set_Configuration); SizeOfCount++)
    {
#ifdef __AVR_ATmega2560__
      FASTSERIAL.Write(UART_NUMB_1, ProgMemReadByte(Ublox_Set_Configuration + SizeOfCount));
#elif defined __arm__ || defined ESP32

#endif
      SCHEDULERTIME.Sleep(5);
    }
  }
}

void GPS_SerialRead(uint8_t ReadData)
{
  if (Get_GPS_Type(GPS_UBLOX))
  {
    switch (Step_Counter)
    {

    case 1:
      if (PREAMBLE2 == ReadData)
      {
        Step_Counter++;
        break;
      }
      Step_Counter = 0;

    case 0:
      if (PREAMBLE1 == ReadData)
        Step_Counter++;
      break;

    case 2:
      Step_Counter++;
      Check_Packet_B = Check_Packet_A = ReadData;
      break;

    case 3:
      Step_Counter++;
      Check_Packet_B += (Check_Packet_A += ReadData);
      Get_GPS_Message_ID = ReadData;
      break;

    case 4:
      Step_Counter++;
      Check_Packet_B += (Check_Packet_A += ReadData);
      Payload_Length = ReadData;
      break;

    case 5:
      Step_Counter++;
      Check_Packet_B += (Check_Packet_A += ReadData);
      Payload_Length += (uint16_t)(ReadData << 8);
      if (Payload_Length > UBLOX_BUFFER_SIZE)
      {
        Payload_Length = 0;
        Step_Counter = 0;
      }
      Payload_Counter = 0;
      break;

    case 6:
      Check_Packet_B += (Check_Packet_A += ReadData);
      if (Payload_Counter < UBLOX_BUFFER_SIZE)
      {
        Buffer.Bytes_Array[Payload_Counter] = ReadData;
      }
      if (++Payload_Counter == Payload_Length)
      {
        Step_Counter++;
      }
      break;

    case 7:
      Step_Counter++;
      if (Check_Packet_A != ReadData)
      {
        Step_Counter = 0;
      }
      break;

    case 8:
      Step_Counter = 0;
      if (Check_Packet_B != ReadData)
      {
        break;
      }
      UBLOX_GetAllGPSData();
    }
  }
  else if (Get_GPS_Type(GPS_DJI_NAZA))
  {
#ifdef USE_NAZA_GPS
    DjiNazaGpsNewFrame(ReadData);
#endif
  }
}

void UBLOX_GetAllGPSData(void)
{
  switch (Get_GPS_Message_ID)
  {

  case MSG_POSLLH:
    GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE] = Buffer.PositionLLH.Latitude;
    GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE] = Buffer.PositionLLH.Longitude;
    GPS_Resources.Navigation.Misc.Get.Altitude = Buffer.PositionLLH.Altitude_MSL / 1000;
    GPS_Resources.Navigation.Misc.Get.EstimatedPositionHorizontal = (Buffer.PositionLLH.Horizontal_Accuracy / 10);
    GPS_Resources.Navigation.Misc.Get.EstimatedPositionVertical = (Buffer.PositionLLH.Vertical_Accuracy / 10);
    break;

  case MSG_STATUS:
    GPS_Resources.Navigation.Misc.Get.Marked3DFix = (Buffer.Solution.Fix_Status & 1) && (Buffer.Solution.Fix_Type == FIX_3D);
    break;

  case MSG_SOL:
    GPS_Resources.Navigation.Misc.Get.Marked3DFix = (Buffer.Solution.Fix_Status & 1) && (Buffer.Solution.Fix_Type == FIX_3D);
    GPS_Resources.Navigation.Misc.Get.Satellites = Buffer.Solution.Satellites;
    GPS_Resources.Navigation.Misc.Get.HDOP = Buffer.Solution.Position_DOP;
    break;

  case MSG_VELNED:
    GPS_Resources.Navigation.Misc.Get.GroundSpeed = Buffer.VelocityNED.Speed_2D;
    GPS_Resources.Navigation.Misc.Get.GroundCourse = (uint16_t)(Buffer.VelocityNED.Heading_2D / 10000);
    GPS_Resources.Navigation.Misc.Velocity.Get[NORTH] = Buffer.VelocityNED.North;
    GPS_Resources.Navigation.Misc.Velocity.Get[EAST] = Buffer.VelocityNED.East;
    GPS_Resources.Navigation.Misc.Velocity.Get[DOWN] = Buffer.VelocityNED.Down;
    GPS_Resources.Navigation.Misc.Velocity.NEDStatus = true;
    break;
  }
}