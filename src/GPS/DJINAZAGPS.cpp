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

#include "DJINAZAGPS.h"
#include "Math/MATHSUPPORT.h"
#include "GPSUBLOX.h"
#include "Common/ENUM.h"
#include "GPSNavigation/NAVIGATION.h"

typedef struct
{
  uint8_t res[4]; //0
  uint8_t fw[4];  //4
  uint8_t hw[4];  //8
} NazaGPS_Version;

//COMPASS DATA
typedef struct
{
  uint16_t MagAxisRoll;  //0
  uint16_t MagAxisPitch; //2
  uint16_t MagAxisYaw;   //4
} Struct_NazaGPS_Mag_Data;

//GPS DATA
typedef struct
{
  uint32_t Unused_Time;
  int32_t Longitude;      //4
  int32_t Latitude;       //8
  int32_t Altitude;       //12
  int32_t Horizontal_Acc; //16
  int32_t Vertical_Acc;   //20
  int32_t Null;
  int32_t NED_North;
  int32_t NED_East;
  int32_t NED_Down;      //39
  uint16_t Position_DOP; //40
  uint16_t Unused_VDOP;
  uint16_t Unused_NDOP;
  uint16_t Unused_EDOP;
  uint8_t Satellites; //48
  uint8_t Null2;
  uint8_t Fix_Type; //50
  uint8_t Null3;
  uint8_t Unused_FS; //52
  uint8_t Null4;
  uint8_t Null5;
  uint8_t Data_Mask; //55
} Struct_NazaGPS_GPS_Data;

//PROTOCOLOS
enum
{
  HEADER1 = 0x55,
  HEADER2 = 0xAA,
  ID_NAV = 0x10,
  ID_MAG = 0x20
};

//GPS FIX
typedef enum
{
  GPS_NO_FIX = 0,
  GPS_FIX_2D,
  GPS_FIX_3D
} GPS_Fix_Type;

//LEITURA DE TODOS OS DADOS SERIAIS DO GPS
static union
{
  Struct_NazaGPS_Mag_Data NazaGPS_Mag_Data;
  Struct_NazaGPS_GPS_Data NazaGPS_GPS_Data;
  NazaGPS_Version Version;
  uint8_t bytes[256];
} NazaGPS_Buffer_Read;

//PRINCIPAIS INFORMAÇÕES DO GPS
typedef struct
{
  int32_t Latitude;
  int32_t Longitude;
  int32_t Altitude;
} Struct_GPS_Location_Data;

typedef struct
{
  GPS_Fix_Type FixType;                       //INSTANCIA
  Struct_GPS_Location_Data GPS_Location_Data; //INSTANCIA
  uint8_t GPS_NumSat;
  int16_t GPS_Read_Compass[3];
  int16_t VelocityNED[3];
  int16_t GroundSpeed;
  int16_t GroundCourse;
  uint16_t HDOP_State;
  int32_t Horizontal_Acc;
  int32_t Vertical_Acc;
} Struct_SolutionData;

Struct_SolutionData GPSSolutionData; //INSTANCIA

//VERIFICÇÃO DE PACOTES DE DADOS DO GPS
static uint8_t CheckPacket_A;
static uint8_t CheckPacket_B;

//ESTADO DE MAQUINA
static bool GPS_New_Information;
static bool Next_Packet;
static uint8_t Step;
static uint8_t Decode_Message_ID;
static uint16_t Payload_Lenght;
static uint16_t Payload_Counter;

int16_t DJINaza_Compass_Roll;
int16_t DJINaza_Compass_Pitch;
int16_t DJINaza_Compass_Yaw;

int16_t Decode16BitsValues(uint16_t Index, uint8_t Data_Mask)
{
  union
  {
    uint16_t UnsignedShort;
    uint8_t Byte[2];
  } Value;
  Value.UnsignedShort = Index;
  Value.Byte[0] ^= Data_Mask;
  Value.Byte[1] ^= Data_Mask;
  return Value.UnsignedShort;
}

int32_t Decode32BitsValues(uint32_t Index, uint8_t Data_Mask)
{
  union
  {
    uint32_t UnsignedLong;
    uint8_t Byte[4];
  } Value;
  Value.UnsignedLong = Index;
  Value.Byte[0] ^= Data_Mask;
  Value.Byte[1] ^= Data_Mask;
  Value.Byte[2] ^= Data_Mask;
  Value.Byte[3] ^= Data_Mask;
  return Value.UnsignedLong;
}

uint16_t HDOPMaxError(uint32_t Value)
{
  return (Value > 9999) ? 9999 : Value;
}

static void NazaGPS_Check_Valid_Data(void)
{
  uint8_t Data_Mask;
  uint8_t Data_Mask_Mag;

  switch (Decode_Message_ID)
  {

  case ID_NAV:
  {
    Data_Mask = NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Data_Mask;

    //DECODE LATITUDE,LONGITUDE & ALTITUDE
    GPSSolutionData.GPS_Location_Data.Longitude = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Longitude, Data_Mask);
    GPSSolutionData.GPS_Location_Data.Latitude = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Latitude, Data_Mask);
    GPSSolutionData.GPS_Location_Data.Altitude = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Altitude, Data_Mask) / 10.0f;
    GPSSolutionData.Horizontal_Acc = NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Horizontal_Acc;
    GPSSolutionData.Vertical_Acc = NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Vertical_Acc;

    //DECODE GPS FIX
    uint8_t FixType = NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Fix_Type ^ Data_Mask;
    if (FixType == FIX_2D)
    {
      GPSSolutionData.FixType = GPS_FIX_2D;
    }
    else if (FixType == FIX_3D)
    {
      GPSSolutionData.FixType = GPS_FIX_3D;
    }
    else
    {
      GPSSolutionData.FixType = GPS_NO_FIX;
    }

    //DECODE A VELOCIDADE NED
    GPSSolutionData.VelocityNED[NORTH] = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.NED_North, Data_Mask);
    GPSSolutionData.VelocityNED[EAST] = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.NED_East, Data_Mask);
    GPSSolutionData.VelocityNED[DOWN] = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.NED_Down, Data_Mask);

    //DECODE DO PDOP QUE AGORA VAI SER HDOP
    uint16_t Position_DOP = Decode16BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Position_DOP, Data_Mask);
    GPSSolutionData.HDOP_State = HDOPMaxError(Position_DOP);

    //DECODE O NÚMERO DE SATELITES
    GPSSolutionData.GPS_NumSat = NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Satellites;

    //CALCULA O GROUND SPEED DADO PELO GPS A PARTIR DAS VELOCIDADES NORTH & EAST
    GPSSolutionData.GroundSpeed = Fast_SquareRoot(Fast_Pow(GPSSolutionData.VelocityNED[NORTH], 2) + Fast_Pow(GPSSolutionData.VelocityNED[EAST], 2));

    //CALCULA O GROUND COURSE DADO PELO GPS A PARTIR DAS VELOCIDADES NORTH & EASTH
    GPSSolutionData.GroundCourse = (uint16_t)(fmodf((Fast_Atan2(GPSSolutionData.VelocityNED[EAST], GPSSolutionData.VelocityNED[NORTH]) * 57.295779513082320876798154814105f) + 3600.0f, 3600.0f));

    GPS_Resources.Navigation.Misc.Velocity.NEDStatus = true;

    //NOVAS INFORMAÇÕES
    GPS_New_Information = true;
    break;
  }

  case ID_MAG:
  {
    Data_Mask_Mag = (NazaGPS_Buffer_Read.NazaGPS_Mag_Data.MagAxisYaw) & 0xFF;
    Data_Mask_Mag = (((Data_Mask_Mag ^ (Data_Mask_Mag >> 4)) & 0x0F) | ((Data_Mask_Mag << 3) & 0xF0)) ^ (((Data_Mask_Mag & 0x01) << 3) | ((Data_Mask_Mag & 0x01) << 7));

    //DECODE OS EIXOS X,Y E Z DO MAGNETOMETRO
    GPSSolutionData.GPS_Read_Compass[ROLL] = Decode16BitsValues(NazaGPS_Buffer_Read.NazaGPS_Mag_Data.MagAxisRoll, Data_Mask_Mag);
    GPSSolutionData.GPS_Read_Compass[PITCH] = Decode16BitsValues(NazaGPS_Buffer_Read.NazaGPS_Mag_Data.MagAxisPitch, Data_Mask_Mag);
    GPSSolutionData.GPS_Read_Compass[YAW] = NazaGPS_Buffer_Read.NazaGPS_Mag_Data.MagAxisYaw ^ (Data_Mask_Mag << 8);
    break;
  }
  }

  if (GPS_New_Information) //ATUALIZA A DECODIFICAÇÃO
  {
    GPS_New_Information = false;
  }
}

void DjiNazaGpsNewFrame(uint8_t SerialReceiverBuffer)
{

  switch (Step)
  {

  case 0: //SINCRONIZAÇÃO DE DATA (0X55)
    if (HEADER1 == SerialReceiverBuffer)
    { //SEM FALHA
      Next_Packet = false;
      Step++;
    }
    break;

  case 1: //SINCRONIZAÇÃO DE DATA (0XAA)
    if (HEADER2 != SerialReceiverBuffer)
    { //FALHA
      Step = 0;
      break;
    }
    Step++; //SEM FALHA
    break;

  case 2:
    Step++;
    CheckPacket_B = CheckPacket_A = SerialReceiverBuffer; //RESETA OS PACOTES DE DADOS
    //ID DO PACOTE DE DADOS PARA DECODIFICAÇÃO
    Decode_Message_ID = SerialReceiverBuffer;
    break;

  case 3:
    //LEITURA DO PACOTE DE DADOS
    Step++;
    CheckPacket_B += (CheckPacket_A += SerialReceiverBuffer); //PACOTE DE DADOS OK
    Payload_Lenght = SerialReceiverBuffer;                    //LEITURA DE VALORES LOWBYTE
    if (Payload_Lenght > 256)
    { //VALOR DO PACOTE MAIOR QUE 256 BYTES?SIM...FALHA,RETORNA AO STEP 0
      Step = 0;
      break;
    }
    //REINICIA PARA NOVOS DADOS
    Payload_Counter = 0;
    if (Payload_Lenght == 0)
    {
      Step = 6;
    }
    break;

  case 4:
    //PACOTE DE DADOS OK
    CheckPacket_B += (CheckPacket_A += SerialReceiverBuffer);
    if (Payload_Counter < 256)
    {
      NazaGPS_Buffer_Read.bytes[Payload_Counter] = SerialReceiverBuffer;
    }
    if (Payload_Counter++ >= Payload_Lenght)
    {
      Step++;
    }
    break;

  case 5:
    Step++;
    if (CheckPacket_A != SerialReceiverBuffer)
    {
      Next_Packet = true; //FALHA
    }
    break;

  case 6:
    Step = 0;
    if (CheckPacket_B != SerialReceiverBuffer)
    {
      break; //FALHA
    }
    if (Next_Packet)
    {
      break;
    }
    NazaGPS_Check_Valid_Data();
  }
  GPS_Resources.Navigation.Misc.Get.Satellites = GPSSolutionData.GPS_NumSat;
  GPS_Resources.Navigation.Misc.Get.HDOP = GPSSolutionData.HDOP_State;
  DJINaza_Compass_Roll = GPSSolutionData.GPS_Read_Compass[ROLL];
  DJINaza_Compass_Pitch = GPSSolutionData.GPS_Read_Compass[PITCH];
  DJINaza_Compass_Yaw = GPSSolutionData.GPS_Read_Compass[YAW];
  GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE] = GPSSolutionData.GPS_Location_Data.Latitude;
  GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE] = GPSSolutionData.GPS_Location_Data.Longitude;
  GPS_Resources.Navigation.Misc.Get.Altitude = (uint16_t)GPSSolutionData.GPS_Location_Data.Altitude;
  GPS_Resources.Navigation.Misc.Get.EstimatedPositionHorizontal = (GPSSolutionData.Horizontal_Acc / 10);
  GPS_Resources.Navigation.Misc.Get.EstimatedPositionVertical = (GPSSolutionData.Vertical_Acc / 10);
  GPS_Resources.Navigation.Misc.Get.GroundCourse = (uint16_t)GPSSolutionData.GroundCourse;
  GPS_Resources.Navigation.Misc.Get.GroundSpeed = (uint16_t)GPSSolutionData.GroundSpeed;
  GPS_Resources.Navigation.Misc.Velocity.Get[NORTH] = GPSSolutionData.VelocityNED[NORTH];
  GPS_Resources.Navigation.Misc.Velocity.Get[EAST] = GPSSolutionData.VelocityNED[EAST];
  GPS_Resources.Navigation.Misc.Velocity.Get[DOWN] = GPSSolutionData.VelocityNED[DOWN];
}
