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

#include "HALSERIAL.h"
#include "HALLIBRARY.h"
#include "Common/ENUM.h"
#include "PID/RCPID.h"
#include "GPS/GPSUBLOX.h"

HALSerialClass HAL_SERIAL;

void HALSerialClass::Initialization()
{
    //DEBUG E GCS
    Serial_Begin(UART_NUMB_0, 115200);
    //GPS
    Serial_Begin(UART_NUMB_1, 57600);
    GPS_SerialInit(57600);
    //IBUS & SBUS
    if (RC_Resources.ReceiverTypeEnabled == PPM_RECEIVER)
    {
        Serial_Begin(UART_NUMB_2, 115200);
    }
    if (RC_Resources.ReceiverTypeEnabled == SBUS_RECEIVER)
    {
        //CONFIGURAÇÃO DA UART_NUMB_2 PARA SBUS
        Serial_Begin(UART_NUMB_2, 100000);
#ifdef __AVR_ATmega2560__
        (*(volatile uint8_t *)(0xD2)) |= (1 << 5) | (1 << 3);
#endif
    }
    else if (RC_Resources.ReceiverTypeEnabled == IBUS_RECEIVER)
    {
        //CONFIGURAÇÃO DA UART_NUMB_2 PARA IBUS
        Serial_Begin(UART_NUMB_2, 115200);
    }
    //CARTÃO SD PARA CAIXA PRETA OU OSD
    Serial_Begin(UART_NUMB_3, 115200);
}

void HALSerialClass::Begin(uint8_t SerialPort, uint32_t BaudRate)
{
    Serial_Begin(SerialPort, BaudRate);
}

void HALSerialClass::UartSendData(uint8_t SerialPort)
{
    Serial_UartSendData(SerialPort);
}

bool HALSerialClass::TXFree(uint8_t SerialPort)
{
    return Serial_TXFree(SerialPort);
}

void HALSerialClass::UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort)
{
    Serial_UartBufferStore(UartBuffer, SerialPort);
}

uint8_t HALSerialClass::Read(uint8_t SerialPort)
{
    return Serial_Read(SerialPort);
}

uint8_t HALSerialClass::Available(uint8_t SerialPort)
{
    return Serial_Available(SerialPort);
}

uint8_t HALSerialClass::UsedTXBuffer(uint8_t SerialPort)
{
    return Serial_UsedTXBuffer(SerialPort);
}

void HALSerialClass::StoreTX(uint8_t SerialPort, uint8_t WriteTX)
{
    Serial_StoreTX(SerialPort, WriteTX);
}

void HALSerialClass::Write(uint8_t SerialPort, uint8_t WriteData)
{
    Serial_Write(SerialPort, WriteData);
}