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

#include "AVRSERIAL.h"
#include "Common/ENUM.h"

#ifdef __AVR_ATmega2560__

static uint8_t UARTBufferRX[256][4];
static uint8_t UARTBufferTX[128][4];
static volatile uint8_t UARTHeadRX[4];
static volatile uint8_t UARTHeadTX[4];
static volatile uint8_t UARTTailRX[4];
static volatile uint8_t UARTTailTX[4];

void Serial_Begin(uint8_t SerialPort, uint32_t BaudRate)
{
    uint8_t CalcedHighByte = ((16000000L / 4 / BaudRate - 1) / 2) >> 8;
    uint8_t CalcedLowByte = ((16000000L / 4 / BaudRate - 1) / 2);
    switch (SerialPort)
    {

    case UART_NUMB_0:
        (*(volatile uint8_t *)(0xC0)) = (1 << 1);
        (*(volatile uint8_t *)(0xC5)) = CalcedHighByte;
        (*(volatile uint8_t *)(0xC4)) = CalcedLowByte;
        (*(volatile uint8_t *)(0XC1)) |= (1 << 4) | (1 << 3) | (1 << 7);
        break;

    case UART_NUMB_1:
        (*(volatile uint8_t *)(0xC8)) = (1 << 1);
        (*(volatile uint8_t *)(0xCD)) = CalcedHighByte;
        (*(volatile uint8_t *)(0xCC)) = CalcedLowByte;
        (*(volatile uint8_t *)(0XC9)) |= (1 << 4) | (1 << 3) | (1 << 7);
        break;

    case UART_NUMB_2:
        (*(volatile uint8_t *)(0xD0)) = (1 << 1);
        (*(volatile uint8_t *)(0xD5)) = CalcedHighByte;
        (*(volatile uint8_t *)(0xD4)) = CalcedLowByte;
        (*(volatile uint8_t *)(0xD1)) |= (1 << 4) | (1 << 3) | (1 << 7);
        break;

    case UART_NUMB_3:
        (*(volatile uint8_t *)(0x130)) = (1 << 1);
        (*(volatile uint8_t *)(0x135)) = CalcedHighByte;
        (*(volatile uint8_t *)(0x134)) = CalcedLowByte;
        (*(volatile uint8_t *)(0x131)) |= (1 << 4) | (1 << 3) | (1 << 7);
        break;
    }
}

void Serial_UartSendData(uint8_t SerialPort)
{
    switch (SerialPort)
    {

    case UART_NUMB_0:
        (*(volatile uint8_t *)(0XC1)) |= (1 << 5);
        break;

    case UART_NUMB_1:
        (*(volatile uint8_t *)(0XC9)) |= (1 << 5);
        break;

    case UART_NUMB_2:
        (*(volatile uint8_t *)(0XD1)) |= (1 << 5);
        break;

    case UART_NUMB_3:
        (*(volatile uint8_t *)(0X131)) |= (1 << 5);
        break;
    }
}

bool Serial_TXFree(uint8_t SerialPort)
{
    return (UARTHeadTX[SerialPort] == UARTTailTX[SerialPort]);
}

void Serial_UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort)
{
    uint8_t RXBuffer = UARTHeadRX[SerialPort];
    UARTBufferRX[RXBuffer++][SerialPort] = UartBuffer;
    if (RXBuffer >= 256)
    {
        RXBuffer = 0;
    }
    UARTHeadRX[SerialPort] = RXBuffer;
}

uint8_t Serial_Read(uint8_t SerialPort)
{
    uint8_t CheckRXBuffer = UARTTailRX[SerialPort];
    uint8_t RXBuffer = UARTBufferRX[CheckRXBuffer][SerialPort];
    if (UARTHeadRX[SerialPort] != CheckRXBuffer)
    {
        if (++CheckRXBuffer >= 256)
        {
            CheckRXBuffer = 0;
        }
        UARTTailRX[SerialPort] = CheckRXBuffer;
    }
    return RXBuffer;
}

uint8_t Serial_Available(uint8_t SerialPort)
{
    return ((uint8_t)(UARTHeadRX[SerialPort] - UARTTailRX[SerialPort])) % 256;
}

uint8_t Serial_UsedTXBuffer(uint8_t SerialPort)
{
    return ((uint8_t)(UARTHeadTX[SerialPort] - UARTTailTX[SerialPort])) % 128;
}

void Serial_StoreTX(uint8_t SerialPort, uint8_t WriteTX)
{
    uint8_t TXBuffer = UARTHeadTX[SerialPort];
    if (++TXBuffer >= 128)
    {
        TXBuffer = 0;
    }
    UARTBufferTX[TXBuffer][SerialPort] = WriteTX;
    UARTHeadTX[SerialPort] = TXBuffer;
}

void Serial_Write(uint8_t SerialPort, uint8_t WriteData)
{
    Serial_StoreTX(SerialPort, WriteData);
    Serial_UartSendData(SerialPort);
}

//SERIAL 0 (DEBUG & GCS)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
extern "C" void __vector_26(void) __attribute__((signal, used, externally_visible));
void __vector_26(void)
{
    uint8_t TXBuffer = UARTTailTX[UART_NUMB_0];
    if (UARTHeadTX[UART_NUMB_0] != TXBuffer)
    {
        if (++TXBuffer >= 128)
        {
            TXBuffer = 0;
        }
        (*(volatile uint8_t *)(0XC6)) = UARTBufferTX[TXBuffer][UART_NUMB_0];
        UARTTailTX[UART_NUMB_0] = TXBuffer;
    }
    if (TXBuffer == UARTHeadTX[UART_NUMB_0])
    {
        (*(volatile uint8_t *)(0XC1)) &= ~(1 << 5);
    }
}

//SERIAL 1 (GPS)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
extern "C" void __vector_37(void) __attribute__((signal, used, externally_visible));
void __vector_37(void)
{
    uint8_t TXBuffer = UARTTailTX[UART_NUMB_1];
    if (UARTHeadTX[UART_NUMB_1] != TXBuffer)
    {
        if (++TXBuffer >= 128)
        {
            TXBuffer = 0;
        }
        (*(volatile uint8_t *)(0XCE)) = UARTBufferTX[TXBuffer][UART_NUMB_1];
        UARTTailTX[UART_NUMB_1] = TXBuffer;
    }
    if (TXBuffer == UARTHeadTX[UART_NUMB_1])
    {
        (*(volatile uint8_t *)(0XC9)) &= ~(1 << 5);
    }
}

//SERIAL 2 (SBUS & IBUS)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
extern "C" void __vector_52(void) __attribute__((signal, used, externally_visible));
void __vector_52(void)
{
    uint8_t TXBuffer = UARTTailTX[UART_NUMB_2];
    if (UARTHeadTX[UART_NUMB_2] != TXBuffer)
    {
        if (++TXBuffer >= 128)
        {
            TXBuffer = 0;
        }
        (*(volatile uint8_t *)(0XD6)) = UARTBufferTX[TXBuffer][UART_NUMB_2];
        UARTTailTX[UART_NUMB_2] = TXBuffer;
    }
    if (TXBuffer == UARTHeadTX[UART_NUMB_2])
    {
        (*(volatile uint8_t *)(0XD1)) &= ~(1 << 5);
    }
}

//SERIAL 3 (CARTÃO SD PARA CAIXA PRETA OU OSD)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
extern "C" void __vector_55(void) __attribute__((signal, used, externally_visible));
void __vector_55(void)
{
    uint8_t TXBuffer = UARTTailTX[UART_NUMB_3];
    if (UARTHeadTX[UART_NUMB_3] != TXBuffer)
    {
        if (++TXBuffer >= 128)
        {
            TXBuffer = 0;
        }
        (*(volatile uint8_t *)(0X136)) = UARTBufferTX[TXBuffer][UART_NUMB_3];
        UARTTailTX[UART_NUMB_3] = TXBuffer;
    }
    if (TXBuffer == UARTHeadTX[UART_NUMB_3])
    {
        (*(volatile uint8_t *)(0X131)) &= ~(1 << 5);
    }
}

extern "C" void __vector_25(void) __attribute__((signal, used, externally_visible));
void __vector_25(void)

{
    Serial_UartBufferStore((*(volatile uint8_t *)(0XC6)), UART_NUMB_0);
}

extern "C" void __vector_36(void) __attribute__((signal, used, externally_visible));
void __vector_36(void)
{
    Serial_UartBufferStore((*(volatile uint8_t *)(0XCE)), UART_NUMB_1);
}

extern "C" void __vector_51(void) __attribute__((signal, used, externally_visible));
void __vector_51(void)
{
    Serial_UartBufferStore((*(volatile uint8_t *)(0XD6)), UART_NUMB_2);
}

extern "C" void __vector_54(void) __attribute__((signal, used, externally_visible));
void __vector_54(void)
{
    Serial_UartBufferStore((*(volatile uint8_t *)(0X136)), UART_NUMB_3);
}

#endif