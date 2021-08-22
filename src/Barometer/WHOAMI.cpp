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

#include "WHOAMI.h"
#include "Common/ENUM.h"
#include "I2C/I2C.h"
#include "Scheduler/SCHEDULERTIME.h"

//ERA PRA ESSAS FUNÇÕES FUNCIONAREM,MAS ELAS MATAM O CICLO DE MAQUINA DO ATMEGA2560 E GERA UM RESET AUTOMATICO
bool GetMS5611DeviceDetect(void)
{
    I2C.WriteRegister(ADDRESS_BAROMETER_MS5611, 0x1E, 0x01);
    SCHEDULERTIME.Sleep(5);
    for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++)
    {
        uint8_t CompareByte = 0;
        SCHEDULERTIME.Sleep(10);
        I2C.RegisterBuffer(ADDRESS_BAROMETER_MS5611, 0xA0, &CompareByte, 0x01);
        if (CompareByte != 0xFF)
        {
            return true;
        }
    }
    return false;
}

bool GetBMP280DeviceDetect(void)
{
    for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++)
    {
        uint8_t CompareByte = 0;
        SCHEDULERTIME.Sleep(100);
        I2C.RegisterBuffer(ADDRESS_BAROMETER_BMP280, 0xD0, &CompareByte, 0x01);
        if (CompareByte == 0x58 || CompareByte == 0x60)
        {
            return true;
        }
    }
    return false;
}
