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

#include "ADC.h"
#include "HAL/HALADC.h"
#include "Build/BOARDDEFS.h"

AnalogReadClass ANALOGSOURCE;

int16_t AnalogReadClass::Read(uint8_t AnalogPin)
{
  return HAL_ADC.AnalogRead(AnalogPin);
}

float AnalogReadClass::Read_Voltage_Ratiometric(uint8_t AnalogPin)
{
#ifndef __AVR_ATmega2560__

  return (float)ANALOGSOURCE.Read(AnalogPin) * (ADC_VOLTAGE_OPERATION / ADC_MAX_SAMPLES);

#else

  return (float)ANALOGSOURCE.Read(AnalogPin) * (ADC_VOLTAGE_OPERATION / ADC_MAX_SAMPLES);
  
#endif
}