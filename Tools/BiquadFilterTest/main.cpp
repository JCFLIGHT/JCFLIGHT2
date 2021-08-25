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

#include "BIQUAD.h"

static BiquadFilter_Struct Smooth_AnalogRead;

#define THIS_LOOP_FREQ 1000 //FREQUENCIA DO LOOP INFINITO
#define FILTER_CUTOFF_FREQ 20    //20HZ

void setup()
{
    Serial.begin(115200);
    BIQUADFILTER.Settings(&Smooth_AnalogRead, FILTER_CUTOFF_FREQ, 0, THIS_LOOP_FREQ, LPF);
}

void loop()
{
    int16_t GetAnalogReadValue = analogRead(0); //PINO ANALOGICO A0
    Serial.print("RealAnalog:");
    Serial.print(GetAnalogReadValue);
    Serial.print(" AnalogFiltered:");
    Serial.println(BIQUADFILTER.ApplyAndGet(&Smooth_AnalogRead, GetAnalogReadValue));
    delay(1); //1KHZ LOOP
}