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

#ifndef BIQUAD_H
#define BIQUAD_H
#include "Build/LIBDEPENDENCIES.h"
#include "Common/ENUM.h"
typedef struct
{
  float Beta0, Beta1, Beta2, Alpha1, Alpha2;
  float SampleX1, SampleX2, SampleY1, SampleY2;
} BiquadFilter_Struct;
class BiQuadFilter
{
public:
  void Settings(BiquadFilter_Struct *Filter, uint16_t FilterFreq, int16_t CutOffFreq, uint32_t SampleIntervalMicros, uint8_t FilterType);
  float ApplyAndGet(BiquadFilter_Struct *Filter, float DeviceToFilter);
};
extern BiQuadFilter BIQUADFILTER;
#endif
