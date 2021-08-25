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

#ifndef ABG_H_
#define ABG_H_
#include <inttypes.h>
#include "Common/STRUCTS.h"

typedef struct
{
  float A, B, G, E;
  float aK, vK, xK, jK;
  float DeltaTime;
  float DeltaTime2;
  float DeltaTime3;
  float HalfLife;
  float Boost;
  PT1_Filter_Struct BoostFilter;
  PT1_Filter_Struct VelocityFilter;
  PT1_Filter_Struct AccelerationFilter;
  PT1_Filter_Struct JerkFilter;
} AlphaBetaGammaFilter_Struct;

class ABGFilterClass
{
public:
  void Initialization(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Alpha, int16_t BoostGain, int16_t HalfLife, float DeltaTime);
  float Apply(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Input);
};
extern ABGFilterClass ABGFILTER;
#endif