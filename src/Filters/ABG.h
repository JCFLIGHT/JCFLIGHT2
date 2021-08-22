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

typedef struct PT1Filter
{
  float State;
  float K;
} PT1_Filter_For_ABG_Struct;

typedef struct
{
  float A, B, G, E;
  float aK, vK, xK, jK;
  float DeltaTime, DeltaTime2, DeltaTime3;
  float HalfLife, Boost;
  PT1_Filter_For_ABG_Struct BoostFilter, VelocityFilter, AcelerationFilter, JerkFilter;
} AlphaBetaGammaFilter_Struct;

void ABG_Initialization(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Alpha, int16_t BoostGain, int16_t HalfLife, float DeltaTime);
float AlphaBetaGammaApply(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Input);
#endif