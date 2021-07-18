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

#ifndef GAUSSNEWTON_H_
#define GAUSSNEWTON_H_
#include "Common/STRUCTS.h"
void ClearGaussNewtonMatrices(Jacobian_Struct *JacobianPointer);
void GaussNewtonPushSampleForOffSetCalculation(Jacobian_Struct *JacobianPointer, int32_t SensorSample[3]);
void GaussNewtonPushSampleForScaleCalculation(Jacobian_Struct *JacobianPointer, int16_t AxisIndex, int32_t SensorSample[3], int16_t Target);
bool GaussNewtonSolveForOffSet(Jacobian_Struct *JacobianPointer, float Result[3]);
bool GaussNewtonSolveForScale(Jacobian_Struct *JacobianPointer, float Result[3]);
#endif