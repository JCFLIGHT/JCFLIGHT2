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

#include "PRECISIONLAND.h"
#include "Filters/PT1.h"
#include "Math/MATHSUPPORT.h"
#include "Common/STRUCTS.h"
#include "InertialNavigation/INS.h"
#include "Build/BOARDDEFS.h"
#include "FastSerial/PRINTF.h"

#ifndef __AVR_ATmega2560__

#define LOOP_RATE_IN_HZ 400                              //HZ
#define LAND_CHECK_ACCEL_MOVING JCF_Param.Land_Check_Acc //M/S^2

#else

#define LOOP_RATE_IN_HZ THIS_LOOP_RATE_IN_US //HZ - O FILTRO SÓ FUNCIONA CORRETAMENTE COM ESSE VALOR EM 1KHZ NA VERSÃO CLASSIC
#define LAND_CHECK_ACCEL_MOVING 3.0f         //M/S^2

#endif

#define LPF_CUTOFF_IN_HZ 1.0f //HZ

//DEBUG
//#define PRINTLN_PRECISION_LAND

PT1_Filter_Struct AccelerationEarthFrame_Smooth[3];

void Update_PrecisionLand(void)
{
  PT1FilterApply(&AccelerationEarthFrame_Smooth[NORTH], INS.NewAccelerationEarthFrame.Roll, LPF_CUTOFF_IN_HZ, 1.0f / LOOP_RATE_IN_HZ);
  PT1FilterApply(&AccelerationEarthFrame_Smooth[EAST], INS.NewAccelerationEarthFrame.Pitch, LPF_CUTOFF_IN_HZ, 1.0f / LOOP_RATE_IN_HZ);
  PT1FilterApply(&AccelerationEarthFrame_Smooth[UP], INS.NewAccelerationEarthFrame.Yaw, LPF_CUTOFF_IN_HZ, 1.0f / LOOP_RATE_IN_HZ);

#ifdef PRINTLN_PRECISION_LAND

  DEBUG("GetAccelerationTotal:%.4f GetLandSuccess:%d", GetAccelerationTotal(), GetLandSuccess());

#endif
}

//CALCULA A RAIZ QUADRADA DE TODAS AS ACELERAÇÕES PRESENTES NO EARTH-FRAME COM 1G SUBTRAIDO NO EIXO Z
float GetAccelerationTotal(void)
{
  return Fast_SquareRoot(SquareFloat(AccelerationEarthFrame_Smooth[NORTH].State) +
                         SquareFloat(AccelerationEarthFrame_Smooth[EAST].State) +
                         SquareFloat(AccelerationEarthFrame_Smooth[UP].State));
}

//SE A VELOCIDADE FOR MAIOR OU IGUAL AO PARAMETRO "LAND_CHECK_ACCEL_MOVING" ISSO QUER DIZER QUE O UAV NÃO ESTÁ NO CHÃO
bool GetLandSuccess(void)
{
  if (GetAccelerationTotal() >= LAND_CHECK_ACCEL_MOVING)
  {
    return false;
  }
  return true;
}
