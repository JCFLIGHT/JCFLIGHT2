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

#include "TPA.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"
#include "PID/RCPID.h"
#include "ASPA.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

TPA_Parameters_Struct TPA_Parameters;

void TPA_Initialization(void)
{
  TPA_Parameters.BreakPointer = STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR);
  TPA_Parameters.ThrottlePercent = STORAGEMANAGER.Read_8Bits(TPA_PERCENT_ADDR); //ESSE PARAMETRO ACIMA DE 50% FUNCIONA BEM PARA AEROS E ASA-FIXA
}

float CalculateFixedWingTPAFactor(int16_t Throttle)
{

#ifdef TEST_AIRSPEED_PID_TPA

  if (Get_ASPA_Enabled())
  {
    return Get_ASPA_Scaler();
  }

#endif

  if (TPA_Parameters.ThrottlePercent != 0 && RC_Resources.Attitude.ThrottleMin < TPA_Parameters.BreakPointer)
  {
    if (Throttle > RC_Resources.Attitude.ThrottleMin)
    {
      TPA_Parameters.Factor = 0.5f + ((float)(TPA_Parameters.BreakPointer - RC_Resources.Attitude.ThrottleMin) / (Throttle - RC_Resources.Attitude.ThrottleMin) / 2.0f);
      TPA_Parameters.Factor = Constrain_Float(TPA_Parameters.Factor, 0.5f, 2.0f);
    }
    else
    {
      TPA_Parameters.Factor = 2.0f;
    }
    TPA_Parameters.Factor = 1.0f + (TPA_Parameters.Factor - 1.0f) * (TPA_Parameters.ThrottlePercent / 100.0f);
  }
  else
  {
    TPA_Parameters.Factor = 1.0f;
  }
  return TPA_Parameters.Factor;
}

float CalculateMultirotorTPAFactor(int16_t Throttle)
{
  if (TPA_Parameters.ThrottlePercent == 0 || Throttle < TPA_Parameters.BreakPointer)
  {
    TPA_Parameters.Factor = 1.0f;
  }
  else if (Throttle < RC_Resources.Attitude.ThrottleMax)
  {
    TPA_Parameters.Factor = (100 - (uint16_t)TPA_Parameters.ThrottlePercent * (Throttle - TPA_Parameters.BreakPointer) / (float)(RC_Resources.Attitude.ThrottleMax - TPA_Parameters.BreakPointer)) / 100.0f;
  }
  else
  {
    TPA_Parameters.Factor = (100 - TPA_Parameters.ThrottlePercent) / 100.0f;
  }
  return TPA_Parameters.Factor;
}