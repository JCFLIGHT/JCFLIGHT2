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

#include "RCPID.h"
#include "SimpleMode/SIMPLEMODE.h"
#include "TPA.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSMOOTH.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "RadioControl/CURVESRC.h"
#include "MotorsControl/THRBOOST.h"
#include "Filters/PT1.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "RadioControl/DECODE.h"
#include "Common/RCDEFINES.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

RC_Resources_Struct RC_Resources;

#ifndef __AVR_ATmega2560__
PT1_Filter_Struct FixedWingTPA_Smooth;
#endif

//DEBUG
//#define PRINTLN_TPA

bool FixedWingTPAFilterInitialized = false;

static void GetRCDataConvertedAndApplyFilter(void)
{
  int32_t CalcedThrottle;
  CalcedThrottle = Constrain_16Bits(DECODE.GetRxChannelOutput(THROTTLE), RC_Resources.Attitude.ThrottleMin, MAX_STICKS_PULSE);
  CalcedThrottle = (uint32_t)(CalcedThrottle - RC_Resources.Attitude.ThrottleMin) * MIN_STICKS_PULSE / (MAX_STICKS_PULSE - RC_Resources.Attitude.ThrottleMin);
  RC_Resources.Attitude.Controller[THROTTLE] = CalcedLookupThrottle(CalcedThrottle);
  RC_Resources.Attitude.Controller[YAW] = -CalcedAttitudeRC(YAW, RC_Resources.Expo.PitchRoll);
  RC_Resources.Attitude.Controller[PITCH] = CalcedAttitudeRC(PITCH, RC_Resources.Expo.PitchRoll);
  RC_Resources.Attitude.Controller[ROLL] = CalcedAttitudeRC(ROLL, RC_Resources.Expo.Yaw);

  //APLICA O FILTRO LPF NO RC DA ATTITUDE
  RCInterpolationApply();

  //FAZ UMA PEQUENA ZONA MORTA NOS CANAIS DA ATTITUDE
  if (ABS(RC_Resources.Attitude.Controller[YAW]) < 5)
  {
    RC_Resources.Attitude.Controller[YAW] = 0;
  }

  if (ABS(RC_Resources.Attitude.Controller[ROLL]) < 5)
  {
    RC_Resources.Attitude.Controller[ROLL] = 0;
  }

  if (ABS(RC_Resources.Attitude.Controller[PITCH]) < 5)
  {
    RC_Resources.Attitude.Controller[PITCH] = 0;
  }
}

void RC_PID_Update(void)
{
  //CONVERTE AS DATAS DOS RADIO E APLICA O FILTRO LPF
  GetRCDataConvertedAndApplyFilter();

  //APLICA O BOOST NO THROTTLE PARA O MODO STABILIZE
  ApplyThrottleBoost();

#ifndef __AVR_ATmega2560__
  if (GetAirPlaneEnabled() && (TPA_Parameters.FixedWingTauMS > 0))
  {
    if (!FixedWingTPAFilterInitialized)
    {
      FixedWingTPA_Smooth.RC = TPA_Parameters.FixedWingTauMS * 1e-3f;
      FixedWingTPA_Smooth.State = RC_Resources.Attitude.ThrottleMin;
      FixedWingTPAFilterInitialized = true;
    }
    int16_t FilteredThrottle = PT1FilterApply2(&FixedWingTPA_Smooth, RC_Resources.Attitude.Controller[THROTTLE], SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);
    if (FilteredThrottle != TPA_Parameters.PreviousThrottle)
    {
      TPA_Parameters.PreviousThrottle = FilteredThrottle;
      TPA_Parameters.UpdateRequired = true;
    }
  }
  else
#endif
  {
    if (RC_Resources.Attitude.Controller[THROTTLE] != TPA_Parameters.PreviousThrottle)
    {
      TPA_Parameters.PreviousThrottle = RC_Resources.Attitude.Controller[THROTTLE];
      TPA_Parameters.UpdateRequired = true;
    }
  }

  //THROTTLE PID ATTENUATION
  //AJUSTE DINAMICO DO PID DE ACORDO COM O VALOR DO THROTTLE
  if (TPA_Parameters.UpdateRequired)
  {
    if (GetMultirotorEnabled())
    {
      TPA_Parameters.CalcedValue = CalculateMultirotorTPAFactor(RC_Resources.Attitude.Controller[THROTTLE]);
    }
    else if (GetAirPlaneEnabled())
    {
      TPA_Parameters.CalcedValue = CalculateFixedWingTPAFactor(TPA_Parameters.PreviousThrottle);
    }
    TPA_Parameters.UpdateRequired = false;
  }
#if defined(PRINTLN_TPA)
  DEBUG("TPA:%d", TPA_Parameters.CalcedValue);
#endif
  Simple_Mode_Update();
}