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

#include "RCSMOOTH.h"
#include "Math/MATHSUPPORT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "BAR/BAR.h"
#include "Filters/BIQUAD.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "PID/RCPID.h"
#include "FastSerial/PRINTF.h"

//DEBUG
//#define PRINTLN_RC_INTERPOLATION

#ifndef __AVR_ATmega2560__
static BiquadFilter_Struct Smooth_RC_Throttle;
static BiquadFilter_Struct Smooth_RC_Yaw;
static BiquadFilter_Struct Smooth_RC_Roll;
static BiquadFilter_Struct Smooth_RC_Pitch;

int16_t RC_LPF_CutOff;
int16_t RCControllerUnFiltered[4];
int16_t RCAttitudeFiltered[4];
#endif

void RCInterpolationInit(void)
{
#ifndef __AVR_ATmega2560__
  RC_LPF_CutOff = STORAGEMANAGER.Read_16Bits(RC_LPF_ADDR);
  BIQUADFILTER.Settings(&Smooth_RC_Throttle, RC_LPF_CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&Smooth_RC_Yaw, RC_LPF_CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&Smooth_RC_Roll, RC_LPF_CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&Smooth_RC_Pitch, RC_LPF_CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
#endif
}

void RCInterpolationApply(void)
{
#ifndef __AVR_ATmega2560__
  if (RC_LPF_CutOff > 0)
  {
    //GUARDA OS VALORES ANTERIOR
    RCControllerUnFiltered[THROTTLE] = RC_Resources.Attitude.Controller[THROTTLE];
    RCControllerUnFiltered[YAW] = RC_Resources.Attitude.Controller[YAW];
    RCControllerUnFiltered[PITCH] = RC_Resources.Attitude.Controller[PITCH];
    RCControllerUnFiltered[ROLL] = RC_Resources.Attitude.Controller[ROLL];

    //APLICA O FILTRO
    RCAttitudeFiltered[THROTTLE] = BIQUADFILTER.ApplyAndGet(&Smooth_RC_Throttle, RCControllerUnFiltered[THROTTLE]);
    RCAttitudeFiltered[YAW] = BIQUADFILTER.ApplyAndGet(&Smooth_RC_Yaw, RCControllerUnFiltered[YAW]);
    RCAttitudeFiltered[PITCH] = BIQUADFILTER.ApplyAndGet(&Smooth_RC_Pitch, RCControllerUnFiltered[PITCH]);
    RCAttitudeFiltered[ROLL] = BIQUADFILTER.ApplyAndGet(&Smooth_RC_Roll, RCControllerUnFiltered[ROLL]);

    //OBTÉM O VALOR FILTRADO
    RC_Resources.Attitude.Controller[THROTTLE] = ((RCAttitudeFiltered[THROTTLE]) >= (RC_Resources.Attitude.ThrottleMin) ? (RCAttitudeFiltered[THROTTLE]) : (RC_Resources.Attitude.ThrottleMin));
    RC_Resources.Attitude.Controller[YAW] = RCAttitudeFiltered[YAW];
    RC_Resources.Attitude.Controller[PITCH] = RCAttitudeFiltered[PITCH];
    RC_Resources.Attitude.Controller[ROLL] = RCAttitudeFiltered[ROLL];
  }
  else
#endif
  {
    RC_Resources.Attitude.Controller[THROTTLE] = ((RC_Resources.Attitude.Controller[THROTTLE]) >= (RC_Resources.Attitude.ThrottleMin) ? (RC_Resources.Attitude.Controller[THROTTLE]) : (RC_Resources.Attitude.ThrottleMin));
  }
#if defined(PRINTLN_RC_INTERPOLATION)
  static uint32_t Refresh = SCHEDULERTIME.GetMillis();
  if (SCHEDULERTIME.GetMillis() - Refresh >= 20)
  {
    DEBUG("NotFiltered:%d Filtered:%d",
          RCControllerUnFiltered[ROLL],
          RC_Resources.Attitude.Controller[ROLL]);
    Refresh = SCHEDULERTIME.GetMillis();
  }
#endif
}
