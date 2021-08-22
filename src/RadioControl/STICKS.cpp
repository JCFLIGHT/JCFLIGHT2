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

#include "STICKS.h"
#include "LedRGB/LEDRGB.h"
#include "FlightModes/AUXFLIGHT.h"
#include "BatteryMonitor/BATTERY.h"
#include "RCSTATES.h"
#include "Buzzer/BUZZER.h"
#include "Arming/ARMING.h"
#include "Common/RCDEFINES.h"
#include "RCSTATES.h"
#include "Compass/COMPASSREAD.h"
#include "FailSafe/FAILSAFE.h"
#include "AHRS/AHRS.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "BitArray/BITARRAY.h"

SticksClass STICKS;

void SticksClass::Update()
{
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    if ((AUXFLIGHT.GetModeConfiguration[SECONDARY_ARM_DISARM] == NONE) && (GetSticksStateToArm()))
    {
      if (!BATTERY.GetExhausted())
      {
        if (GetArmDelayedState())
        {
          STICKS.PreArm_Run = true;
        }
      }
    }
    else
    {
      ResetArmDelayed();
    }
  }
  else
  {
    if ((AUXFLIGHT.GetModeConfiguration[SECONDARY_ARM_DISARM] == NONE) && (GetSticksStateToDisarm()))
    {
      if (GetDisarmDelayedState())
      {
        DISABLE_THIS_STATE(PRIMARY_ARM_DISARM);
        BEEPER.Play(BEEPER_DISARMING);
      }
    }
    else
    {
      ResetDisarmDelayed();
    }
  }

  if (GetActualThrottleStatus(THROTTLE_LOW))
  {
    if (!FastSystemFailSafe())
    {
      if (AUXFLIGHT.GetModeConfiguration[SECONDARY_ARM_DISARM] > NONE)
      {
        if (IS_STATE_ACTIVE(SECONDARY_ARM_DISARM))
        {
          if (!SystemInFailSafe() && !IS_FLIGHT_MODE_ACTIVE(RTH_MODE) && !IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE))
          {
            if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
            {
              ENABLE_THIS_STATE(PRIMARY_ARM_DISARM);
              Calibration.Magnetometer.SimpleModeHeading = Attitude.EulerAngles.Yaw;
            }
          }
        }
        else
        {
          DISABLE_THIS_STATE(PRIMARY_ARM_DISARM);
        }
      }
    }
  }
}

void SticksClass::Pre_Arm(void)
{
  if (AUXFLIGHT.GetModeConfiguration[SECONDARY_ARM_DISARM] > NONE)
  {
    return; //FAÇA UMA RAPIDA SAÍDA SE O ARM-DISARM ESTIVER CONFIGURADO PELA CHAVE AUX
  }

  //ROTINA PRE-ARM
  if (STICKS.PreArm_Run)
  {
    STICKS.PreArm_Run_Count++;
    if (STICKS.PreArm_Run_Count > 30)
    {
      if (!PREARM.CheckSafeState()) //CONDIÇÕES INCORRETAS?SIM...NÃO ARMA OS MOTORES
      {
        DISABLE_THIS_STATE(PRIMARY_ARM_DISARM);
      }
      else //IMU CALIBRADA?SIM...ARMA OS MOTORES
      {
        ENABLE_THIS_STATE(PRIMARY_ARM_DISARM);
        Calibration.Magnetometer.SimpleModeHeading = Attitude.EulerAngles.Yaw;
      }
      STICKS.PreArm_Run = false;
      STICKS.PreArm_Run_Count = 0;
    }
  }
}

void SticksClass::Pre_Arm_Leds(void)
{
  //ROTINA PRE-ARM LED INDICADOR
  if ((STICKS.PreArm_Run_Count > 0 && STICKS.PreArm_Run_Count <= 20))
  {
    RGB.Function(CALL_LED_PRE_ARM_INIT);
    BEEPER.Play(BEEPER_ARM);
  }

  if (!PREARM.CheckSafeState()) //SE TIVER ALGUMA CONDIÇÃO INCORRETA,NÃO ARMA
  {
    if ((STICKS.PreArm_Run_Count > 20 && STICKS.PreArm_Run_Count <= 30))
    {
      RGB.Function(CALL_LED_PRE_ARM_FAIL);
      if (STICKS.PreArm_Run_Count == 21)
      {
        BEEPER.Play(BEEPER_ACTION_FAIL);
      }
    }
  }
  else //CASO CONTRARIO
  {
    if ((STICKS.PreArm_Run_Count > 20 && STICKS.PreArm_Run_Count <= 30))
    {
      RGB.Function(CALL_LED_PRE_ARM_SUCESS);
    }
  }
}