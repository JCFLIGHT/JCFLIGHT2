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

#include "AUTOLAUNCH.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "AHRS/AHRS.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSTATES.h"
#include "AHRS/VECTOR.h"
#include "Buzzer/BUZZER.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "PID/RCPID.h"
#include "Barometer/BAROBACKEND.h"
#include "GPS/GPSUBLOX.h"
#include "GPS/GPSSTATES.h"
#include "Param/PARAM.h"
#include "I2C/I2C.h"
#include "GPSNavigation/NAVIGATION.h"
#include "RadioControl/DECODE.h"
#include "BitArray/BITARRAY.h"
#include "RadioControl/CURVESRC.h"
#include "PID/PIDPARAMS.h"
#include "InertialNavigation/INS.h"

AutoLaunchClass AUTOLAUNCH;

#ifdef __AVR_ATmega2560__

#define AHRS_BANKED_ANGLE 45                                 //25 GRAUS MAXIMO DE BANK ANGLE (CONSIDERANDO EM RADIANOS = 436)
#define LAUNCH_MOTOR_IDLE_SPINUP_TIME 1500                   //ARMA O MOTOR DEPOIS DE 1.5 SEGUNDO APÓS DETECTAR O AUTO-LAUNCH
#define AUTO_LAUNCH_ANGLE 18                                 //VALOR DO PITCH (ELEVATOR) AO FAZER O AUTO-LAUNCH (VALOR EM GRAUS)
#define SWING_LAUNCH_MIN_ROTATION_RATE ConvertToRadians(100) //NO MINIMO UM RATE DE 100DPS NO GYRO
#define LAUNCH_VELOCITY_THRESH 3                             //METROS/S
#define MOTOR_SPINUP_VALUE 100                               //VALOR DA INCREMENTAÇÃO DO THROTTLE PARA PLANES COM RODAS
#define MOTOR_SPINUP_TIME 300                                //VAI SUBINDO O THROTTLE AOS POUCOS,BOM PARA AERO COM RODAS (TEMPO EM MS)
#define AUTO_LAUCH_EXIT_FUNCTION 5000                        //TEMPO DE PARA SAIR DO MODO AUTO-LAUCH APÓS A DETECÇÃO (TEMPO EM MS)
#define AUTO_LAUNCH_THROTTLE_MAX 1700                        //VALOR MAXIMO DE ACELERAÇÃO
#define AUTO_LAUCH_MAX_ALTITUDE 0                            //ALTITUDE MAXIMA PARA VALIDAR O AUTO-LAUNCH (VALOR EM METROS)

#else

#define AHRS_BANKED_ANGLE JCF_Param.AutoLaunch_AHRS_BankAngle                  //'N' GRAUS MAXIMO DE BANK ANGLE
#define LAUNCH_MOTOR_IDLE_SPINUP_TIME JCF_Param.AutoLaunch_Trigger_Motor_Delay //ARMA O MOTOR DEPOIS DE 'N' SEGUNDO APÓS DETECTAR O AUTO-LAUNCH (TEMPO EM MS)
#define AUTO_LAUNCH_ANGLE JCF_Param.AutoLaunch_Elevator                        //VALOR DO PITCH (ELEVATOR) AO FAZER O AUTO-LAUNCH (VALOR EM GRAUS)
#define SWING_LAUNCH_MIN_ROTATION_RATE ConvertToRadians(100)                   //NO MINIMO UM RATE DE 100DPS NO GYRO
#define LAUNCH_VELOCITY_THRESH JCF_Param.AutoLaunch_Velocity_Thresh            //METROS/S
#define MOTOR_SPINUP_VALUE JCF_Param.AutoLaunch_SpinUp                         //VALOR DA INCREMENTAÇÃO DO THROTTLE PARA PLANES COM RODAS
#define MOTOR_SPINUP_TIME JCF_Param.AutoLaunch_SpinUp_Time                     //VAI SUBINDO O THROTTLE AOS POUCOS,BOM PARA AERO COM RODAS (TEMPO EM MS)
#define AUTO_LAUCH_EXIT_FUNCTION JCF_Param.AutoLaunch_Exit                     //TEMPO DE PARA SAIR DO MODO AUTO-LAUCH APÓS A DETECÇÃO (TEMPO EM MS)
#define AUTO_LAUNCH_THROTTLE_MAX JCF_Param.AutoLaunch_MaxThrottle              //VALOR MAXIMO DE ACELERAÇÃO
#define AUTO_LAUCH_MAX_ALTITUDE JCF_Param.AutoLaunch_Altitude                  //ALTITUDE MAXIMA PARA VALIDAR O AUTO-LAUNCH (VALOR EM METROS)

#endif

bool LaunchEnabled = false;
bool LaunchedDetect = false;
bool BeeperOnce = false;
bool IgnoreFirstPeakSpinUpThrottle = false;
bool IgnoreFirstPeakOverFlowTime = false;

uint32_t ThrottleSpinUpStart = 0;
uint32_t AutoLaunchDetectorPreviousTime = 0;
uint32_t AutoLaunchDetectorSum = 0;
uint32_t AutoLaunchAbortTime = 0;

const float GetPitchAccelerationInMSS(void)
{
  return BodyFrameAcceleration.Pitch;
}

const float GetRollAccelerationInMSS(void)
{
  return BodyFrameAcceleration.Roll;
}

const float GetYawRotationInRadians(void)
{
  return BodyFrameRotation.Yaw;
}

bool AutoLaunchClass::GetSwingVelocityState(void)
{
  const float SwingVelocity = (ABS(GetYawRotationInRadians()) > SWING_LAUNCH_MIN_ROTATION_RATE) ? (GetRollAccelerationInMSS() / GetYawRotationInRadians()) : 0;
  return (SwingVelocity > ConverMetersToCM(LAUNCH_VELOCITY_THRESH)) && (GetPitchAccelerationInMSS() > 0);
}

bool AutoLaunchClass::GetForwardState(void)
{
  return Get_GPS_Heading_Is_Valid() && (GetRollAccelerationInMSS() > 0) && (GPS_Resources.Navigation.Misc.Get.GroundSpeed > ConverMetersToCM(LAUNCH_VELOCITY_THRESH));
}

void AutoLaunchClass::AutomaticDetector()
{
  if (AHRS.CheckAnglesInclination(AHRS_BANKED_ANGLE) ||
      AUTOLAUNCH.GetSwingVelocityState() ||
      AUTOLAUNCH.GetForwardState())
  {
    AutoLaunchDetectorSum += (SCHEDULERTIME.GetMillis() - AutoLaunchDetectorPreviousTime);
    AutoLaunchDetectorPreviousTime = SCHEDULERTIME.GetMillis();
    if (AutoLaunchDetectorSum >= 40)
    {
      LaunchEnabled = true;
    }
  }
  else
  {
    AutoLaunchDetectorPreviousTime = SCHEDULERTIME.GetMillis();
    AutoLaunchDetectorSum = 0;
  }
}

void AutoLaunchClass::Update(void)
{
  if (!GetAirPlaneEnabled())
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(LAUNCH_MODE))
  {
    if (AUTOLAUNCH.GetValidStateToRunLaunch() && !LaunchedDetect)
    {
      if (!LaunchEnabled)
      {
        BEEPER.Play(BEEPER_AUTO_LAUNCH);
      }
      else
      {
        AUTOLAUNCH.RCControllerThrottle_Apply_Logic();

        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
          ENABLE_THIS_STATE(PRIMARY_ARM_DISARM);
        }

        if (AUTOLAUNCH.GetStatusCompleted())
        {
          LaunchedDetect = true;
          LaunchEnabled = false;
        }

        if (!BeeperOnce)
        {
          BEEPER.Silence();
          BEEPER.Play(BEEPER_LAUNCHED);
          BeeperOnce = true;
        }
      }

      AUTOLAUNCH.AutomaticDetector();
      AUTOLAUNCH.RCControllerYawPitchRoll_Apply_Logic();
    }
  }

  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && !AUTOLAUNCH.GetValidStateToRunLaunch())
  {
    AUTOLAUNCH.ResetParameters();
  }
}

void AutoLaunchClass::RCControllerThrottle_Apply_Logic(void)
{
  if (SCHEDULERTIME.GetMillis() - ThrottleSpinUpStart >= LAUNCH_MOTOR_IDLE_SPINUP_TIME)
  {
    if (IgnoreFirstPeakSpinUpThrottle)
    {
      RC_Resources.Attitude.Controller[THROTTLE] = AUTO_LAUNCH_THROTTLE_MAX;
    }
    else
    {
      ThrottleSpinUpStart = SCHEDULERTIME.GetMillis();
    }
    IgnoreFirstPeakSpinUpThrottle = true;
  }
  else
  {
    RC_Resources.Attitude.Controller[THROTTLE] = RC_Resources.Attitude.ThrottleMin; //VAMOS MANTER A VELOCIDADE MINIMA DEFINIDA PELO USUARIO
  }
}

void AutoLaunchClass::RCControllerYawPitchRoll_Apply_Logic(void)
{
  RC_Resources.Attitude.Controller[ROLL] = 0;
  RC_Resources.Attitude.Controller[PITCH] = PIDAngleToRcController(-ConvertDegreesToDecidegrees(AUTO_LAUNCH_ANGLE), ConvertDegreesToDecidegrees(GET_SET[MAX_PITCH_LEVEL].MaxValue));
  RC_Resources.Attitude.Controller[YAW] = 0;
}

bool AutoLaunchClass::GetValidStateToRunLaunch(void)
{
  return GetActualThrottleStatus(THROTTLE_MIDDLE);
}

bool AutoLaunchClass::GetTimerOverFlow(void)
{
  if (SCHEDULERTIME.GetMillis() - AutoLaunchAbortTime >= AUTO_LAUCH_EXIT_FUNCTION)
  {
    if (!IgnoreFirstPeakOverFlowTime)
    {
      AutoLaunchAbortTime = SCHEDULERTIME.GetMillis();
    }
    else
    {
      return true;
    }
    IgnoreFirstPeakOverFlowTime = true;
  }
  return false;
}

bool AutoLaunchClass::GetMaxAltitudeReached(void)
{
  if (!I2CResources.Found.Barometer)
  {
    return false;
  }

  return ((AUTO_LAUCH_MAX_ALTITUDE * 100) > 0) && (INS_Resources.Estimated.Position.Yaw >= (AUTO_LAUCH_MAX_ALTITUDE * 100));
}

bool AutoLaunchClass::GetStatusCompleted(void)
{
  return ((AUTO_LAUCH_MAX_ALTITUDE * 100) > 0 ? false : AUTOLAUNCH.GetTimerOverFlow()) || (GetSticksDeflected(15)) || (AUTOLAUNCH.GetMaxAltitudeReached());
}

bool AutoLaunchClass::GetLaunchFinalized(void)
{
  return !LaunchEnabled;
}

void AutoLaunchClass::ResetParameters(void)
{
  //RESETA OS PARAMETROS QUANDO ESTIVER DESARMADO
  BeeperOnce = false;
  LaunchedDetect = false;
  IgnoreFirstPeakSpinUpThrottle = false;
  IgnoreFirstPeakOverFlowTime = false;
  ThrottleSpinUpStart = 0;
  AutoLaunchAbortTime = 0;
}