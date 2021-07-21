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

#include "ALTITUDEHOLD.h"
#include "Scheduler/SCHEDULER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "RadioControl/RCSTATES.h"
#include "RadioControl/DECODE.h"
#include "GPSNavigation/NAVIGATION.h"
#include "PID/RCPID.h"
#include "FlightModes/FLIGHTMODES.h"
#include "PID/PIDPARAMS.h"
#include "Barometer/BAROBACKEND.h"
#include "GPS/GPSSTATES.h"
#include "BitArray/BITARRAY.h"
#include "Common/RCDEFINES.h"

AltitudeHold_Controller_Struct AltitudeHoldController;

#define SAFE_ALTITUDE 5              //ALTITUDE SEGURA PARA O LAND E TAKE-OFF EM METROS
#define MIN_TARGET_POS_Z 50          //GANHO DE ALTITUDE DURANTE O LAND E ACIMA DA ALTITUDE DADA PELO PARAM "SAFE_ALTITUDE" EM CM DE 30 ~ 100
#define VALID_ALT_REACHED 50         //RESULTADO DA SUBTRAÇÃO ENTRE A ALTITUDE MARCADA COMO HOLD E A ALTIUDE ATUAL PARA VALIDAR QUE ALTITUDE FOI ALCANÇADA EM CM
#define MIN_VEL_Z_TO_VALID_GROUND 15 //VELOCIDADE VERTICAL MINIMA PARA INDICAR QUE A VEL Z DO INS ESTÁ EM REPOUSO
#define LANDED_TIME 4000             //ESTOURO DE TEMPO EM MS PARA INDICAR QUE REALMENTE O SOLO FOI DETECTADO
#define MAX_ALTITUDE_SUPORTED 150    //ALTITUDE MAXIMA SUPORTADA PELO ALGORITIMO EM METROS,PODE SER INCREMENTADO,MAS É NECESSARIO TESTES DE FUNCIONAMENTO
#define THR_DIFF_COMPLETE_TAKEOFF 70 //1500 INDICA PARA O CONTROLADOR DO ALT-HOLD QUE O TAKEOFF ESTÁ EM ANDAMENDO OU FOI CONCLUIDO.

//#define THR_SMOOTH_TEST

#ifdef THR_SMOOTH_TEST

//É NECESSARIO TESTES PARA DEFINIR SE SERÁ IMPLEMENTADO OU NÃO

#include "FastSerial/PRINTF.h"
#include "Filters/PT1.h"

PT1_Filter_Struct Smooth_ThrottleHover;

#define ALT_HOLD_LPF_CUTOFF 4 //HZ

#endif

static void ResetIntegralOfVelZError(void)
{
  AltitudeHoldController.PID.IntegratorSum = Constrain_32Bits(AltitudeHoldController.PID.IntegratorSum, -8192000, 8192000);
  AltitudeHoldController.PID.IntegratorError = Constrain_16Bits(AltitudeHoldController.PID.IntegratorError, -125, 125);
}

static void ResetLandDetector(void)
{
  AltitudeHoldController.Time.LandDetectorStart = SCHEDULERTIME.GetMillis();
  AltitudeHoldController.Time.OnLand = 0;
  AltitudeHoldController.Flags.GroundAltitudeSet = false;
}

static void RunLandDetector(void)
{
  if (GetGroundDetected())
  {
    AltitudeHoldController.Time.OnLand = SCHEDULERTIME.GetMillis() - AltitudeHoldController.Time.LandDetectorStart;
  }
  else
  {
    ResetLandDetector();
  }
  if (!AltitudeHoldController.Flags.GroundAltitudeSet && (AltitudeHoldController.Time.OnLand >= 100))
  {
    AltitudeHoldController.Flags.GroundAltitudeSet = true;
    Barometer.Altitude.GroundOffSet = Barometer.Altitude.Actual;
  }
}

bool ApplyAltitudeHoldControl(void)
{
  static Scheduler_Struct AltitudeHoldControlTimer;
  if (Scheduler(&AltitudeHoldControlTimer, SCHEDULER_SET_FREQUENCY(50, "Hz")))
  {
    static bool BaroModeActivated = false;
    if ((Do_Altitude_Hold || Do_RTH_Or_Land_Call_Alt_Hold) && IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
      if (!BaroModeActivated)
      {
        BaroModeActivated = true;
        AltitudeHoldController.Flags.Hovering = false;
        AltitudeHoldController.Flags.TakeOffInProgress = false;
        if ((AltitudeHoldController.Throttle.Hovering < (MIDDLE_STICKS_PULSE - 250)) ||
            (AltitudeHoldController.Throttle.Hovering > (MIDDLE_STICKS_PULSE + 250)))
        {
          AltitudeHoldController.Throttle.Hovering = Constrain_16Bits(AltitudeHoldController.Throttle.Hover, MIDDLE_STICKS_PULSE - 250, MIDDLE_STICKS_PULSE + 250);
        }
        ResetLandDetector();
      }
      RunLandDetector();
      if (Do_RTH_Or_Land_Call_Alt_Hold)
      {
        AltitudeHoldController.Flags.TakeOffInProgress = false;
        if (Get_GPS_Used_To_Land())
        {
          AltitudeHoldController.Flags.Hovering = false;
          SetNewAltitudeToHold(Barometer.INS.Altitude.Estimated);
          if (Barometer.INS.Altitude.Estimated > ConverMetersToCM(SAFE_ALTITUDE))
          {
            AltitudeHoldController.Target.Position.Z = MIN_TARGET_POS_Z + ((int32_t)(250 - MIN_TARGET_POS_Z) * (Barometer.INS.Altitude.Estimated - ConverMetersToCM(SAFE_ALTITUDE)) / (ConverMetersToCM(GPS_Resources.Home.Altitude) - ConverMetersToCM(SAFE_ALTITUDE)));
          }
          AltitudeHoldController.Target.Position.Z = -AltitudeHoldController.Target.Position.Z;
        }
        else
        {
          AltitudeHoldController.Flags.Hovering = true;
          AltitudeHoldController.Target.Position.Z = ((AltitudeHoldController.Target.Altitude - Barometer.INS.Altitude.Estimated) * GET_SET[PID_POSITION_Z].kP) / 2;
          if (Barometer.INS.Altitude.Estimated > ConverMetersToCM(SAFE_ALTITUDE))
          {
            AltitudeHoldController.Target.Position.Z = Constrain_32Bits(AltitudeHoldController.Target.Position.Z, -250, 250);
          }
          else
          {
            AltitudeHoldController.Target.Position.Z = Constrain_32Bits(AltitudeHoldController.Target.Position.Z, -83, 83);
          }
        }
      }
      else
      {
        AltitudeHoldController.Throttle.Difference = DECODE.GetRxChannelOutput(THROTTLE) - MIDDLE_STICKS_PULSE;
        if (!AltitudeHoldController.Flags.TakeOffInProgress && GetActualThrottleStatus(THROTTLE_LOW) && GetGroundDetectedFor100ms())
        {
          AltitudeHoldController.Flags.TakeOffInProgress = true;
        }
        else
        {
          if ((AltitudeHoldController.Throttle.Difference > THR_DIFF_COMPLETE_TAKEOFF) &&
              (Barometer.INS.Velocity.Vertical >= MIN_VEL_Z_TO_VALID_GROUND))
          {
            AltitudeHoldController.Flags.TakeOffInProgress = false;
          }
        }
        if (AltitudeHoldController.Flags.TakeOffInProgress ||
            (ABS(AltitudeHoldController.Throttle.Difference) > THR_DIFF_COMPLETE_TAKEOFF))
        {
          AltitudeHoldController.Flags.Hovering = false;
          if (ABS(AltitudeHoldController.Throttle.Difference) <= THR_DIFF_COMPLETE_TAKEOFF)
          {
            AltitudeHoldController.Target.Position.Z = 0;
          }
          else
          {
            AltitudeHoldController.Target.Position.Z = ((AltitudeHoldController.Throttle.Difference - ((AltitudeHoldController.Throttle.Difference > 0) ? THR_DIFF_COMPLETE_TAKEOFF : -THR_DIFF_COMPLETE_TAKEOFF)) * GET_SET[PID_POSITION_Z].kP) / 4;
          }
        }
        else
        {
          if (!AltitudeHoldController.Flags.Hovering)
          {
            AltitudeHoldController.Flags.Hovering = true;
            AltitudeHoldController.Target.Altitude = Barometer.INS.Altitude.Estimated;
          }
          AltitudeHoldController.Target.Position.Z = ((AltitudeHoldController.Target.Altitude - Barometer.INS.Altitude.Estimated) * GET_SET[PID_POSITION_Z].kP) / 2;
        }
      }
      AltitudeHoldController.Target.Position.Z = Constrain_32Bits(AltitudeHoldController.Target.Position.Z, -350, 350);
      AltitudeHoldController.Target.Velocity.Z = Constrain_32Bits(AltitudeHoldController.Target.Position.Z - Barometer.INS.Velocity.Vertical, -600, 600);
      AltitudeHoldController.PID.IntegratorSum += Constrain_32Bits(((AltitudeHoldController.Target.Velocity.Z * GET_SET[PID_VELOCITY_Z].kI * AltitudeHoldControlTimer.ActualTime) / 128) / ((AltitudeHoldController.Flags.Hovering && ABS(AltitudeHoldController.Target.Position.Z) < 100) ? 2 : 1), -16384000, 16384000);
      AltitudeHoldController.PID.IntegratorError = Constrain_16Bits((AltitudeHoldController.PID.IntegratorSum / 65536), -250, 250);
      AltitudeHoldController.PID.Control = ((AltitudeHoldController.Target.Velocity.Z * GET_SET[PID_VELOCITY_Z].kP) / 32) + AltitudeHoldController.PID.IntegratorError - (((int32_t)INS.AccelerationEarthFrame_Filtered[INS_VERTICAL_Z] * GET_SET[PID_VELOCITY_Z].kD) / 64);

      RC_Resources.Attitude.Controller[THROTTLE] = Constrain_16Bits(AltitudeHoldController.Throttle.Hovering + AltitudeHoldController.PID.Control, RC_Resources.Attitude.ThrottleMin + ALT_HOLD_DEADBAND, RC_Resources.Attitude.ThrottleMax - ALT_HOLD_DEADBAND);

#ifdef THR_SMOOTH_TEST

      int16_t PrevThrottle = RC_Resources.Attitude.Controller[THROTTLE];

      RC_Resources.Attitude.Controller[THROTTLE] = (int16_t)PT1FilterApply(&Smooth_ThrottleHover, RC_Resources.Attitude.Controller[THROTTLE], ALT_HOLD_LPF_CUTOFF, AltitudeHoldControlTimer.ActualTime * 1e-6f);

      DEBUG("ThrottleUnFilt:%d ThrottleFilt:%d", PrevThrottle, RC_Resources.Attitude.Controller[THROTTLE]);

#endif

      return true;
    }
    else
    {
      if (BaroModeActivated)
      {
        BaroModeActivated = false;
        AltitudeHoldController.Flags.TakeOffInProgress = false;
        AltitudeHoldController.Target.Position.Z = 0;
        AltitudeHoldController.Throttle.Hover = STORAGEMANAGER.Read_16Bits(HOVER_THROTTLE_ADDR);
        ResetIntegralOfVelZError();
        ResetLandDetector();
      }
    }
  }
  return false;
}

void SetNewAltitudeToHold(int32_t AltitudeSetPoint)
{
  float Max_Altitude_Suported = ConverMetersToCM(MAX_ALTITUDE_SUPORTED);
  if (AltitudeSetPoint > Max_Altitude_Suported)
  {
    AltitudeSetPoint = Max_Altitude_Suported;
  }
  AltitudeHoldController.Target.Altitude = AltitudeSetPoint;
}

bool GetTakeOffInProgress(void)
{
  return AltitudeHoldController.Flags.TakeOffInProgress;
}

bool GetAltitudeReached(void)
{
  return ABS(AltitudeHoldController.Target.Altitude - Barometer.INS.Altitude.Estimated) < VALID_ALT_REACHED;
}

bool GetGroundDetected(void)
{
  return (ABS(Barometer.INS.Velocity.Vertical) < MIN_VEL_Z_TO_VALID_GROUND) &&
         (AltitudeHoldController.PID.IntegratorError <= -185) &&
         (Barometer.INS.Altitude.Estimated < ConverMetersToCM(SAFE_ALTITUDE));
}

bool GetGroundDetectedFor100ms(void)
{
  return AltitudeHoldController.Flags.GroundAltitudeSet;
}

bool GetLanded(void)
{
  return AltitudeHoldController.Flags.GroundAltitudeSet &&
         (AltitudeHoldController.Time.OnLand >= LANDED_TIME);
}