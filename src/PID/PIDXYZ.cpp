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

#include "PIDXYZ.h"
#include "RCPID.h"
#include "FlightModes/AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "BAR/BAR.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Yaw/HEADINGHOLD.h"
#include "RadioControl/CURVESRC.h"
#include "Scheduler/SCHEDULER.h"
#include "Filters/BIQUADFILTER.h"
#include "Filters/PT1.h"
#include "AirSpeed/AIRSPEED.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "RadioControl/RCSTATES.h"
#include "MotorsControl/THRCLIPPING.h"
#include "TPA.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "GPSNavigation/NAVIGATION.h"
#include "FlightModes/FLIGHTMODES.h"
#include "AHRS/AHRS.h"
#include "AHRS/QUATERNION.h"
#include "IMU/ACCGYROREAD.h"
#include "PID/PIDPARAMS.h"
#include "TECS/TECS.h"
#include "BitArray/BITARRAY.h"
#include "Common/RCDEFINES.h"
#include "RadioControl/DECODE.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

PIDXYZClass PIDXYZ;
PID_Resources_Struct PID_Resources;

static BiquadFilter_Struct Derivative_Roll_Smooth;
static BiquadFilter_Struct Derivative_Pitch_Smooth;
static BiquadFilter_Struct Derivative_Yaw_Smooth;
static BiquadFilter_Struct ControlDerivative_Roll_Smooth;
static BiquadFilter_Struct ControlDerivative_Pitch_Smooth;
static BiquadFilter_Struct ControlDerivative_Yaw_Smooth;
#ifdef USE_DERIVATIVE_BOOST_PID
static BiquadFilter_Struct DerivativeBoost_Roll_Smooth;
static BiquadFilter_Struct DerivativeBoost_Pitch_Smooth;
static BiquadFilter_Struct DerivativeBoost_Yaw_Smooth;
#endif

PT1_Filter_Struct Angle_Roll_Smooth;
PT1_Filter_Struct Angle_Pitch_Smooth;
PT1_Filter_Struct WindUpRoll_Smooth;
PT1_Filter_Struct WindUpPitch_Smooth;
#ifdef USE_DERIVATIVE_BOOST_PID
PT1_Filter_Struct DerivativeBoost_PT1_Roll_Smooth;
PT1_Filter_Struct DerivativeBoost_PT1_Pitch_Smooth;
PT1_Filter_Struct DerivativeBoost_PT1_Yaw_Smooth;
#endif

//MIGRAR ESSES PARAMETROS PARA A LISTA COMPLETA DE PARAMETROS
/////////////////////////////////////////////////////////////////

//SAÍDA MAXIMA DE PITCH E ROLL
#define MAX_PID_SUM_LIMIT 500

//SAÍDA MAXIMA DO YAW
#define MAX_YAW_PID_SUM_LIMIT 350

//FREQUENCIA DE CORTE DO GYRO APLICADO AO DERIVATIVE BOOST
#define DERIVATIVE_BOOST_GYRO_CUTOFF 80 //Hz

//FREQUENCIA DE CORTE DA ACELERAÇÃO CALCULADA PELO DERIVATIVE BOOST
#define DERIVATIVE_BOOST_CUTOFF 10 //Hz

int16_t FixedWingIntegralTermThrowLimit = 165; //AJUSTAVEL PELO USUARIO -> (0 a 500)
int16_t MinThrottleDownPitchAngle = 0;         //AJUSTAVEL PELO USUARIO -> (0 a 450)
float CoordinatedPitchGain = 1.0f;             //AJUSTAVEL PELO USUARIO -> (0.0 a 2.0 (float))
float CoordinatedYawGain = 1.0f;               //AJUSTAVEL PELO USUARIO -> (0.0 a 2.0 (float))
float DerivativeBoostFactor = 1.25f;           //AJUSTAVEL PELO USUARIO -> (-1.0 a 3.0 (float))
float DerivativeBoostMaxAceleration = 7500.0f; //AJUSTAVEL PELO USUARIO -> (1000 a 16000)

///////////////////////////////////////////////////////////////

float MotorIntegralTermWindUpPoint;
float AntiWindUpScaler;
float CoordinatedTurnRateEarthFrame;

void PIDXYZClass::Initialization(void)
{
  Load_All_PID_Params();
  PID_Resources.Param.PitchLevelTrim = STORAGEMANAGER.Read_16Bits(PITCH_LEVEL_TRIM_ADDR);
  PID_Resources.Filter.DerivativeCutOff = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
  PID_Resources.Filter.IntegralRelaxCutOff = STORAGEMANAGER.Read_16Bits(INTEGRAL_RELAX_LPF_ADDR);
  PID_Resources.Filter.ControlDerivativeCutOff = STORAGEMANAGER.Read_16Bits(KCD_OR_FF_LPF_ADDR);
  PID_Resources.Param.IntegralTermWindUpPercent = STORAGEMANAGER.Read_8Bits(INTEGRAL_WINDUP_ADDR);
  PID_Resources.Param.ReferenceAirSpeed = STORAGEMANAGER.Read_16Bits(AIR_SPEED_REFERENCE_ADDR);
  BIQUADFILTER.Settings(&Derivative_Roll_Smooth, PID_Resources.Filter.DerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&Derivative_Pitch_Smooth, PID_Resources.Filter.DerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&Derivative_Yaw_Smooth, PID_Resources.Filter.DerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Roll_Smooth, PID_Resources.Filter.ControlDerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Pitch_Smooth, PID_Resources.Filter.ControlDerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Yaw_Smooth, PID_Resources.Filter.ControlDerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
#ifdef USE_DERIVATIVE_BOOST_PID
  BIQUADFILTER.Settings(&DerivativeBoost_Roll_Smooth, DERIVATIVE_BOOST_GYRO_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&DerivativeBoost_Pitch_Smooth, DERIVATIVE_BOOST_GYRO_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
#endif
  PT1FilterInit(&WindUpRoll_Smooth, PID_Resources.Filter.IntegralRelaxCutOff, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);
  PT1FilterInit(&WindUpPitch_Smooth, PID_Resources.Filter.IntegralRelaxCutOff, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);
  MotorIntegralTermWindUpPoint = 1.0f - (PID_Resources.Param.IntegralTermWindUpPercent / 100.0f);
  GPS_Resources.Navigation.HeadingHoldLimit = Fast_Cosine(ConvertToRadians(GET_SET[MAX_ROLL_LEVEL].MaxValue)) * Fast_Cosine(ConvertToRadians(GET_SET[MAX_PITCH_LEVEL].MaxValue));
}

void PIDXYZClass::Update(float DeltaTime)
{
  PID_Resources.RcRateTarget.Roll = RCControllerToRate(RC_Resources.Attitude.Controller[ROLL], RC_Resources.Rate.PitchRoll);
  PID_Resources.RcRateTarget.Pitch = RCControllerToRate(RC_Resources.Attitude.Controller[PITCH], RC_Resources.Rate.PitchRoll);
  PID_Resources.RcRateTarget.Yaw = RCControllerToRate(RC_Resources.Attitude.Controller[YAW], RC_Resources.Rate.Yaw);

  PID_Resources.RcRateTarget.GCS.Roll = PID_Resources.RcRateTarget.Roll;
  PID_Resources.RcRateTarget.GCS.Pitch = PID_Resources.RcRateTarget.Pitch;
  PID_Resources.RcRateTarget.GCS.Yaw = PID_Resources.RcRateTarget.Yaw;

  UpdateHeadingHold();

  if (GetSafeStateOfHeadingHold())
  {
    PID_Resources.RcRateTarget.Yaw = GetHeadingHoldValue(DeltaTime);
  }

  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    PID_Resources.RcRateTarget.Roll = PIDXYZ.LevelRoll(DeltaTime);
    PID_Resources.RcRateTarget.Pitch = PIDXYZ.LevelPitch(DeltaTime);
  }

  if (GetMultirotorEnabled())
  {
    AntiWindUpScaler = Constrain_Float((1.0f - GetMotorMixRange()) / MotorIntegralTermWindUpPoint, 0.0f, 1.0f);
    PIDXYZ.ApplyMulticopterRateControllerRoll(DeltaTime);
    PIDXYZ.ApplyMulticopterRateControllerPitch(DeltaTime);
    PIDXYZ.ApplyMulticopterRateControllerYaw(DeltaTime);
  }
  else if (GetAirPlaneEnabled())
  {
    PIDXYZ.GetNewControllerForPlaneWithTurn();
    PIDXYZ.ApplyFixedWingRateControllerRoll(DeltaTime);
    PIDXYZ.ApplyFixedWingRateControllerPitch(DeltaTime);
    PIDXYZ.ApplyFixedWingRateControllerYaw(DeltaTime);
  }

  if (GetActualThrottleStatus(THROTTLE_LOW) || !IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE) || !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    PIDXYZ.Reset_Integral_Accumulators();
  }
}

float PIDXYZClass::ProportionalTermProcess(uint8_t kP, float RateError, float TPA_Value)
{
  const float NewPTermCalced = ((float)kP / 31.0f * TPA_Value) * RateError;
  return NewPTermCalced;
}

float PIDXYZClass::DerivativeTermProcessRoll(float ActualGyro, float PreviousRateGyro, float PreviousRateTarget, float DeltaTime)
{
  float NewDTermCalced;

  if (GET_SET[PID_ROLL].kD == 0)
  {
    NewDTermCalced = 0.0f;
  }
  else
  {
    float GyroDifference = PreviousRateGyro - ActualGyro;

    if (PID_Resources.Filter.DerivativeCutOff > 0)
    {
      GyroDifference = BIQUADFILTER.ApplyAndGet(&Derivative_Roll_Smooth, GyroDifference);
    }

    NewDTermCalced = GyroDifference * (((float)GET_SET[PID_ROLL].kD / 1905.0f * TPA_Parameters.CalcedValue) / DeltaTime) * PIDXYZ.ApplyDerivativeBoostRoll(IMU.Gyroscope.ReadFloat[ROLL], PreviousRateGyro, PID_Resources.RcRateTarget.Roll, PreviousRateTarget, DeltaTime);
  }

  return NewDTermCalced;
}

float PIDXYZClass::DerivativeTermProcessPitch(float ActualGyro, float PreviousRateGyro, float PreviousRateTarget, float DeltaTime)
{
  float NewDTermCalced;

  if (GET_SET[PID_PITCH].kD == 0)
  {
    NewDTermCalced = 0.0f;
  }
  else
  {
    float GyroDifference = PreviousRateGyro - ActualGyro;

    if (PID_Resources.Filter.DerivativeCutOff > 0)
    {
      NewDTermCalced = BIQUADFILTER.ApplyAndGet(&Derivative_Pitch_Smooth, GyroDifference);
    }

    NewDTermCalced = GyroDifference * (((float)GET_SET[PID_PITCH].kD / 1905.0f * TPA_Parameters.CalcedValue) / DeltaTime) * PIDXYZ.ApplyDerivativeBoostPitch(IMU.Gyroscope.ReadFloat[PITCH], PreviousRateGyro, PID_Resources.RcRateTarget.Pitch, PreviousRateTarget, DeltaTime);
  }

  return NewDTermCalced;
}

float PIDXYZClass::DerivativeTermProcessYaw(float ActualGyro, float PreviousRateGyro, float PreviousRateTarget, float TPA_Value, float DeltaTime)
{
  float NewDTermCalced;

  if (GET_SET[PID_YAW].kD == 0)
  {
    NewDTermCalced = 0.0f;
  }
  else
  {
    float GyroDifference = PreviousRateGyro - ActualGyro;

    if (PID_Resources.Filter.DerivativeCutOff > 0)
    {
      NewDTermCalced = BIQUADFILTER.ApplyAndGet(&Derivative_Yaw_Smooth, GyroDifference);
    }

    NewDTermCalced = GyroDifference * (((float)GET_SET[PID_YAW].kD / 1905.0f * TPA_Value) / DeltaTime) * PIDXYZ.ApplyDerivativeBoostYaw(IMU.Gyroscope.ReadFloat[YAW], PreviousRateGyro, PID_Resources.RcRateTarget.Yaw, PreviousRateTarget, DeltaTime);
  }

  return NewDTermCalced;
}

float PIDXYZClass::ApplyIntegralTermRelaxRoll(float CurrentPIDSetpoint, float IntegralTermErrorRate)
{
  const float SetPointLPF = PT1FilterApply3(&WindUpRoll_Smooth, CurrentPIDSetpoint);
  const float SetPointHPF = ABS(CurrentPIDSetpoint - SetPointLPF);
  const float IntegralTermRelaxFactor = MAX(0, 1 - SetPointHPF / 40.0f);
  return IntegralTermErrorRate * IntegralTermRelaxFactor;
}

float PIDXYZClass::ApplyIntegralTermRelaxPitch(float CurrentPIDSetpoint, float IntegralTermErrorRate)
{
  const float SetPointLPF = PT1FilterApply3(&WindUpPitch_Smooth, CurrentPIDSetpoint);
  const float SetPointHPF = ABS(CurrentPIDSetpoint - SetPointLPF);
  const float IntegralTermRelaxFactor = MAX(0, 1 - SetPointHPF / 40.0f);
  return IntegralTermErrorRate * IntegralTermRelaxFactor;
}

float PIDXYZClass::ApplyIntegralTermLimiting(uint8_t Axis, float ErrorGyroIntegral)
{
  if ((MixerIsOutputSaturated() && GetMultirotorEnabled()) || (GetAirPlaneEnabled() && PIDXYZ.FixedWingIntegralTermLimitActive(Axis)))
  {
    ErrorGyroIntegral = Constrain_Float(ErrorGyroIntegral, -PID_Resources.Controller.Integral.ErrorGyroLimit[Axis], PID_Resources.Controller.Integral.ErrorGyroLimit[Axis]);
  }
  else
  {
    PID_Resources.Controller.Integral.ErrorGyroLimit[Axis] = ABS(ErrorGyroIntegral);
  }
  return ErrorGyroIntegral;
}

float PIDXYZClass::ApplyDerivativeBoostRoll(float ActualGyro, float PrevGyro, float ActualRateTagert, float PrevRateTagert, float DeltaTime)
{
  float DerivativeBoost = 1.0f;

#ifdef USE_DERIVATIVE_BOOST_PID

  if (DerivativeBoostFactor > 1)
  {
    const float DerivativeBoostGyroDelta = (ActualGyro - PrevGyro) / DeltaTime;
    const float DerivativeBoostGyroAcceleration = ABS(BIQUADFILTER.ApplyAndGet(&DerivativeBoost_Roll_Smooth, DerivativeBoostGyroDelta));
    const float DerivativeBoostRateAcceleration = ABS((ActualRateTagert - PrevRateTagert) / DeltaTime);
    const float Acceleration = MAX(DerivativeBoostGyroAcceleration, DerivativeBoostRateAcceleration);
    DerivativeBoost = ScaleRangeFloat(Acceleration, 0.0f, DerivativeBoostMaxAceleration, 1.0f, DerivativeBoostFactor);
    DerivativeBoost = PT1FilterApply(&DerivativeBoost_PT1_Roll_Smooth, DerivativeBoost, DERIVATIVE_BOOST_CUTOFF, DeltaTime);
    DerivativeBoost = Constrain_Float(DerivativeBoost, 1.0f, DerivativeBoostFactor);
  }

#endif

  return DerivativeBoost;
}

float PIDXYZClass::ApplyDerivativeBoostPitch(float ActualGyro, float PrevGyro, float ActualRateTagert, float PrevRateTagert, float DeltaTime)
{
  float DerivativeBoost = 1.0f;

#ifdef USE_DERIVATIVE_BOOST_PID

  if (DerivativeBoostFactor > 1)
  {
    const float DerivativeBoostGyroDelta = (ActualGyro - PrevGyro) / DeltaTime;
    const float DerivativeBoostGyroAcceleration = ABS(BIQUADFILTER.ApplyAndGet(&DerivativeBoost_Pitch_Smooth, DerivativeBoostGyroDelta));
    const float DerivativeBoostRateAcceleration = ABS((ActualRateTagert - PrevRateTagert) / DeltaTime);
    const float Acceleration = MAX(DerivativeBoostGyroAcceleration, DerivativeBoostRateAcceleration);
    DerivativeBoost = ScaleRangeFloat(Acceleration, 0.0f, DerivativeBoostMaxAceleration, 1.0f, DerivativeBoostFactor);
    DerivativeBoost = PT1FilterApply(&DerivativeBoost_PT1_Pitch_Smooth, DerivativeBoost, DERIVATIVE_BOOST_CUTOFF, DeltaTime);
    DerivativeBoost = Constrain_Float(DerivativeBoost, 1.0f, DerivativeBoostFactor);
  }

#endif

  return DerivativeBoost;
}

float PIDXYZClass::ApplyDerivativeBoostYaw(float ActualGyro, float PrevGyro, float ActualRateTagert, float PrevRateTagert, float DeltaTime)
{
  float DerivativeBoost = 1.0f;

#ifdef USE_DERIVATIVE_BOOST_PID

  if (DerivativeBoostFactor > 1)
  {
    const float DerivativeBoostGyroDelta = (ActualGyro - PrevGyro) / DeltaTime;
    const float DerivativeBoostGyroAcceleration = ABS(BIQUADFILTER.ApplyAndGet(&DerivativeBoost_Yaw_Smooth, DerivativeBoostGyroDelta));
    const float DerivativeBoostRateAcceleration = ABS((ActualRateTagert - PrevRateTagert) / DeltaTime);
    const float Acceleration = MAX(DerivativeBoostGyroAcceleration, DerivativeBoostRateAcceleration);
    DerivativeBoost = ScaleRangeFloat(Acceleration, 0.0f, DerivativeBoostMaxAceleration, 1.0f, DerivativeBoostFactor);
    DerivativeBoost = PT1FilterApply(&DerivativeBoost_PT1_Yaw_Smooth, DerivativeBoost, DERIVATIVE_BOOST_CUTOFF, DeltaTime);
    DerivativeBoost = Constrain_Float(DerivativeBoost, 1.0f, DerivativeBoostFactor);
  }

#endif

  return DerivativeBoost;
}

float PIDXYZClass::LevelRoll(float DeltaTime)
{
  float RcControllerAngle = 0.0f;

  if (GPS_Resources.Navigation.AutoPilot.Control.Enabled)
  {
    RcControllerAngle = RcControllerToAngle(GPS_Resources.Navigation.AutoPilot.Control.Angle[ROLL], ConvertDegreesToDecidegrees(GET_SET[MAX_ROLL_LEVEL].MaxValue));
  }
  else
  {
    if (GetMultirotorEnabled() && IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
    {
      RcControllerAngle = RcControllerToAngle(RC_Resources.Attitude.Controller[ROLL], ConvertDegreesToDecidegrees(GET_SET[ATTACK_BANK_MAX].MaxValue));
    }
    else
    {
      RcControllerAngle = RcControllerToAngle(RC_Resources.Attitude.Controller[ROLL], ConvertDegreesToDecidegrees(GET_SET[MAX_ROLL_LEVEL].MaxValue));
    }
  }

  float AngleErrorInDegrees = ConvertDeciDegreesToDegrees(RcControllerAngle - Attitude.Raw[ROLL]);

  float AngleRateTarget = Constrain_Float(AngleErrorInDegrees * (GET_SET[PI_AUTO_LEVEL].kP / 6.56f), -ConvertDegreesToDecidegrees(RC_Resources.Rate.PitchRoll), ConvertDegreesToDecidegrees(RC_Resources.Rate.PitchRoll));

  if (GET_SET[PI_AUTO_LEVEL].kI > 0)
  {

#ifndef __AVR_ATmega2560__

    AngleRateTarget = PT1FilterApply(&Angle_Roll_Smooth, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].kI, DeltaTime);

#else

    AngleRateTarget = PT1FilterApply(&Angle_Roll_Smooth, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].kI, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);

#endif
  }
  return AngleRateTarget;
}

float PIDXYZClass::LevelPitch(float DeltaTime)
{
  float RcControllerAngle = 0.0f;

  if (GPS_Resources.Navigation.AutoPilot.Control.Enabled)
  {
    RcControllerAngle = RcControllerToAngle(GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH], ConvertDegreesToDecidegrees(GET_SET[MAX_PITCH_LEVEL].MaxValue));
  }
  else
  {
    if (GetMultirotorEnabled() && IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
    {
      RcControllerAngle = RcControllerToAngle(RC_Resources.Attitude.Controller[PITCH], ConvertDegreesToDecidegrees(GET_SET[ATTACK_BANK_MAX].MaxValue));
    }
    else
    {
      RcControllerAngle = RcControllerToAngle(RC_Resources.Attitude.Controller[PITCH], ConvertDegreesToDecidegrees(GET_SET[MAX_PITCH_LEVEL].MaxValue));
    }
  }

  RcControllerAngle += TECS.AutoPitchDown(MinThrottleDownPitchAngle);

  if (GetAirPlaneEnabled())
  {
    RcControllerAngle -= ConvertDegreesToDecidegrees(PID_Resources.Param.PitchLevelTrim);
  }

  float AngleErrorInDegrees = ConvertDeciDegreesToDegrees(RcControllerAngle - Attitude.Raw[PITCH]);

  float AngleRateTarget = Constrain_Float(AngleErrorInDegrees * (GET_SET[PI_AUTO_LEVEL].kP / 6.56f), -ConvertDegreesToDecidegrees(RC_Resources.Rate.PitchRoll), ConvertDegreesToDecidegrees(RC_Resources.Rate.PitchRoll));

  if (GET_SET[PI_AUTO_LEVEL].kI > 0)
  {

#ifndef __AVR_ATmega2560__

    AngleRateTarget = PT1FilterApply(&Angle_Pitch_Smooth, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].kI, DeltaTime);
#else

    AngleRateTarget = PT1FilterApply(&Angle_Pitch_Smooth, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].kI, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);

#endif
  }
  return AngleRateTarget;
}

void PIDXYZClass::ApplyMulticopterRateControllerRoll(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Roll - IMU.Gyroscope.ReadFloat[ROLL];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_ROLL].kP, RateError, TPA_Parameters.CalcedValue);
  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessRoll(IMU.Gyroscope.ReadFloat[ROLL], PreviousRateGyro, PreviousRateTarget, DeltaTime);

  const float RateTargetDelta = PID_Resources.RcRateTarget.Roll - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.ApplyAndGet(&ControlDerivative_Roll_Smooth, RateTargetDelta);

  const float NewControlDerivativeTerm = RateTargetDeltaFiltered * (((float)GET_SET[PID_ROLL].kFF / 7270.0f * TPA_Parameters.CalcedValue) / DeltaTime);

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + PID_Resources.Controller.Integral.ErrorGyro[ROLL] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxRoll(PID_Resources.RcRateTarget.Roll, RateError);

  float NewControlTracking;

  if ((GET_SET[PID_ROLL].kP != 0) && (GET_SET[PID_ROLL].kI != 0))
  {
    NewControlTracking = 2.0f / ((((float)GET_SET[PID_ROLL].kP / 31.0f * TPA_Parameters.CalcedValue) /
                                  ((float)GET_SET[PID_ROLL].kI / 4.0f)) +
                                 (((float)GET_SET[PID_ROLL].kD / 1905.0f * TPA_Parameters.CalcedValue) /
                                  ((float)GET_SET[PID_ROLL].kP / 31.0f * TPA_Parameters.CalcedValue)));
  }
  else
  {
    NewControlTracking = 0;
  }

  PID_Resources.Controller.Integral.ErrorGyro[ROLL] += (IntegralTermErrorRate * ((float)GET_SET[PID_ROLL].kI / 4.0f) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  PID_Resources.Controller.Integral.ErrorGyro[ROLL] = PIDXYZ.ApplyIntegralTermLimiting(ROLL, PID_Resources.Controller.Integral.ErrorGyro[ROLL]);

  PID_Resources.Controller.Output.Calced[ROLL] = NewOutputLimited;

  PreviousRateTarget = PID_Resources.RcRateTarget.Roll;
  PreviousRateGyro = IMU.Gyroscope.ReadFloat[ROLL];
}

void PIDXYZClass::ApplyMulticopterRateControllerPitch(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Pitch - IMU.Gyroscope.ReadFloat[PITCH];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_PITCH].kP, RateError, TPA_Parameters.CalcedValue);
  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessPitch(IMU.Gyroscope.ReadFloat[PITCH], PreviousRateGyro, PreviousRateTarget, DeltaTime);

  const float RateTargetDelta = PID_Resources.RcRateTarget.Pitch - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.ApplyAndGet(&ControlDerivative_Pitch_Smooth, RateTargetDelta);

  const float NewControlDerivativeTerm = RateTargetDeltaFiltered * (((float)GET_SET[PID_PITCH].kFF / 7270.0f * TPA_Parameters.CalcedValue) / DeltaTime);

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + PID_Resources.Controller.Integral.ErrorGyro[PITCH] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxPitch(PID_Resources.RcRateTarget.Pitch, RateError);

  float NewControlTracking;

  if ((GET_SET[PID_PITCH].kP != 0) && (GET_SET[PID_PITCH].kI != 0))
  {
    NewControlTracking = 2.0f / ((((float)GET_SET[PID_PITCH].kP / 31.0f * TPA_Parameters.CalcedValue) /
                                  ((float)GET_SET[PID_PITCH].kI / 4.0f)) +
                                 (((float)GET_SET[PID_PITCH].kD / 1905.0f * TPA_Parameters.CalcedValue) /
                                  ((float)GET_SET[PID_PITCH].kP / 31.0f * TPA_Parameters.CalcedValue)));
  }
  else
  {
    NewControlTracking = 0;
  }

  PID_Resources.Controller.Integral.ErrorGyro[PITCH] += (IntegralTermErrorRate * (GET_SET[PID_PITCH].kI / 4.0f * TPA_Parameters.CalcedValue) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  PID_Resources.Controller.Integral.ErrorGyro[PITCH] = PIDXYZ.ApplyIntegralTermLimiting(PITCH, PID_Resources.Controller.Integral.ErrorGyro[PITCH]);

  PID_Resources.Controller.Output.Calced[PITCH] = NewOutputLimited;

  PreviousRateTarget = PID_Resources.RcRateTarget.Pitch;
  PreviousRateGyro = IMU.Gyroscope.ReadFloat[PITCH];
}

void PIDXYZClass::ApplyMulticopterRateControllerYaw(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Yaw - IMU.Gyroscope.ReadFloat[YAW];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[YAW].kP, RateError, 1.0f);

  const float RateTargetDelta = PID_Resources.RcRateTarget.Yaw - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.ApplyAndGet(&ControlDerivative_Yaw_Smooth, RateTargetDelta);

  const float NewControlDerivativeTerm = RateTargetDeltaFiltered * (((float)GET_SET[PID_YAW].kFF / 7270.0f) / DeltaTime);

  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessYaw(IMU.Gyroscope.ReadFloat[YAW], PreviousRateGyro, PreviousRateTarget, 1.0f, DeltaTime);

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + PID_Resources.Controller.Integral.ErrorGyro[YAW] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_YAW_PID_SUM_LIMIT, +MAX_YAW_PID_SUM_LIMIT);

  //float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxYaw(PID_Resources.RcRateTarget.Yaw, RateError);

  float IntegralTermErrorRate = RateError;

  float NewControlTracking;

  if ((GET_SET[PID_YAW].kP != 0) && (GET_SET[PID_YAW].kI != 0))
  {
    NewControlTracking = 2.0f / ((((float)GET_SET[PID_YAW].kP / 31.0f) /
                                  ((float)GET_SET[PID_YAW].kI / 4.0f)) +
                                 (((float)GET_SET[PID_YAW].kD / 1905.0f) /
                                  ((float)GET_SET[PID_YAW].kP / 31.0f)));
  }
  else
  {
    NewControlTracking = 0;
  }

  PID_Resources.Controller.Integral.ErrorGyro[YAW] += (IntegralTermErrorRate * (GET_SET[PID_YAW].kI / 4.0f) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  PID_Resources.Controller.Integral.ErrorGyro[YAW] = PIDXYZ.ApplyIntegralTermLimiting(YAW, PID_Resources.Controller.Integral.ErrorGyro[YAW]);

  PID_Resources.Controller.Output.Calced[YAW] = NewOutputLimited;

  PreviousRateTarget = PID_Resources.RcRateTarget.Yaw;
  PreviousRateGyro = IMU.Gyroscope.ReadFloat[YAW];
}

void PIDXYZClass::ApplyFixedWingRateControllerRoll(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Roll - IMU.Gyroscope.ReadFloat[ROLL];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_ROLL].kP, RateError, TPA_Parameters.CalcedValue);
  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessRoll(IMU.Gyroscope.ReadFloat[ROLL], PreviousRateGyro, PreviousRateTarget, DeltaTime);
  const float NewFeedForwardTerm = PID_Resources.RcRateTarget.Roll * ((float)GET_SET[PID_ROLL].kFF / 31.0f * TPA_Parameters.CalcedValue);

  PID_Resources.Controller.Integral.ErrorGyro[ROLL] += RateError * ((float)GET_SET[PID_ROLL].kI / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  PID_Resources.Controller.Integral.ErrorGyro[ROLL] = PIDXYZ.ApplyIntegralTermLimiting(ROLL, PID_Resources.Controller.Integral.ErrorGyro[ROLL]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    PID_Resources.Controller.Integral.ErrorGyro[ROLL] = Constrain_Float(PID_Resources.Controller.Integral.ErrorGyro[ROLL], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PID_Resources.Controller.Output.Calced[ROLL] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + PID_Resources.Controller.Integral.ErrorGyro[ROLL] + NewDerivativeTerm, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  PreviousRateTarget = PID_Resources.RcRateTarget.Roll;
  PreviousRateGyro = IMU.Gyroscope.ReadFloat[ROLL];
}

void PIDXYZClass::ApplyFixedWingRateControllerPitch(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Pitch - IMU.Gyroscope.ReadFloat[PITCH];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_PITCH].kP, RateError, TPA_Parameters.CalcedValue);
  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessPitch(IMU.Gyroscope.ReadFloat[PITCH], PreviousRateGyro, PreviousRateTarget, DeltaTime);
  const float NewFeedForwardTerm = PID_Resources.RcRateTarget.Pitch * ((float)GET_SET[PID_PITCH].kFF / 31.0f * TPA_Parameters.CalcedValue);

  PID_Resources.Controller.Integral.ErrorGyro[PITCH] += RateError * ((float)GET_SET[PID_PITCH].kI / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  PID_Resources.Controller.Integral.ErrorGyro[PITCH] = PIDXYZ.ApplyIntegralTermLimiting(PITCH, PID_Resources.Controller.Integral.ErrorGyro[PITCH]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    PID_Resources.Controller.Integral.ErrorGyro[PITCH] = Constrain_Float(PID_Resources.Controller.Integral.ErrorGyro[PITCH], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PID_Resources.Controller.Output.Calced[PITCH] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + PID_Resources.Controller.Integral.ErrorGyro[PITCH] + NewDerivativeTerm, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  PreviousRateTarget = PID_Resources.RcRateTarget.Pitch;
  PreviousRateGyro = IMU.Gyroscope.ReadFloat[PITCH];
}

void PIDXYZClass::ApplyFixedWingRateControllerYaw(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Yaw - IMU.Gyroscope.ReadFloat[YAW];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_YAW].kP, RateError, TPA_Parameters.CalcedValue);
  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessYaw(IMU.Gyroscope.ReadFloat[YAW], PreviousRateGyro, PreviousRateTarget, TPA_Parameters.CalcedValue, DeltaTime);
  const float NewFeedForwardTerm = PID_Resources.RcRateTarget.Yaw * ((float)GET_SET[PID_YAW].kFF / 31.0f * TPA_Parameters.CalcedValue);

  PID_Resources.Controller.Integral.ErrorGyro[YAW] += RateError * ((float)GET_SET[PID_YAW].kI / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  PID_Resources.Controller.Integral.ErrorGyro[YAW] = PIDXYZ.ApplyIntegralTermLimiting(YAW, PID_Resources.Controller.Integral.ErrorGyro[YAW]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    PID_Resources.Controller.Integral.ErrorGyro[YAW] = Constrain_Float(PID_Resources.Controller.Integral.ErrorGyro[YAW], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PID_Resources.Controller.Output.Calced[YAW] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + PID_Resources.Controller.Integral.ErrorGyro[YAW] + NewDerivativeTerm, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  PreviousRateTarget = PID_Resources.RcRateTarget.Yaw;
  PreviousRateGyro = IMU.Gyroscope.ReadFloat[YAW];
}

bool PIDXYZClass::FixedWingIntegralTermLimitActive(uint8_t Axis)
{
  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    return false;
  }
  float StickPosition = (float)Constrain_16Bits(DECODE.GetRxChannelOutput(Axis) - MIDDLE_STICKS_PULSE, -500, 500) / 500.0f;
  return ABS(StickPosition) > 0.5f; //MAIS DE 50% DE DEFLEXÃO
}

void PIDXYZClass::GetNewControllerForPlaneWithTurn(void)
{
  Vector3x3_Struct TurnControllerRates;
  TurnControllerRates.Roll = 0;
  TurnControllerRates.Pitch = 0;

  if (!IS_FLIGHT_MODE_ACTIVE(TURN_MODE) || !IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    return;
  }
  else
  {
    if (AHRS.CheckAnglesInclination(10)) //10 GRAUS DE INCLINAÇÃO
    {
      //SE O PITOT NÃO ESTIVER A BORDO,UTILIZE O VALOR PADRÃO DE 1500CM/S = 54KM/H
      float AirSpeedForCoordinatedTurn = Get_AirSpeed_Enabled() ? AIRSPEED.Get_True_Value("In Centimeters") : PID_Resources.Param.ReferenceAirSpeed;
      //LIMITE DE 10KM/H - 216KM/H
      AirSpeedForCoordinatedTurn = Constrain_Float(AirSpeedForCoordinatedTurn, 300, 6000);
      float BankAngleTarget = ConvertDeciDegreesToRadians(RcControllerToAngle(RC_Resources.Attitude.Controller[ROLL], ConvertDegreesToDecidegrees(GET_SET[NAV_ROLL_BANK_MAX].MaxValue)));
      float FinalBankAngleTarget = Constrain_Float(BankAngleTarget, -ConvertToRadians(60), ConvertToRadians(60));
      float PitchAngleTarget = ConvertDeciDegreesToRadians(RcControllerToAngle(RC_Resources.Attitude.Controller[PITCH], ConvertDegreesToDecidegrees(GET_SET[NAV_PITCH_BANK_MAX].MaxValue)));
      float TurnRatePitchAdjustmentFactor = Fast_Cosine(ABS(PitchAngleTarget));
      CoordinatedTurnRateEarthFrame = ConvertToDegrees(GRAVITY_CMSS * Fast_Tangent(-FinalBankAngleTarget) / AirSpeedForCoordinatedTurn * TurnRatePitchAdjustmentFactor);
      TurnControllerRates.Yaw = CoordinatedTurnRateEarthFrame;
    }
    else
    {
      return;
    }
  }

  //CONVERTE DE EARTH-FRAME PARA BODY-FRAME
  AHRS.TransformVectorEarthFrameToBodyFrame(&TurnControllerRates);

  //LIMITA O VALOR MINIMO E MAXIMO DE SAÍDA A PARTIR DOS VALOR DE RATE DEFINIDO PELO USUARIO NO GCS
  PID_Resources.RcRateTarget.Roll = Constrain_16Bits(PID_Resources.RcRateTarget.Roll + TurnControllerRates.Roll, -ConvertDegreesToDecidegrees(RC_Resources.Rate.PitchRoll), ConvertDegreesToDecidegrees(RC_Resources.Rate.PitchRoll));
  PID_Resources.RcRateTarget.Pitch = Constrain_16Bits(PID_Resources.RcRateTarget.Pitch + TurnControllerRates.Pitch * CoordinatedPitchGain, -ConvertDegreesToDecidegrees(RC_Resources.Rate.PitchRoll), ConvertDegreesToDecidegrees(RC_Resources.Rate.PitchRoll));
  PID_Resources.RcRateTarget.Yaw = Constrain_16Bits(PID_Resources.RcRateTarget.Yaw + TurnControllerRates.Yaw * CoordinatedYawGain, -ConvertDegreesToDecidegrees(RC_Resources.Rate.Yaw), ConvertDegreesToDecidegrees(RC_Resources.Rate.Yaw));
}

void PIDXYZClass::Reset_Integral_Accumulators(void)
{
  PID_Resources.Controller.Integral.ErrorGyro[ROLL] = 0;
  PID_Resources.Controller.Integral.ErrorGyro[PITCH] = 0;
  PID_Resources.Controller.Integral.ErrorGyro[YAW] = 0;
  PID_Resources.Controller.Integral.ErrorGyroLimit[ROLL] = 0;
  PID_Resources.Controller.Integral.ErrorGyroLimit[PITCH] = 0;
  PID_Resources.Controller.Integral.ErrorGyroLimit[YAW] = 0;
}