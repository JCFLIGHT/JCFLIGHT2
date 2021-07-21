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

#include "SERVOSAUTOTRIM.h"
#include "AirPlane/AIRPLANE.h"
#include "PID/PIDXYZ.h"
#include "SwitchFlag/SWITCHFLAG.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Buzzer/BUZZER.h"
#include "MotorsControl/MOTORS.h"
#include "BitArray/BITARRAY.h"
#include "SERVOSMASTER.h"
#include "ServosMaster/SERVOSMASTER.h"
#include "AHRS/AHRS.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Filters/PT1.h"
#include "GPS/GPSSTATES.h"
#include "PID/PIDXYZ.h"
#include "Build/BOARDDEFS.h"
#include "Param/PARAM.h"

static PT1_Filter_Struct RotationRateFilter;
static PT1_Filter_Struct TargetRateFilter;

//DEFINIÇÕES PARA O TRIM CONTINUO DOS SERVOS
#define SERVO_AUTOTRIM_FILTER_CUTOFF 1
#define SERVO_AUTOTRIM_UPDATE_SIZE 5
#define SERVO_AUTOTRIM_CENTER_MIN 1300
#define SERVO_AUTOTRIM_CENTER_MAX 1700

//DEFINIÇÕES PARA O AUTO-TRIM
#define SERVO_AUTOTRIM_OVERFLOW 2000
#define SAVE_OVERFLOW 2500

void ServosSaveAndUpdateMiddlePoint(void)
{
    SAVE_SERVO_MIDDLE(SERVO1_MID_ADDR, Servo.Pulse.Middle[SERVO1]);
    SAVE_SERVO_MIDDLE(SERVO2_MID_ADDR, Servo.Pulse.Middle[SERVO2]);
    SAVE_SERVO_MIDDLE(SERVO3_MID_ADDR, Servo.Pulse.Middle[SERVO3]);
    SAVE_SERVO_MIDDLE(SERVO4_MID_ADDR, Servo.Pulse.Middle[SERVO4]);
    SERVOSMASTER.UpdateMiddlePoint();
}

void ServoAutoTrimRun(const float DeltaTime)
{

    if (Servo.ContinousTrim.Enabled)
    {
        ProcessContinuousServoAutoTrim(DeltaTime);
        return;
    }

    static ServoAutoTrimState_Enum ServoAutoTrimState = SERVO_AUTOTRIM_IDLE;

    Servo.AutoTrim.ActualPulse[SERVO1] = MotorControl[MOTOR2];
    Servo.AutoTrim.ActualPulse[SERVO2] = MotorControl[MOTOR3];
    Servo.AutoTrim.ActualPulse[SERVO3] = MotorControl[MOTOR4];
    Servo.AutoTrim.ActualPulse[SERVO4] = MotorControl[MOTOR5];

    if (Servo.AutoTrim.Enabled)
    {
        switch (ServoAutoTrimState)
        {
        case SERVO_AUTOTRIM_IDLE:
            if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
            {
                for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
                {
                    Servo.AutoTrim.MiddleBackup[ServoIndex] = Servo.Pulse.Middle[ServoIndex];
                    Servo.AutoTrim.MiddleAccum[ServoIndex] = 0;
                }

                Servo.AutoTrim.PreviousTime = SCHEDULERTIME.GetMillis();
                Servo.AutoTrim.MiddleAccumCount = 0;
                ServoAutoTrimState = SERVO_AUTOTRIM_COLLECTING;
            }
            else
            {
                break;
            }

        case SERVO_AUTOTRIM_COLLECTING:
            if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
            {
                Servo.AutoTrim.MiddleAccumCount++;

                for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
                {
                    Servo.AutoTrim.MiddleAccum[ServoIndex] += Servo.AutoTrim.ActualPulse[ServoIndex];
                }

                if ((SCHEDULERTIME.GetMillis() - Servo.AutoTrim.PreviousTime) > SERVO_AUTOTRIM_OVERFLOW)
                {
                    for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
                    {
                        Servo.Pulse.Middle[ServoIndex] = Servo.AutoTrim.MiddleAccum[ServoIndex] / Servo.AutoTrim.MiddleAccumCount;
                    }
                    ServoAutoTrimState = SERVO_AUTOTRIM_SAVE_PENDING;
                    PIDXYZ.Reset_Integral_Accumulators();
                }
            }
            else
            {
                ServoAutoTrimState = SERVO_AUTOTRIM_IDLE;
            }
            break;

        case SERVO_AUTOTRIM_SAVE_PENDING:
            if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
            {
                if ((SCHEDULERTIME.GetMillis() - Servo.AutoTrim.SavePreviousTime) > SAVE_OVERFLOW)
                {
                    ServosSaveAndUpdateMiddlePoint();
                    BEEPER.Play(BEEPER_ACTION_SUCCESS);
                    ServoAutoTrimState = SERVO_AUTOTRIM_DONE;
                }
            }
            else
            {
                Servo.AutoTrim.SavePreviousTime = SCHEDULERTIME.GetMillis();
            }
            break;

        case SERVO_AUTOTRIM_DONE:
            break;
        }
    }
    else
    {
        if (ServoAutoTrimState == SERVO_AUTOTRIM_SAVE_PENDING)
        {
            for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
            {
                Servo.Pulse.Middle[ServoIndex] = Servo.AutoTrim.MiddleBackup[ServoIndex];
            }
        }
        ServoAutoTrimState = SERVO_AUTOTRIM_IDLE;
    }
}

void PIDReduceErrorAccumulators(int8_t Delta, uint8_t Axis)
{
    PID_Resources.Controller.Integral.ErrorGyro[Axis] -= Delta;
    PID_Resources.Controller.Integral.ErrorGyroLimit[Axis] -= Delta;
}

float GetTotalRateTarget(void)
{
    return sqrtf(SquareFloat(PID_Resources.RcRateTarget.Roll) +
                 SquareFloat(PID_Resources.RcRateTarget.Pitch) +
                 SquareFloat(PID_Resources.RcRateTarget.Yaw));
}

float GetNewIntegralTerm(uint8_t Axis)
{
    return PID_Resources.Controller.Integral.ErrorGyro[Axis];
}

void ProcessContinuousServoAutoTrim(float DeltaTime)
{
#ifdef __AVR_ATmega2560__
    DeltaTime = THIS_LOOP_RATE_IN_US * 1e-6f;
#endif

    static ServoAutoTrimState_Enum ServoAutoTrimState = SERVO_AUTOTRIM_IDLE;
    static uint32_t PreviousTime;

    const float RotationRateMagnitude = sqrtf(VectorNormSquared(&BodyFrameRotation));
    const float RotationRateMagnitudeFiltered = PT1FilterApply(&RotationRateFilter, RotationRateMagnitude, SERVO_AUTOTRIM_FILTER_CUTOFF, DeltaTime);
    const float TargetRateMagnitude = GetTotalRateTarget();
    const float TargetRateMagnitudeFiltered = PT1FilterApply(&TargetRateFilter, TargetRateMagnitude, SERVO_AUTOTRIM_FILTER_CUTOFF, DeltaTime);

    if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
        ServoAutoTrimState = SERVO_AUTOTRIM_COLLECTING;
        if ((SCHEDULERTIME.GetMillis() - PreviousTime) > 500)
        {
            const bool FuselageIsFlyingStraight = RotationRateMagnitudeFiltered <= ConvertToRadians((float)JCF_Param.Continuous_Servo_Trim_Rot_Limit);
            const bool NoRotationCommanded = TargetRateMagnitudeFiltered <= (float)JCF_Param.Continuous_Servo_Trim_Rot_Limit;
            const bool FuselageIsFlyingLevel = AHRS.CosineTiltAngle() >= 0.878153032f;

            if (FuselageIsFlyingStraight && NoRotationCommanded && FuselageIsFlyingLevel && !IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE) && Get_GPS_Heading_Is_Valid())
            {
                for (uint8_t IndexCount = 0; IndexCount < 3; IndexCount++)
                {
                    const float NewIntegralTerm = GetNewIntegralTerm(IndexCount);
                    if (ABS(NewIntegralTerm) > SERVO_AUTOTRIM_UPDATE_SIZE)
                    {
                        const int8_t IntegralTermUpdate = NewIntegralTerm > 0.0f ? SERVO_AUTOTRIM_UPDATE_SIZE : -SERVO_AUTOTRIM_UPDATE_SIZE;
                        for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
                        {
                            const float ServoRate = Servo.Rate.GetAndSet[ServoIndex] / 100.0f;
                            const float ServoWeight = Servo.Weight.GetAndSet[ServoIndex] / 100.0f;
                            Servo.Pulse.Middle[ServoIndex] += IntegralTermUpdate * ServoRate * ServoWeight;
                            Servo.Pulse.Middle[ServoIndex] = Constrain_16Bits(Servo.Pulse.Middle[ServoIndex], SERVO_AUTOTRIM_CENTER_MIN, SERVO_AUTOTRIM_CENTER_MAX);
                        }
                        PIDReduceErrorAccumulators(IntegralTermUpdate, IndexCount);
                    }
                }
            }
            PreviousTime = SCHEDULERTIME.GetMillis();
        }
    }
    else if (ServoAutoTrimState == SERVO_AUTOTRIM_COLLECTING)
    {
        ServosSaveAndUpdateMiddlePoint();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        ServoAutoTrimState = SERVO_AUTOTRIM_IDLE;
    }
}
