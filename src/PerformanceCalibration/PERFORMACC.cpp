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

#include "PERFORMACC.h"
#include "IMU/ACCGYROREAD.h"
#include "ParamsToGCS/IMUCALGCS.h"
#include "Buzzer/BUZZER.h"
#include "LedRGB/LEDRGB.h"
#include "IMU/IMUHEALTH.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "GaussNewton/GAUSSNEWTON.h"
#include "DEVICE.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

AccCalibClass ACCCALIBRATION;

Calibration_Struct Calibration;
Device_Struct AccDevice[3];

#define CALIBRATING_ACC_TIME_MS 500    //TEMPO MAXIMO DE CALIBRAÇÃO DO ACC EM MS
#define ACC_CALIBRATION_VARIANCE 0.05f //5% MAXIMO ACEITAVEL DE DESVIO DURANTE A CALIBRAÇÃO

enum Acc_Calibration_Enum
{
    ACC_CALIBRATION_NONE = 0,
    ACC_CALIBRATION_IN_PROGRESS,
    ACC_CALIBRATION_DONE,
    ACC_CALIBRATION_FAIL,
};

static bool GetAllOrientationsHaveCalibrationDataCollected(void)
{
    return Calibration.Accelerometer.Flags.CalibratedPosition[0] &&
           Calibration.Accelerometer.Flags.CalibratedPosition[1] &&
           Calibration.Accelerometer.Flags.CalibratedPosition[2] &&
           Calibration.Accelerometer.Flags.CalibratedPosition[3] &&
           Calibration.Accelerometer.Flags.CalibratedPosition[4] &&
           Calibration.Accelerometer.Flags.CalibratedPosition[5];
}

void AccCalibClass::Start(void)
{
    int8_t GetActualPositionOfAcc = GetAxisInclinedToCalibration(IMU.Accelerometer.Read);

    if (GetActualPositionOfAcc == -1)
    {
        return;
    }

    if (GetActualPositionOfAcc == 0)
    {
        for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
        {
            Calibration.Accelerometer.Flags.CalibratedPosition[AxisIndex] = false;
            Calibration.Accelerometer.Samples.Window[AxisIndex][ROLL] = 0;
            Calibration.Accelerometer.Samples.Window[AxisIndex][PITCH] = 0;
            Calibration.Accelerometer.Samples.Window[AxisIndex][YAW] = 0;
        }
    }

    //RESETA ALGUNS PARÂMETROS PARA INICIAR UMA NOVA CALIBRAÇÃO
    Calibration.Accelerometer.Flags.State = ACC_CALIBRATION_IN_PROGRESS;
    Calibration.Accelerometer.Time.Start = SCHEDULERTIME.GetMillis();
    Calibration.Accelerometer.Samples.Count = 0;
    Calibration.Accelerometer.Flags.InCalibration = true;
    for (uint8_t IndexCount = 0; IndexCount < 3; IndexCount++)
    {
        Calibration.Accelerometer.Samples.Sum[IndexCount] = 0;
        DeviceClear(&AccDevice[IndexCount]);
    }

    //INDICA COM O LED RGB QUE A CALIBRAÇÃO DO ACC INICIOU
    RGB.Function(CALL_LED_ACC_CALIBRATION);
}

bool AccCalibClass::GetRunning(void)
{
    return Calibration.Accelerometer.Flags.InCalibration;
}

static void PerformAccelerationCalibration(void)
{
    if (Calibration.Accelerometer.Flags.State != ACC_CALIBRATION_IN_PROGRESS)
    {
        return;
    }

    int8_t GetActualPositionOfAcc = GetAxisInclinedToCalibration(IMU.Accelerometer.Read);

    if (GetActualPositionOfAcc == -1)
    {
        return;
    }

    Vector3x3_Struct Acc_Vector;

    if (!Calibration.Accelerometer.Flags.CalibratedPosition[GetActualPositionOfAcc])
    {
        Acc_Vector.Vector[ROLL] = IMU.Accelerometer.Read[ROLL];
        Acc_Vector.Vector[PITCH] = IMU.Accelerometer.Read[PITCH];
        Acc_Vector.Vector[YAW] = IMU.Accelerometer.Read[YAW];

        if (Calibration.Accelerometer.Flags.State == ACC_CALIBRATION_IN_PROGRESS)
        {
            //ADICIONA NOVOS VALORES A CALIBRAÇÃO
            for (uint8_t IndexCount = 0; IndexCount < 3; IndexCount++)
            {
                Calibration.Accelerometer.Samples.Sum[IndexCount] += Acc_Vector.Vector[IndexCount];
                DevicePushValues(&AccDevice[IndexCount], Acc_Vector.Vector[IndexCount]);
            }

            Calibration.Accelerometer.Samples.Count++; //REALIZA A CONTAGEM DAS AMOSTRAS

            //VERIFICA SE A CALIBRAÇÃO CONCLUIU
            if ((SCHEDULERTIME.GetMillis() - Calibration.Accelerometer.Time.Start) > CALIBRATING_ACC_TIME_MS)
            {
                Calibration.Accelerometer.Flags.Fail = false;
                for (uint8_t IndexCount = 0; IndexCount < 3 && !Calibration.Accelerometer.Flags.Fail; IndexCount++)
                {
                    const float CheckDeviationLevel = DeviceStandardDeviation(&AccDevice[IndexCount]);
                    if (CheckDeviationLevel > (IMU.Accelerometer.GravityForce.OneG * ACC_CALIBRATION_VARIANCE))
                    {
                        Calibration.Accelerometer.Flags.Fail = true;
                    }
                }

                if (Calibration.Accelerometer.Flags.Fail) //A CALIBRAÇÃO FALHOU?SIM...
                {
                    Calibration.Accelerometer.Flags.State = ACC_CALIBRATION_FAIL;
                }
                else
                {
                    Calibration.Accelerometer.Samples.Sum[ROLL] = Calibration.Accelerometer.Samples.Sum[ROLL] / Calibration.Accelerometer.Samples.Count;
                    Calibration.Accelerometer.Samples.Sum[PITCH] = Calibration.Accelerometer.Samples.Sum[PITCH] / Calibration.Accelerometer.Samples.Count;
                    Calibration.Accelerometer.Samples.Sum[YAW] = Calibration.Accelerometer.Samples.Sum[YAW] / Calibration.Accelerometer.Samples.Count;
                    Calibration.Accelerometer.Flags.State = ACC_CALIBRATION_DONE;
                }
            }
        }

        if (Calibration.Accelerometer.Flags.State != ACC_CALIBRATION_IN_PROGRESS)
        {
            if (Calibration.Accelerometer.Flags.State == ACC_CALIBRATION_DONE)
            {
                Acc_Vector.Vector[ROLL] = Calibration.Accelerometer.Samples.Sum[ROLL];
                Acc_Vector.Vector[PITCH] = Calibration.Accelerometer.Samples.Sum[PITCH];
                Acc_Vector.Vector[YAW] = Calibration.Accelerometer.Samples.Sum[YAW];

                Calibration.Accelerometer.Samples.Window[GetActualPositionOfAcc][ROLL] = Acc_Vector.Vector[ROLL];
                Calibration.Accelerometer.Samples.Window[GetActualPositionOfAcc][PITCH] = Acc_Vector.Vector[PITCH];
                Calibration.Accelerometer.Samples.Window[GetActualPositionOfAcc][YAW] = Acc_Vector.Vector[YAW];

                Calibration.Accelerometer.Flags.CalibratedPosition[GetActualPositionOfAcc] = true;
            }
            else
            {
                Calibration.Accelerometer.Flags.CalibratedPosition[GetActualPositionOfAcc] = false;
            }
            Calibration.Accelerometer.Flags.InCalibration = false;
            BEEPER.Play(BEEPER_CALIBRATION_DONE);
        }
    }

    if (GetAllOrientationsHaveCalibrationDataCollected())
    {
        Jacobian_Struct Jacobian_Matrices_To_Acc;
        float AccOffSetAndScaleBeta[3];
        bool CalibrationFailed = false;

        //CALCULA O OFFSET E VERIFICA SE A CALIBRAÇÃO FALHOU
        ClearGaussNewtonMatrices(&Jacobian_Matrices_To_Acc);

        for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
        {
            GaussNewtonPushSampleForOffSetCalculation(&Jacobian_Matrices_To_Acc, Calibration.Accelerometer.Samples.Window[AxisIndex]);
        }

        //CALCULA O OFFSET E VERIFICA SE A CALIBRAÇÃO FALHOU
        if (!GaussNewtonSolveForOffSet(&Jacobian_Matrices_To_Acc, AccOffSetAndScaleBeta))
        {
            AccOffSetAndScaleBeta[ROLL] = 0.0f;
            AccOffSetAndScaleBeta[PITCH] = 0.0f;
            AccOffSetAndScaleBeta[YAW] = 0.0f;
            CalibrationFailed = true;
        }

        Calibration.Accelerometer.OffSet[ROLL] = lrintf(AccOffSetAndScaleBeta[ROLL]);
        Calibration.Accelerometer.OffSet[PITCH] = lrintf(AccOffSetAndScaleBeta[PITCH]);
        Calibration.Accelerometer.OffSet[YAW] = lrintf(AccOffSetAndScaleBeta[YAW]);

        //LIMPA A MATRIX AFIM DE NÃO COMPENSAR AS AMOSTRAS MÉDIAS,ESCALAS E GANHOS
        ClearGaussNewtonMatrices(&Jacobian_Matrices_To_Acc);

        for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
        {
            int32_t AccSample[3];

            AccSample[ROLL] = Calibration.Accelerometer.Samples.Window[AxisIndex][ROLL] - Calibration.Accelerometer.OffSet[ROLL];
            AccSample[PITCH] = Calibration.Accelerometer.Samples.Window[AxisIndex][PITCH] - Calibration.Accelerometer.OffSet[PITCH];
            AccSample[YAW] = Calibration.Accelerometer.Samples.Window[AxisIndex][YAW] - Calibration.Accelerometer.OffSet[YAW];

            GaussNewtonPushSampleForScaleCalculation(&Jacobian_Matrices_To_Acc, AxisIndex / 2, AccSample, IMU.Accelerometer.GravityForce.OneG);
        }

        //CALCULA A ESCALA E VERIFICA SE A CALIBRAÇÃO FALHOU
        if (!GaussNewtonSolveForScale(&Jacobian_Matrices_To_Acc, AccOffSetAndScaleBeta))
        {
            AccOffSetAndScaleBeta[ROLL] = 1.0f;
            AccOffSetAndScaleBeta[PITCH] = 1.0f;
            AccOffSetAndScaleBeta[YAW] = 1.0f;
            CalibrationFailed = true;
        }

        for (uint8_t AxisIndex = 0; AxisIndex < 3; AxisIndex++)
        {
            Calibration.Accelerometer.Scale[AxisIndex] = lrintf(AccOffSetAndScaleBeta[AxisIndex] * 4096);
        }

        if (!CalibrationFailed) //A CALIBRAÇÃO FALHOU?NÃO...
        {
            SaveIMUCalibration();
        }
        else
        {
            for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
            {
                Calibration.Accelerometer.Flags.CalibratedPosition[AxisIndex] = false;
            }
        }
    }
}

void AccCalibClass::Update(void)
{
    PerformAccelerationCalibration();

    //APLICA A ACELERAÇÃO ZERO
    IMU.Accelerometer.Read[ROLL] = (((int32_t)(IMU.Accelerometer.Read[ROLL] - Calibration.Accelerometer.OffSet[ROLL])) * Calibration.Accelerometer.Scale[ROLL]) / 4096;
    IMU.Accelerometer.Read[PITCH] = (((int32_t)(IMU.Accelerometer.Read[PITCH] - Calibration.Accelerometer.OffSet[PITCH])) * Calibration.Accelerometer.Scale[PITCH]) / 4096;
    IMU.Accelerometer.Read[YAW] = (((int32_t)(IMU.Accelerometer.Read[YAW] - Calibration.Accelerometer.OffSet[YAW])) * Calibration.Accelerometer.Scale[YAW]) / 4096;
}