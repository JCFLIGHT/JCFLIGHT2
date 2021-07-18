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

#include "PERFORMCOMPASS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "LedRGB/LEDRGB.h"
#include "IMU/IMUHEALTH.h"
#include "Buzzer/BUZZER.h"
#include "Common/STRUCTS.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "IMU/ACCGYROREAD.h"
#include "Param/PARAM.h"
#include "Math/MATHSUPPORT.h"
#include "GaussNewton/GAUSSNEWTON.h"

PerformCompassClass COMPASSCALIBRATION;

void PerformCompassClass::Initialization(void)
{
    //CARREGA OS OFFSET'S
    Calibration.Magnetometer.OffSet[ROLL] = STORAGEMANAGER.Read_16Bits(MAG_ROLL_OFFSET_ADDR);
    Calibration.Magnetometer.OffSet[PITCH] = STORAGEMANAGER.Read_16Bits(MAG_PITCH_OFFSET_ADDR);
    Calibration.Magnetometer.OffSet[YAW] = STORAGEMANAGER.Read_16Bits(MAG_YAW_OFFSET_ADDR);

    //CARREGA OS GANHOS
    Calibration.Magnetometer.Gain[ROLL] = STORAGEMANAGER.Read_16Bits(MAG_ROLL_GAIN_ADDR);
    Calibration.Magnetometer.Gain[PITCH] = STORAGEMANAGER.Read_16Bits(MAG_PITCH_GAIN_ADDR);
    Calibration.Magnetometer.Gain[YAW] = STORAGEMANAGER.Read_16Bits(MAG_YAW_GAIN_ADDR);
}

void PerformCompassClass::Update(void)
{
    if (!Calibration.Magnetometer.Calibrating) //CALIBRAÇÃO INICIADA?NÃO...
    {
        return;
    }

    static Jacobian_Struct Jacobian_Matrices_To_Compass;

    Calibration.Magnetometer.Count++;

    if (Calibration.Magnetometer.Count == 1) //INICIALMENTE RESETA TUDO
    {
        for (uint8_t IndexCount = 0; IndexCount < 3; IndexCount++)
        {
            Calibration.Magnetometer.OffSet[IndexCount] = 0;
            Calibration.Magnetometer.Gain[IndexCount] = 1024;
            Calibration.Magnetometer.Previous[IndexCount] = 0;
            Calibration.Magnetometer.Deviation[IndexCount] = 0;
            Calibration.Magnetometer.GaussNewtonOffSet[IndexCount] = 0.0f;
        }
        ClearGaussNewtonMatrices(&Jacobian_Matrices_To_Compass);
    }

    if (Calibration.Magnetometer.Count < (JCF_Param.Compass_Cal_Timer * 10))
    {
        RGB.Function(CALL_LED_MAG_CALIBRATION);

        Calibration.Magnetometer.Difference = 0.0f;
        Calibration.Magnetometer.Average = 0.0f;

        for (uint8_t IndexCount = 0; IndexCount < 3; IndexCount++)
        {
            Calibration.Magnetometer.Difference += (IMU.Compass.Read[IndexCount] - Calibration.Magnetometer.Previous[IndexCount]) * (IMU.Compass.Read[IndexCount] - Calibration.Magnetometer.Previous[IndexCount]);
            Calibration.Magnetometer.Average += (IMU.Compass.Read[IndexCount] + Calibration.Magnetometer.Previous[IndexCount]) * (IMU.Compass.Read[IndexCount] + Calibration.Magnetometer.Previous[IndexCount]) / 4.0f;

            if (ABS(IMU.Compass.Read[IndexCount]) > ABS(Calibration.Magnetometer.Deviation[IndexCount]))
            {
                Calibration.Magnetometer.Deviation[IndexCount] = IMU.Compass.Read[IndexCount];
            }
        }

        if ((Calibration.Magnetometer.Average > 0.01f) && ((Calibration.Magnetometer.Difference / Calibration.Magnetometer.Average) > (0.01960000023f)))
        {
            GaussNewtonPushSampleForOffSetCalculation(&Jacobian_Matrices_To_Compass, (int32_t *)IMU.Compass.Read);
            Calibration.Magnetometer.Previous[ROLL] = IMU.Compass.Read[ROLL];
            Calibration.Magnetometer.Previous[PITCH] = IMU.Compass.Read[PITCH];
            Calibration.Magnetometer.Previous[YAW] = IMU.Compass.Read[YAW];
        }
    }
    else
    {
        GaussNewtonSolveForOffSet(&Jacobian_Matrices_To_Compass, Calibration.Magnetometer.GaussNewtonOffSet);

        Calibration.Magnetometer.OffSet[ROLL] = lrintf(Calibration.Magnetometer.GaussNewtonOffSet[ROLL]);
        Calibration.Magnetometer.OffSet[PITCH] = lrintf(Calibration.Magnetometer.GaussNewtonOffSet[PITCH]);
        Calibration.Magnetometer.OffSet[YAW] = lrintf(Calibration.Magnetometer.GaussNewtonOffSet[YAW]);

        Calibration.Magnetometer.Gain[ROLL] = ABS(Calibration.Magnetometer.Deviation[ROLL] - Calibration.Magnetometer.OffSet[ROLL]);
        Calibration.Magnetometer.Gain[PITCH] = ABS(Calibration.Magnetometer.Deviation[PITCH] - Calibration.Magnetometer.OffSet[PITCH]);
        Calibration.Magnetometer.Gain[YAW] = ABS(Calibration.Magnetometer.Deviation[YAW] - Calibration.Magnetometer.OffSet[YAW]);

        STORAGEMANAGER.Write_16Bits(MAG_ROLL_OFFSET_ADDR, Calibration.Magnetometer.OffSet[ROLL]);
        STORAGEMANAGER.Write_16Bits(MAG_PITCH_OFFSET_ADDR, Calibration.Magnetometer.OffSet[PITCH]);
        STORAGEMANAGER.Write_16Bits(MAG_YAW_OFFSET_ADDR, Calibration.Magnetometer.OffSet[YAW]);

        STORAGEMANAGER.Write_16Bits(MAG_ROLL_GAIN_ADDR, Calibration.Magnetometer.Gain[ROLL]);
        STORAGEMANAGER.Write_16Bits(MAG_PITCH_GAIN_ADDR, Calibration.Magnetometer.Gain[PITCH]);
        STORAGEMANAGER.Write_16Bits(MAG_YAW_GAIN_ADDR, Calibration.Magnetometer.Gain[YAW]);

        COMPASSCALIBRATION.Initialization();

        Calibration.Magnetometer.Calibrating = false;
        Calibration.Magnetometer.Count = 0;

        BEEPER.Play(BEEPER_CALIBRATION_DONE);
    }
}

void PerformCompassClass::Apply(void)
{
    //AJUSTA O VALOR FINAL DO COMPASS DE ACORDO COM A CALIBRAÇÃO DO MESMO
    if (!Calibration.Magnetometer.Calibrating)
    {
        IMU.Compass.Read[ROLL] = ((int32_t)IMU.Compass.Read[ROLL] - Calibration.Magnetometer.OffSet[ROLL]) * 1024 / Calibration.Magnetometer.Gain[ROLL];
        IMU.Compass.Read[PITCH] = ((int32_t)IMU.Compass.Read[PITCH] - Calibration.Magnetometer.OffSet[PITCH]) * 1024 / Calibration.Magnetometer.Gain[PITCH];
        IMU.Compass.Read[YAW] = ((int32_t)IMU.Compass.Read[YAW] - Calibration.Magnetometer.OffSet[YAW]) * 1024 / Calibration.Magnetometer.Gain[YAW];
    }
}