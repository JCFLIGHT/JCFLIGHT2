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

#include "PERFORMGYRO.h"
#include "IMU/ACCGYROREAD.h"
#include "DEVICE.h"
#include "Buzzer/BUZZER.h"
#include "Math/MATHSUPPORT.h"
#include "LedRGB/LEDRGB.h"
#include "PERFORMACC.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

GyroCalClass GYROCALIBRATION;

Device_Struct GyroDevice[3];

#define CALIBRATING_GYRO_TIME_MS 2000 //TEMPO MAXIMO DE CALIBRAÇÃO DO GYRO EM MS
#define GYRO_CALIBRATION_VARIANCE 32  //DESVIO MAXIMO SUPORTADO NO GYRO PRA COMPLETAR A CALIBRAÇÃO.ISSO SERVE PARA EVITAR COM QUE A CALIBRAÇÃO CONCLUA COM O USUARIO MOVENDO O UAV

bool GyroCalClass::GetRunning(void)
{
    return !Calibration.Gyroscope.Flags.Calibrated;
}

void GyroCalClass::Update(void)
{
    if (!BEEPER.GetSafeToOthersBeeps())
    {
        return;
    }

    if (!Calibration.Gyroscope.Flags.Calibrated)
    {
        RGB.Function(CALL_LED_GYRO_CALIBRATION);

        if (Calibration.Gyroscope.Flags.Restart)
        {
            Calibration.Gyroscope.Samples.Sum[ROLL] = 0;
            Calibration.Gyroscope.Samples.Sum[PITCH] = 0;
            Calibration.Gyroscope.Samples.Sum[YAW] = 0;
            DeviceClear(&GyroDevice[ROLL]);
            DeviceClear(&GyroDevice[PITCH]);
            DeviceClear(&GyroDevice[YAW]);
            Calibration.Gyroscope.Time.Previous = SCHEDULERTIME.GetMillis();
            Calibration.Gyroscope.Samples.Count = 0;
            Calibration.Gyroscope.Flags.Restart = false;
        }

        if ((SCHEDULERTIME.GetMillis() - Calibration.Gyroscope.Time.Previous) >= CALIBRATING_GYRO_TIME_MS)
        {
            Calibration.Gyroscope.Deviation[ROLL] = DeviceStandardDeviation(&GyroDevice[ROLL]);
            Calibration.Gyroscope.Deviation[PITCH] = DeviceStandardDeviation(&GyroDevice[PITCH]);
            Calibration.Gyroscope.Deviation[YAW] = DeviceStandardDeviation(&GyroDevice[YAW]);
            //CHECA SE A IMU FOI MOVIDA DURANTE A CALIBRAÇÃO
            if ((Calibration.Gyroscope.Deviation[ROLL] > GYRO_CALIBRATION_VARIANCE) ||
                (Calibration.Gyroscope.Deviation[PITCH] > GYRO_CALIBRATION_VARIANCE) ||
                (Calibration.Gyroscope.Deviation[YAW] > GYRO_CALIBRATION_VARIANCE))
            {
                BEEPER.Play(BEEPER_ACTION_FAIL);            //SINALIZA COM O BUZZER QUE HOUVE UM ERRO
                Calibration.Gyroscope.Flags.Restart = true; //REINICIA A CALIBRAÇÃO DO GYRO
            }
            else
            {
                Calibration.Gyroscope.Samples.Sum[ROLL] = Calibration.Gyroscope.Samples.Sum[ROLL] / Calibration.Gyroscope.Samples.Count;
                Calibration.Gyroscope.Samples.Sum[PITCH] = Calibration.Gyroscope.Samples.Sum[PITCH] / Calibration.Gyroscope.Samples.Count;
                Calibration.Gyroscope.Samples.Sum[YAW] = Calibration.Gyroscope.Samples.Sum[YAW] / Calibration.Gyroscope.Samples.Count;
                BEEPER.Play(BEEPER_CALIBRATION_DONE); //SINALIZA COM O BUZZER QUE TUDO OCORREU BEM
                LOG_WITH_ARGS("Giroscopio Calib OffSets:Roll = %ld Pitch = %ld Yaw = %d",
                              Calibration.Gyroscope.Samples.Sum[ROLL],
                              Calibration.Gyroscope.Samples.Sum[PITCH],
                              Calibration.Gyroscope.Samples.Sum[YAW]);
                LINE_SPACE;
                Calibration.Gyroscope.Flags.Calibrated = true; //CALIBRAÇÃO CONCLUIDA
            }
        }
        else
        {
            Calibration.Gyroscope.Samples.Sum[ROLL] += IMU.Gyroscope.Read[ROLL];
            Calibration.Gyroscope.Samples.Sum[PITCH] += IMU.Gyroscope.Read[PITCH];
            Calibration.Gyroscope.Samples.Sum[YAW] += IMU.Gyroscope.Read[YAW];
            DevicePushValues(&GyroDevice[ROLL], IMU.Gyroscope.Read[ROLL]);
            DevicePushValues(&GyroDevice[PITCH], IMU.Gyroscope.Read[PITCH]);
            DevicePushValues(&GyroDevice[YAW], IMU.Gyroscope.Read[YAW]);
            Calibration.Gyroscope.Samples.Count++;
        }
    }

    //APLICA A ROTAÇÃO ZERO
    IMU.Gyroscope.Read[ROLL] = (IMU.Gyroscope.Read[ROLL] - Calibration.Gyroscope.Samples.Sum[ROLL]);
    IMU.Gyroscope.Read[PITCH] = (IMU.Gyroscope.Read[PITCH] - Calibration.Gyroscope.Samples.Sum[PITCH]);
    IMU.Gyroscope.Read[YAW] = (IMU.Gyroscope.Read[YAW] - Calibration.Gyroscope.Samples.Sum[YAW]);
}