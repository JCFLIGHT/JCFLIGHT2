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

#include "PERFORMGRAVITY.h"
#include "DEVICE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "PERFORMACC.h"
#include "Math/MATHSUPPORT.h"
#include "FastSerial/PRINTF.h"

GravityClass GRAVITYCALIBRATION;

Device_Struct GravityDevice;

#define CALIBRATING_GRAVITY_TIME_MS 2000   //TEMPO MAXIMO DE CALIBRAÇÃO DO 1G DO EIXO Z EM MS
#define GRAVITY_CALIBRATION_VARIANCE 0.05f //5% MAXIMO ACEITAVEL DE DESVIO DURANTE A CALIBRAÇÃO

void GravityClass::Update(Vector3x3_Struct *VectorPointer)
{
    if (!Calibration.Accelerometer.Gravity.Flags.Calibrated)
    {

        if (Calibration.Accelerometer.Gravity.Flags.Restart)
        {
            DeviceClear(&GravityDevice);
            Calibration.Accelerometer.Gravity.Samples.Count = 0;
            Calibration.Accelerometer.Gravity.Samples.Sum = 0;
            Calibration.Accelerometer.Gravity.Time.Previous = SCHEDULERTIME.GetMillis();
            Calibration.Accelerometer.Gravity.Flags.Calibrated = false;
            Calibration.Accelerometer.Gravity.Flags.Restart = false;
        }

        if ((SCHEDULERTIME.GetMillis() - Calibration.Accelerometer.Gravity.Time.Previous) >= CALIBRATING_GRAVITY_TIME_MS)
        {
            //5% DE DESVIO MAXIMO SUPORTADO NO EIXO Z DO NEU PRA COMPLETAR A CALIBRAÇÃO,CASO CONTRARIO SERÁ REINICIADA.
            //A CALIBRAÇÃO DA GRAVIDADE OCORRE AO MESMO TEMPO QUE A CALIBRAÇÃO DO GYRO,ENTÃO SE A CALIBRAÇÃO DO GYRO FALHAR,
            //A DA GRAVIDADE TAMBÉM FALHARÁ.A FALHA SÓ OCORRERÁ SE O USUARIO MOVER O UAV,OU PROVAVEL IMU COM DEFEITO.
            if (DeviceStandardDeviation(&GravityDevice) > (GRAVITY_CMSS * GRAVITY_CALIBRATION_VARIANCE))
            {
                Calibration.Accelerometer.Gravity.Flags.Restart = true;
            }
            else
            {
                Calibration.Accelerometer.Gravity.Samples.Sum = Calibration.Accelerometer.Gravity.Samples.Sum / Calibration.Accelerometer.Gravity.Samples.Count;
                LOG_WITH_ARGS("Gravidade Calib OffSet:%d", (int16_t)Calibration.Accelerometer.Gravity.Samples.Sum);
                LINE_SPACE;
                Calibration.Accelerometer.Gravity.Flags.Calibrated = true;
            }
        }
        else
        {
            Calibration.Accelerometer.Gravity.Samples.Sum = Calibration.Accelerometer.Gravity.Samples.Sum + VectorPointer->Yaw;
            DevicePushValues(&GravityDevice, VectorPointer->Yaw);
            Calibration.Accelerometer.Gravity.Samples.Count++;
        }

        //RESETA TODOS OS EIXOS DO NEU SE A CALIBRAÇÃO AINDA ESTIVER CORRENDO
        VectorPointer->Roll = 0.0f;
        VectorPointer->Pitch = 0.0f;
        VectorPointer->Yaw = 0.0f;
    }
    else //SUBTRAI O 1G QUE FOI CALCULADO DURANTE A CALIBRAÇÃO DO EIXO Z DO NEU
    {
        VectorPointer->Yaw = VectorPointer->Yaw - Calibration.Accelerometer.Gravity.Samples.Sum;
    }
}