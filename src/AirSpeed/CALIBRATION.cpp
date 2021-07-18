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

#include "CALIBRATION.h"
#include "Math/MATRIXF.h"
#include "GPSNavigation/NAVIGATION.h"
#include "Math/MATHSUPPORT.h"
#include "Param/PARAM.h"
#include "AHRS/AHRS.h"
#include "GPS/GPSSTATES.h"
#include "AIRSPEED.h"
#include "Barometer/BAROFRONTEND.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Scheduler/SCHEDULER.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Build/BOARDDEFS.h"
#include "BAR/BAR.h"

AirSpeedCalibrationClass AIRSPEEDCALIBRATION;

Matrix3x3Float CovariancePrediction(100.0f, 0.0f, 0.0f,
                                    0.0f, 100.0f, 0.0f,
                                    0.0f, 0.0f, 0.000001f);

Vector3x3Float StateEstimate(0.0f, 0.0f, 0.0f);

#define AIR_SPEED_MIN 9              //LEITURA MINIMA DO TUBO DE PITOT EM M/S PARA ENTRAR NA CALIBRAÇÃO EM VOO
#define AIR_SPEED_MAX 22             //LEITURA MAXIMA DO TUBO DE PITOT EM M/S
#define FUSELAGE_ROLL_MAX 45         //BANK ANGLE MAXIMO EM GRAUS SUPORTADO PARA ENTRAR NA CALIBRAÇÃO EM VOO
#define FUSELAGE_PITCH_MAX 25        //PITCH MAXIMO EM GRAUS SUPORTADO PARA ENTRAR NA CALIBRAÇÃO EM VOO
#define KF_PROCESS_NOISE 0.01f       //NOISE GERADO PELO SENSOR AO LONGO DO PROCESSO
#define KF_MEDICION_NOISE 0.0000005f //ERRO DE MEDIÇÃO AO LONGO DO PROCESSO
#define KF_NOISE_BASE 1.0f           //ADIÇÃO MANUAL DE ERRO AO FILTRO DE KALMAN EM M/S

#ifdef __AVR_ATmega2560__

#define CALIBRATION_SAMPLES 15

#else

#define CALIBRATION_SAMPLES JCF_Param.AirSpeed_Samples

#endif

void AirSpeedCalibrationClass::Initialization(void)
{
#ifdef USE_AIRSPEED_AUTO_SCALE_CALIBRATION

    AIRSPEEDCALIBRATION.Previous_Scale = AirSpeed.Param.Factor;
    StateEstimate.Z = 1.0f / sqrtf(AirSpeed.Param.Factor);

#endif
}

bool AirSpeedCalibrationClass::Calibrate(void)
{
    if (!AirSpeed.Calibration.Initialized)
    {
        AirSpeed.Calibration.Start_MS = SCHEDULERTIME.GetMillis();
        AirSpeed.Calibration.Initialized = true;
    }

    if (AirSpeed.Calibration.Start_MS == 0)
    {
        return true;
    }

    if (SCHEDULERTIME.GetMillis() - AirSpeed.Calibration.Start_MS >= 1000 && AirSpeed.Calibration.Read_Count > CALIBRATION_SAMPLES)
    {
        if (AirSpeed.Calibration.Count > 0)
        {
            AirSpeed.Calibration.OffSet = AirSpeed.Calibration.Sum / AirSpeed.Calibration.Count;
        }
        AirSpeed.Calibration.Start_MS = 0;
        return false;
    }

    //DESCARTA AS 5 PRIMEIRAS AMOSTRAS
    if (AirSpeed.Calibration.Read_Count > 5)
    {
        AirSpeed.Calibration.Sum += AirSpeed.Raw.Pressure;
        AirSpeed.Calibration.Count++;
    }

    AirSpeed.Calibration.Read_Count++;
    return false;
}

#ifdef USE_AIRSPEED_AUTO_SCALE_CALIBRATION

static float Get_Scale_Calibration(float True_AirSpeed, const Vector3x3Float &GPSVelocity)
{
    CovariancePrediction.A.X += KF_PROCESS_NOISE;
    CovariancePrediction.B.Y += KF_PROCESS_NOISE;
    CovariancePrediction.C.Z += KF_MEDICION_NOISE;

    float TAS_Predicted = StateEstimate.Z * sqrtf(SquareFloat(GPSVelocity.X - StateEstimate.X) + SquareFloat(GPSVelocity.Y - StateEstimate.Y) + SquareFloat(GPSVelocity.Z));
    float TAS_Masured = True_AirSpeed;

    float JacobianObservationTAS = SquareFloat(GPSVelocity.Y - StateEstimate.Y) + SquareFloat(GPSVelocity.X - StateEstimate.X);

    if (JacobianObservationTAS < 0.000001f)
    {
        //EVITA DIVIDIR POR NÚMEROS PEQUENOS
        return StateEstimate.Z;
    }

    float JacobianScale = 1 / sqrtf(JacobianObservationTAS);

    Vector3x3Float JacobianObservation(-(StateEstimate.Z * JacobianScale * (2 * GPSVelocity.X - 2 * StateEstimate.X)) / 2,
                                       -(StateEstimate.Z * JacobianScale * (2 * GPSVelocity.Y - 2 * StateEstimate.Y)) / 2,
                                       1 / JacobianScale);

    Vector3x3Float FusionInnovation = CovariancePrediction * JacobianObservation;

    Vector3x3Float KalmanGain = FusionInnovation / (JacobianObservation * FusionInnovation + KF_NOISE_BASE);

    StateEstimate += KalmanGain * (TAS_Masured - TAS_Predicted); //6 ESTADOS

    Vector3x3Float JacobianCovariance = JacobianObservation * CovariancePrediction;
    CovariancePrediction -= KalmanGain.Multiply_Row_Column(JacobianCovariance);

    //FORÇA SIMETRIA NA COVARIANCIA DA MATRIX
    float P12 = 0.5f * (CovariancePrediction.A.Y + CovariancePrediction.B.X);
    float P13 = 0.5f * (CovariancePrediction.A.Z + CovariancePrediction.C.X);
    float P23 = 0.5f * (CovariancePrediction.B.Z + CovariancePrediction.C.Y);
    CovariancePrediction.A.Y = CovariancePrediction.B.X = P12;
    CovariancePrediction.A.Z = CovariancePrediction.C.X = P13;
    CovariancePrediction.B.Z = CovariancePrediction.C.Y = P23;

    CovariancePrediction.A.X = MAX(CovariancePrediction.A.X, 0.0f);
    CovariancePrediction.B.Y = MAX(CovariancePrediction.B.Y, 0.0f);
    CovariancePrediction.C.Z = MAX(CovariancePrediction.C.Z, 0.0f);

    StateEstimate.X = Constrain_Float(StateEstimate.X, -AIR_SPEED_MAX, AIR_SPEED_MAX);
    StateEstimate.Y = Constrain_Float(StateEstimate.Y, -AIR_SPEED_MAX, AIR_SPEED_MAX);
    StateEstimate.Z = Constrain_Float(StateEstimate.Z, 0.5f, 1.0f);

    return StateEstimate.Z;
}

#endif

void AirSpeedCalibrationClass::Scale_Update(void)
{
#ifdef USE_AIRSPEED_AUTO_SCALE_CALIBRATION

    static Scheduler_Struct AirSpeedScale_Scheduler;

    if (!Scheduler(&AirSpeedScale_Scheduler, SCHEDULER_SET_FREQUENCY(2, "Hz")))
    {
        return;
    }

    //if () //TUNNING
    {
        return;
    }

    if (!AirSpeed.Healthy || (AirSpeed.Raw.IASPressureInCM < ConverMetersToCM(AIR_SPEED_MIN)) || !Get_GPS_Heading_Is_Valid())
    {
        //NÃO CALIBRA SE O AIR-SPEED NÃO ESTIVER HABILITADO OU SE O UAV ESTIVER PARADO
        return;
    }

    if ((ABS(Attitude.EulerAngles.Roll) > ConvertDegreesToDecidegrees(FUSELAGE_ROLL_MAX)) ||
        (ABS(Attitude.EulerAngles.Pitch) > ConvertDegreesToDecidegrees(FUSELAGE_PITCH_MAX)))
    {
        //NÃO CALIBRA SE A FUSELAGEM ESTIVER GIRANDO OU INCLINANDO
        return;
    }

    //OBTÉM E CONVERTE O NED DADO PELO GPS DE CM/S PARA M/S
    const Vector3x3Float &GPSVelocity = Vector3x3Float(ConvertCMToMeters(GPS_Resources.Navigation.Misc.Velocity.Get[NORTH]),
                                                       ConvertCMToMeters(GPS_Resources.Navigation.Misc.Velocity.Get[EAST]),
                                                       ConvertCMToMeters(GPS_Resources.Navigation.Misc.Velocity.Get[DOWN]));

    //VELOCIDADE VERDADEIRA DO AR SEM O GANHO DE 'AirSpeed.Param.Factor'
    float True_AirSpeed = sqrtf(AirSpeed.Raw.DifferentialPressure) * Get_EAS2TAS();

    float CurrentScale = Constrain_Float(AirSpeed.Param.Factor, 1.0f, 4.0f);

    StateEstimate.Z = 1.0f / sqrtf(CurrentScale);

    float EstimatedZScale = Get_Scale_Calibration(True_AirSpeed, GPSVelocity);

    if (isnan(EstimatedZScale) || isinf(EstimatedZScale))
    {
        return;
    }

    EstimatedZScale = Constrain_Float(EstimatedZScale, 0.5f, 1.0f);
    AirSpeed.Param.Factor = 1.0f / SquareFloat(EstimatedZScale);
    if (AIRSPEEDCALIBRATION.Scale_Counter > 60)
    {
        if (AIRSPEEDCALIBRATION.Previous_Scale < (0.95f * AirSpeed.Param.Factor) ||
            AIRSPEEDCALIBRATION.Previous_Scale > (1.05f * AirSpeed.Param.Factor))
        {
            //GUARDA UM NOVO VALOR DE CALIBRAÇÃO NA EEPROM A CADA 2 MINUTOS
            STORAGEMANAGER.Write_32Bits(AIRSPEED_FACTOR_ADDR, (int32_t)(AirSpeed.Param.Factor * 10000.0f));
            AIRSPEEDCALIBRATION.Previous_Scale = AirSpeed.Param.Factor;
            AIRSPEEDCALIBRATION.Scale_Counter = 0;
        }
    }
    else
    {
        AIRSPEEDCALIBRATION.Scale_Counter++;
    }

#endif
}