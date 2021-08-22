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

#include "INS.h"
#include "I2C/I2C.h"
#include "GPSNavigation/NAVIGATION.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "Math/MATHSUPPORT.h"
#include "GPS/GPSSTATES.h"
#include "AHRS/AHRS.h"
#include "IMU/ACCGYROREAD.h"
#include "Barometer/BAROBACKEND.h"
#include "PerformanceCalibration/PERFORMGRAVITY.h"
#include "BitArray/BITARRAY.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "I2C/I2C.h"
#include "GPSNavigation/NAVIGATIONGEO.h"
#include "Scheduler/SCHEDULER.h"
#include "FastSerial/PRINTF.h"

InertialNavigationClass INERTIALNAVIGATION;
INS_Resources_Struct INS_Resources;

#define GPS_DEFAULT_EPH 200.0f                           //2M MAXIMO DE ERRO NA POSIÇÃO HORIZONTAL
#define GPS_DEFAULT_EPV 500.0f                           //5M MAXIMO DE ERRO NA POSIÇÃO VERTICAL
#define GPS_TIMEOUT_MS 1500                              //TEMPO MAXIMO DE ESPERA DE RESPOSTA DO GPS (MS)
#define BARO_TIMEOUT_MS 200                              //TEMPO MAXIMO DE ESPERA DE RESPOSTA DO BARÔMETRO (MS)
#define ACC_BIAS_ACCEPTANCE_VALUE (GRAVITY_CMSS * 0.25f) //0.25G
#define ACC_CLIPPING_RC_CONSTANT (0.010f)                //10MS DE DELAY
#define ACC_CLIPPING_THRESHOLD_G 7.9f                    //7.9G

//CLI
#define Weight_Acc_Bias 0.01f         //GANHO PARA CALCULAR A ESTIMATIVA DE DRIFT DO ACELERÔMETRO
#define Weight_XYZ_Acc_Position 1.0f  //GANHO DA CORREÇÃO ACELERÔMETRO NO INS
#define Weight_Z_Baro_Position 0.35f  //GANHO DA MEDIÇÃO DO BARÔMETRO PARA CALCULAR A TAXA DE SUBIDA E A ALTITUDE ESTIMADA
#define Weight_Z_GPS_Position 0.2f    //GANHO DA MEDIÇÃO DE ALTITUDE DADA PELO GPS PARA CALCULAR A ALTITUDE ESTIMADA (AERONAVES DE ASA-FIXA APENAS)
#define Weight_Z_GPS_Velocity 0.1f    //GANHO DA MEDIÇÃO DO GPS PARA CALCULAR A TAXA DE SUBIDA E A ALTITUDE ESTIMADA
#define Weight_XY_GPS_Position 1.0f   //GANHO DA MEDIÇÃO DAS COORDENADAS PARA CALCULAR A POSIÇÃO E VELOCIDADE DO UAV
#define Weight_XY_GPS_Velocity 2.0f   //GANHO DA MEDIÇÃO DE VELOCIDADE DO GPS PARA CALCULAR A VELOCIDADADE ESTIMADA DO UAV
#define Weight_XY_Coeff_Velocity 0.5f //COEFICIENTE DE DECAIMENTO PARA A VELOCIDADE ESTIMADA QUANDO A REFERÊNCIA DA POSIÇÃO DO GPS É PERDIDA
#define Weight_Z_Coeff_Velocity 0.5f  //COEFICIENTE DE DECAIMENTO PARA A TAXA DE SUBIDA ESTIMADA QUANDO A REFERÊNCIA DA ALTITUDE DO BARO OU GPS É PERDIDA
#define Max_INS_H_V_Error 1000.0f     //VALOR MAXIMO DE ERRO SUPORTADO PELO INS PARA AS POSIÇÕES X,Y E Z [CM]
#define Max_INS_Baro_Error 100.0f     //VALOR MAXIMO DE ERRO DO BARÔMETRO SUPORTADO PELO INS [CM]

//DEBUG
//#define PRINTLN_ALL_INS
//#define PRINTLN_INS_YAW_COS_SIN
//#define PRINTLN_INS_ACC_NEU
//#define PRINTLN_INS_POS_VEL_XY
//#define PRINTLN_INS_POS_VEL_Z

void InertialNavigationClass::Initialization(void)
{
  INS_Resources.Estimated.EstimatedPositionHorizontal = Max_INS_H_V_Error + 0.001f;
  INS_Resources.Estimated.EstimatedPositionVertical = Max_INS_H_V_Error + 0.001f;

  INS_Resources.IMU.LastUpdateTime = 0;
  INS_Resources.GPS.LastUpdateTime = 0;
  INS_Resources.Barometer.LastUpdateTime = 0;
  INS_Resources.IMU.AccWeightFactor = 0.0f;

  for (uint8_t IndexCount = 0; IndexCount < 3; IndexCount++)
  {
    INS_Resources.IMU.AccelerationBias.Vector[IndexCount] = 0.0f;
    INS_Resources.Estimated.Position.Vector[IndexCount] = 0.0f;
    INS_Resources.Estimated.Velocity.Vector[IndexCount] = 0.0f;
  }
}

void InertialNavigationClass::Update(void)
{
  const uint32_t ActualTimeInUs = SCHEDULERTIME.GetMicros();

  INERTIALNAVIGATION.UpdateIMU(ActualTimeInUs);
  INERTIALNAVIGATION.UpdateBarometer(ActualTimeInUs);
  INERTIALNAVIGATION.UpdateEstimationPredictXYZ(ActualTimeInUs);

  //OBTÉM O SENO E COSSENO DO YAW DADO PELO AHRS
  INS_Resources.Math.Cosine.Yaw = AHRS.GetCosineYaw();
  INS_Resources.Math.Sine.Yaw = AHRS.GetSineYaw();

#ifdef PRINTLN_ALL_INS

  DEBUG("%.f %.f %.f %.f %ld %.f %.f",
        INS_Resources.Estimated.Position.X,  //POSIÇÃO FINAL X ESTIMADA PELO INS
        INS_Resources.Estimated.Position.Y,  //POSIÇÃO FINAL Y ESTIMADA PELO INS
        INS_Resources.Estimated.Velocity.X,  //VELOCIDADE FINAL X ESTIMADA PELO INS
        INS_Resources.Estimated.Velocity.Y,  //VELOCIDADE FINAL Y ESTIMADA PELO INS
        Barometer.Altitude.Actual,           //REAL BARO ALT
        INS_Resources.Estimated.Position.Z,  //ALTITUDE FINAL ESTIMADA PELO INS
        INS_Resources.Estimated.Velocity.Z); //VELOCIDADE VERTICAL(Z) FINAL ESTIMADA PELO INS

#endif

#ifdef PRINTLN_INS_YAW_COS_SIN

  DEBUG("%d %.2f %.2f",
        Attitude.EulerAngles.YawDecidegrees,
        INS_Resources.Math.Cosine.Yaw,
        INS_Resources.Math.Sine.Yaw);

#endif

#ifdef PRINTLN_INS_ACC_NEU

  //INS_Resources.IMU.AccelerationNEU.X -> POSITIVO MOVENDO PARA O NORTE
  //INS_Resources.IMU.AccelerationNEU.Y -> POSITIVO MOVENDO PARA O OESTE
  //INS_Resources.IMU.AccelerationNEU.Z -> POSITIVO MOVENDO PARA CIMA

  DEBUG("NORTH:%.4f EAST:%.4f UP:%.4f",
        INS_Resources.IMU.AccelerationNEU.X,
        INS_Resources.IMU.AccelerationNEU.Y,
        INS_Resources.IMU.AccelerationNEU.Z);

#endif

#ifdef PRINTLN_INS_POS_VEL_XY

  DEBUG("%.f %.f %.f %.f",
        INS_Resources.Estimated.Position.X,
        INS_Resources.Estimated.Position.Y,
        INS_Resources.Estimated.Velocity.X,
        INS_Resources.Estimated.Velocity.Y);

#endif

#ifdef PRINTLN_INS_POS_VEL_Z

  //Barometer.INS_Resources.Altitude.Estimated -> POSITIVO SUBINDO E NEGATIVO DESCENDO
  //Barometer.INS_Resources.Velocity.Vertical  -> POSITIVO SUBINDO E NEGATIVO DESCENDO

  DEBUG("%ld %d",
        Barometer.INS_Resources.Altitude.Estimated,
        Barometer.INS_Resources.Velocity.Vertical);

#endif

  INERTIALNAVIGATION.UpdateGPS();
}

bool InertialNavigationClass::WaitForSample(void)
{
#define SAMPLE_DELAY 2.0f //SEGUNDOS

  static uint16_t SampleCount = 0;

  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    SampleCount = 0;
  }

  if (SampleCount >= (50 * SAMPLE_DELAY))
  {
    return true;
  }
  else
  {
    SampleCount++;
  }

  return false;
}

void InertialNavigationClass::UpdateIMU(uint32_t ActualTimeInUs)
{
  const float DeltaTime = (ActualTimeInUs - INS_Resources.IMU.LastUpdateTime) * 1e-6f;
  INS_Resources.IMU.LastUpdateTime = ActualTimeInUs;

  //ATUALIZA O WEIGHT BASEADO NAS VIBRAÇÕES DO ACC
  const bool AccClipped = ABS(IMU.Accelerometer.ReadFloat[ROLL]) > ACC_CLIPPING_THRESHOLD_G ||
                          ABS(IMU.Accelerometer.ReadFloat[PITCH]) > ACC_CLIPPING_THRESHOLD_G ||
                          ABS(IMU.Accelerometer.ReadFloat[YAW]) > ACC_CLIPPING_THRESHOLD_G;

  //SE O ACELEROMETRO FOR CLIPADO,O WEIGHT SERÁ REDUZIDO A ZERO.CASO CONTRARIO SERÁ RESTAURADO PARA 1 GRADUALMENTE.
  if (AccClipped)
  {
    INS_Resources.IMU.AccWeightFactor = 0.0f;
  }
  else
  {
    const float DTAlpha = DeltaTime / (DeltaTime + ACC_CLIPPING_RC_CONSTANT);
    INS_Resources.IMU.AccWeightFactor = INS_Resources.IMU.AccWeightFactor * (1.0f - DTAlpha) + 1.0f * DTAlpha;
  }

  //RESETA O VETOR DA ACELERAÇÃO DO BODY-FRAME
  VectorZero(&INS_Resources.NewAccelerationEarthFrame);

  //PASSA A ACELERAÇÃO DO EARTH-FRAME PARA UM NOVO VETOR
  INS_Resources.NewAccelerationEarthFrame.X = BodyFrameAcceleration.X;
  INS_Resources.NewAccelerationEarthFrame.Y = BodyFrameAcceleration.Y;
  INS_Resources.NewAccelerationEarthFrame.Z = BodyFrameAcceleration.Z;

  //TRANSFORMA O VETOR DA ACELERAÇÃO PARA EARTH-FRAME
  AHRS.TransformVectorBodyFrameToEarthFrame(&INS_Resources.NewAccelerationEarthFrame);

  //CALIBRA O 1G DA ACELERAÇÃO
  GRAVITYCALIBRATION.Update(&INS_Resources.NewAccelerationEarthFrame);

  //CORRIGE A BIAS DO ACELEROMETRO E PASSA A ACELERAÇÃO EM NEU PARA NOVAS VARIAVEIS PARA SEREM APLICADAS NO INS
  INS_Resources.IMU.AccelerationNEU.X = INS_Resources.NewAccelerationEarthFrame.X - INS_Resources.IMU.AccelerationBias.X;
  INS_Resources.IMU.AccelerationNEU.Y = INS_Resources.NewAccelerationEarthFrame.Y - INS_Resources.IMU.AccelerationBias.Y;
  INS_Resources.IMU.AccelerationNEU.Z = INS_Resources.NewAccelerationEarthFrame.Z - INS_Resources.IMU.AccelerationBias.Z;
}

void InertialNavigationClass::UpdateBarometer(uint32_t ActualTimeInUs)
{
  if (I2CResources.Found.Barometer)
  {
    INS_Resources.Barometer.ActualAltitude = Barometer.Altitude.Actual;
    INS_Resources.Barometer.EstimatedPositionVertical = Max_INS_Baro_Error;
    INS_Resources.Barometer.LastUpdateTime = ActualTimeInUs;
  }
  else
  {
    INS_Resources.Barometer.ActualAltitude = 0;
    INS_Resources.Barometer.LastUpdateTime = 0;
  }
}

void InertialNavigationClass::UpdateGPS(void)
{
  const uint32_t ActualTimeInUs = SCHEDULERTIME.GetMicros();

  if (Get_GPS_In_Bad_Condition())
  {
    INS_Resources.GPS.EstimatedPositionHorizontal = GPS_DEFAULT_EPH;
    INS_Resources.GPS.EstimatedPositionVertical = GPS_DEFAULT_EPV;
    return;
  }

  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS_Resources.GPS.AltitudeOffSet = GPS_Resources.Navigation.Misc.Get.Altitude;
  }

  INS_Resources.GPS.Position.X = GPS_Resources.Home.INS.Distance[COORD_LATITUDE];
  INS_Resources.GPS.Position.Y = GPS_Resources.Home.INS.Distance[COORD_LONGITUDE];
  INS_Resources.GPS.Position.Z = GPS_Resources.Navigation.Misc.Get.Altitude - INS_Resources.GPS.AltitudeOffSet; //ALTITUDE DO GPS PARA AERONAVES DE ASA-FIXA APENAS

  INS_Resources.GPS.Velocity.X = GPS_Resources.Navigation.Misc.Velocity.Get[NORTH];
  INS_Resources.GPS.Velocity.Y = GPS_Resources.Navigation.Misc.Velocity.Get[EAST];
  INS_Resources.GPS.Velocity.Z = -GPS_Resources.Navigation.Misc.Velocity.Get[DOWN];

  INS_Resources.GPS.EstimatedPositionHorizontal = GPS_Resources.Navigation.Misc.Get.EstimatedPositionHorizontal;
  INS_Resources.GPS.EstimatedPositionVertical = GPS_Resources.Navigation.Misc.Get.EstimatedPositionVertical;

  INS_Resources.GPS.LastUpdateTime = ActualTimeInUs;
}

void InertialNavigationClass::UpdatePredictXYZ(INS_Context_Struct *Context)
{
  const float AccWeight = INS_Resources.IMU.AccWeightFactor * Weight_XYZ_Acc_Position;

  //PREDIÇÃO Z
  if ((Context->NewFlags & EST_Z_VALID))
  {
    INS_Resources.Estimated.Position.Z += INS_Resources.Estimated.Velocity.Z * Context->DeltaTime;
    INS_Resources.Estimated.Position.Z += INS_Resources.IMU.AccelerationNEU.Z * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
    INS_Resources.Estimated.Velocity.Z += INS_Resources.IMU.AccelerationNEU.Z * Context->DeltaTime * SquareFloat(AccWeight);
  }

  //PREDIÇÃO X E Y
  if ((Context->NewFlags & EST_XY_VALID))
  {
    //PREDIÇÃO DA POSIÇÃO BASEADO NA VELOCIDADE
    INS_Resources.Estimated.Position.X += INS_Resources.Estimated.Velocity.X * Context->DeltaTime;
    INS_Resources.Estimated.Position.Y += INS_Resources.Estimated.Velocity.Y * Context->DeltaTime;

    //VERIFICA SE ESTÁ OK PARA A INTEGRAÇÃO NEU
    if (I2CResources.Found.Compass || (GetAirPlaneEnabled() && GPS_Resources.Navigation.Misc.Get.HeadingInitialized))
    {
      INS_Resources.Estimated.Position.X += INS_Resources.IMU.AccelerationNEU.X * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
      INS_Resources.Estimated.Position.Y += INS_Resources.IMU.AccelerationNEU.Y * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
      INS_Resources.Estimated.Velocity.X += INS_Resources.IMU.AccelerationNEU.X * Context->DeltaTime * SquareFloat(AccWeight);
      INS_Resources.Estimated.Velocity.Y += INS_Resources.IMU.AccelerationNEU.Y * Context->DeltaTime * SquareFloat(AccWeight);
    }
  }
}

bool InertialNavigationClass::CorrectXYStateWithGPS(INS_Context_Struct *Context)
{
  if (Context->NewFlags & EST_GPS_XY_VALID)
  {
    //RESETA AS COORDENAS E VELOCIDADES
    if (!(Context->NewFlags & EST_XY_VALID))
    {
      Context->EstimatedPosistionCorrected.X += INS_Resources.GPS.Position.X - INS_Resources.Estimated.Position.X;
      Context->EstimatedPosistionCorrected.Y += INS_Resources.GPS.Position.Y - INS_Resources.Estimated.Position.Y;
      Context->EstimatedVelocityCorrected.X += INS_Resources.GPS.Velocity.X - INS_Resources.Estimated.Velocity.X;
      Context->EstimatedVelocityCorrected.Y += INS_Resources.GPS.Velocity.Y - INS_Resources.Estimated.Velocity.Y;
      Context->NewEstimatedPositionHorizontal = INS_Resources.GPS.EstimatedPositionHorizontal;
    }
    else
    {
      const float GPSPositionXResidual = INS_Resources.GPS.Position.X - INS_Resources.Estimated.Position.X;
      const float GPSPositionYResidual = INS_Resources.GPS.Position.Y - INS_Resources.Estimated.Position.Y;
      const float GPSVelocityXResidual = INS_Resources.GPS.Velocity.X - INS_Resources.Estimated.Velocity.X;
      const float GPSVelocityYResidual = INS_Resources.GPS.Velocity.Y - INS_Resources.Estimated.Velocity.Y;
      const float GPSPositionResidualMag = sqrtf(SquareFloat(GPSPositionXResidual) + SquareFloat(GPSPositionYResidual));

      const float New_Weight_XY_GPS_Position = Weight_XY_GPS_Position;
      const float New_Weight_XY_GPS_Velocity = Weight_XY_GPS_Velocity;

      //NOVAS COORDENADAS
      Context->EstimatedPosistionCorrected.X += GPSPositionXResidual * New_Weight_XY_GPS_Position * Context->DeltaTime;
      Context->EstimatedPosistionCorrected.Y += GPSPositionYResidual * New_Weight_XY_GPS_Position * Context->DeltaTime;

      //AJUSTA AS VELOCIDADES PARA APLICAR NA COORDENADAS
      Context->EstimatedVelocityCorrected.X += GPSPositionXResidual * SquareFloat(New_Weight_XY_GPS_Position) * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Y += GPSPositionYResidual * SquareFloat(New_Weight_XY_GPS_Position) * Context->DeltaTime;

      //AJUSTA AS VELOCIDADES DE ACORDO COM A MEDIÇÃO DO RESIDUAL X E Y
      Context->EstimatedVelocityCorrected.X += GPSVelocityXResidual * New_Weight_XY_GPS_Velocity * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Y += GPSVelocityYResidual * New_Weight_XY_GPS_Velocity * Context->DeltaTime;

      //AJUSTA A BIAS DO ACELEROMETRO
      Context->AccBiasCorrected.X -= GPSPositionXResidual * SquareFloat(New_Weight_XY_GPS_Position);
      Context->AccBiasCorrected.Y -= GPSPositionYResidual * SquareFloat(New_Weight_XY_GPS_Position);

      //AJUSTA A ESTIMATIVA DE POSIÇÃO HORIZONTAL
      Context->NewEstimatedPositionHorizontal = INS_Resources.Estimated.EstimatedPositionHorizontal + (MAX(INS_Resources.GPS.EstimatedPositionHorizontal, GPSPositionResidualMag) - INS_Resources.Estimated.EstimatedPositionHorizontal) * New_Weight_XY_GPS_Position * Context->DeltaTime;
    }

    return true;
  }

  return false;
}

bool InertialNavigationClass::CorrectZStateWithBaroOrGPS(INS_Context_Struct *Context)
{
  if (Context->NewFlags & EST_BARO_VALID)
  {
    uint32_t ActualTimeInUs = SCHEDULERTIME.GetMicros();

    if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
      INS_Resources.State.BaroGroundAlt = INS_Resources.Estimated.Position.Z;
      INS_Resources.State.BaroGroundValid = true;
      INS_Resources.State.BaroGroundTimeout = ActualTimeInUs + 250000;
    }
    else
    {
      if (INS_Resources.Estimated.Velocity.Z > 15)
      {
        if (ActualTimeInUs > INS_Resources.State.BaroGroundTimeout)
        {
          INS_Resources.State.BaroGroundValid = false;
        }
      }
      else
      {
        INS_Resources.State.BaroGroundTimeout = ActualTimeInUs + 250000;
      }
    }

    //VERIFICA SE O EFEITO ALMOFADA EXISTE
    bool AirCushionEffectDetected = IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && ((Context->NewFlags & EST_BARO_VALID) && INS_Resources.State.BaroGroundValid && INS_Resources.Barometer.ActualAltitude < INS_Resources.State.BaroGroundAlt);

    //CALCULA A ALTITUDE COM BASE NO BARO
    const float BaroAltResidual = (AirCushionEffectDetected ? INS_Resources.State.BaroGroundAlt : INS_Resources.Barometer.ActualAltitude) - INS_Resources.Estimated.Position.Z;
    Context->EstimatedPosistionCorrected.Z += BaroAltResidual * Weight_Z_Baro_Position * Context->DeltaTime;
    Context->EstimatedVelocityCorrected.Z += BaroAltResidual * SquareFloat(Weight_Z_Baro_Position) * Context->DeltaTime;

    //VERIFICIA SE A ESTIMATIVA DE ALTITUDE DO GPS É VALIDA
    if (Context->NewFlags & EST_GPS_Z_VALID)
    {
      const float GPSRocResidual = INS_Resources.GPS.Velocity.Z - INS_Resources.Estimated.Velocity.Z;
      const float GPSRocScaler = Sine_Curve(GPSRocResidual, 250.0f);
      Context->EstimatedVelocityCorrected.Z += GPSRocResidual * Weight_Z_GPS_Velocity * GPSRocScaler * Context->DeltaTime;
    }

    //AJUSTA A ESTIMATIVA DE POSIÇÃO VERTICAL
    Context->NewEstimatedPositionVertical = INS_Resources.Estimated.EstimatedPositionVertical + (INS_Resources.Barometer.EstimatedPositionVertical - INS_Resources.Estimated.EstimatedPositionVertical) * Weight_Z_Baro_Position * Context->DeltaTime;

    //AJUSTA A BIAS DO Z
    if (!AirCushionEffectDetected)
    {
      Context->AccBiasCorrected.Z -= BaroAltResidual * SquareFloat(Weight_Z_Baro_Position);
    }

    return true;
  }
  else if (GetAirPlaneEnabled() && (Context->NewFlags & EST_GPS_Z_VALID))
  {
    //RESETA A POSIÇÃO E VELOCIDADE Z
    if (!(Context->NewFlags & EST_Z_VALID))
    {
      Context->EstimatedPosistionCorrected.Z += INS_Resources.GPS.Position.Z - INS_Resources.Estimated.Position.Z;
      Context->EstimatedVelocityCorrected.Z += INS_Resources.GPS.Velocity.Z - INS_Resources.Estimated.Velocity.Z;
      Context->NewEstimatedPositionVertical = INS_Resources.GPS.EstimatedPositionVertical;
    }
    else
    {
      //CALCULA A ALTITUDE BASEADO NO GPS
      const float GPSAltResudual = INS_Resources.GPS.Position.Z - INS_Resources.Estimated.Position.Z;

      Context->EstimatedPosistionCorrected.Z += GPSAltResudual * Weight_Z_GPS_Position * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Z += GPSAltResudual * SquareFloat(Weight_Z_GPS_Position) * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Z += (INS_Resources.GPS.Velocity.Z - INS_Resources.Estimated.Velocity.Z) * Weight_Z_GPS_Velocity * Context->DeltaTime;

      //AJUSTA A ESTIMATIVA DE POSIÇÃO VERTICAL
      Context->NewEstimatedPositionVertical = INS_Resources.Estimated.EstimatedPositionVertical + (MAX(INS_Resources.GPS.EstimatedPositionVertical, GPSAltResudual) - INS_Resources.Estimated.EstimatedPositionVertical) * Weight_Z_GPS_Position * Context->DeltaTime;

      //AJUSTA A BIAS DO Z
      Context->AccBiasCorrected.Z -= GPSAltResudual * SquareFloat(Weight_Z_GPS_Position);
    }

    return true;
  }

  return false;
}

static uint32_t CalculateCurrentValidityFlags(uint32_t ActualTimeInUs)
{
  uint32_t NewFlags = 0;

  if (((ActualTimeInUs - INS_Resources.GPS.LastUpdateTime) <= (GPS_TIMEOUT_MS * 1000L)) && (INS_Resources.GPS.EstimatedPositionHorizontal < Max_INS_H_V_Error))
  {
    if (INS_Resources.GPS.EstimatedPositionVertical < Max_INS_H_V_Error)
    {
      NewFlags |= EST_GPS_XY_VALID | EST_GPS_Z_VALID;
    }
    else
    {
      NewFlags |= EST_GPS_XY_VALID;
    }
  }

  if (I2CResources.Found.Barometer && ((ActualTimeInUs - INS_Resources.Barometer.LastUpdateTime) <= (BARO_TIMEOUT_MS * 1000L)))
  {
    NewFlags |= EST_BARO_VALID;
  }

  if (INS_Resources.Estimated.EstimatedPositionHorizontal < Max_INS_H_V_Error)
  {
    NewFlags |= EST_XY_VALID;
  }

  if (INS_Resources.Estimated.EstimatedPositionVertical < Max_INS_H_V_Error)
  {
    NewFlags |= EST_Z_VALID;
  }

  return NewFlags;
}

void InertialNavigationClass::UpdateEstimationPredictXYZ(uint32_t ActualTimeInUs)
{
  INS_Context_Struct Context;

  Context.DeltaTime = (ActualTimeInUs - INS_Resources.Estimated.LastUpdateTime) * 1e-6f;
  INS_Resources.Estimated.LastUpdateTime = ActualTimeInUs;

  //CALCULA UMA NOVA ESTIMATIVA DE POSIÇÃO HORIZONTAL E VERTICAL
  Context.NewEstimatedPositionHorizontal = INS_Resources.Estimated.EstimatedPositionHorizontal * ((INS_Resources.Estimated.EstimatedPositionHorizontal <= Max_INS_H_V_Error) ? 1.0f + Context.DeltaTime : 1.0f);
  Context.NewEstimatedPositionVertical = INS_Resources.Estimated.EstimatedPositionVertical * ((INS_Resources.Estimated.EstimatedPositionVertical <= Max_INS_H_V_Error) ? 1.0f + Context.DeltaTime : 1.0f);

  //OBTÉM AS FLAGS DE ESTADO DO INS
  Context.NewFlags = CalculateCurrentValidityFlags(ActualTimeInUs);

  //RESETA OS VETORES
  VectorZero(&Context.EstimatedPosistionCorrected);
  VectorZero(&Context.EstimatedVelocityCorrected);
  VectorZero(&Context.AccBiasCorrected);

  //PREDIÇÃO X,Y E Z
  INERTIALNAVIGATION.UpdatePredictXYZ(&Context);

  //CORREÇÃO Z
  const bool EstimatorZCorrectState = INERTIALNAVIGATION.CorrectZStateWithBaroOrGPS(&Context);

  //CORREÇÃO X E Y
  const bool EstimatorXYCorrectState = INERTIALNAVIGATION.CorrectXYStateWithGPS(&Context);

  if (!EstimatorXYCorrectState || Context.NewEstimatedPositionHorizontal > Max_INS_H_V_Error)
  {
    Context.EstimatedVelocityCorrected.X = (0.0f - INS_Resources.Estimated.Velocity.X) * Weight_XY_Coeff_Velocity * Context.DeltaTime;
    Context.EstimatedVelocityCorrected.Y = (0.0f - INS_Resources.Estimated.Velocity.Y) * Weight_XY_Coeff_Velocity * Context.DeltaTime;
  }

  if (!EstimatorZCorrectState || Context.NewEstimatedPositionVertical > Max_INS_H_V_Error)
  {
    Context.EstimatedVelocityCorrected.Z = (0.0f - INS_Resources.Estimated.Velocity.Z) * Weight_Z_Coeff_Velocity * Context.DeltaTime;
  }

  //APLICA AS CORREÇÕES
  VectorAdd(&INS_Resources.Estimated.Position, &INS_Resources.Estimated.Position, &Context.EstimatedPosistionCorrected);
  VectorAdd(&INS_Resources.Estimated.Velocity, &INS_Resources.Estimated.Velocity, &Context.EstimatedVelocityCorrected);

  if (Weight_Acc_Bias > 0.0f)
  {
    const float AccelerationBiasMagnitude = SquareFloat(Context.AccBiasCorrected.X) + SquareFloat(Context.AccBiasCorrected.Y) + SquareFloat(Context.AccBiasCorrected.Z);
    if (AccelerationBiasMagnitude < SquareFloat(ACC_BIAS_ACCEPTANCE_VALUE))
    {
      //TRANSFORMA DE EARTH PARA BODY
      AHRS.TransformVectorEarthFrameToBodyFrame(&Context.AccBiasCorrected);

      //AJUSTA A BIAS DA ACELERAÇÃO
      INS_Resources.IMU.AccelerationBias.X += Context.AccBiasCorrected.X * Weight_Acc_Bias * Context.DeltaTime;
      INS_Resources.IMU.AccelerationBias.Y += Context.AccBiasCorrected.Y * Weight_Acc_Bias * Context.DeltaTime;
      INS_Resources.IMU.AccelerationBias.Z += Context.AccBiasCorrected.Z * Weight_Acc_Bias * Context.DeltaTime;
    }
  }

  INS_Resources.Estimated.EstimatedPositionHorizontal = Context.NewEstimatedPositionHorizontal;
  INS_Resources.Estimated.EstimatedPositionVertical = Context.NewEstimatedPositionVertical;

  INS_Resources.Flags = Context.NewFlags;
}