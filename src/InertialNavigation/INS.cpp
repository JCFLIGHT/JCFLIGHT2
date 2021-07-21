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

#define GPS_DEFAULT_EPH 200.0f                           //2M
#define GPS_DEFAULT_EPV 500.0f                           //5M
#define GPS_ACCEPTANCE_EPE 500.0f                        //5M
#define GPS_GLITCH_RADIUS 250.0f                         //2.5M GPS
#define GPS_GLITCH_ACCEL 1000.0f                         //10M/S/S
#define GPS_TIMEOUT_MS 1500                              //MS
#define BARO_TIMEOUT_MS 200                              //MS
#define ACC_BIAS_ACCEPTANCE_VALUE (GRAVITY_CMSS * 0.25f) //0.25G
#define POSITION_RATE 50                                 //HZ
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
//#define PRINTLN_INS_COS_SIN
//#define PRINTLN_INS_ACC_NEU
//#define PRINTLN_INS_POS_VEL_XY
//#define PRINTLN_INS_POS_VEL_Z

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
  INS_Resources.NewAccelerationEarthFrame.Roll = BodyFrameAcceleration.Roll;
  INS_Resources.NewAccelerationEarthFrame.Pitch = BodyFrameAcceleration.Pitch;
  INS_Resources.NewAccelerationEarthFrame.Yaw = BodyFrameAcceleration.Yaw;

  //TRANSFORMA O VETOR DA ACELERAÇÃO PARA EARTH-FRAME
  AHRS.TransformVectorBodyFrameToEarthFrame(&INS_Resources.NewAccelerationEarthFrame);

  //CALIBRA O 1G DA ACELERAÇÃO
  GRAVITYCALIBRATION.Update(&INS_Resources.NewAccelerationEarthFrame);

  //CORRIGE A BIAS DO ACELEROMETRO E PASSA A ACELERAÇÃO EM NEU PARA NOVAS VARIAVEIS PARA SEREM APLICADAS NO INS
  INS_Resources.IMU.AccelerationNEU.Roll = INS_Resources.NewAccelerationEarthFrame.Roll - INS_Resources.IMU.AccelerationBias.Roll;
  INS_Resources.IMU.AccelerationNEU.Pitch = INS_Resources.NewAccelerationEarthFrame.Pitch - INS_Resources.IMU.AccelerationBias.Pitch;
  INS_Resources.IMU.AccelerationNEU.Yaw = INS_Resources.NewAccelerationEarthFrame.Yaw - INS_Resources.IMU.AccelerationBias.Yaw;
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

  INS_Resources.GPS.Position.Roll = GPS_Resources.Home.INS.Distance[COORD_LATITUDE];
  INS_Resources.GPS.Position.Pitch = GPS_Resources.Home.INS.Distance[COORD_LONGITUDE];
  INS_Resources.GPS.Position.Yaw = GPS_Resources.Navigation.Misc.Get.Altitude - INS_Resources.GPS.AltitudeOffSet; //ALTITUDE DO GPS PARA AERONAVES DE ASA-FIXA APENAS

  INS_Resources.GPS.Velocity.Roll = GPS_Resources.Navigation.Misc.Velocity.Get[NORTH];
  INS_Resources.GPS.Velocity.Pitch = GPS_Resources.Navigation.Misc.Velocity.Get[EAST];
  INS_Resources.GPS.Velocity.Yaw = -GPS_Resources.Navigation.Misc.Velocity.Get[DOWN];

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
    INS_Resources.Estimated.Position.Yaw += INS_Resources.Estimated.Velocity.Yaw * Context->DeltaTime;
    INS_Resources.Estimated.Position.Yaw += INS_Resources.IMU.AccelerationNEU.Yaw * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
    INS_Resources.Estimated.Velocity.Yaw += INS_Resources.IMU.AccelerationNEU.Yaw * Context->DeltaTime * SquareFloat(AccWeight);
  }

  //PREDIÇÃO X E Y
  if ((Context->NewFlags & EST_XY_VALID))
  {
    //PREDIÇÃO DA POSIÇÃO BASEADO NA VELOCIDADE
    INS_Resources.Estimated.Position.Roll += INS_Resources.Estimated.Velocity.Roll * Context->DeltaTime;
    INS_Resources.Estimated.Position.Pitch += INS_Resources.Estimated.Velocity.Pitch * Context->DeltaTime;

    //VERIFICA SE ESTÁ OK PARA A INTEGRAÇÃO NEU
    if (I2CResources.Found.Compass || (GetAirPlaneEnabled() && GPS_Resources.Navigation.Misc.Get.HeadingInitialized))
    {
      INS_Resources.Estimated.Position.Roll += INS_Resources.IMU.AccelerationNEU.Roll * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
      INS_Resources.Estimated.Position.Pitch += INS_Resources.IMU.AccelerationNEU.Pitch * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
      INS_Resources.Estimated.Velocity.Roll += INS_Resources.IMU.AccelerationNEU.Roll * Context->DeltaTime * SquareFloat(AccWeight);
      INS_Resources.Estimated.Velocity.Pitch += INS_Resources.IMU.AccelerationNEU.Pitch * Context->DeltaTime * SquareFloat(AccWeight);
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
      Context->EstimatedPosistionCorrected.Roll += INS_Resources.GPS.Position.Roll - INS_Resources.Estimated.Position.Roll;
      Context->EstimatedPosistionCorrected.Pitch += INS_Resources.GPS.Position.Pitch - INS_Resources.Estimated.Position.Pitch;
      Context->EstimatedVelocityCorrected.Roll += INS_Resources.GPS.Velocity.Roll - INS_Resources.Estimated.Velocity.Roll;
      Context->EstimatedVelocityCorrected.Pitch += INS_Resources.GPS.Velocity.Pitch - INS_Resources.Estimated.Velocity.Pitch;
      Context->NewEstimatedPositionHorizontal = INS_Resources.GPS.EstimatedPositionHorizontal;
    }
    else
    {
      const float GPSPositionXResidual = INS_Resources.GPS.Position.Roll - INS_Resources.Estimated.Position.Roll;
      const float GPSPositionYResidual = INS_Resources.GPS.Position.Pitch - INS_Resources.Estimated.Position.Pitch;
      const float GPSVelocityXResidual = INS_Resources.GPS.Velocity.Roll - INS_Resources.Estimated.Velocity.Roll;
      const float GPSVelocityYResidual = INS_Resources.GPS.Velocity.Pitch - INS_Resources.Estimated.Velocity.Pitch;
      const float GPSPositionResidualMag = sqrtf(SquareFloat(GPSPositionXResidual) + SquareFloat(GPSPositionYResidual));

      //const float GPSWeightScaler = ScaleRangeFloat(Sine_Curve(GPSPositionResidualMag, GPS_ACCEPTANCE_EPE), 0.0f, 1.0f, 0.1f, 1.0f);
      const float GPSWeightScaler = 1.0f;

      const float New_Weight_XY_GPS_Position = Weight_XY_GPS_Position * GPSWeightScaler;
      const float New_Weight_XY_GPS_Velocity = Weight_XY_GPS_Velocity * SquareFloat(GPSWeightScaler);

      //NOVAS COORDENADAS
      Context->EstimatedPosistionCorrected.Roll += GPSPositionXResidual * New_Weight_XY_GPS_Position * Context->DeltaTime;
      Context->EstimatedPosistionCorrected.Pitch += GPSPositionYResidual * New_Weight_XY_GPS_Position * Context->DeltaTime;

      //AJUSTA AS VELOCIDADES PARA APLICAR NA COORDENADAS
      Context->EstimatedVelocityCorrected.Roll += GPSPositionXResidual * SquareFloat(New_Weight_XY_GPS_Position) * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Pitch += GPSPositionYResidual * SquareFloat(New_Weight_XY_GPS_Position) * Context->DeltaTime;

      //AJUSTA AS VELOCIDADES DE ACORDO COM A MEDIÇÃO DO RESIDUAL X E Y
      Context->EstimatedVelocityCorrected.Roll += GPSVelocityXResidual * New_Weight_XY_GPS_Velocity * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Pitch += GPSVelocityYResidual * New_Weight_XY_GPS_Velocity * Context->DeltaTime;

      //AJUSTA A BIAS DO ACELEROMETRO
      Context->AccBiasCorrected.Roll -= GPSPositionXResidual * SquareFloat(New_Weight_XY_GPS_Position);
      Context->AccBiasCorrected.Pitch -= GPSPositionYResidual * SquareFloat(New_Weight_XY_GPS_Position);

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
      INS_Resources.State.BaroGroundAlt = INS_Resources.Estimated.Position.Yaw;
      INS_Resources.State.BaroGroundValid = true;
      INS_Resources.State.BaroGroundTimeout = ActualTimeInUs + 250000;
    }
    else
    {
      if (INS_Resources.Estimated.Velocity.Yaw > 15)
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
    const float BaroAltResidual = (AirCushionEffectDetected ? INS_Resources.State.BaroGroundAlt : INS_Resources.Barometer.ActualAltitude) - INS_Resources.Estimated.Position.Yaw;
    Context->EstimatedPosistionCorrected.Yaw += BaroAltResidual * Weight_Z_Baro_Position * Context->DeltaTime;
    Context->EstimatedVelocityCorrected.Yaw += BaroAltResidual * SquareFloat(Weight_Z_Baro_Position) * Context->DeltaTime;

    if (Context->NewFlags & EST_GPS_Z_VALID)
    {
      const float GPSRocResidual = INS_Resources.GPS.Velocity.Yaw - INS_Resources.Estimated.Velocity.Yaw;
      const float GPSRocScaler = Sine_Curve(GPSRocResidual, 250.0f);
      Context->EstimatedVelocityCorrected.Yaw += GPSRocResidual * Weight_Z_GPS_Velocity * GPSRocScaler * Context->DeltaTime;
    }

    //AJUSTA A ESTIMATIVA DE POSIÇÃO VERTICAL
    Context->NewEstimatedPositionVertical = INS_Resources.Estimated.EstimatedPositionVertical + (INS_Resources.Barometer.EstimatedPositionVertical - INS_Resources.Estimated.EstimatedPositionVertical) * Weight_Z_Baro_Position * Context->DeltaTime;

    //AJUSTA A BIAS DO Z
    if (!AirCushionEffectDetected)
    {
      Context->AccBiasCorrected.Yaw -= BaroAltResidual * SquareFloat(Weight_Z_Baro_Position);
    }

    return true;
  }
  else if (GetAirPlaneEnabled() && (Context->NewFlags & EST_GPS_Z_VALID))
  {
    //RESETA A POSIÇÃO E VELOCIDADE Z
    if (!(Context->NewFlags & EST_Z_VALID))
    {
      Context->EstimatedPosistionCorrected.Yaw += INS_Resources.GPS.Position.Yaw - INS_Resources.Estimated.Position.Yaw;
      Context->EstimatedVelocityCorrected.Yaw += INS_Resources.GPS.Velocity.Yaw - INS_Resources.Estimated.Velocity.Yaw;
      Context->NewEstimatedPositionVertical = INS_Resources.GPS.EstimatedPositionVertical;
    }
    else
    {
      //CALCULA A ALTITUDE BASEADO NO GPS
      const float GPSAltResudual = INS_Resources.GPS.Position.Yaw - INS_Resources.Estimated.Position.Yaw;

      Context->EstimatedPosistionCorrected.Yaw += GPSAltResudual * Weight_Z_GPS_Position * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Yaw += GPSAltResudual * SquareFloat(Weight_Z_GPS_Position) * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Yaw += (INS_Resources.GPS.Velocity.Yaw - INS_Resources.Estimated.Velocity.Yaw) * Weight_Z_GPS_Velocity * Context->DeltaTime;

      //AJUSTA A ESTIMATIVA DE POSIÇÃO VERTICAL
      Context->NewEstimatedPositionVertical = INS_Resources.Estimated.EstimatedPositionVertical + (MAX(INS_Resources.GPS.EstimatedPositionVertical, GPSAltResudual) - INS_Resources.Estimated.EstimatedPositionVertical) * Weight_Z_GPS_Position * Context->DeltaTime;

      //AJUSTA A BIAS DO Z
      Context->AccBiasCorrected.Yaw -= GPSAltResudual * SquareFloat(Weight_Z_GPS_Position);
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
    Context.EstimatedVelocityCorrected.Roll = (0.0f - INS_Resources.Estimated.Velocity.Roll) * Weight_XY_Coeff_Velocity * Context.DeltaTime;
    Context.EstimatedVelocityCorrected.Pitch = (0.0f - INS_Resources.Estimated.Velocity.Pitch) * Weight_XY_Coeff_Velocity * Context.DeltaTime;
  }

  if (!EstimatorZCorrectState || Context.NewEstimatedPositionVertical > Max_INS_H_V_Error)
  {
    Context.EstimatedVelocityCorrected.Yaw = (0.0f - INS_Resources.Estimated.Velocity.Yaw) * Weight_Z_Coeff_Velocity * Context.DeltaTime;
  }

  //APLICA AS CORREÇÕES
  VectorAdd(&INS_Resources.Estimated.Position, &INS_Resources.Estimated.Position, &Context.EstimatedPosistionCorrected);
  VectorAdd(&INS_Resources.Estimated.Velocity, &INS_Resources.Estimated.Velocity, &Context.EstimatedVelocityCorrected);

  if (Weight_Acc_Bias > 0.0f)
  {
    const float accelBiasCorrMagnitudeSq = SquareFloat(Context.AccBiasCorrected.Roll) + SquareFloat(Context.AccBiasCorrected.Pitch) + SquareFloat(Context.AccBiasCorrected.Yaw);
    if (accelBiasCorrMagnitudeSq < SquareFloat(ACC_BIAS_ACCEPTANCE_VALUE))
    {
      //TRANFORMA DE EARTH PARA BODY
      AHRS.TransformVectorEarthFrameToBodyFrame(&Context.AccBiasCorrected);

      //AJUSTA A BIAS DA ACELERAÇÃO
      INS_Resources.IMU.AccelerationBias.Roll += Context.AccBiasCorrected.Roll * Weight_Acc_Bias * Context.DeltaTime;
      INS_Resources.IMU.AccelerationBias.Pitch += Context.AccBiasCorrected.Pitch * Weight_Acc_Bias * Context.DeltaTime;
      INS_Resources.IMU.AccelerationBias.Yaw += Context.AccBiasCorrected.Yaw * Weight_Acc_Bias * Context.DeltaTime;
    }
  }

  INS_Resources.Estimated.EstimatedPositionHorizontal = Context.NewEstimatedPositionHorizontal;
  INS_Resources.Estimated.EstimatedPositionVertical = Context.NewEstimatedPositionVertical;

  INS_Resources.Flags = Context.NewFlags;
}

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
  static uint32_t LastTriggeredTime;

  INERTIALNAVIGATION.UpdateIMU(ActualTimeInUs);
  INERTIALNAVIGATION.UpdateBarometer(ActualTimeInUs);
  INERTIALNAVIGATION.UpdateEstimationPredictXYZ(ActualTimeInUs);

  //OBTÉM O SENO E COSSENO DO YAW DADO PELO AHRS
  INS_Resources.Math.Cosine.Yaw = AHRS.GetCosineYaw();
  INS_Resources.Math.Sine.Yaw = AHRS.GetSineYaw();

  if ((ActualTimeInUs - LastTriggeredTime) >= SCHEDULER_SET_FREQUENCY(POSITION_RATE, "Hz"))
  {
    LastTriggeredTime = ActualTimeInUs;

    /*
    DEBUG("%.f %.f %.f %.f %ld %.f %.f",
          INS_Resources.Estimated.Position.Roll,  //X POS
          INS_Resources.Estimated.Position.Pitch, //Y POS
          INS_Resources.Estimated.Velocity.Roll,  //X VEL
          INS_Resources.Estimated.Velocity.Pitch, //Y VEL
          Barometer.Altitude.Actual,             //BARO ALT
          INS_Resources.Estimated.Position.Yaw,   //Z POS
          INS_Resources.Estimated.Velocity.Yaw);  //Z VEL
*/

    INS_Resources.EarthFrame.Position[INS_LATITUDE] = INS_Resources.Estimated.Position.Roll;   //POSIÇÃO FINAL X ESTIMADA PELO INS
    INS_Resources.EarthFrame.Position[INS_LONGITUDE] = INS_Resources.Estimated.Position.Pitch; //POSIÇÃO FINAL Y ESTIMADA PELO INS
    INS_Resources.EarthFrame.Velocity[INS_LATITUDE] = INS_Resources.Estimated.Velocity.Roll;   //VELOCIDADE FINAL X ESTIMADA PELO INS
    INS_Resources.EarthFrame.Velocity[INS_LONGITUDE] = INS_Resources.Estimated.Velocity.Pitch; //VELOCIDADE FINAL Y ESTIMADA PELO INS
    Barometer.INS.Altitude.Estimated = INS_Resources.Estimated.Position.Yaw;                   //ALTITUDE FINAL ESTIMADA PELO INS
    Barometer.INS.Velocity.Vertical = INS_Resources.Estimated.Velocity.Yaw;                    //VELOCIDADE VERTICAL(Z) FINAL ESTIMADA PELO INS

#ifdef PRINTLN_INS_COS_SIN

    DEBUG("%d %.2f %.2f",
          Attitude.EulerAngles.YawDecidegrees,
          INS_Resources.Math.Cosine.Yaw,
          INS_Resources.Math.Sine.Yaw);

#endif

#ifdef PRINTLN_INS_ACC_NEU

    //INS_Resources.IMU.AccelerationNEU.Roll  -> POSITIVO MOVENDO PARA O NORTE
    //INS_Resources.IMU.AccelerationNEU.Pitch -> POSITIVO MOVENDO PARA O OESTE
    //INS_Resources.IMU.AccelerationNEU.Yaw   -> POSITIVO MOVENDO PARA CIMA

    DEBUG("NORTH:%.4f EAST:%.4f UP:%.4f",
          INS_Resources.IMU.AccelerationNEU.Roll,
          INS_Resources.IMU.AccelerationNEU.Pitch,
          INS_Resources.IMU.AccelerationNEU.Yaw);

#endif

#ifdef PRINTLN_INS_POS_VEL_XY

    DEBUG("%.f %.f %.f %.f",
          INS_Resources.EarthFrame.Position[INS_LATITUDE],
          INS_Resources.EarthFrame.Position[INS_LONGITUDE],
          INS_Resources.EarthFrame.Velocity[INS_LATITUDE],
          INS_Resources.EarthFrame.Velocity[INS_LONGITUDE]);

#endif

#ifdef PRINTLN_INS_POS_VEL_Z

    //Barometer.INS_Resources.Altitude.Estimated -> POSITIVO SUBINDO E NEGATIVO DESCENDO
    //Barometer.INS_Resources.Velocity.Vertical  -> POSITIVO SUBINDO E NEGATIVO DESCENDO

    DEBUG("%ld %d",
          Barometer.INS_Resources.Altitude.Estimated,
          Barometer.INS_Resources.Velocity.Vertical);

#endif
  }

  INERTIALNAVIGATION.UpdateGPS();
}