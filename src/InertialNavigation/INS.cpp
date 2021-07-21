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
#include "FastSerial/PRINTF.h"

InertialNavigationClass INERTIALNAVIGATION;
INS_Struct INS;

//#define TEST_NEW_INS

//DEBUG
//#define PRINTLN_INS_COS_SIN
//#define PRINTLN_INS_ACC_NEU
//#define PRINTLN_INS_POS_VEL_XY
//#define PRINTLN_INS_POS_VEL_Z

#ifndef TEST_NEW_INS

#define MAX_INS_POSITION_ERROR 1000.0f //ERRO MAXIMO DE POSIÇÃO QUE SE PODE ACUMULAR NO INS (EM CM)

void InertialNavigationClass::Calculate_AccelerationXYZ_To_EarthFrame(void)
{
  VectorZero(&INS.NewAccelerationEarthFrame);

  //OBTÉM O SENO E COSSENO DO YAW DADO PELO AHRS
  INS.Math.Cosine.Yaw = AHRS.GetCosineYaw();
  INS.Math.Sine.Yaw = AHRS.GetSineYaw();

#ifdef PRINTLN_INS_COS_SIN

  //COM O COMPASS CALIBRADO E APONTANTO PARA 1000:COSSENO DEVE SER NEGATIVO E SENO DEVE SER POSITIVO

  DEBUG("%d %.2f %.2f",
        Attitude.EulerAngles.YawDecidegrees,
        INS.Math.Cosine.Yaw,
        INS.Math.Sine.Yaw);

#endif

  INS.NewAccelerationEarthFrame.Roll = BodyFrameAcceleration.Roll;
  INS.NewAccelerationEarthFrame.Pitch = BodyFrameAcceleration.Pitch;
  INS.NewAccelerationEarthFrame.Yaw = BodyFrameAcceleration.Yaw;

  AHRS.TransformVectorBodyFrameToEarthFrame(&INS.NewAccelerationEarthFrame);

  GRAVITYCALIBRATION.Update(&INS.NewAccelerationEarthFrame);

  INS.EarthFrame.AccelerationNEU[NORTH] = INS.NewAccelerationEarthFrame.Roll;
  INS.EarthFrame.AccelerationNEU[EAST] = INS.NewAccelerationEarthFrame.Pitch;
  INS.EarthFrame.AccelerationNEU[UP] = INS.NewAccelerationEarthFrame.Yaw;

  //NORTH
  INS.Bias.Difference[NORTH] = INS.EarthFrame.AccelerationNEU[NORTH] - INS.Bias.Adjust[NORTH];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[NORTH] = INS.Bias.Adjust[NORTH] * 0.985f + INS.EarthFrame.AccelerationNEU[NORTH] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[NORTH]) <= 80.0f)
  {
    INS.Bias.Adjust[NORTH] = INS.Bias.Adjust[NORTH] * 0.9987f + INS.EarthFrame.AccelerationNEU[NORTH] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.AccelerationNEU[NORTH] = INS.Bias.Difference[NORTH];
  INS.AccelerationEarthFrame_LPF[NORTH] = INS.AccelerationEarthFrame_LPF[NORTH] * 0.85714285714285714285714285714286f + INS.EarthFrame.AccelerationNEU[NORTH] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[NORTH] += INS.AccelerationEarthFrame_LPF[NORTH];
  INS.AccelerationEarthFrame_Sum_Count[NORTH]++;

  //EAST
  INS.Bias.Difference[EAST] = INS.EarthFrame.AccelerationNEU[EAST] - INS.Bias.Adjust[EAST];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[EAST] = INS.Bias.Adjust[EAST] * 0.985f + INS.EarthFrame.AccelerationNEU[EAST] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[EAST]) <= 80.0f)
  {
    INS.Bias.Adjust[EAST] = INS.Bias.Adjust[EAST] * 0.9987f + INS.EarthFrame.AccelerationNEU[EAST] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.AccelerationNEU[EAST] = INS.Bias.Difference[EAST];
  INS.AccelerationEarthFrame_LPF[EAST] = INS.AccelerationEarthFrame_LPF[EAST] * 0.85714285714285714285714285714286f + INS.EarthFrame.AccelerationNEU[EAST] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[EAST] += INS.AccelerationEarthFrame_LPF[EAST];
  INS.AccelerationEarthFrame_Sum_Count[EAST]++;

  //UP
  INS.Bias.Difference[UP] = INS.EarthFrame.AccelerationNEU[UP] - INS.Bias.Adjust[UP];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[UP] = INS.Bias.Adjust[UP] * 0.985f + INS.EarthFrame.AccelerationNEU[UP] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[UP]) <= 80.0f)
  {
    INS.Bias.Adjust[UP] = INS.Bias.Adjust[UP] * 0.9987f + INS.EarthFrame.AccelerationNEU[UP] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.AccelerationNEU[UP] = INS.Bias.Difference[UP];
  INS.AccelerationEarthFrame_LPF[UP] = INS.AccelerationEarthFrame_LPF[UP] * 0.85714285714285714285714285714286f + INS.EarthFrame.AccelerationNEU[UP] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[UP] += INS.AccelerationEarthFrame_LPF[UP];
  INS.AccelerationEarthFrame_Sum_Count[UP]++;

#ifdef PRINTLN_INS_ACC_NEU

  //INS.EarthFrame.AccelerationNEU[NORTH] -> POSITIVO MOVENDO PARA O NORTE
  //INS.EarthFrame.AccelerationNEU[EAST]  -> POSITIVO MOVENDO PARA O OESTE
  //INS.EarthFrame.AccelerationNEU[UP]    -> POSITIVO MOVENDO PARA CIMA

  DEBUG("NORTH:%.4f EAST:%.4f UP:%.4f",
        INS.EarthFrame.AccelerationNEU[NORTH],
        INS.EarthFrame.AccelerationNEU[EAST],
        INS.EarthFrame.AccelerationNEU[UP]);

#endif
}

#endif

void InertialNavigationClass::UpdateAccelerationEarthFrame_Filtered(uint8_t ArrayCount)
{
#ifndef TEST_NEW_INS

  INS.AccelerationEarthFrame_Filtered[ArrayCount] = INS.AccelerationEarthFrame_Sum[ArrayCount] / INS.AccelerationEarthFrame_Sum_Count[ArrayCount];
  INS.AccelerationEarthFrame_Sum[ArrayCount] = 0.0f;
  INS.AccelerationEarthFrame_Sum_Count[ArrayCount] = 0;

#endif
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

void InertialNavigationClass::Calculate_AccelerationXY(float DeltaTime)
{
#ifndef TEST_NEW_INS

  INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_LATITUDE);
  INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_LONGITUDE);
  if (Get_State_Armed_With_GPS() && INERTIALNAVIGATION.WaitForSample())
  {
    INERTIALNAVIGATION.CorrectXYStateWithGPS(DeltaTime);
    INERTIALNAVIGATION.EstimationPredictXY(DeltaTime);
    INERTIALNAVIGATION.SaveXYPositionToHistory();
  }
  else
  {
    INERTIALNAVIGATION.ResetXYState();
  }

#endif
}

void InertialNavigationClass::CorrectXYStateWithGPS(float DeltaTime)
{
#ifndef TEST_NEW_INS

  float PositionError[2] = {0, 0};
  PositionError[INS_LATITUDE] = GPS_Resources.Home.INS.Distance[COORD_LATITUDE] - INS.History.XYPosition[INS_LATITUDE][INS.History.XYCount];
  PositionError[INS_LONGITUDE] = GPS_Resources.Home.INS.Distance[COORD_LONGITUDE] - INS.History.XYPosition[INS_LONGITUDE][INS.History.XYCount];
  PositionError[INS_LATITUDE] = Constrain_Float(PositionError[INS_LATITUDE], -MAX_INS_POSITION_ERROR, MAX_INS_POSITION_ERROR);
  PositionError[INS_LONGITUDE] = Constrain_Float(PositionError[INS_LONGITUDE], -MAX_INS_POSITION_ERROR, MAX_INS_POSITION_ERROR);
  INS.EarthFrame.Velocity[INS_LATITUDE] += PositionError[INS_LATITUDE] * (DeltaTime * 0.48f);
  INS.EarthFrame.Velocity[INS_LONGITUDE] += PositionError[INS_LONGITUDE] * (DeltaTime * 0.48f);
  INS.EarthFrame.Position[INS_LATITUDE] += PositionError[INS_LATITUDE] * (DeltaTime * 1.2f);
  INS.EarthFrame.Position[INS_LONGITUDE] += PositionError[INS_LONGITUDE] * (DeltaTime * 1.2f);

#endif
}

void InertialNavigationClass::EstimationPredictXY(float DeltaTime)
{
#ifndef TEST_NEW_INS

  float VelocityIncrease[2] = {0, 0};
  VelocityIncrease[INS_LATITUDE] = INS.AccelerationEarthFrame_Filtered[INS_LATITUDE] * DeltaTime;
  VelocityIncrease[INS_LONGITUDE] = INS.AccelerationEarthFrame_Filtered[INS_LONGITUDE] * DeltaTime;
  INS.EarthFrame.Position[INS_LATITUDE] += (INS.EarthFrame.Velocity[INS_LATITUDE] + VelocityIncrease[INS_LATITUDE] * 0.5f) * DeltaTime;    //POSIÇÃO FINAL X ESTIMADA PELO INS
  INS.EarthFrame.Position[INS_LONGITUDE] += (INS.EarthFrame.Velocity[INS_LONGITUDE] + VelocityIncrease[INS_LONGITUDE] * 0.5f) * DeltaTime; //POSIÇÃO FINAL Y ESTIMADA PELO INS
  INS.EarthFrame.Velocity[INS_LATITUDE] += VelocityIncrease[INS_LATITUDE];                                                                 //VELOCIDADE FINAL X ESTIMADA PELO INS
  INS.EarthFrame.Velocity[INS_LONGITUDE] += VelocityIncrease[INS_LONGITUDE];                                                               //VELOCIDADE FINAL Y ESTIMADA PELO INS

#ifdef PRINTLN_INS_POS_VEL_XY

  DEBUG("%.f %.f %.f %.f",
        INS.EarthFrame.Position[INS_LATITUDE],
        INS.EarthFrame.Position[INS_LONGITUDE],
        INS.EarthFrame.Velocity[INS_LATITUDE],
        INS.EarthFrame.Velocity[INS_LONGITUDE]);

#endif

#endif
}

void InertialNavigationClass::SaveXYPositionToHistory(void)
{
#ifndef TEST_NEW_INS

  INS.History.XYPosition[INS_LATITUDE][INS.History.XYCount] = INS.EarthFrame.Position[INS_LATITUDE];
  INS.History.XYPosition[INS_LONGITUDE][INS.History.XYCount] = INS.EarthFrame.Position[INS_LONGITUDE];
  INS.History.XYCount++;
  if (INS.History.XYCount >= 10)
  {
    INS.History.XYCount = 0;
  }

#endif
}

void InertialNavigationClass::ResetXYState(void)
{
#ifndef TEST_NEW_INS

  INS.History.XYCount = 0;
  for (uint8_t IndexCount = 0; IndexCount < 2; IndexCount++)
  {
    INS.EarthFrame.Velocity[IndexCount] = (ABS(GPS_Resources.Navigation.Speed[IndexCount]) > 50) ? GPS_Resources.Navigation.Speed[IndexCount] : 0.0f;
    INS.EarthFrame.Position[IndexCount] = GPS_Resources.Home.INS.Distance[IndexCount];
    for (uint8_t SecondIndexCount = 0; SecondIndexCount < 10; SecondIndexCount++)
    {
      INS.History.XYPosition[IndexCount][SecondIndexCount] = GPS_Resources.Home.INS.Distance[IndexCount];
    }
  }

#endif
}

void InertialNavigationClass::Calculate_AccelerationZ(float DeltaTime)
{
#ifndef TEST_NEW_INS

  INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_VERTICAL_Z);
  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INERTIALNAVIGATION.CorrectZStateWithBaro(DeltaTime);
    INERTIALNAVIGATION.EstimationPredictZ(DeltaTime);
    INERTIALNAVIGATION.SaveZPositionToHistory();
  }
  else
  {
    INERTIALNAVIGATION.ResetZState();
  }
#endif
}

void InertialNavigationClass::CorrectZStateWithBaro(float DeltaTime)
{
#ifndef TEST_NEW_INS

  bool AirCushionEffectDetected = (GetTakeOffInProgress() || GetGroundDetected()) && (Barometer.Altitude.Actual < Barometer.Altitude.GroundOffSet);
  float BaroAltitudeResidual = (AirCushionEffectDetected ? Barometer.Altitude.GroundOffSet : Barometer.Altitude.Actual) - INS.History.ZPosition[INS.History.ZCount];
  INS.EarthFrame.Position[INS_VERTICAL_Z] += BaroAltitudeResidual * (0.66666666666666666666666666666667f * DeltaTime);
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] += BaroAltitudeResidual * (0.19753086419753086419753086419753f * DeltaTime);

#endif
}

void InertialNavigationClass::EstimationPredictZ(float DeltaTime)
{
#ifndef TEST_NEW_INS

  float VelocityIncrease = INS.AccelerationEarthFrame_Filtered[INS_VERTICAL_Z] * DeltaTime;
  INS.EarthFrame.Position[INS_VERTICAL_Z] += (INS.EarthFrame.Velocity[INS_VERTICAL_Z] + VelocityIncrease * 0.5f) * DeltaTime;
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] += VelocityIncrease;
  Barometer.INS.Altitude.Estimated = INS.EarthFrame.Position[INS_VERTICAL_Z]; //ALTITUDE FINAL ESTIMADA PELO INS
  Barometer.INS.Velocity.Vertical = INS.EarthFrame.Velocity[INS_VERTICAL_Z];  //VELOCIDADE VERTICAL(Z) FINAL ESTIMADA PELO INS

#ifdef PRINTLN_INS_POS_VEL_Z

  //Barometer.INS.Altitude.Estimated -> POSITIVO SUBINDO E NEGATIVO DESCENDO
  //Barometer.INS.Velocity.Vertical  -> POSITIVO SUBINDO E NEGATIVO DESCENDO

  DEBUG("%ld %d",
        Barometer.INS.Altitude.Estimated,
        Barometer.INS.Velocity.Vertical);

#endif

#endif
}

void InertialNavigationClass::SaveZPositionToHistory(void)
{
#ifndef TEST_NEW_INS

  INS.History.ZPosition[INS.History.ZCount] = Barometer.INS.Altitude.Estimated;
  INS.History.ZCount++;
  if (INS.History.ZCount >= 10)
  {
    INS.History.ZCount = 0;
  }

#endif
}

void InertialNavigationClass::ResetZState(void)
{
#ifndef TEST_NEW_INS

  INS.EarthFrame.Position[INS_VERTICAL_Z] = 0.0f;
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] = 0.0f;
  INS.History.ZCount = 0;
  for (uint8_t IndexCount = 0; IndexCount < 10; IndexCount++)
  {
    INS.History.ZPosition[IndexCount] = 0;
  }

#endif
}

#ifdef TEST_NEW_INS

#include "Scheduler/SCHEDULERTIME.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "I2C/I2C.h"
#include "GPSNavigation/NAVIGATIONGEO.h"
#include "Scheduler/SCHEDULER.h"

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

enum PositionEstimatorFlags_Enum
{
  EST_GPS_XY_VALID = (1 << 0),
  EST_GPS_Z_VALID = (1 << 1),
  EST_BARO_VALID = (1 << 2),
  EST_XY_VALID = (1 << 3),
  EST_Z_VALID = (1 << 4)
};

typedef struct
{
  uint32_t LastUpdateTime;
  Vector3x3_Struct Position;
  Vector3x3_Struct Velocity;
  float EstimatedPositionHorizontal;
  float EstimatedPositionVertical;
  float AltitudeOffSet;
} PositionEstimatorGPS_Struct;

typedef struct
{
  uint32_t LastUpdateTime;
  float ActualAltitude;
  float EstimatedPositionVertical;
} PositionEstimatorBarometer_Struct;

typedef struct
{
  uint32_t LastUpdateTime;
  Vector3x3_Struct Position;
  Vector3x3_Struct Velocity;
  float EstimatedPositionHorizontal;
  float EstimatedPositionVertical;
} PositionEstimatorEstimate_Struct;

typedef struct
{
  uint32_t LastUpdateTime;
  Vector3x3_Struct AccelerationNEU;
  Vector3x3_Struct AccelerationBias;
  float AccWeightFactor;
} PositionEstimatorIMU_Struct;

typedef struct
{
  bool BaroGroundValid;
  float BaroGroundAlt;
  uint32_t BaroGroundTimeout;
} PositionEstimatorState_Struct;

typedef struct
{
  uint32_t Flags;
  PositionEstimatorIMU_Struct IMU;
  PositionEstimatorBarometer_Struct Barometer;
  PositionEstimatorGPS_Struct GPS;
  PositionEstimatorEstimate_Struct Estimate;
  PositionEstimatorState_Struct State;
} PositionEstimator_Struct;

typedef struct
{
  float DeltaTime;
  float NewEstimatedPositionVertical;
  float NewEstimatedPositionHorizontal;
  uint32_t NewFlags;
  Vector3x3_Struct EstimatedPosistionCorrected;
  Vector3x3_Struct EstimatedVelocityCorrected;
  Vector3x3_Struct AccBiasCorrected;
} EstimationContext_Struct;

PositionEstimator_Struct PositionEstimator;

static void UpdateNewGPSData(void)
{
  const uint32_t ActualTimeInUs = SCHEDULERTIME.GetMicros();

  if (Get_GPS_In_Bad_Condition())
  {
    PositionEstimator.GPS.EstimatedPositionHorizontal = GPS_DEFAULT_EPH;
    PositionEstimator.GPS.EstimatedPositionVertical = GPS_DEFAULT_EPV;
    return;
  }

  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    PositionEstimator.GPS.AltitudeOffSet = GPS_Resources.Navigation.Misc.Get.Altitude;
  }

  PositionEstimator.GPS.Position.Roll = GPS_Resources.Home.INS.Distance[COORD_LATITUDE];
  PositionEstimator.GPS.Position.Pitch = GPS_Resources.Home.INS.Distance[COORD_LONGITUDE];
  PositionEstimator.GPS.Position.Yaw = GPS_Resources.Navigation.Misc.Get.Altitude - PositionEstimator.GPS.AltitudeOffSet; //ALTITUDE DO GPS PARA AERONAVES DE ASA-FIXA APENAS

  PositionEstimator.GPS.Velocity.Roll = GPS_Resources.Navigation.Misc.Velocity.Get[NORTH];
  PositionEstimator.GPS.Velocity.Pitch = GPS_Resources.Navigation.Misc.Velocity.Get[EAST];
  PositionEstimator.GPS.Velocity.Yaw = -GPS_Resources.Navigation.Misc.Velocity.Get[DOWN];

  PositionEstimator.GPS.EstimatedPositionHorizontal = GPS_Resources.Navigation.Misc.Get.EstimatedPositionHorizontal;
  PositionEstimator.GPS.EstimatedPositionVertical = GPS_Resources.Navigation.Misc.Get.EstimatedPositionVertical;

  PositionEstimator.GPS.LastUpdateTime = ActualTimeInUs;
}

static void UpdatePositionEstimatorBaroTopic(uint32_t ActualTimeInUs)
{
  if (I2CResources.Found.Barometer)
  {
    PositionEstimator.Barometer.ActualAltitude = Barometer.Altitude.Actual;
    PositionEstimator.Barometer.EstimatedPositionVertical = Max_INS_Baro_Error;
    PositionEstimator.Barometer.LastUpdateTime = ActualTimeInUs;
  }
  else
  {
    PositionEstimator.Barometer.ActualAltitude = 0;
    PositionEstimator.Barometer.LastUpdateTime = 0;
  }
}

static void UpdateIMUEstimationWeight(const float DeltaTime)
{
  const bool AccClipped = ABS(IMU.Accelerometer.ReadFloat[ROLL]) > ACC_CLIPPING_THRESHOLD_G ||
                          ABS(IMU.Accelerometer.ReadFloat[PITCH]) > ACC_CLIPPING_THRESHOLD_G ||
                          ABS(IMU.Accelerometer.ReadFloat[YAW]) > ACC_CLIPPING_THRESHOLD_G;

  //SE O ACELEROMETRO FOR CLIPADO,O WEIGHT SERÁ REDUZIDO A ZERO.CASO CONTRARIO SERÁ RESTAURADO PARA 1 GRADUALMENTE.
  if (AccClipped)
  {
    PositionEstimator.IMU.AccWeightFactor = 0.0f;
  }
  else
  {
    const float DTAlpha = DeltaTime / (DeltaTime + ACC_CLIPPING_RC_CONSTANT);
    PositionEstimator.IMU.AccWeightFactor = PositionEstimator.IMU.AccWeightFactor * (1.0f - DTAlpha) + 1.0f * DTAlpha;
  }
}

static void UpdateIMUTopic(uint32_t ActualTimeInUs)
{
  const float DeltaTime = (ActualTimeInUs - PositionEstimator.IMU.LastUpdateTime) * 1e-6f;
  PositionEstimator.IMU.LastUpdateTime = ActualTimeInUs;

  //ATUALIZA O WEIGHT BASEADO NAS VIBRAÇÕES DO ACC
  UpdateIMUEstimationWeight(DeltaTime);

  //RESETA O VETOR DA ACELERAÇÃO DO BODY-FRAME
  VectorZero(&INS.NewAccelerationEarthFrame);

  //PASSA A ACELERAÇÃO DO EARTH-FRAME PARA UM NOVO VETOR
  INS.NewAccelerationEarthFrame.Roll = BodyFrameAcceleration.Roll;
  INS.NewAccelerationEarthFrame.Pitch = BodyFrameAcceleration.Pitch;
  INS.NewAccelerationEarthFrame.Yaw = BodyFrameAcceleration.Yaw;

  //TRANSFORMA O VETOR DA ACELERAÇÃO PARA EARTH-FRAME
  AHRS.TransformVectorBodyFrameToEarthFrame(&INS.NewAccelerationEarthFrame);

  //CALIBRA O 1G DA ACELERAÇÃO
  GRAVITYCALIBRATION.Update(&INS.NewAccelerationEarthFrame);

  //CORRIGE A BIAS DO ACELEROMETRO E PASSA A ACELERAÇÃO EM NEU PARA NOVAS VARIAVEIS PARA SEREM APLICADAS NO INS
  PositionEstimator.IMU.AccelerationNEU.Roll = INS.NewAccelerationEarthFrame.Roll - PositionEstimator.IMU.AccelerationBias.Roll;
  PositionEstimator.IMU.AccelerationNEU.Pitch = INS.NewAccelerationEarthFrame.Pitch - PositionEstimator.IMU.AccelerationBias.Pitch;
  PositionEstimator.IMU.AccelerationNEU.Yaw = INS.NewAccelerationEarthFrame.Yaw - PositionEstimator.IMU.AccelerationBias.Yaw;
}

static float UpdateEstimatedPosition(const float OldEPE, const float DeltaTime, const float NewEPE, const float Weight)
{
  return OldEPE + (NewEPE - OldEPE) * Weight * DeltaTime;
}

static uint32_t CalculateCurrentValidityFlags(uint32_t ActualTimeInUs)
{
  uint32_t NewFlags = 0;

  if (((ActualTimeInUs - PositionEstimator.GPS.LastUpdateTime) <= (GPS_TIMEOUT_MS * 1000L)) && (PositionEstimator.GPS.EstimatedPositionHorizontal < Max_INS_H_V_Error))
  {
    if (PositionEstimator.GPS.EstimatedPositionVertical < Max_INS_H_V_Error)
    {
      NewFlags |= EST_GPS_XY_VALID | EST_GPS_Z_VALID;
    }
    else
    {
      NewFlags |= EST_GPS_XY_VALID;
    }
  }

  if (I2CResources.Found.Barometer && ((ActualTimeInUs - PositionEstimator.Barometer.LastUpdateTime) <= (BARO_TIMEOUT_MS * 1000L)))
  {
    NewFlags |= EST_BARO_VALID;
  }

  if (PositionEstimator.Estimate.EstimatedPositionHorizontal < Max_INS_H_V_Error)
  {
    NewFlags |= EST_XY_VALID;
  }

  if (PositionEstimator.Estimate.EstimatedPositionVertical < Max_INS_H_V_Error)
  {
    NewFlags |= EST_Z_VALID;
  }

  return NewFlags;
}

static void EstimationPredict(EstimationContext_Struct *Context)
{
  const float AccWeight = PositionEstimator.IMU.AccWeightFactor * Weight_XYZ_Acc_Position;

  //PREDIÇÃO Z
  if ((Context->NewFlags & EST_Z_VALID))
  {
    PositionEstimator.Estimate.Position.Yaw += PositionEstimator.Estimate.Velocity.Yaw * Context->DeltaTime;
    PositionEstimator.Estimate.Position.Yaw += PositionEstimator.IMU.AccelerationNEU.Yaw * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
    PositionEstimator.Estimate.Velocity.Yaw += PositionEstimator.IMU.AccelerationNEU.Yaw * Context->DeltaTime * SquareFloat(AccWeight);
  }

  //PREDIÇÃO X E Y
  if ((Context->NewFlags & EST_XY_VALID))
  {
    //PREDIÇÃO DA POSIÇÃO BASEADO NA VELOCIDADE
    PositionEstimator.Estimate.Position.Roll += PositionEstimator.Estimate.Velocity.Roll * Context->DeltaTime;
    PositionEstimator.Estimate.Position.Pitch += PositionEstimator.Estimate.Velocity.Pitch * Context->DeltaTime;

    //VERIFICA SE ESTÁ OK PARA A INTEGRAÇÃO NEU
    if (I2CResources.Found.Compass || (GetAirPlaneEnabled() && GPS_Resources.Navigation.Misc.Get.HeadingInitialized))
    {
      PositionEstimator.Estimate.Position.Roll += PositionEstimator.IMU.AccelerationNEU.Roll * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
      PositionEstimator.Estimate.Position.Pitch += PositionEstimator.IMU.AccelerationNEU.Pitch * SquareFloat(Context->DeltaTime) / 2.0f * AccWeight;
      PositionEstimator.Estimate.Velocity.Roll += PositionEstimator.IMU.AccelerationNEU.Roll * Context->DeltaTime * SquareFloat(AccWeight);
      PositionEstimator.Estimate.Velocity.Pitch += PositionEstimator.IMU.AccelerationNEU.Pitch * Context->DeltaTime * SquareFloat(AccWeight);
    }
  }
}

static bool EstimationCalculateCorrection_Z(EstimationContext_Struct *Context)
{
  if (Context->NewFlags & EST_BARO_VALID)
  {
    uint32_t ActualTimeInUs = SCHEDULERTIME.GetMicros();

    if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
      PositionEstimator.State.BaroGroundAlt = PositionEstimator.Estimate.Position.Yaw;
      PositionEstimator.State.BaroGroundValid = true;
      PositionEstimator.State.BaroGroundTimeout = ActualTimeInUs + 250000;
    }
    else
    {
      if (PositionEstimator.Estimate.Velocity.Yaw > 15)
      {
        if (ActualTimeInUs > PositionEstimator.State.BaroGroundTimeout)
        {
          PositionEstimator.State.BaroGroundValid = false;
        }
      }
      else
      {
        PositionEstimator.State.BaroGroundTimeout = ActualTimeInUs + 250000;
      }
    }

    //VERIFICA SE O EFEITO ALMOFADA EXISTE
    bool AirCushionEffectDetected = IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && ((Context->NewFlags & EST_BARO_VALID) && PositionEstimator.State.BaroGroundValid && PositionEstimator.Barometer.ActualAltitude < PositionEstimator.State.BaroGroundAlt);

    //CALCULA A ALTITUDE COM BASE NO BARO
    const float BaroAltResidual = (AirCushionEffectDetected ? PositionEstimator.State.BaroGroundAlt : PositionEstimator.Barometer.ActualAltitude) - PositionEstimator.Estimate.Position.Yaw;
    Context->EstimatedPosistionCorrected.Yaw += BaroAltResidual * Weight_Z_Baro_Position * Context->DeltaTime;
    Context->EstimatedVelocityCorrected.Yaw += BaroAltResidual * SquareFloat(Weight_Z_Baro_Position) * Context->DeltaTime;

    if (Context->NewFlags & EST_GPS_Z_VALID)
    {
      const float GPSRocResidual = PositionEstimator.GPS.Velocity.Yaw - PositionEstimator.Estimate.Velocity.Yaw;
      const float GPSRocScaler = Sine_Curve(GPSRocResidual, 250.0f);
      Context->EstimatedVelocityCorrected.Yaw += GPSRocResidual * Weight_Z_GPS_Velocity * GPSRocScaler * Context->DeltaTime;
    }

    Context->NewEstimatedPositionVertical = UpdateEstimatedPosition(PositionEstimator.Estimate.EstimatedPositionVertical, Context->DeltaTime, PositionEstimator.Barometer.EstimatedPositionVertical, Weight_Z_Baro_Position);

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
      Context->EstimatedPosistionCorrected.Yaw += PositionEstimator.GPS.Position.Yaw - PositionEstimator.Estimate.Position.Yaw;
      Context->EstimatedVelocityCorrected.Yaw += PositionEstimator.GPS.Velocity.Yaw - PositionEstimator.Estimate.Velocity.Yaw;
      Context->NewEstimatedPositionVertical = PositionEstimator.GPS.EstimatedPositionVertical;
    }
    else
    {
      //CALCULA A ALTITUDE BASEADO NO GPS
      const float GPSAltResudual = PositionEstimator.GPS.Position.Yaw - PositionEstimator.Estimate.Position.Yaw;

      Context->EstimatedPosistionCorrected.Yaw += GPSAltResudual * Weight_Z_GPS_Position * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Yaw += GPSAltResudual * SquareFloat(Weight_Z_GPS_Position) * Context->DeltaTime;
      Context->EstimatedVelocityCorrected.Yaw += (PositionEstimator.GPS.Velocity.Yaw - PositionEstimator.Estimate.Velocity.Yaw) * Weight_Z_GPS_Velocity * Context->DeltaTime;
      Context->NewEstimatedPositionVertical = UpdateEstimatedPosition(PositionEstimator.Estimate.EstimatedPositionVertical, Context->DeltaTime, MAX(PositionEstimator.GPS.EstimatedPositionVertical, GPSAltResudual), Weight_Z_GPS_Position);

      //AJUSTA A BIAS DO Z
      Context->AccBiasCorrected.Yaw -= GPSAltResudual * SquareFloat(Weight_Z_GPS_Position);
    }

    return true;
  }

  return false;
}

static bool EstimationCalculateCorrection_XY_GPS(EstimationContext_Struct *Context)
{
  if (Context->NewFlags & EST_GPS_XY_VALID)
  {
    //RESETA AS COORDENAS E VELOCIDADES
    if (!(Context->NewFlags & EST_XY_VALID))
    {
      Context->EstimatedPosistionCorrected.Roll += PositionEstimator.GPS.Position.Roll - PositionEstimator.Estimate.Position.Roll;
      Context->EstimatedPosistionCorrected.Pitch += PositionEstimator.GPS.Position.Pitch - PositionEstimator.Estimate.Position.Pitch;
      Context->EstimatedVelocityCorrected.Roll += PositionEstimator.GPS.Velocity.Roll - PositionEstimator.Estimate.Velocity.Roll;
      Context->EstimatedVelocityCorrected.Pitch += PositionEstimator.GPS.Velocity.Pitch - PositionEstimator.Estimate.Velocity.Pitch;
      Context->NewEstimatedPositionHorizontal = PositionEstimator.GPS.EstimatedPositionHorizontal;
    }
    else
    {
      const float GPSPositionXResidual = PositionEstimator.GPS.Position.Roll - PositionEstimator.Estimate.Position.Roll;
      const float GPSPositionYResidual = PositionEstimator.GPS.Position.Pitch - PositionEstimator.Estimate.Position.Pitch;
      const float GPSVelocityXResidual = PositionEstimator.GPS.Velocity.Roll - PositionEstimator.Estimate.Velocity.Roll;
      const float GPSVelocityYResidual = PositionEstimator.GPS.Velocity.Pitch - PositionEstimator.Estimate.Velocity.Pitch;
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
      Context->NewEstimatedPositionHorizontal = UpdateEstimatedPosition(PositionEstimator.Estimate.EstimatedPositionHorizontal, Context->DeltaTime, MAX(PositionEstimator.GPS.EstimatedPositionHorizontal, GPSPositionResidualMag), New_Weight_XY_GPS_Position);
    }

    return true;
  }

  return false;
}

static void UpdateEstimatedTopic(uint32_t ActualTimeInUs)
{
  EstimationContext_Struct Context;

  Context.DeltaTime = (ActualTimeInUs - PositionEstimator.Estimate.LastUpdateTime) * 1e-6f;
  PositionEstimator.Estimate.LastUpdateTime = ActualTimeInUs;

  //CALCULA UMA NOVA ESTIMATIVA DE POSIÇÃO HORIZONTAL E VERTICAL
  Context.NewEstimatedPositionHorizontal = PositionEstimator.Estimate.EstimatedPositionHorizontal * ((PositionEstimator.Estimate.EstimatedPositionHorizontal <= Max_INS_H_V_Error) ? 1.0f + Context.DeltaTime : 1.0f);
  Context.NewEstimatedPositionVertical = PositionEstimator.Estimate.EstimatedPositionVertical * ((PositionEstimator.Estimate.EstimatedPositionVertical <= Max_INS_H_V_Error) ? 1.0f + Context.DeltaTime : 1.0f);

  //OBTÉM AS FLAGS DE ESTADO DO INS
  Context.NewFlags = CalculateCurrentValidityFlags(ActualTimeInUs);

  //RESETA OS VETORES
  VectorZero(&Context.EstimatedPosistionCorrected);
  VectorZero(&Context.EstimatedVelocityCorrected);
  VectorZero(&Context.AccBiasCorrected);

  //PREDIÇÃO X,Y E Z
  EstimationPredict(&Context);

  //CORREÇÃO Z
  const bool EstimatorZCorrectState = EstimationCalculateCorrection_Z(&Context);

  //CORREÇÃO X E Y
  const bool EstimatorXYCorrectState = EstimationCalculateCorrection_XY_GPS(&Context);

  if (!EstimatorXYCorrectState || Context.NewEstimatedPositionHorizontal > Max_INS_H_V_Error)
  {
    Context.EstimatedVelocityCorrected.Roll = (0.0f - PositionEstimator.Estimate.Velocity.Roll) * Weight_XY_Coeff_Velocity * Context.DeltaTime;
    Context.EstimatedVelocityCorrected.Pitch = (0.0f - PositionEstimator.Estimate.Velocity.Pitch) * Weight_XY_Coeff_Velocity * Context.DeltaTime;
  }

  if (!EstimatorZCorrectState || Context.NewEstimatedPositionVertical > Max_INS_H_V_Error)
  {
    Context.EstimatedVelocityCorrected.Yaw = (0.0f - PositionEstimator.Estimate.Velocity.Yaw) * Weight_Z_Coeff_Velocity * Context.DeltaTime;
  }

  //APLICA AS CORREÇÕES
  VectorAdd(&PositionEstimator.Estimate.Position, &PositionEstimator.Estimate.Position, &Context.EstimatedPosistionCorrected);
  VectorAdd(&PositionEstimator.Estimate.Velocity, &PositionEstimator.Estimate.Velocity, &Context.EstimatedVelocityCorrected);

  if (Weight_Acc_Bias > 0.0f)
  {
    const float accelBiasCorrMagnitudeSq = SquareFloat(Context.AccBiasCorrected.Roll) + SquareFloat(Context.AccBiasCorrected.Pitch) + SquareFloat(Context.AccBiasCorrected.Yaw);
    if (accelBiasCorrMagnitudeSq < SquareFloat(ACC_BIAS_ACCEPTANCE_VALUE))
    {
      //TRANFORMA DE EARTH PARA BODY
      AHRS.TransformVectorEarthFrameToBodyFrame(&Context.AccBiasCorrected);

      //AJUSTA A BIAS DA ACELERAÇÃO
      PositionEstimator.IMU.AccelerationBias.Roll += Context.AccBiasCorrected.Roll * Weight_Acc_Bias * Context.DeltaTime;
      PositionEstimator.IMU.AccelerationBias.Pitch += Context.AccBiasCorrected.Pitch * Weight_Acc_Bias * Context.DeltaTime;
      PositionEstimator.IMU.AccelerationBias.Yaw += Context.AccBiasCorrected.Yaw * Weight_Acc_Bias * Context.DeltaTime;
    }
  }

  PositionEstimator.Estimate.EstimatedPositionHorizontal = Context.NewEstimatedPositionHorizontal;
  PositionEstimator.Estimate.EstimatedPositionVertical = Context.NewEstimatedPositionVertical;

  PositionEstimator.Flags = Context.NewFlags;
}

static void initializePositionEstimator(void)
{
  int axis;

  PositionEstimator.Estimate.EstimatedPositionHorizontal = Max_INS_H_V_Error + 0.001f;
  PositionEstimator.Estimate.EstimatedPositionVertical = Max_INS_H_V_Error + 0.001f;

  PositionEstimator.IMU.LastUpdateTime = 0;
  PositionEstimator.GPS.LastUpdateTime = 0;
  PositionEstimator.Barometer.LastUpdateTime = 0;
  PositionEstimator.IMU.AccWeightFactor = 0.0f;

  for (axis = 0; axis < 3; axis++)
  {
    PositionEstimator.IMU.AccelerationBias.Vector[axis] = 0.0f;
    PositionEstimator.Estimate.Position.Vector[axis] = 0.0f;
    PositionEstimator.Estimate.Velocity.Vector[axis] = 0.0f;
  }
}

void InertialNavigationClass::Calculate_AccelerationXYZ_To_EarthFrame(void)
{
  static bool isInitialized = false;

  if (!isInitialized)
  {
    initializePositionEstimator();
    isInitialized = true;
  }

  const uint32_t ActualTimeInUs = SCHEDULERTIME.GetMicros();
  static uint32_t LastTriggeredTime;

  UpdateIMUTopic(ActualTimeInUs);

  UpdatePositionEstimatorBaroTopic(ActualTimeInUs);

  UpdateEstimatedTopic(ActualTimeInUs);

  //OBTÉM O SENO E COSSENO DO YAW DADO PELO AHRS
  INS.Math.Cosine.Yaw = AHRS.GetCosineYaw();
  INS.Math.Sine.Yaw = AHRS.GetSineYaw();

  if ((ActualTimeInUs - LastTriggeredTime) >= SCHEDULER_SET_FREQUENCY(POSITION_RATE, "Hz"))
  {
    LastTriggeredTime = ActualTimeInUs;

    /*
    DEBUG("%.f %.f %.f %.f %ld %.f %.f",
          PositionEstimator.Estimate.Position.Roll,  //X POS
          PositionEstimator.Estimate.Position.Pitch, //Y POS
          PositionEstimator.Estimate.Velocity.Roll,  //X VEL
          PositionEstimator.Estimate.Velocity.Pitch, //Y VEL
          Barometer.Altitude.Actual,                 //BARO ALT
          PositionEstimator.Estimate.Position.Yaw,   //Z POS
          PositionEstimator.Estimate.Velocity.Yaw);  //Z VEL
*/

    INS.EarthFrame.Position[INS_LATITUDE] = PositionEstimator.Estimate.Position.Roll;   //POSIÇÃO FINAL X ESTIMADA PELO INS
    INS.EarthFrame.Position[INS_LONGITUDE] = PositionEstimator.Estimate.Position.Pitch; //POSIÇÃO FINAL Y ESTIMADA PELO INS
    INS.EarthFrame.Velocity[INS_LATITUDE] = PositionEstimator.Estimate.Velocity.Roll;   //VELOCIDADE FINAL X ESTIMADA PELO INS
    INS.EarthFrame.Velocity[INS_LONGITUDE] = PositionEstimator.Estimate.Velocity.Pitch; //VELOCIDADE FINAL Y ESTIMADA PELO INS
    Barometer.INS.Altitude.Estimated = PositionEstimator.Estimate.Position.Yaw;         //ALTITUDE FINAL ESTIMADA PELO INS
    Barometer.INS.Velocity.Vertical = PositionEstimator.Estimate.Velocity.Yaw;          //VELOCIDADE VERTICAL(Z) FINAL ESTIMADA PELO INS

#ifdef PRINTLN_INS_COS_SIN

    DEBUG("%d %.2f %.2f",
          Attitude.EulerAngles.YawDecidegrees,
          INS.Math.Cosine.Yaw,
          INS.Math.Sine.Yaw);

#endif

#ifdef PRINTLN_INS_ACC_NEU

    //PositionEstimator.IMU.AccelerationNEU.Roll  -> POSITIVO MOVENDO PARA O NORTE
    //PositionEstimator.IMU.AccelerationNEU.Pitch -> POSITIVO MOVENDO PARA O OESTE
    //PositionEstimator.IMU.AccelerationNEU.Yaw   -> POSITIVO MOVENDO PARA CIMA

    DEBUG("NORTH:%.4f EAST:%.4f UP:%.4f",
          PositionEstimator.IMU.AccelerationNEU.Roll,
          PositionEstimator.IMU.AccelerationNEU.Pitch,
          PositionEstimator.IMU.AccelerationNEU.Yaw);

#endif

#ifdef PRINTLN_INS_POS_VEL_XY

    DEBUG("%.f %.f %.f %.f",
          INS.EarthFrame.Position[INS_LATITUDE],
          INS.EarthFrame.Position[INS_LONGITUDE],
          INS.EarthFrame.Velocity[INS_LATITUDE],
          INS.EarthFrame.Velocity[INS_LONGITUDE]);

#endif

#ifdef PRINTLN_INS_POS_VEL_Z

    //Barometer.INS.Altitude.Estimated -> POSITIVO SUBINDO E NEGATIVO DESCENDO
    //Barometer.INS.Velocity.Vertical  -> POSITIVO SUBINDO E NEGATIVO DESCENDO

    DEBUG("%ld %d",
          Barometer.INS.Altitude.Estimated,
          Barometer.INS.Velocity.Vertical);

#endif
  }

  UpdateNewGPSData();
}

#endif