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

#include "AHRS.h"
#include "Common/STRUCTS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "I2C/I2C.h"
#include "BAR/BAR.h"
#include "Yaw/HEADINGHOLD.h"
#include "QUATERNION.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "GPS/GPSSTATES.h"
#include "GPS/GPSUBLOX.h"
#include "IMU/ACCGYROREAD.h"
#include "GPSNavigation/NAVIGATION.h"
#include "PID/PIDPARAMS.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

AHRSClass AHRS;

#ifdef __AVR_ATmega2560__

#define NEARNESS 100.0f //FATOR DE GANHO DE CORREÇÃO DO ACELEROMETRO NO AHRS

#else

#define NEARNESS 1.0f //FATOR DE GANHO DE CORREÇÃO DO ACELEROMETRO NO AHRS

#endif

#define SPIN_RATE_LIMIT 20     //VALOR DE GYRO^2 PARA CORTAR A CORREÇÃO DO INTEGRAL NO AHRS
#define MAX_ACC_NEARNESS 0.33f //33% (0.67G - 1.33G)

Attitude_Struct Attitude;
Vector3x3_Struct BodyFrameAcceleration;
Vector3x3_Struct BodyFrameRotation;
Quaternion_Struct Orientation;
Matrix3x3_Struct Rotation;

static Vector3x3_Struct CorrectedMagneticFieldNorth;
static AHRS_Configuration_Struct AHRSConfiguration;

static void ComputeRotationMatrix(void)
{
  float q1q1 = Orientation.q1 * Orientation.q1;
  float q2q2 = Orientation.q2 * Orientation.q2;
  float q3q3 = Orientation.q3 * Orientation.q3;
  float q0q1 = Orientation.q0 * Orientation.q1;
  float q0q2 = Orientation.q0 * Orientation.q2;
  float q0q3 = Orientation.q0 * Orientation.q3;
  float q1q2 = Orientation.q1 * Orientation.q2;
  float q1q3 = Orientation.q1 * Orientation.q3;
  float q2q3 = Orientation.q2 * Orientation.q3;
  Rotation.Matrix3x3[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
  Rotation.Matrix3x3[0][1] = 2.0f * (q1q2 + -q0q3);
  Rotation.Matrix3x3[0][2] = 2.0f * (q1q3 - -q0q2);
  Rotation.Matrix3x3[1][0] = 2.0f * (q1q2 - -q0q3);
  Rotation.Matrix3x3[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
  Rotation.Matrix3x3[1][2] = 2.0f * (q2q3 + -q0q1);
  Rotation.Matrix3x3[2][0] = 2.0f * (q1q3 + -q0q2);
  Rotation.Matrix3x3[2][1] = 2.0f * (q2q3 - -q0q1);
  Rotation.Matrix3x3[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void AHRSClass::Initialization(void)
{
#ifndef __AVR_ATmega2560__

  AHRSConfiguration.kP_Accelerometer = JCF_Param.kP_Acc_AHRS / 10000.0f;
  AHRSConfiguration.kI_Accelerometer = JCF_Param.kI_Acc_AHRS / 10000.0f;
  AHRSConfiguration.kP_Magnetometer = JCF_Param.kP_Mag_AHRS / 10000.0f;
  AHRSConfiguration.kI_Magnetometer = JCF_Param.kI_Mag_AHRS > 0 ? JCF_Param.kI_Mag_AHRS / 10000.0f : 0.0f;
  AHRSConfiguration.Cosine_Z = JCF_Param.AngleLevelBlockArm;

#endif

  //CALCULA A DECLINAÇÃO MAGNETICA
  const int16_t Degrees = ((int16_t)(STORAGEMANAGER.Read_Float(MAG_DECLINATION_ADDR) * 100)) / 100;
  const int16_t Remainder = ((int16_t)(STORAGEMANAGER.Read_Float(MAG_DECLINATION_ADDR) * 100)) % 100;
  const float CalcedValueInRadians = -ConvertToRadians(Degrees + Remainder / 60.0f);
  CorrectedMagneticFieldNorth.Roll = Fast_Cosine(CalcedValueInRadians);
  CorrectedMagneticFieldNorth.Pitch = Fast_Sine(CalcedValueInRadians);
  CorrectedMagneticFieldNorth.Yaw = 0;

  //RESETA O QUATERNION E A MATRIX
  QuaternionInit(&Orientation);
  ComputeRotationMatrix();
}

static bool ValidateQuaternion(const Quaternion_Struct *Quaternion)
{
  const float CheckAbsoluteValue = ABS(Quaternion->q0) +
                                   ABS(Quaternion->q1) +
                                   ABS(Quaternion->q2) +
                                   ABS(Quaternion->q3);

  if (!isnan(CheckAbsoluteValue) && !isinf(CheckAbsoluteValue))
  {
    return true;
  }

  const float QuatSquared = QuaternionNormalizedSquared(&Orientation);

  if (QuatSquared > (1.0f - 1e-6f) && QuatSquared < (1.0f + 1e-6f))
  {
    return true;
  }

  return false;
}

static void ResetOrientationQuaternion(const Vector3x3_Struct *AccelerationBodyFrame)
{
  const float AccVectorSquared = Fast_SquareRoot(VectorNormSquared(AccelerationBodyFrame));
  Orientation.q0 = AccelerationBodyFrame->Yaw + AccVectorSquared;
  Orientation.q1 = AccelerationBodyFrame->Pitch;
  Orientation.q2 = -AccelerationBodyFrame->Roll;
  Orientation.q3 = 0.0f;
  QuaternionNormalize(&Orientation, &Orientation);
}

static void CheckAndResetOrientationQuaternion(const Quaternion_Struct *Quaternion, const Vector3x3_Struct *AccelerationBodyFrame)
{
  //CHECA SE O QUATERNION ESTÁ NORMAL
  if (ValidateQuaternion(&Orientation))
  {
    return;
  }

  //ORIENTAÇÃO INVALIDA,É NECESSARIO RESETAR O QUATERNION
  if (ValidateQuaternion(Quaternion))
  {
    //OBTÉM O VALOR ANTERIOR VALIDO
    Orientation = *Quaternion;
  }
  else
  {
    //REFERENCIA INVALIDA,O ACELEROMETRO PODE ESTAR RUIM
    ResetOrientationQuaternion(AccelerationBodyFrame);
  }
}

static void MahonyAHRSUpdate(float DeltaTime,
                             const Vector3x3_Struct *RotationBodyFrame,
                             const Vector3x3_Struct *AccelerationBodyFrame,
                             const Vector3x3_Struct *MagnetometerBodyFrame,
                             bool SafeToUseGPSHeading, float CourseOverGround,
                             float AccelerometerWeightScaler,
                             float MagnetometerWeightScaler)
{
  static Vector3x3_Struct GyroDriftEstimate = {0};

  Quaternion_Struct PreviousOrientation = Orientation;
  Vector3x3_Struct RotationRate = *RotationBodyFrame;

  //CALCULA O VALOR DO SPIN RATE EM RADIANOS/S
  const float Spin_Rate_Square = VectorNormSquared(&RotationRate);

  //CORREÇÃO DO ROLL E PITCH USANDO O VETOR DO ACELEROMETRO
  if (AccelerationBodyFrame)
  {
    static const Vector3x3_Struct Gravity = {.Vector = {0.0f, 0.0f, 1.0f}};

    Vector3x3_Struct EstimatedGravity;
    Vector3x3_Struct AccelerationVector;
    Vector3x3_Struct VectorError;

    //ESTIMA A GRAVIDADE NO BODY FRAME
    QuaternionRotateVector(&EstimatedGravity, &Gravity, &Orientation);

    //ESTIMA A DIREÇÃO DA GRAVIDADE
    VectorNormalize(&AccelerationVector, AccelerationBodyFrame);
    VectorCrossProduct(&VectorError, &AccelerationVector, &EstimatedGravity);

    //CALCULA E APLICA O FEEDBACK INTEGRAL
    if (AHRSConfiguration.kI_Accelerometer > 0.0f)
    {
      //DESATIVA A INTEGRAÇÃO SE O SPIN RATE ULTRAPASSAR UM CERTO VALOR
      if (Spin_Rate_Square < SquareFloat(ConvertToRadians(SPIN_RATE_LIMIT)))
      {
        Vector3x3_Struct OldVector;
        //CALCULA O ERRO ESCALADO POR kI
        VectorScale(&OldVector, &VectorError, AHRSConfiguration.kI_Accelerometer * DeltaTime);
        VectorAdd(&GyroDriftEstimate, &GyroDriftEstimate, &OldVector);
      }
    }

    //CALCULA O GANHO DE kP E APLICA O FEEDBACK PROPORCIONAL
    VectorScale(&VectorError, &VectorError, AHRSConfiguration.kP_Accelerometer * AccelerometerWeightScaler);
    VectorAdd(&RotationRate, &RotationRate, &VectorError);
  }

  //CORREÇÃO DO YAW
  if (MagnetometerBodyFrame || SafeToUseGPSHeading)
  {
    static const Vector3x3_Struct Forward = {.Vector = {1.0f, 0.0f, 0.0f}};

    Vector3x3_Struct VectorError = {.Vector = {0.0f, 0.0f, 0.0f}};

    if (MagnetometerBodyFrame && VectorNormSquared(MagnetometerBodyFrame) > 0.01f)
    {
      Vector3x3_Struct MagnetormeterVector;

      //CALCULA O NORTE MAGNETICO
      QuaternionRotateVectorInverse(&MagnetormeterVector, MagnetometerBodyFrame, &Orientation);

      //IGNORA A INCLINAÇÃO Z DO MAGNETOMETRO
      MagnetormeterVector.Yaw = 0.0f;

      //VERIFICA SE O MAGNETOMETRO^2 ESTÁ OK
      if (VectorNormSquared(&MagnetormeterVector) > 0.01f)
      {
        //NORMALIZA O VETOR DO MAGNETOMETRO
        VectorNormalize(&MagnetormeterVector, &MagnetormeterVector);

        //CALCULA A REFERENCIA DO COMPASS
        VectorCrossProduct(&VectorError, &MagnetormeterVector, &CorrectedMagneticFieldNorth);

        //CALCULA O ERRO DE ROTAÇÃO NO BODY FRAME
        QuaternionRotateVector(&VectorError, &VectorError, &Orientation);
      }
    }
    else if (SafeToUseGPSHeading)
    {
      Vector3x3_Struct HeadingEarthFrame;

      //USE O COG DO GPS PARA CALCULAR O HEADING
      while (CourseOverGround > 3.14159265358979323846f)
      {
        CourseOverGround -= (6.283185482f);
      }

      while (CourseOverGround < -3.14159265358979323846f)
      {
        CourseOverGround += (6.283185482f);
      }

      //CALCULA O VALOR DE HEADING COM BASE NO COG
      Vector3x3_Struct CourseOverGroundVector = {.Vector = {-Fast_Cosine(CourseOverGround), Fast_Sine(CourseOverGround), 0.0f}};

      //ROTACIONA O VETOR DO BODY FRAME PARA EARTH FRAME
      QuaternionRotateVectorInverse(&HeadingEarthFrame, &Forward, &Orientation);
      HeadingEarthFrame.Yaw = 0.0f;

      //CORRIJA APENAS SE O SQUARE FOR POSITIVO E MAIOR QUE ZERO
      if (VectorNormSquared(&HeadingEarthFrame) > 0.01f)
      {
        //NORMALIZA O VETOR
        VectorNormalize(&HeadingEarthFrame, &HeadingEarthFrame);

        //CALCULA O ERRO
        VectorCrossProduct(&VectorError, &CourseOverGroundVector, &HeadingEarthFrame);

        //ROTACIONA O ERRO NO BODY FRAME
        QuaternionRotateVector(&VectorError, &VectorError, &Orientation);
      }
    }

    //CALCULA E APLICA O FEEDBACK INTEGRAL
    if (AHRSConfiguration.kI_Magnetometer > 0.0f)
    {
      //PARE A INTEGRAÇÃO SE O SPIN RATE FOR MAIOR QUE O LIMITE
      if (Spin_Rate_Square < SquareFloat(ConvertToRadians(SPIN_RATE_LIMIT)))
      {
        Vector3x3_Struct OldVector;
        //CALCULA O ERRO INTEGRAL ESCALADO POR kI
        VectorScale(&OldVector, &VectorError, AHRSConfiguration.kI_Magnetometer * DeltaTime);
        VectorAdd(&GyroDriftEstimate, &GyroDriftEstimate, &OldVector);
      }
    }

    //CALCULA O GANHO DE kP E APLICA O FEEDBACK DO PROPORCIONAL
    VectorScale(&VectorError, &VectorError, AHRSConfiguration.kP_Magnetometer * MagnetometerWeightScaler);
    VectorAdd(&RotationRate, &RotationRate, &VectorError);
  }

  //APLICA A CORREÇÃO DE DRIFT DO GYROSCOPIO
  VectorAdd(&RotationRate, &RotationRate, &GyroDriftEstimate);

  //TAXA DE MUDANÇA DO QUATERNION
  Vector3x3_Struct Theta; //THETA É A ROTAÇÃO DE EIXO/ÂNGULO.
  Quaternion_Struct QuaternionDelta;

  VectorScale(&Theta, &RotationRate, 0.5f * DeltaTime);
  QuaternionInitFromVector(&QuaternionDelta, &Theta);
  const float ThetaSquared = VectorNormSquared(&Theta);

  //ATUALIZE O QUATERNION APENAS SE A ROTAÇÃO FOR MAIOR QUE ZERO
  if (ThetaSquared >= 1e-20)
  {
    //CONFORME THETA SE APROXIMA DE ZERO,AS OPERAÇÕES DE SENO E COSENO SE TORNAM CADA VEZ MAIS INSTAVEIS,
    //PARA CONTORNAR ISSO,USAMOS A SÉRIE DE TAYLOR,VERIFICAMOS SE O SQUARE DE THETA É MENOR QUE A PRECISÃO
    //FLOAT DO MICROCONTROLADOR (FLOAT = 24 BITS COM EXPONENTE DE 8 BITS),SENDO ASSIM SER POSSIVEL CALCULAR
    //COM SEGURANÇA O VALOR DE UM ÂNGULO PEQUENO SEM PERDER A PRECISÃO NUMÉRICA.
    if (ThetaSquared < Fast_SquareRoot(24.0f * 1e-6f))
    {
      QuaternionScale(&QuaternionDelta, &QuaternionDelta, 1.0f - ThetaSquared / 6.0f);
      QuaternionDelta.q0 = 1.0f - ThetaSquared / 2.0f;
    }
    else
    {
      const float ThetaMagnitude = Fast_SquareRoot(ThetaSquared);
      QuaternionScale(&QuaternionDelta, &QuaternionDelta, Fast_Sine(ThetaMagnitude) / ThetaMagnitude);
      QuaternionDelta.q0 = Fast_Cosine(ThetaMagnitude);
    }

    //CALCULA O VALOR FINA DA ORIENTAÇÃO E RENORMALIZA O QUATERNION
    QuaternionMultiply(&Orientation, &Orientation, &QuaternionDelta);
    QuaternionNormalize(&Orientation, &Orientation);
  }

  //CHECA SE O NOVO VALOR DO QUATERNION É VALIDO,SE SIM RESETA O VALOR ANTIGO
  CheckAndResetOrientationQuaternion(&PreviousOrientation, AccelerationBodyFrame);

  //CALCULA O QUATERNION PARA A MATRIX ROTATIVA
  ComputeRotationMatrix();
}

static float CalculateAccelerometerWeight(void)
{
  float AccelerometerMagnitudeSquare = 0;

  //CALCULA O SQUARE DE TODOS OS EIXOS DO ACELEROMETRO PARA EXTRAIR A MAGNITUDE
  AccelerometerMagnitudeSquare += SquareFloat(IMU.Accelerometer.ReadFloat[ROLL]);
  AccelerometerMagnitudeSquare += SquareFloat(IMU.Accelerometer.ReadFloat[PITCH]);
  AccelerometerMagnitudeSquare += SquareFloat(IMU.Accelerometer.ReadFloat[YAW]);

  //CALCULA A CURVA DE SENO DA MAGNITUDE DO ACELEROMETRO
  const float AccWeight_Nearness = Sine_Curve(Fast_SquareRoot(AccelerometerMagnitudeSquare) - 1.0f, MAX_ACC_NEARNESS);

  return AccWeight_Nearness;
}

static void ComputeQuaternionFromRPY(int16_t InitialRoll, int16_t InitialPitch, int16_t InitialYaw)
{
  if (InitialRoll > 1800)
  {
    InitialRoll -= 3600;
  }

  if (InitialPitch > 1800)
  {
    InitialPitch -= 3600;
  }

  if (InitialYaw > 1800)
  {
    InitialYaw -= 3600;
  }

  const float CosineRoll = Fast_Cosine(ConvertDeciDegreesToRadians(InitialRoll) * 0.5f);
  const float SineRoll = Fast_Sine(ConvertDeciDegreesToRadians(InitialRoll) * 0.5f);

  const float CosinePitch = Fast_Cosine(ConvertDeciDegreesToRadians(InitialPitch) * 0.5f);
  const float SinePitch = Fast_Sine(ConvertDeciDegreesToRadians(InitialPitch) * 0.5f);

  const float CosineYaw = Fast_Cosine(ConvertDeciDegreesToRadians(-InitialYaw) * 0.5f);
  const float SineYaw = Fast_Sine(ConvertDeciDegreesToRadians(-InitialYaw) * 0.5f);

  Orientation.q0 = CosineRoll * CosinePitch * CosineYaw + SineRoll * SinePitch * SineYaw;
  Orientation.q1 = SineRoll * CosinePitch * CosineYaw - CosineRoll * SinePitch * SineYaw;
  Orientation.q2 = CosineRoll * SinePitch * CosineYaw + SineRoll * CosinePitch * SineYaw;
  Orientation.q3 = CosineRoll * CosinePitch * SineYaw - SineRoll * SinePitch * CosineYaw;

  ComputeRotationMatrix();
}

void GetMeasuredAcceleration(Vector3x3_Struct *MeasureAcceleration)
{
  MeasureAcceleration->Vector[ROLL] = IMU.Accelerometer.ReadFloat[ROLL] * GRAVITY_CMSS;
  MeasureAcceleration->Vector[PITCH] = IMU.Accelerometer.ReadFloat[PITCH] * GRAVITY_CMSS;
  MeasureAcceleration->Vector[YAW] = IMU.Accelerometer.ReadFloat[YAW] * GRAVITY_CMSS;
}

void GetMeasuredRotationRate(Vector3x3_Struct *MeasureRotation)
{
  MeasureRotation->Vector[ROLL] = ConvertToRadians(IMU.Gyroscope.ReadFloat[ROLL]);
  MeasureRotation->Vector[PITCH] = ConvertToRadians(IMU.Gyroscope.ReadFloat[PITCH]);
  MeasureRotation->Vector[YAW] = ConvertToRadians(IMU.Gyroscope.ReadFloat[YAW]);
}

void AHRSClass::Update(float DeltaTime)
{
  bool SafeToUseCompass = false;
  bool SafeToUseGPSHeading = false;
  float CourseOverGround = 0;

  GetMeasuredAcceleration(&BodyFrameAcceleration); //CALCULA A ACELERAÇÃO DA IMU EM CM/S^2
  GetMeasuredRotationRate(&BodyFrameRotation);     //CALCULA A ROTAÇÃO DA IMU EM RADIANOS/S

  if (GetAirPlaneEnabled())
  {
    const bool SafeToUseCOG = Get_GPS_Heading_Is_Valid();

    if (I2CResources.Found.Compass)
    {
      SafeToUseCompass = true;
      GPS_Resources.Navigation.Misc.Get.HeadingInitialized = true;
    }
    else if (SafeToUseCOG)
    {
      if (GPS_Resources.Navigation.Misc.Get.HeadingInitialized)
      {
        CourseOverGround = ConvertDeciDegreesToRadians(GPS_Resources.Navigation.Misc.Get.GroundCourse);
        SafeToUseGPSHeading = true;
      }
      else
      {
        ComputeQuaternionFromRPY(Attitude.EulerAngles.Roll, Attitude.EulerAngles.Pitch, GPS_Resources.Navigation.Misc.Get.GroundCourse);
        GPS_Resources.Navigation.Misc.Get.HeadingInitialized = true;
      }
    }
  }
  else
  {
    if (I2CResources.Found.Compass)
    {
      SafeToUseCompass = true;
    }
  }

  Vector3x3_Struct MagnetometerBodyFrame = {.Vector = {(float)IMU.Compass.Read[ROLL],
                                                       (float)IMU.Compass.Read[PITCH],
                                                       (float)IMU.Compass.Read[YAW]}};

  const float CalcedCompassWeight = 10.0f;
  const float CalcedAccelerometerWeight = NEARNESS * CalculateAccelerometerWeight();
  const bool SafeToUseAccelerometer = (CalcedAccelerometerWeight > 0.001f);

  //ATUALIZA O AHRS
  MahonyAHRSUpdate(DeltaTime, &BodyFrameRotation,
                   SafeToUseAccelerometer ? &BodyFrameAcceleration : NULL,
                   SafeToUseCompass ? &MagnetometerBodyFrame : NULL,
                   SafeToUseGPSHeading, CourseOverGround,
                   CalcedAccelerometerWeight,
                   CalcedCompassWeight);

  //SAÍDA DOS EIXOS DO APÓS O AHRS
  //PITCH
  Attitude.EulerAngles.Pitch = ConvertRadiansToDeciDegrees(Fast_Atan2(Rotation.Matrix3x3[2][1], Rotation.Matrix3x3[2][2]));
  //ROLL
  Attitude.EulerAngles.Roll = ConvertRadiansToDeciDegrees((0.5f * 3.14159265358979323846f) - Fast_AtanCosine(-Rotation.Matrix3x3[2][0]));
  //YAW
  Attitude.EulerAngles.YawDecidegrees = ConvertRadiansToDeciDegrees(-Fast_Atan2(Rotation.Matrix3x3[1][0], Rotation.Matrix3x3[0][0]));
  //CONVERTE O VALOR DE COMPASS HEADING PARA O VALOR ACEITAVEL
  if (Attitude.EulerAngles.YawDecidegrees < 0)
  {
    Attitude.EulerAngles.YawDecidegrees += 3600;
  }
  Attitude.EulerAngles.Yaw = ConvertDeciDegreesToDegrees(Attitude.EulerAngles.YawDecidegrees);
}

float AHRSClass::CosineTiltAngle(void)
{
  return 1.0f - 2.0f * SquareFloat(Orientation.q1) - 2.0f * SquareFloat(Orientation.q2);
}

bool AHRSClass::CheckAnglesInclination(int16_t Angle)
{
  if (AHRS.CosineTiltAngle() < Fast_Cosine(ConvertToRadians(Angle)))
  {
    return true;
  }
  return false;
}

bool AHRSClass::Get_Cosine_Z_Overflowed(void)
{
  return AHRS.CheckAnglesInclination(AHRSConfiguration.Cosine_Z);
}

void AHRSClass::TransformVectorEarthFrameToBodyFrame(Vector3x3_Struct *VectorPointer)
{
  VectorPointer->Pitch = -VectorPointer->Pitch;
  QuaternionRotateVector(VectorPointer, VectorPointer, &Orientation);
}

void AHRSClass::TransformVectorBodyFrameToEarthFrame(Vector3x3_Struct *VectorPointer)
{
  QuaternionRotateVectorInverse(VectorPointer, VectorPointer, &Orientation);
  VectorPointer->Pitch = -VectorPointer->Pitch;
}

float AHRSClass::GetSineRoll(void)
{
  return Fast_Sine(ConvertDeciDegreesToRadians(Attitude.EulerAngles.Roll));
}

float AHRSClass::GetCosineRoll(void)
{
  return Fast_Cosine(ConvertDeciDegreesToRadians(Attitude.EulerAngles.Roll));
}

float AHRSClass::GetSinePitch(void)
{
  return Fast_Sine(ConvertDeciDegreesToRadians(Attitude.EulerAngles.Pitch));
}

float AHRSClass::GetCosinePitch(void)
{
  return Fast_Cosine(ConvertDeciDegreesToRadians(Attitude.EulerAngles.Pitch));
}

float AHRSClass::GetSineYaw(void)
{
  return Fast_Sine(ConvertCentiDegreesToRadians(ConvertDecidegreesToCentiDegrees(Attitude.EulerAngles.YawDecidegrees)));
}

float AHRSClass::GetCosineYaw(void)
{
  return Fast_Cosine(ConvertCentiDegreesToRadians(ConvertDecidegreesToCentiDegrees(Attitude.EulerAngles.YawDecidegrees)));
}