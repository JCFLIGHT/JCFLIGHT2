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

#include "ACCGYROREAD.h"
#include "I2C/I2C.h"
#include "Filters/KALMANFILTER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "Filters/BIQUADFILTER.h"
#include "Scheduler/SCHEDULER.h"
#include "Compass/COMPASSREAD.h"
#include "BAR/BAR.h"
#include "Build/BOARDDEFS.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "PerformanceCalibration/PERFORMGYRO.h"
#include "FILTERANDRATE.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

IMU_Struct IMU;

#ifdef USE_IMU_FILTERS

//INSTANCIAS PARA O LPF
static BiquadFilter_Struct BiquadAccLPF[3];
static BiquadFilter_Struct BiquadGyroLPF[3];

//INSTANCIAS PARA O NOTCH
static BiquadFilter_Struct BiquadAccNotch[3];
static BiquadFilter_Struct BiquadGyroNotch[3];

#endif

bool KalmanFilterEnabled = false;

#ifdef USE_IMU_FILTERS

int16_t Biquad_Acc_LPF = 0;
int16_t Biquad_Gyro_LPF = 0;
int16_t Biquad_Acc_Notch = 0;
int16_t Biquad_Gyro_Notch = 0;

#endif

static void IMU_Filters_Initialization(void)
{
  //ATUALIZA O ESTADO GUARDADO DO ESTADO DO KALMAN
  KalmanFilterEnabled = STORAGEMANAGER.Read_8Bits(KALMAN_ADDR) == NONE ? false : true;
  KALMAN.Initialization();

#ifdef USE_IMU_FILTERS

  //CARREGA OS VALORES GUARDADOS DO LPF
  Biquad_Acc_LPF = STORAGEMANAGER.Read_16Bits(BI_ACC_LPF_ADDR);
  Biquad_Gyro_LPF = STORAGEMANAGER.Read_16Bits(BI_GYRO_LPF_ADDR);
  //GERA UM COEFICIENTE PARA O LPF DO ACELEROMETRO
  BIQUADFILTER.Settings(&BiquadAccLPF[ROLL], Biquad_Acc_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&BiquadAccLPF[PITCH], Biquad_Acc_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&BiquadAccLPF[YAW], Biquad_Acc_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  //GERA UM COEFICIENTE PARA O LPF DO GYROSCOPIO
  BIQUADFILTER.Settings(&BiquadGyroLPF[ROLL], Biquad_Gyro_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&BiquadGyroLPF[PITCH], Biquad_Gyro_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&BiquadGyroLPF[YAW], Biquad_Gyro_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  //CARREGA OS VALORES GUARDADOS DO NOTCH
  Biquad_Acc_Notch = STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR);
  Biquad_Gyro_Notch = STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR);
  //GERA UM COEFICIENTE PARA O NOTCH DO ACELEROMETRO
  BIQUADFILTER.Settings(&BiquadAccNotch[ROLL], Biquad_Acc_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), NOTCH);
  BIQUADFILTER.Settings(&BiquadAccNotch[PITCH], Biquad_Acc_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), NOTCH);
  BIQUADFILTER.Settings(&BiquadAccNotch[YAW], Biquad_Acc_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), NOTCH);
  //GERA UM COEFICIENTE PARA O NOTCH DO GYROSCOPIO
  BIQUADFILTER.Settings(&BiquadGyroNotch[ROLL], Biquad_Gyro_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), NOTCH);
  BIQUADFILTER.Settings(&BiquadGyroNotch[PITCH], Biquad_Gyro_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), NOTCH);
  BIQUADFILTER.Settings(&BiquadGyroNotch[YAW], Biquad_Gyro_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), NOTCH);

#endif
}

static MPUDetectionResult_Enum MPU6050DeviceDetect(void)
{
  uint8_t Who_Am_I;
  uint8_t RevisionBuffer[6];
  uint8_t ForceRepetition = 5;

  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x6B, 0x80);

  do
  {
    SCHEDULERTIME.Sleep(150);

    I2C.RegisterBuffer(ADDRESS_IMU_MPU6050, 0x75, &Who_Am_I, 0x01);

    Who_Am_I &= 0x7E;

    if (Who_Am_I == ADDRESS_IMU_MPU6050)
    {
      break;
    }

    if (!ForceRepetition)
    {
      return MPU6050_NONE;
    }

  } while (ForceRepetition--);

  I2C.RegisterBuffer(ADDRESS_IMU_MPU6050, 0x06, RevisionBuffer, 0x06);

  uint8_t Revision = ((RevisionBuffer[5] & 0x01) << 2) | ((RevisionBuffer[3] & 0x01) << 1) | (RevisionBuffer[1] & 0x01);

  if (Revision)
  {
    if (Revision == 1)
    {
      return MPU6050_HALF_RESOLUTION;
    }
    else if (Revision == 2)
    {
      return MPU6050_FULL_RESOLUTION;
    }
    else if ((Revision == 3) || (Revision == 7))
    {
      return MPU6050_FULL_RESOLUTION;
    }
    else
    {
      return MPU6050_NONE;
    }
  }
  else
  {
    uint8_t Product_ID;

    I2C.RegisterBuffer(ADDRESS_IMU_MPU6050, 0x0C, &Product_ID, 0x01);

    Revision = Product_ID & 0x0F;

    if (!Revision)
    {
      return MPU6050_NONE;
    }
    else if (Revision == 4)
    {
      return MPU6050_HALF_RESOLUTION;
    }
    else
    {
      return MPU6050_FULL_RESOLUTION;
    }
  }

  return MPU6050_NONE;
}

static void AccResolutionDetect(void)
{
  MPUDetectionResult_Enum Resolution = MPU6050DeviceDetect();

  IMU.Gyroscope.Scale = 1.0f / 16.4f; //MPU6050

  if (Resolution == MPU6050_NONE)
  {
    LOG("SEM IMU");
    LINE_SPACE;
    IMU.Accelerometer.GravityForce.OneG = 256;
    return;
  }
  else
  {
    LOG("DISPOSITIVO ENCONTRADO - 0x68 << MPU-6050");
    LOG_WITH_ARGS("IMU DO TIPO:%s", Resolution == MPU6050_FULL_RESOLUTION ? (char *)"FULL RESOLUTION 1G = 2048" : (char *)"HALF RESOLUTION 1G = 1024");
    LINE_SPACE;
  }

  if (Resolution == MPU6050_FULL_RESOLUTION)
  {
    IMU.Accelerometer.GravityForce.OneG = 2048;
  }
  else
  {
    IMU.Accelerometer.GravityForce.OneG = 1024;
  }
}

static uint8_t GetConfiguratedIntenalGyroLPF(void)
{
  uint8_t RetVal = 0;

  switch (STORAGEMANAGER.Read_8Bits(HW_GYRO_LPF_ADDR)) //LPF INTERNO DA IMU
  {

    //"CASE" DE ACORDO COM O COMBOBOX DO GCS

  case 0:
    RetVal = GYRO_LPF_NONE;
    break;

  case 1:
    RetVal = GYRO_LPF_10HZ;
    break;

  case 2:
    RetVal = GYRO_LPF_20HZ;
    break;

  case 3:
    RetVal = GYRO_LPF_42HZ;
    break;

  case 4:
    RetVal = GYRO_LPF_98HZ;
    break;

  case 5:
    RetVal = GYRO_LPF_188HZ;
    break;

  case 6:
    RetVal = GYRO_LPF_256HZ;
    break;
  }

  return RetVal;
}

void MPU6050AccAndGyroInitialization(void)
{
  AccResolutionDetect();

  uint8_t DesiredLPF_Register = 0x00; //1KHZ POR PADRÃO PARA A VERSÃO CLASSIC
  uint8_t GyroLPF_Register = GetConfiguratedIntenalGyroLPF();

#ifdef USE_GYRO_FILTER_RATE_TABLE

  //ITERVALO EM US
  //500HZ = 2000
  //666HZ = 1500
  //1KHZ = 1000
  //2KHZ = 500
  //4KHZ = 250
  //8KHZ = 125
  const uint32_t MainRateIntervalUS = SCHEDULER_SET_FREQUENCY(THIS_LOOP_RATE_IN_US, "KHz");

  const MPUGyroFilterAndRate_Struct *Configuration = IMUChooseGyroConfig(GetConfiguratedIntenalGyroLPF(), 1000000 / MainRateIntervalUS);

  DesiredLPF_Register = Configuration->Gyro_I2C_Register[MPU_DLPF];
  GyroLPF_Register = Configuration->Gyro_I2C_Register[MPU_GYRO_LPF];

  //const uint32_t NewRateForIntegralLoopTask = Configuration->GyroRateInHz; //NOVO RATE PARA A TASK PRINCIPAL COM BASE NAS CONFIGURAÇÕES DO GYRO

#endif

  //RESETA A MPU-6050
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x6B, 0x80);
  SCHEDULERTIME.Sleep(150);

  //ATIVA O CLOCK DO PLL COM REFERENCIA DO EIXO Z DO GIROSCOPIO
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x6B, 0x03);
  SCHEDULERTIME.MicroSecondsSleep(15);

  //SETA O RATE DE SAÍDA DO GIROSCOPIO
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x19, DesiredLPF_Register);
  SCHEDULERTIME.MicroSecondsSleep(15);

  //SETA A FREQUENCIA DE CORTE DO LPF INTERNO DO GIROSCOPIO
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1A, GyroLPF_Register);
  SCHEDULERTIME.MicroSecondsSleep(1);

  //SETA O GIROSCOPIO EM ESCALA COMPLETA +/- 2000 DPS
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1B, 0x18);
  SCHEDULERTIME.MicroSecondsSleep(15);

  //SETA O ACELEROMETRO EM ESCALA COMPLETA +/- 16 G
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1C, 0x18);
  SCHEDULERTIME.MicroSecondsSleep(15);

  //ATIVA O I2C BYPASS PARA O USO DO COMPASS
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x37, 0x02);
  SCHEDULERTIME.MicroSecondsSleep(15);

  /*
  //ATIVA O PINO INTERRUPÇÃO DA IMU
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x38, 1 << 0);
  SCHEDULERTIME.MicroSecondsSleep(15);
  */

  IMU_Filters_Initialization();
}

void IMU_Get_Data(void)
{
  uint8_t Data_Read_Buffer[14];
  I2C.RegisterBuffer(ADDRESS_IMU_MPU6050, 0x3B, Data_Read_Buffer, 14);

#ifdef __AVR_ATmega2560__

  IMU.Accelerometer.Read[ROLL] = -(((Data_Read_Buffer[0] << 8) | Data_Read_Buffer[1]) >> 3);
  IMU.Accelerometer.Read[PITCH] = -(((Data_Read_Buffer[2] << 8) | Data_Read_Buffer[3]) >> 3);
  IMU.Accelerometer.Read[YAW] = ((Data_Read_Buffer[4] << 8) | Data_Read_Buffer[5]) >> 3;

  IMU.Accelerometer.Temperature = ConvertDeciDegreesToDegrees(((Data_Read_Buffer[6] << 8) | Data_Read_Buffer[7]) / 34 + 365);

  IMU.Gyroscope.Read[PITCH] = -((Data_Read_Buffer[8] << 8) | Data_Read_Buffer[9]) >> 2;
  IMU.Gyroscope.Read[ROLL] = ((Data_Read_Buffer[10] << 8) | Data_Read_Buffer[11]) >> 2;
  IMU.Gyroscope.Read[YAW] = -((Data_Read_Buffer[12] << 8) | Data_Read_Buffer[13]) >> 2;

#else

  IMU.Accelerometer.Read[ROLL] = -((Data_Read_Buffer[0] << 8) | Data_Read_Buffer[1]);
  IMU.Accelerometer.Read[PITCH] = -((Data_Read_Buffer[2] << 8) | Data_Read_Buffer[3]);
  IMU.Accelerometer.Read[YAW] = ((Data_Read_Buffer[4] << 8) | Data_Read_Buffer[5]);

  IMU.Accelerometer.Temperature = ConvertDeciDegreesToDegrees(((Data_Read_Buffer[6] << 8) | Data_Read_Buffer[7]) / 34 + 365);

  IMU.Gyroscope.Read[PITCH] = -((Data_Read_Buffer[8] << 8) | Data_Read_Buffer[9]);
  IMU.Gyroscope.Read[ROLL] = ((Data_Read_Buffer[10] << 8) | Data_Read_Buffer[11]);
  IMU.Gyroscope.Read[YAW] = -((Data_Read_Buffer[12] << 8) | Data_Read_Buffer[13]);

#endif
}

void Update_Accelerometer(void)
{
  IMU_Get_Data();

  ACCCALIBRATION.Update();

  //KALMAN
  if (KalmanFilterEnabled)
  {
    KALMAN.Apply_In_Acc(IMU.Accelerometer.Read);
  }

#ifdef USE_IMU_FILTERS

  //LPF
  if (Biquad_Acc_LPF > 0)
  {
    //APLICA O FILTRO
    IMU.Accelerometer.Read[ROLL] = BIQUADFILTER.ApplyAndGet(&BiquadAccLPF[ROLL], IMU.Accelerometer.Read[ROLL]);
    IMU.Accelerometer.Read[PITCH] = BIQUADFILTER.ApplyAndGet(&BiquadAccLPF[PITCH], IMU.Accelerometer.Read[PITCH]);
    IMU.Accelerometer.Read[YAW] = BIQUADFILTER.ApplyAndGet(&BiquadAccLPF[YAW], IMU.Accelerometer.Read[YAW]);
  }

  //NOTCH
  if (Biquad_Acc_Notch > 0)
  {
    //APLICA O FILTRO
    IMU.Accelerometer.Read[ROLL] = BIQUADFILTER.ApplyAndGet(&BiquadAccNotch[ROLL], IMU.Accelerometer.Read[ROLL]);
    IMU.Accelerometer.Read[PITCH] = BIQUADFILTER.ApplyAndGet(&BiquadAccNotch[PITCH], IMU.Accelerometer.Read[PITCH]);
    IMU.Accelerometer.Read[YAW] = BIQUADFILTER.ApplyAndGet(&BiquadAccNotch[YAW], IMU.Accelerometer.Read[YAW]);
  }

#endif

  IMU.Accelerometer.ReadFloat[ROLL] = (float)IMU.Accelerometer.Read[ROLL] / (float)IMU.Accelerometer.GravityForce.OneG;
  IMU.Accelerometer.ReadFloat[PITCH] = (float)IMU.Accelerometer.Read[PITCH] / (float)IMU.Accelerometer.GravityForce.OneG;
  IMU.Accelerometer.ReadFloat[YAW] = (float)IMU.Accelerometer.Read[YAW] / (float)IMU.Accelerometer.GravityForce.OneG;
}

void Update_Gyroscope(void)
{
  GYROCALIBRATION.Update();

  //KALMAN
  if (KalmanFilterEnabled)
  {
    KALMAN.Apply_In_Gyro(IMU.Gyroscope.Read);
  }

#ifdef USE_IMU_FILTERS

  //LPF
  if (Biquad_Gyro_LPF > 0)
  {
    //APLICA O FILTRO
    IMU.Gyroscope.Read[ROLL] = BIQUADFILTER.ApplyAndGet(&BiquadGyroLPF[ROLL], IMU.Gyroscope.Read[ROLL]);
    IMU.Gyroscope.Read[PITCH] = BIQUADFILTER.ApplyAndGet(&BiquadGyroLPF[PITCH], IMU.Gyroscope.Read[PITCH]);
    IMU.Gyroscope.Read[YAW] = BIQUADFILTER.ApplyAndGet(&BiquadGyroLPF[YAW], IMU.Gyroscope.Read[YAW]);
  }

  //NOTCH
  if (Biquad_Gyro_Notch > 0)
  {
    //APLICA O FILTRO
    IMU.Gyroscope.Read[ROLL] = BIQUADFILTER.ApplyAndGet(&BiquadGyroNotch[ROLL], IMU.Gyroscope.Read[ROLL]);
    IMU.Gyroscope.Read[PITCH] = BIQUADFILTER.ApplyAndGet(&BiquadGyroNotch[PITCH], IMU.Gyroscope.Read[PITCH]);
    IMU.Gyroscope.Read[YAW] = BIQUADFILTER.ApplyAndGet(&BiquadGyroNotch[YAW], IMU.Gyroscope.Read[YAW]);
  }

#endif

  IMU.Gyroscope.ReadFloat[ROLL] = (float)IMU.Gyroscope.Read[ROLL] * IMU.Gyroscope.Scale;
  IMU.Gyroscope.ReadFloat[PITCH] = (float)IMU.Gyroscope.Read[PITCH] * IMU.Gyroscope.Scale;
  IMU.Gyroscope.ReadFloat[YAW] = (float)IMU.Gyroscope.Read[YAW] * IMU.Gyroscope.Scale;
}