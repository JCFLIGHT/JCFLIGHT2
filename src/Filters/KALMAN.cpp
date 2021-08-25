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

#include "KALMAN.h"
#include "Common/ENUM.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

KalmanClass KALMANFILTER;

StructKalmanState Kalman_Acc[3];
StructKalmanState Kalman_Gyro[3];

#define ACC_PROCESS_NOISE 0.0625 //RUIDO DO PROCESSO
#define ACC_MEDICION_NOISE 1.0   //RUIDO DE MEDIÇÃO
#define ACC_ESTIMATE_ERROR 0.22  //ERRO ESTIMADO

#define GYRO_PROCESS_NOISE 3.0    //RUIDO DO PROCESSO
#define GYRO_MEDICION_NOISE 0.125 //RUIDO DE MEDIÇÃO
#define GYRO_ESTIMATE_ERROR 0.42  //ERRO ESTIMADO

void KalmanClass::Initialization(void)
{
  KALMANFILTER.AccelInitialization();
  KALMANFILTER.GyroInitialization();
}

void KalmanClass::State(StructKalmanState *State, float Q, float R, float P, float Initial_Value)
{
  State->Q = Q;
  State->R = R;
  State->P = P;
  State->X = Initial_Value;
}

void KalmanClass::Update(StructKalmanState *State, int16_t *ErrorInput)
{
  float K;
  float Measurement = *ErrorInput;
  State->P = State->P + State->Q;
  //ATUALIZA A MEDIÇÃO
  K = State->P * (1.0 / (State->P + State->R));
  State->X = State->X + K * (Measurement - State->X);
  State->P = (1 - K) * State->P;
  Measurement = State->X;
  *ErrorInput = (int16_t)Measurement;
}

void KalmanClass::AccelInitialization(void)
{
  KALMANFILTER.State(&Kalman_Acc[ROLL], ACC_PROCESS_NOISE, ACC_MEDICION_NOISE, ACC_ESTIMATE_ERROR, 0);
  KALMANFILTER.State(&Kalman_Acc[PITCH], ACC_PROCESS_NOISE, ACC_MEDICION_NOISE, ACC_ESTIMATE_ERROR, 0);
  KALMANFILTER.State(&Kalman_Acc[YAW], ACC_PROCESS_NOISE, ACC_MEDICION_NOISE, ACC_ESTIMATE_ERROR, 0);
}

void KalmanClass::GyroInitialization(void)
{
  KALMANFILTER.State(&Kalman_Gyro[ROLL], GYRO_PROCESS_NOISE, GYRO_MEDICION_NOISE, GYRO_ESTIMATE_ERROR, 0);
  KALMANFILTER.State(&Kalman_Gyro[PITCH], GYRO_PROCESS_NOISE, GYRO_MEDICION_NOISE, GYRO_ESTIMATE_ERROR, 0);
  KALMANFILTER.State(&Kalman_Gyro[YAW], GYRO_PROCESS_NOISE, GYRO_MEDICION_NOISE, GYRO_ESTIMATE_ERROR, 0);
}

void KalmanClass::Apply_In_Acc(int16_t AccVectorInput[3])
{
  KALMANFILTER.Update(&Kalman_Acc[ROLL], &AccVectorInput[ROLL]);
  KALMANFILTER.Update(&Kalman_Acc[PITCH], &AccVectorInput[PITCH]);
  KALMANFILTER.Update(&Kalman_Acc[YAW], &AccVectorInput[YAW]);
}

void KalmanClass::Apply_In_Gyro(int16_t GyroVectorInput[3])
{
  KALMANFILTER.Update(&Kalman_Gyro[ROLL], &GyroVectorInput[ROLL]);
  KALMANFILTER.Update(&Kalman_Gyro[PITCH], &GyroVectorInput[PITCH]);
  KALMANFILTER.Update(&Kalman_Gyro[YAW], &GyroVectorInput[YAW]);
}
