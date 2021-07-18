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

#include "TPA.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"
#include "PID/RCPID.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

TPA_Parameters_Struct TPA_Parameters;

//#define TEST_AIRSPEED_PID_TPA //PARA TESTES FUTURAMENTE

void TPA_Initialization(void)
{
  TPA_Parameters.BreakPointer = STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR);
  TPA_Parameters.ThrottlePercent = STORAGEMANAGER.Read_8Bits(TPA_PERCENT_ADDR); //ESSE PARAMETRO ACIMA DE 50% FUNCIONA BEM PARA AEROS E ASA-FIXA
}

#ifdef TEST_AIRSPEED_PID_TPA

//AINDA É NECESSARIO TESTES

#include "AirSpeed/AIRSPEED.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"

#define AIRSPEED_MIN 9  //M/S ~ 32,4km/h
#define AIRSPEED_MAX 22 //M/S ~ 79,2km/h

float TPA_Scaling_Speed = 15; //CALCULA A ATENUAÇÃO DO PID COM BASE NA VELOCIDADE DA FUSELAGEM ~ 0 É DESATIVADO / 15 É UM VALOR RECOMENDADO

static float Get_PID_AirSpeed_Scaler(float ScalingSpeed)
{
  float AirSpeedValue = AIRSPEED.Get_True_Value("In Meters");
  float AirSpeed_Scaler = 0.0f;
  if (Get_AirSpeed_Enabled()) //HEALTHY
  {
    if (AirSpeedValue > 0.0001f)
    {
      AirSpeed_Scaler = ScalingSpeed / AirSpeedValue;
    }
    else
    {
      AirSpeed_Scaler = 2.0f;
    }
    float Scale_Min = MIN(0.5f, (0.5f * AIRSPEED_MIN) / ScalingSpeed);
    float Scale_Max = MAX(2.0f, (1.5f * AIRSPEED_MAX) / ScalingSpeed);
    AirSpeed_Scaler = Constrain_Float(AirSpeed_Scaler, Scale_Min, Scale_Max);
  }
  else
  {
    AirSpeed_Scaler = 1.0f;
  }
  return AirSpeed_Scaler;
}

#endif

float CalculateFixedWingTPAFactor(int16_t Throttle)
{

#ifdef TEST_AIRSPEED_PID_TPA

  const float ParseScalingSpeed = TPA_Scaling_Speed;

  if (ParseScalingSpeed > 0)
  {
    return Get_PID_AirSpeed_Scaler(ParseScalingSpeed);
  }

#endif

  if (TPA_Parameters.ThrottlePercent != 0 && RC_Resources.Attitude.ThrottleMin < TPA_Parameters.BreakPointer)
  {
    if (Throttle > RC_Resources.Attitude.ThrottleMin)
    {
      TPA_Parameters.Factor = 0.5f + ((float)(TPA_Parameters.BreakPointer - RC_Resources.Attitude.ThrottleMin) / (Throttle - RC_Resources.Attitude.ThrottleMin) / 2.0f);
      TPA_Parameters.Factor = Constrain_Float(TPA_Parameters.Factor, 0.5f, 2.0f);
    }
    else
    {
      TPA_Parameters.Factor = 2.0f;
    }
    TPA_Parameters.Factor = 1.0f + (TPA_Parameters.Factor - 1.0f) * (TPA_Parameters.ThrottlePercent / 100.0f);
  }
  else
  {
    TPA_Parameters.Factor = 1.0f;
  }
  return TPA_Parameters.Factor;
}

float CalculateMultirotorTPAFactor(int16_t Throttle)
{
  if (TPA_Parameters.ThrottlePercent == 0 || Throttle < TPA_Parameters.BreakPointer)
  {
    TPA_Parameters.Factor = 1.0f;
  }
  else if (Throttle < RC_Resources.Attitude.ThrottleMax)
  {
    TPA_Parameters.Factor = (100 - (uint16_t)TPA_Parameters.ThrottlePercent * (Throttle - TPA_Parameters.BreakPointer) / (float)(RC_Resources.Attitude.ThrottleMax - TPA_Parameters.BreakPointer)) / 100.0f;
  }
  else
  {
    TPA_Parameters.Factor = (100 - TPA_Parameters.ThrottlePercent) / 100.0f;
  }
  return TPA_Parameters.Factor;
}