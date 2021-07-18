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

#include "ASPA.h"
#include "Math/MATHSUPPORT.h"
#include "AirSpeed/AIRSPEED.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

/*
AINDA É NECESSARIO TESTES
AINDA É NECESSARIO TESTES
AINDA É NECESSARIO TESTES
AINDA É NECESSARIO TESTES
AINDA É NECESSARIO TESTES
AINDA É NECESSARIO TESTES
AINDA É NECESSARIO TESTES
AINDA É NECESSARIO TESTES
*/

#ifdef TEST_AIRSPEED_PID_TPA

#define AIRSPEED_MIN 9  //M/S ~ 32,4km/h
#define AIRSPEED_MAX 22 //M/S ~ 79,2km/h

float TPA_Scaling_Speed = 15; //CALCULA O TPA COM BASE NA VELOCIDADE DA FUSELAGEM (0 É DESATIVADO ~ 15 É UM VALOR RECOMENDADO (M/S))

bool Get_ASPA_Enabled(void)
{
  return (TPA_Scaling_Speed > 0);
}

float Get_ASPA_Scaler(void)
{
  float AirSpeedValue = AIRSPEED.Get_True_Value("In Meters");
  float AirSpeed_Scaler = 0.0f;
  if (Get_AirSpeed_Enabled()) //HEALTHY
  {
    if (AirSpeedValue > 0.0001f)
    {
      AirSpeed_Scaler = TPA_Scaling_Speed / AirSpeedValue;
    }
    else
    {
      AirSpeed_Scaler = 2.0f;
    }
    float Scale_Min = MIN(0.5f, (0.5f * AIRSPEED_MIN) / TPA_Scaling_Speed);
    float Scale_Max = MAX(2.0f, (1.5f * AIRSPEED_MAX) / TPA_Scaling_Speed);
    AirSpeed_Scaler = Constrain_Float(AirSpeed_Scaler, Scale_Min, Scale_Max);
  }
  else
  {
    AirSpeed_Scaler = 1.0f;
  }
  return AirSpeed_Scaler;
}

#endif