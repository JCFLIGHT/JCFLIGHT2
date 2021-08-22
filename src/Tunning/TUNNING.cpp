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

#include "TUNNING.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "RadioControl/DECODE.h"
#include "Common/RCDEFINES.h"
#include "Common/ENUM.h"
#include "PID/PIDPARAMS.h"
#include "Math/MATHSUPPORT.h"

TunningClass TUNNING;
Tunning_Enum_Typedef Tunning_Status;

void TunningClass::Initialization(void)
{
#ifdef USE_TUNNING_MODE

  TUNNING.ChannelControl = STORAGEMANAGER.Read_8Bits(CH_TUNNING_ADDR);
  TUNNING.Mode = STORAGEMANAGER.Read_8Bits(TUNNING_ADDR);

#endif
}

int16_t TunningClass::GetConfiguredChannelValue(Tunning_Enum_Typedef OnOffMode)
{
  if (OnOffMode == TUNNING_TYPE_STATE)
  {
    return (DECODE.GetRxChannelOutput(TUNNING.ChannelControl + NON_AUX_CHANNEL_COUNT) > 1400) ? TUNNING_STATE_ENABLED : TUNNING_STATE_DISABLED;
  }

  return DECODE.GetRxChannelOutput(TUNNING.ChannelControl + NON_AUX_CHANNEL_COUNT);
}

void TunningClass::Update(void)
{
#ifdef USE_TUNNING_MODE

  if (TUNNING.Mode == NONE_TUNNING_MODE || TUNNING.ChannelControl == NONE_TUNNING_CHANNEL)
  {
    return;
  }

  switch (TUNNING.Mode)
  {
  case TUNNING_KP_ROLL:
    GET_SET[PID_ROLL].kP = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KI_ROLL:
    GET_SET[PID_ROLL].kI = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KD_ROLL:
    GET_SET[PID_ROLL].kD = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KCD_OR_KFF_ROLL:
    GET_SET[PID_ROLL].kFF = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KP_PITCH:
    GET_SET[PID_PITCH].kP = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KI_PITCH:
    GET_SET[PID_PITCH].kI = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KD_PITCH:
    GET_SET[PID_PITCH].kD = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KCD_OR_KFF_PITCH:
    GET_SET[PID_PITCH].kFF = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KP_YAW:
    GET_SET[PID_YAW].kP = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KI_YAW:
    GET_SET[PID_YAW].kI = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KD_YAW:
    GET_SET[PID_YAW].kD = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_KCD_OR_KFF_YAW:
    GET_SET[PID_YAW].kFF = ScaleRange16Bits(GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE, 0, 200);
    break;

  case TUNNING_PITOT_FACTOR:
    Tunning_Status = TUNNING_PITOT_FACTOR;
    break;
  }

#endif
}

bool TunningClass::GetActivated(Tunning_Enum_Typedef TunningParam)
{
  if (Tunning_Status == TunningParam)
  {
    if (TUNNING.GetConfiguredChannelValue(TUNNING_TYPE_STATE) == TUNNING_STATE_ENABLED)
    {
      return true;
    }
  }
  return false;
}