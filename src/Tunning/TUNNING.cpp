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
#include "FastSerial/PRINTF.h"

TunningClass TUNNING;
Tunning_Enum_Typedef Tunning_Status;

void TunningClass::Initialization(void)
{
#ifdef USE_TUNNING_MODE

  TUNNING.ChannelControl = STORAGEMANAGER.Read_8Bits(CH_TUNNING_ADDR);
  TUNNING.Mode = STORAGEMANAGER.Read_8Bits(TUNNING_ADDR);

#endif

  TUNNING.ChannelControl = 1;
}

int16_t TunningClass::GetConfiguredChannelValue(Tunning_Enum_Typedef OnOffMode)
{
  if (OnOffMode == TUNNING_TYPE_STATE)
  {
    return (DECODE.GetRxChannelOutput(TUNNING.ChannelControl + 3) > 1400) ? TUNNING_STATE_ENABLED : TUNNING_STATE_DISABLED;
  }

  return DECODE.GetRxChannelOutput(TUNNING.ChannelControl + 3);
}

void TunningClass::Update(void)
{

  Tunning_Status = TUNNING_PITOT_FACTOR;

  DEBUG("State:%d VarValue:%d", TUNNING.GetActivated(TUNNING_PITOT_FACTOR), TUNNING.GetConfiguredChannelValue(TUNNING_TYPE_ADJUSTABLE));

#ifdef USE_TUNNING_MODE

  if (TUNNING.Mode == NONE_TUNNING_MODE || TUNNING.ChannelControl == NONE_TUNNING_CHANNEL)
  {
    return;
  }

  switch (TUNNING.Mode)
  {

  case TUNNING_KP_ROLL:
    break;

  case TUNNING_KI_ROLL:
    break;

  case TUNNING_KD_ROLL:
    break;

  case TUNNING_KCD_OR_KFF_ROLL:
    break;

  case TUNNING_KP_PITCH:
    break;

  case TUNNING_KI_PITCH:
    break;

  case TUNNING_KD_PITCH:
    break;

  case TUNNING_KCD_OR_KFF_PITCH:
    break;

  case TUNNING_KP_YAW:
    break;

  case TUNNING_KI_YAW:
    break;

  case TUNNING_KD_YAW:
    break;

  case TUNNING_KCD_OR_KFF_YAW:
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