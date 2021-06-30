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

#ifndef TUNNING_H_
#define TUNNING_H_
#include "inttypes.h"

typedef enum Tunning_Enum
{
    NONE_TUNNING_MODE = 0,
    NONE_TUNNING_CHANNEL = 0,
    TUNNING_TYPE_ADJUSTABLE = 0,
    TUNNING_STATE_DISABLED = 0,
    TUNNING_STATE_ENABLED = 1,
    TUNNING_TYPE_STATE = 1,
    TUNNING_KP_ROLL = 1,
    TUNNING_KI_ROLL,
    TUNNING_KD_ROLL,
    TUNNING_KCD_OR_KFF_ROLL,
    TUNNING_KP_PITCH,
    TUNNING_KI_PITCH,
    TUNNING_KD_PITCH,
    TUNNING_KCD_OR_KFF_PITCH,
    TUNNING_KP_YAW,
    TUNNING_KI_YAW,
    TUNNING_KD_YAW,
    TUNNING_KCD_OR_KFF_YAW,
    TUNNING_PITOT_FACTOR,
} Tunning_Enum_Typedef;

class TunningClass
{
public:
    void Initialization(void);
    void Update(void);
    bool GetActivated(Tunning_Enum_Typedef TunningParam);

private:
    uint8_t ChannelControll = NONE_TUNNING_CHANNEL;
    uint8_t Mode = NONE_TUNNING_MODE;
    int16_t GetConfiguredChannelValue(Tunning_Enum_Typedef StateMode);
};
extern TunningClass TUNNING;
#endif