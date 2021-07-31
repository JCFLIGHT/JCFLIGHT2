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

#include "GIMBAL.h"
#include "FlightModes/AUXFLIGHT.h"
#include "RadioControl/DECODE.h"
#include "MotorsControl/MOTORS.h"
#include "Common/RCDEFINES.h"
#include "Common/ENUM.h"
#include "Math/MATHSUPPORT.h"
#include "Param/PARAM.h"

//CONTROLE DO TILT DO GIMBAL

void Gimbal_Controll(void)
{
  for (uint8_t IndexCount = 0; IndexCount <= RCAUX8; IndexCount++)
  {
    if (AUXFLIGHT.GetModeConfiguration[GIMBAL_MODE] == IndexCount)
    {
      if (AUXFLIGHT.GetModeConfiguration[GIMBAL_MODE] == NONE)
      {
        MotorControl[GIMBAL] = MIDDLE_STICKS_PULSE;
      }
      else
      {
        MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(NON_AUX_CHANNEL_COUNT + IndexCount);
      }
      break;
    }
  }
#ifndef __AVR_ATmega2560__

  Constrain_16Bits(MotorControl[GIMBAL], JCF_Param.GimbalMinValue, JCF_Param.GimbalMaxValue);

#endif
}
