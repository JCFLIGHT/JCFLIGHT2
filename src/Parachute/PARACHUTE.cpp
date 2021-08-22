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

#include "PARACHUTE.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Buzzer/BUZZER.h"
#include "MotorsControl/MOTORS.h"
#include "Common/RCDEFINES.h"

ParachuteClass PARACHUTE;

#define MOTORS_DISARM_TIME 40 //MS - DESARMA OS MOTORES APÓS DETECTAR QUE O PARACHUTE FOI LANÇADO

void ParachuteClass::Auto_Do_Now(bool ActiveParachute)
{
  if (!ActiveParachute)
  {
    MotorControl[PARACHUTE_SERVO] = 400; //0 GRAUS
    PARACHUTE.ParachuteReleased = false;
    return;
  }

  PARACHUTE.ParachuteInAuto = true;
  MotorControl[PARACHUTE_SERVO] = 2400; //180 GRAUS

  if (!PARACHUTE.ParachuteReleased)
  {
    BEEPER.Play(BEEPER_PARACHUTE);
  }

  PARACHUTE.ParachuteReleased = true;
}

void ParachuteClass::Manual_Do_Now(void)
{
  if (!PARACHUTE.ParachuteReleased)
  {
    PARACHUTE.OverFlowTime += SCHEDULERTIME.GetMillis();
  }

  if (PARACHUTE.ParachuteInAuto)
  {
    return;
  }

  if (!AUXFLIGHT.GetModeState[PARACHUTE_MODE])
  {
    MotorControl[PARACHUTE_SERVO] = 400; //0 GRAUS
    PARACHUTE.ParachuteReleased = false;
    return;
  }

  MotorControl[PARACHUTE_SERVO] = 2400; //180 GRAUS

  if (!PARACHUTE.ParachuteReleased)
  {
    BEEPER.Play(BEEPER_PARACHUTE);
  }

  PARACHUTE.ParachuteReleased = true;
}

bool ParachuteClass::GetSafeStateToDisarmMotors(void)
{
  if (AUXFLIGHT.GetModeConfiguration[PARACHUTE_MODE] == NONE)
  {
    return false;
  }

  if (PARACHUTE.Released())
  {
    return true;
  }

  return false;
}

bool ParachuteClass::Released(void)
{
  return (PARACHUTE.ParachuteReleased && PARACHUTE.ReleasedOverFlowTime());
}

bool ParachuteClass::ReleasedOverFlowTime(void)
{
  if (SCHEDULERTIME.GetMillis() - PARACHUTE.OverFlowTime >= MOTORS_DISARM_TIME)
  {
    return true;
  }

  return false;
}
