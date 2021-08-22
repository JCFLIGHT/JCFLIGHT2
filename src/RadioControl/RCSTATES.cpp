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

#include "RCSTATES.h"
#include "RCCONFIG.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Common/RCDEFINES.h"
#include "RadioControl/DECODE.h"
#include "PID/RCPID.h"
#include "Param/PARAM.h"
#include "GPSNavigation/NAVIGATION.h"

#define THIS_LOOP_RATE 50

int16_t ArmDelayedCount = 0;
int16_t DisarmDelayedCount = 0;

bool GetCheckInclinationForArm(void)
{
    if (GetAirPlaneEnabled())
    {
        return false; //PULA A CHECAGEM DE INCLINAÇÃO NO MODO PLANE
    }

    if (GetMultirotorEnabled() && AHRS.Get_Cosine_Z_Overflowed())
    {
        return true; //INVALIDA O ARMAMENTO DO SISTEMA SE HOUVER INCLINAÇÃO NOS EIXOS
    }

    return false; //INCLINAÇÃO NÃO DETECTADA
}

bool GetArmDelayedState(void)
{
    if (ArmDelayedCount >= (THIS_LOOP_RATE * JCF_Param.Arm_Time_Safety))
    {
        return true;
    }
    else
    {
        ArmDelayedCount++;
    }
    return false;
}

bool GetDisarmDelayedState(void)
{
    if (DisarmDelayedCount >= (THIS_LOOP_RATE * JCF_Param.Disarm_Time_Safety))
    {
        return true;
    }
    else
    {
        DisarmDelayedCount++;
    }
    return false;
}

void ResetArmDelayed(void)
{
    ArmDelayedCount = 0;
}

void ResetDisarmDelayed(void)
{
    DisarmDelayedCount = 0;
}

bool GetSticksStateToArm(void)
{
    return (Throttle.Output < MIN_PULSE) && (Yaw.Output > MAX_PULSE) && (Pitch.Output < MIN_PULSE) && (Roll.Output < MIN_PULSE);
}

bool GetSticksStateToDisarm(void)
{
    return (Throttle.Output < MIN_PULSE) && (Yaw.Output < MIN_PULSE) && (Pitch.Output < MIN_PULSE) && (Roll.Output > MAX_PULSE);
}

bool GetSticksInAutoPilotPosition(int16_t AutoPilotValue)
{
    return (ABS(RC_Resources.Attitude.Controller[ROLL]) < AutoPilotValue) && (ABS(RC_Resources.Attitude.Controller[PITCH]) < AutoPilotValue);
}

bool GetSticksDeflected(int16_t MinDeflectionValue)
{
    return (ABS(RC_Resources.Attitude.Controller[ROLL]) > MinDeflectionValue) || (ABS(RC_Resources.Attitude.Controller[PITCH]) > MinDeflectionValue);
}

bool GetActualThrottleStatus(uint8_t ThrottleStatus)
{
    if (DECODE.GetRxChannelOutput(THROTTLE) <= MIN_PULSE && ThrottleStatus == THROTTLE_LOW)
    {
        return true;
    }
    else if (DECODE.GetRxChannelOutput(THROTTLE) >= MAX_PULSE && ThrottleStatus == THROTTLE_HIGH)
    {
        return true;
    }
    else if (DECODE.GetRxChannelOutput(THROTTLE) >= (MIDDLE_STICKS_PULSE - MIDDLE_PULSE_OFF_SET) &&
             DECODE.GetRxChannelOutput(THROTTLE) <= (MIDDLE_STICKS_PULSE + MIDDLE_PULSE_OFF_SET) &&
             ThrottleStatus == THROTTLE_MIDDLE)
    {
        return true;
    }
    return false;
}