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

#include "FLIGHTMODES.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "GPSNavigation/NAVIGATION.h"
#include "GPSNavigation/INSNAVIGATION.h"
#include "RadioControl/RCSTATES.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "GPS/GPSSTATES.h"
#include "Yaw/HEADINGHOLD.h"
#include "BitArray/BITARRAY.h"
#include "AHRS/AHRS.h"
#include "Barometer/BAROBACKEND.h"
#include "Param/PARAM.h"
#include "Common/RCDEFINES.h"
#include "AutoLaunch/AUTOLAUNCH.h"

bool Do_Altitude_Hold = false;
bool Do_RTH_Or_Land_Call_Alt_Hold = false;
bool Do_Pos_Hold_Call_Alt_Hold = false;

static bool Get_Multirotor_GPS_FlightModes_Once(void)
{
  static uint8_t Previous_Mode = 0;
  uint8_t Check_Actual_State = ((IS_FLIGHT_MODE_ACTIVE(LAND_MODE) << 2) +
                                (IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE) << 1) +
                                (IS_FLIGHT_MODE_ACTIVE(RTH_MODE)));
  if (Previous_Mode != Check_Actual_State)
  {
    Previous_Mode = Check_Actual_State;
    return true;
  }
  return false;
}

static void ProcessFlightModesToMultirotor(void)
{
  if (!GetMultirotorEnabled())
  {
    return;
  }

  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    if (Get_GPS_In_Good_Condition())
    {
      if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
      {
        DISABLE_THIS_FLIGHT_MODE(POS_HOLD_MODE);
      }
      else
      {
        if (IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE) && (GPS_Resources.Navigation.AutoPilot.Control.Mode == AUTOPILOT_MODE_ATTI))
        {
          ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(POS_HOLD_MODE, GetSticksInAutoPilotPosition(POS_HOLD_DEADBAND));
        }
      }
      if (Get_Multirotor_GPS_FlightModes_Once())
      {
        if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
        {
          GPS_Resources.Mode.Navigation = DO_START_RTH;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          RESET_THIS_FLIGHT_MODE_ONCE(HEADING_HOLD_MODE);
          Multirotor_Do_Mode_RTH_Now();
        }
        else if (IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE))
        {
          GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
          Do_RTH_Or_Land_Call_Alt_Hold = false;
          Do_Pos_Hold_Call_Alt_Hold = true;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          RESET_THIS_FLIGHT_MODE_ONCE(HEADING_HOLD_MODE);
          SetThisPointToPositionHold();
        }
        else if (IS_FLIGHT_MODE_ACTIVE(LAND_MODE))
        {
          GPS_Resources.Mode.Navigation = DO_LAND_INIT;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          RESET_THIS_FLIGHT_MODE_ONCE(HEADING_HOLD_MODE);
          SetThisPointToPositionHold();
        }
        else
        {
          Do_Pos_Hold_Call_Alt_Hold = false;
          Do_RTH_Or_Land_Call_Alt_Hold = false;
          DISABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          GPS_Reset_Navigation();
        }
      }
    }
    else
    {
      if (Get_GPS_Used_To_Navigation())
      {
        if (Do_RTH_Or_Land_Call_Alt_Hold)
        {
          SetNewAltitudeToHold(Barometer.INS.Altitude.Estimated);
        }
      }
      GPS_Reset_Navigation();
    }
  }
  else
  {
    Do_RTH_Or_Land_Call_Alt_Hold = false;
    GPS_Reset_Navigation();
  }

  if (IS_FLIGHT_MODE_ACTIVE(ALTITUDE_HOLD_MODE) || Do_Pos_Hold_Call_Alt_Hold || Do_RTH_Or_Land_Call_Alt_Hold)
  {
    if (!Do_Altitude_Hold)
    {
      Do_Altitude_Hold = true;
    }
  }
  else
  {
    Do_Altitude_Hold = false;
  }

  if (!IS_FLIGHT_MODE_ACTIVE(RTH_MODE) && !IS_FLIGHT_MODE_ACTIVE(LAND_MODE))
  {
    Do_RTH_Or_Land_Call_Alt_Hold = false;
  }
}

static void ProcessFlightModesToAirPlane(void)
{
  if (!GetAirPlaneEnabled() || !Get_State_Armed_With_GPS())
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
  {
    GPS_Resources.Mode.Navigation = DO_START_RTH; //INDICA PARA O TECS QUE O RTH SERÁ USADO
    ENABLE_THIS_FLIGHT_MODE(CIRCLE_MODE);         //ATIVA O CONTROLE HORIZONTAL X,Y & HEADING
    ENABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);       //ATIVA O CONTROLE VERTICAL Z
  }
  else
  {
    if (IS_FLIGHT_MODE_ACTIVE(ALTITUDE_HOLD_MODE))
    {
      GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
      ENABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
      SetThisPointToPositionHold();
    }
    else if (IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE))
    {
      GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
      ENABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
      SetThisPointToPositionHold();
    }
    else if (IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE))
    {
      GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
      ENABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
      SetThisPointToPositionHold();
    }
    else
    {
      GPS_Resources.Mode.Navigation = DO_NONE;
      DISABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
    }
  }

  if (Get_GPS_Used_To_Navigation() && !IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    ENABLE_THIS_FLIGHT_MODE(STABILIZE_MODE); //FORÇA O MODO DE ESTABILIZAÇÃO EM MODO DE NAVEGAÇÃO POR GPS
  }
}

void FlightModesUpdate(void)
{
  ProcessFlightModesToMultirotor();
  ProcessFlightModesToAirPlane();
  GPS_Resources.Navigation.AutoPilot.Control.Enabled = Get_GPS_Used_To_Navigation();
}