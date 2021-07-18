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

#include "WAYPOINT.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "GPSNavigation/NAVIGATION.h"
#include "RadioControl/RCCONFIG.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/STICKS.h"
#include "Scheduler/SCHEDULER.h"
#include "Yaw/HEADINGHOLD.h"
#include "PID/RCPID.h"
#include "FlightModes/FLIGHTMODES.h"
#include "Param/PARAM.h"
#include "GPSNavigation/NAVIGATIONGEO.h"
#include "GPSNavigation/INSNAVIGATION.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Barometer/BAROBACKEND.h"
#include "RadioControl/DECODE.h"
#include "BitArray/BITARRAY.h"
#include "AutoLaunch/AUTOLAUNCH.h"
#include "TECS/TECS.h"

WayPointClass WAYPOINT;
WayPoint_Resources_Struct WayPoint_Resources;
_GetWayPointPacketOne GetWayPointPacketOne;
_GetWayPointPacketTwo GetWayPointPacketTwo;

#define THROTTLE_TAKEOFF_ASCENT 1600    //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF ATÉ CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_TAKEOFF_NORMALIZE 1500 //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF AO CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_CANCEL_TAKEOFF 1450    //VALOR DO THROTTLE LIDO DO RECEPTOR PARA CANCELAR O AUTO-TAKEOFF E VOLTAR AO CONTROLE NORMAL
#define THROTTLE_INCREMENT 100          //NÚMERO DE INCREMENTAÇÕES A CADA ESTOURO DE TEMPO DEFINIDO PELO PARAMETRO "THROTTLE_INCREMENT_TIME"
#define THROTTLE_INCREMENT_TIME 1       //INCREMENTA A CADA 0.10 SEGUNDOS

void WayPointClass::Initialization(void)
{
  for (int16_t AddressCount = INITIAL_ADDR_OF_COORDINATES; AddressCount <= FINAL_ADDR_OF_COORDINATES; AddressCount += sizeof(int32_t))
  {
    if (AddressCount < COORDINATES_ADDR_TO_COMPARE)
    {
      WayPoint_Resources.Mission.Coordinates.Latitude[WayPoint_Resources.Storage.ArrayCount] = STORAGEMANAGER.Read_32Bits(AddressCount);
    }
    else
    {
      WayPoint_Resources.Mission.Coordinates.Longitude[WayPoint_Resources.Storage.ArrayCount] = STORAGEMANAGER.Read_32Bits(AddressCount);
    }

    WayPoint_Resources.Storage.ArrayCount++;

    if (WayPoint_Resources.Storage.ArrayCount >= WAYPOINTS_MAXIMUM)
    {
      WayPoint_Resources.Storage.ArrayCount = 0;
    }
  }

  WayPoint_Resources.Storage.ArrayCount = 0;

  for (int16_t AddressCount = INITIAL_ADDR_OF_OTHERS_PARAMS; AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount += sizeof(uint8_t))
  {
    if (AddressCount < GPS_HOLD_TIMED_ADDR_TO_COMPARE)
    {
      WayPoint_Resources.Mission.OthersParams.PositionHoldTime[WayPoint_Resources.Storage.ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }
    else if (AddressCount >= GPS_HOLD_TIMED_ADDR_TO_COMPARE && AddressCount < (FLIGHT_MODE_ADDR_TO_COMPARE - 1))
    {
      WayPoint_Resources.Mission.OthersParams.FlightMode[WayPoint_Resources.Storage.ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }
    else if (AddressCount >= FLIGHT_MODE_ADDR_TO_COMPARE && AddressCount < (ALTITUDE_ADDR_TO_COMPARE - 1))
    {
      WayPoint_Resources.Mission.OthersParams.Altitude[WayPoint_Resources.Storage.ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }

    WayPoint_Resources.Storage.ArrayCount++;

    if (WayPoint_Resources.Storage.ArrayCount >= WAYPOINTS_MAXIMUM)
    {
      WayPoint_Resources.Storage.ArrayCount = 0;
    }
  }
}

static void Push_WayPoint_Parameters(void)
{
  //NÃO VAMOS ZERAR AS VARIAVEIS,TALVEZ CONTÉM ALGO NA EEPROM
  if (GetWayPointPacketOne.LatitudeOne == 0 || GetWayPointPacketOne.LongitudeOne == 0)
  {
    return;
  }

  //OBTÉM TODAS AS LATITUDES DE CADA WAYPOINT
  WayPoint_Resources.Mission.Coordinates.Latitude[0] = GetWayPointPacketOne.LatitudeOne;
  WayPoint_Resources.Mission.Coordinates.Latitude[1] = GetWayPointPacketOne.LatitudeTwo;
  WayPoint_Resources.Mission.Coordinates.Latitude[2] = GetWayPointPacketOne.LatitudeThree;
  WayPoint_Resources.Mission.Coordinates.Latitude[3] = GetWayPointPacketOne.LatitudeFour;
  WayPoint_Resources.Mission.Coordinates.Latitude[4] = GetWayPointPacketOne.LatitudeFive;
  WayPoint_Resources.Mission.Coordinates.Latitude[5] = GetWayPointPacketTwo.LatitudeSix;
  WayPoint_Resources.Mission.Coordinates.Latitude[6] = GetWayPointPacketTwo.LatitudeSeven;
  WayPoint_Resources.Mission.Coordinates.Latitude[7] = GetWayPointPacketTwo.LatitudeEight;
  WayPoint_Resources.Mission.Coordinates.Latitude[8] = GetWayPointPacketTwo.LatitudeNine;
  WayPoint_Resources.Mission.Coordinates.Latitude[9] = GetWayPointPacketTwo.LatitudeTen;

  //OBTÉM TODAS AS LONGITUDES DE CADA WAYPOINT
  WayPoint_Resources.Mission.Coordinates.Longitude[0] = GetWayPointPacketOne.LongitudeOne;
  WayPoint_Resources.Mission.Coordinates.Longitude[1] = GetWayPointPacketOne.LongitudeTwo;
  WayPoint_Resources.Mission.Coordinates.Longitude[2] = GetWayPointPacketOne.LongitudeThree;
  WayPoint_Resources.Mission.Coordinates.Longitude[3] = GetWayPointPacketOne.LongitudeFour;
  WayPoint_Resources.Mission.Coordinates.Longitude[4] = GetWayPointPacketOne.LongitudeFive;
  WayPoint_Resources.Mission.Coordinates.Longitude[5] = GetWayPointPacketTwo.LongitudeSix;
  WayPoint_Resources.Mission.Coordinates.Longitude[6] = GetWayPointPacketTwo.LongitudeSeven;
  WayPoint_Resources.Mission.Coordinates.Longitude[7] = GetWayPointPacketTwo.LongitudeEight;
  WayPoint_Resources.Mission.Coordinates.Longitude[8] = GetWayPointPacketTwo.LongitudeNine;
  WayPoint_Resources.Mission.Coordinates.Longitude[9] = GetWayPointPacketTwo.LongitudeTen;

  //OBTÉM A ALTITUDE DE SUBIDA DE CADA WAYPOINT
  WayPoint_Resources.Mission.OthersParams.Altitude[0] = GetWayPointPacketOne.AltitudeOne;
  WayPoint_Resources.Mission.OthersParams.Altitude[1] = GetWayPointPacketOne.AltitudeTwo;
  WayPoint_Resources.Mission.OthersParams.Altitude[2] = GetWayPointPacketOne.AltitudeThree;
  WayPoint_Resources.Mission.OthersParams.Altitude[3] = GetWayPointPacketOne.AltitudeFour;
  WayPoint_Resources.Mission.OthersParams.Altitude[4] = GetWayPointPacketOne.AltitudeFive;
  WayPoint_Resources.Mission.OthersParams.Altitude[5] = GetWayPointPacketTwo.AltitudeSix;
  WayPoint_Resources.Mission.OthersParams.Altitude[6] = GetWayPointPacketTwo.AltitudeSeven;
  WayPoint_Resources.Mission.OthersParams.Altitude[7] = GetWayPointPacketTwo.AltitudeEight;
  WayPoint_Resources.Mission.OthersParams.Altitude[8] = GetWayPointPacketTwo.AltitudeNine;
  WayPoint_Resources.Mission.OthersParams.Altitude[9] = GetWayPointPacketTwo.AltitudeTen;

  //OBTÉM OS MODOS DE VOO DE CADA WAYPOINT
  WayPoint_Resources.Mission.OthersParams.FlightMode[0] = GetWayPointPacketOne.FlightModeOne;
  WayPoint_Resources.Mission.OthersParams.FlightMode[1] = GetWayPointPacketOne.FlightModeTwo;
  WayPoint_Resources.Mission.OthersParams.FlightMode[2] = GetWayPointPacketOne.FlightModeThree;
  WayPoint_Resources.Mission.OthersParams.FlightMode[3] = GetWayPointPacketOne.FlightModeFour;
  WayPoint_Resources.Mission.OthersParams.FlightMode[4] = GetWayPointPacketOne.FlightModeFive;
  WayPoint_Resources.Mission.OthersParams.FlightMode[5] = GetWayPointPacketTwo.FlightModeSix;
  WayPoint_Resources.Mission.OthersParams.FlightMode[6] = GetWayPointPacketTwo.FlightModeSeven;
  WayPoint_Resources.Mission.OthersParams.FlightMode[7] = GetWayPointPacketTwo.FlightModeEight;
  WayPoint_Resources.Mission.OthersParams.FlightMode[8] = GetWayPointPacketTwo.FlightModeNine;
  WayPoint_Resources.Mission.OthersParams.FlightMode[9] = GetWayPointPacketTwo.FlightModeTen;

  //OBTÉM O TEMPO DE VOO DO GPS-HOLD DE CADA WP
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[0] = GetWayPointPacketOne.GPSHoldTimedOne;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[1] = GetWayPointPacketOne.GPSHoldTimedTwo;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[2] = GetWayPointPacketOne.GPSHoldTimedThree;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[3] = GetWayPointPacketOne.GPSHoldTimedFour;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[4] = GetWayPointPacketOne.GPSHoldTimedFive;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[5] = GetWayPointPacketTwo.GPSHoldTimedSix;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[6] = GetWayPointPacketTwo.GPSHoldTimedSeven;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[7] = GetWayPointPacketTwo.GPSHoldTimedEight;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[8] = GetWayPointPacketTwo.GPSHoldTimedNine;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTime[9] = GetWayPointPacketTwo.GPSHoldTimedTen;
}

void WayPointClass::Erase(void)
{
  //OS ÚLTIMOS ENDEREÇOS SÃO OS DE ARMAZENAMENTO DAS ALTITUDES DE SUBIDA DE CADA MISSÃO
  STORAGEMANAGER.Erase(INITIAL_ADDR_OF_COORDINATES, (FINAL_ADDR_OF_OTHERS_PARAMS - WAYPOINTS_MAXIMUM));
  for (int16_t AddressCount = (FINAL_ADDR_OF_OTHERS_PARAMS - WAYPOINTS_MAXIMUM + 1); AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount++)
  {
    //PADRÃO DE 10 METROS PARA AS ALTITUDES DE VOO POR WAYPOINT
    STORAGEMANAGER.Write_8Bits(AddressCount, 10);
  }
}

static void Store_And_Clear_WayPoints(void)
{
  if (WayPoint_Resources.Storage.Function == WAYPOINT_STORAGE_RESET)
  {
    for (uint8_t IndexCount = 0; IndexCount < WAYPOINTS_MAXIMUM; IndexCount++)
    {
      WayPoint_Resources.Mission.Coordinates.Latitude[IndexCount] = 0;
      WayPoint_Resources.Mission.Coordinates.Longitude[IndexCount] = 0;
      WayPoint_Resources.Mission.OthersParams.FlightMode[IndexCount] = 0;
      WayPoint_Resources.Mission.OthersParams.Altitude[IndexCount] = 0;
      WayPoint_Resources.Mission.OthersParams.PositionHoldTime[IndexCount] = 0;
      GetWayPointPacketOne.Reset();
      GetWayPointPacketTwo.Reset();
      WAYPOINT.Erase();
    }
    WayPoint_Resources.Storage.Function = WAYPOINT_STORAGE_NONE;
  }

  if (WayPoint_Resources.Storage.Function == WAYPOINT_STORAGE_SAVE)
  {
    for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++) //REPETE 5 VEZES
    {

      WayPoint_Resources.Storage.ArrayCount = 0;

      for (int16_t AddressCount = INITIAL_ADDR_OF_COORDINATES; AddressCount <= FINAL_ADDR_OF_COORDINATES; AddressCount += sizeof(int32_t))
      {
        if (AddressCount < COORDINATES_ADDR_TO_COMPARE)
        {
          STORAGEMANAGER.Write_32Bits(AddressCount, WayPoint_Resources.Mission.Coordinates.Latitude[WayPoint_Resources.Storage.ArrayCount]);
        }
        else
        {
          STORAGEMANAGER.Write_32Bits(AddressCount, WayPoint_Resources.Mission.Coordinates.Longitude[WayPoint_Resources.Storage.ArrayCount]);
        }

        WayPoint_Resources.Storage.ArrayCount++;

        if (WayPoint_Resources.Storage.ArrayCount >= WAYPOINTS_MAXIMUM)
        {
          WayPoint_Resources.Storage.ArrayCount = 0;
        }
      }

      WayPoint_Resources.Storage.ArrayCount = 0;

      for (int16_t AddressCount = INITIAL_ADDR_OF_OTHERS_PARAMS; AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount += sizeof(uint8_t))
      {
        if (AddressCount < GPS_HOLD_TIMED_ADDR_TO_COMPARE)
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPoint_Resources.Mission.OthersParams.PositionHoldTime[WayPoint_Resources.Storage.ArrayCount]);
        }
        else if (AddressCount >= GPS_HOLD_TIMED_ADDR_TO_COMPARE && AddressCount < (FLIGHT_MODE_ADDR_TO_COMPARE - 1))
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPoint_Resources.Mission.OthersParams.FlightMode[WayPoint_Resources.Storage.ArrayCount]);
        }
        else if (AddressCount >= FLIGHT_MODE_ADDR_TO_COMPARE && AddressCount < (ALTITUDE_ADDR_TO_COMPARE - 1))
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPoint_Resources.Mission.OthersParams.Altitude[WayPoint_Resources.Storage.ArrayCount]);
        }

        WayPoint_Resources.Storage.ArrayCount++;

        if (WayPoint_Resources.Storage.ArrayCount >= WAYPOINTS_MAXIMUM)
        {
          WayPoint_Resources.Storage.ArrayCount = 0;
        }
      }
    }
    WayPoint_Resources.Storage.Function = WAYPOINT_STORAGE_NONE;
  }
}

static bool WayPointSync10Hz(void)
{
  static Scheduler_Struct WayPointSyncTimer;
  return (Scheduler(&WayPointSyncTimer, SCHEDULER_SET_FREQUENCY(10, "Hz")));
}

static void SetWayPointAutoTakeOffState(uint8_t _AutoTakeOffState)
{
  if (_AutoTakeOffState == WAYPOINT_ENABLE_AUTO_TAKEOFF)
  {
    WayPoint_Resources.AutoTakeOff.Flags.State = true;
  }
  if (_AutoTakeOffState == WAYPOINT_DISABLE_AUTO_TAKEOFF)
  {
    WayPoint_Resources.AutoTakeOff.Flags.State = false;
  }
  else if (_AutoTakeOffState == WAYPOINT_NORMALIZE_TAKEOFF)
  {
    WayPoint_Resources.AutoTakeOff.Flags.Normalized = true;
  }
  else if (_AutoTakeOffState == WAYPOINT_NORMALIZE_RESET)
  {
    WayPoint_Resources.AutoTakeOff.Flags.Normalized = false;
  }
}

static bool GetWayPointAutoTakeOffState(void)
{
  return WayPoint_Resources.AutoTakeOff.Flags.State;
}

static bool GetWayPointAutoTakeOffNormalized(void)
{
  return WayPoint_Resources.AutoTakeOff.Flags.Normalized;
}

static void WayPointAutoTakeOffUpdate(void)
{
  if (!GetWayPointAutoTakeOffState())
  {
    return;
  }

  if (GetMultirotorEnabled())
  {
    if (WayPointSync10Hz())
    {
      if (WayPoint_Resources.AutoTakeOff.Throttle.Increment < THROTTLE_TAKEOFF_ASCENT)
      {
        if (WayPoint_Resources.AutoTakeOff.Throttle.IncrementCount >= THROTTLE_INCREMENT_TIME)
        {
          WayPoint_Resources.AutoTakeOff.Throttle.Increment += THROTTLE_INCREMENT;
          WayPoint_Resources.AutoTakeOff.Throttle.IncrementCount = 0;
        }
        else
        {
          WayPoint_Resources.AutoTakeOff.Throttle.IncrementCount++;
        }
      }
      if (GetWayPointAutoTakeOffNormalized())
      {
        WayPoint_Resources.AutoTakeOff.Throttle.Increment = THROTTLE_TAKEOFF_NORMALIZE;
      }
    }
    WayPoint_Resources.AutoTakeOff.Throttle.Increment = Constrain_16Bits(WayPoint_Resources.AutoTakeOff.Throttle.Increment, RC_Resources.Attitude.ThrottleMin, RC_Resources.Attitude.ThrottleMax);
    DECODE.SetRxChannelInput(THROTTLE, WayPoint_Resources.AutoTakeOff.Throttle.Increment);
    RC_Resources.Attitude.Controller[THROTTLE] = WayPoint_Resources.AutoTakeOff.Throttle.Increment;
  }
  else if (GetAirPlaneEnabled())
  {
    ENABLE_THIS_FLIGHT_MODE(LAUNCH_MODE);
  }
}

static void ResetAutoTakeOff(void)
{
  WayPoint_Resources.AutoTakeOff.Throttle.Increment = 1000;
  WayPoint_Resources.AutoTakeOff.Throttle.IncrementCount = 0;
  SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
  SetWayPointAutoTakeOffState(WAYPOINT_NORMALIZE_RESET);
}

static void WayPointPredictPositionAndSetAltitude(void)
{
  if (!WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_Z])
  {
    GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
    Do_Pos_Hold_Call_Alt_Hold = true;
    SetNewAltitudeToHold(ConverMetersToCM(WayPoint_Resources.Mission.OthersParams.Altitude[WayPoint_Resources.Mission.OthersParams.Number]));
    SetThisPointToPositionHold();
    WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_Z] = true;
  }
}

static void ResetWayPointNavigation(void)
{
  WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_INIT;
  WayPoint_Resources.Mission.Flags.Reached = false;
  WayPoint_Resources.Mission.OthersParams.Number = 0;
  WayPoint_Resources.Mission.OthersParams.PositionHoldTimeToCompare = 0;
  WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_Z] = false;
  WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY] = false;
  if (!WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_RESET_POS_Z])
  {
    GPS_Resources.Mode.Navigation = DO_NONE;
    Do_Pos_Hold_Call_Alt_Hold = false;
    //DESTIVA ALGUNS MODOS DE VOO USADO PELO MODO WP
    DISABLE_THIS_FLIGHT_MODE(RTH_MODE);
    DISABLE_THIS_FLIGHT_MODE(CRUISE_MODE);
    DISABLE_THIS_FLIGHT_MODE(CIRCLE_MODE);
    WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_RESET_POS_Z] = true;
  }
}

void WayPointClass::Update(void)
{
  Push_WayPoint_Parameters();
  Store_And_Clear_WayPoints();

  if (!IS_FLIGHT_MODE_ACTIVE(WAYPOINT_MODE))
  {
    ResetWayPointNavigation();
    ResetAutoTakeOff();
    return;
  }

  if (WayPoint_Resources.Mission.Coordinates.Latitude[0] == 0 || WayPoint_Resources.Mission.Coordinates.Longitude[0] == 0)
  {
    return;
  }

  int16_t Navigation_Speed_Result = 0;

  WayPointAutoTakeOffUpdate();

  WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_RESET_POS_Z] = false;

  switch (WayPoint_Resources.Mission.OthersParams.Mode)
  {

  case WAYPOINT_INIT:
    for (uint8_t IndexCount = 0; IndexCount < WAYPOINTS_MAXIMUM; IndexCount++)
    {
      if (WayPoint_Resources.Mission.OthersParams.FlightMode[IndexCount] == WAYPOINT_TAKEOFF)
      {
        WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_RUN_TAKEOFF;
        return;
      }
      else
      {
        WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_SET_ALTITUDE;
      }
    }
    break;

  case WAYPOINT_RUN_TAKEOFF:
    /*
    WayPointPredictPositionAndSetAltitude();
    if (GetAltitudeReached())
    {
      if (GetMultirotorEnabled())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_NORMALIZE_TAKEOFF);
      }
      else if (GetAirPlaneEnabled())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
        DISABLE_THIS_FLIGHT_MODE(LAUNCH_MODE);
      }
      WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_START_MISSION;
    }
    else
    {
      if (GetMultirotorEnabled())
      {
        if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
          SetWayPointAutoTakeOffState(WAYPOINT_ENABLE_AUTO_TAKEOFF);
        }
        else
        {
          STICKS.PreArm_Run = true;
        }
      }
      else if (GetAirPlaneEnabled())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_ENABLE_AUTO_TAKEOFF);
      }
    }
*/
    if (GetMultirotorEnabled())
    {
      WayPointPredictPositionAndSetAltitude();
      if (GetAltitudeReached())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_NORMALIZE_TAKEOFF);
        WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_START_MISSION;
      }
      else
      {
        if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
          SetWayPointAutoTakeOffState(WAYPOINT_ENABLE_AUTO_TAKEOFF);
        }
        else
        {
          STICKS.PreArm_Run = true;
        }
      }
    }
    else if (GetAirPlaneEnabled())
    {
      if (!AUTOLAUNCH.GetLaunchFinalized())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_ENABLE_AUTO_TAKEOFF);
      }
      else
      {
        SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
        DISABLE_THIS_FLIGHT_MODE(LAUNCH_MODE);
        WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_SET_ALTITUDE;
      }
    }
    break;

  case WAYPOINT_SET_ALTITUDE:
    /*
   WayPointPredictPositionAndSetAltitude();
      if (GetAltitudeReached())
      {
        WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_START_MISSION;
      }
*/
    if (GetMultirotorEnabled())
    {
      WayPointPredictPositionAndSetAltitude();
      if (GetAltitudeReached())
      {
        WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_START_MISSION;
      }
    }
    else if (GetAirPlaneEnabled())
    {
      GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
      ENABLE_THIS_FLIGHT_MODE(CRUISE_MODE);
      WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_START_MISSION;
    }
    break;

  case WAYPOINT_START_MISSION:
    /*
    WayPoint_Resources.Mission.OthersParams.PositionHoldTimeToCompare = 0;
    WayPoint_Resources.Mission.Flags.Reached = true;
    WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_Z] = false;
    WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY] = false;
    Set_Next_Point_To_Navigation(WayPoint_Resources.Mission.Coordinates.Latitude[WayPoint_Resources.Mission.OthersParams.Number], WayPoint_Resources.Mission.Coordinates.Longitude[WayPoint_Resources.Mission.OthersParams.Number]);
    WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_MISSION_ENROUTE;
    */
    WayPoint_Resources.Mission.OthersParams.PositionHoldTimeToCompare = 0;
    WayPoint_Resources.Mission.Flags.Reached = true;
    WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_Z] = false;
    WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY] = false;
    Set_Next_Point_To_Navigation(WayPoint_Resources.Mission.Coordinates.Latitude[WayPoint_Resources.Mission.OthersParams.Number], WayPoint_Resources.Mission.Coordinates.Longitude[WayPoint_Resources.Mission.OthersParams.Number]);
    if (GetAirPlaneEnabled())
    {
      RESET_THIS_FLIGHT_MODE_ONCE(CRUISE_MODE); //RESETA O MODO CRUISE PARA OBTER OS NOVOS PARAMETROS DE NAVEGAÇÃO
      TECS_Resources.Position.DestinationNEU.X = INS.Position.Hold[INS_LATITUDE];
      TECS_Resources.Position.DestinationNEU.Y = INS.Position.Hold[INS_LONGITUDE];
      TECS_Resources.Position.DestinationNEU.Altitude = ConverMetersToCM(WayPoint_Resources.Mission.OthersParams.Altitude[WayPoint_Resources.Mission.OthersParams.Number]);
    }
    WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_MISSION_ENROUTE;
    break;

  case WAYPOINT_MISSION_ENROUTE:
    if (GetMultirotorEnabled())
    {
      //CALCULA UMA NOVA VELOCIDADE DE NAVEGAÇÃO
      Navigation_Speed_Result = Calculate_Navigation_Speed(JCF_Param.Navigation_Vel);
      GPSCalculateNavigationRate(Navigation_Speed_Result);
      //DESATIVA O TAKEOFF SE A MISSÃO NÃO ESTIVER CONFIGURADA PARA O MESMO E SE O THROTTLE ESTIVER ACIMA DE UM CERTO NIVEL
      if (WayPoint_Resources.Mission.OthersParams.FlightMode[WayPoint_Resources.Mission.OthersParams.Number] != WAYPOINT_TAKEOFF && Throttle.Output >= THROTTLE_CANCEL_TAKEOFF)
      {
        SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
      }
    }

    GPS_Resources.Navigation.HeadingHoldTarget = WRap_18000(GPS_Resources.Navigation.Bearing.ActualTarget) / 100;
    if ((GPS_Resources.Navigation.Coordinates.Distance <= ConverMetersToCM(JCF_Param.GPS_WP_Radius)) || GetWaypointMissed())
    {
      for (uint8_t MissionCount = 0; MissionCount < WAYPOINTS_MAXIMUM; MissionCount++)
      {
        if (WayPoint_Resources.Mission.Flags.Reached &&
            WayPoint_Resources.Mission.OthersParams.Number == MissionCount &&
            WayPoint_Resources.Mission.Coordinates.Latitude[MissionCount + 1] != 0 &&
            WayPoint_Resources.Mission.Coordinates.Longitude[MissionCount + 1] != 0)
        {
          WayPoint_Resources.Mission.OthersParams.Number = MissionCount + 1;
          WayPoint_Resources.Mission.Flags.Reached = false;
          return;
        }
      }

      //AVANÇA O WAYPOINT
      if (WayPoint_Resources.Mission.OthersParams.FlightMode[WayPoint_Resources.Mission.OthersParams.Number] == WAYPOINT_ADVANCE)
      {
        WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_SET_ALTITUDE;
        Do_RTH_Or_Land_Call_Alt_Hold = false;
        Do_Pos_Hold_Call_Alt_Hold = false;
      }

      //GPS-HOLD TIMERIZADO
      if (WayPoint_Resources.Mission.OthersParams.FlightMode[WayPoint_Resources.Mission.OthersParams.Number] == WAYPOINT_TIMED)
      {
        //CONTAGEM DE TEMPO
        if (WayPointSync10Hz())
        {
          WayPoint_Resources.Mission.OthersParams.PositionHoldTimeToCompare++; //10 ITERAÇÕES = 1 SEGUNDO
        }
        //FAÇA ALGUMAS TAREFAS APENAS 1 VEZ
        if (!WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY])
        {
          GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
          Do_RTH_Or_Land_Call_Alt_Hold = false;
          Do_Pos_Hold_Call_Alt_Hold = false;
          SetThisPointToPositionHold();
          if (GetAirPlaneEnabled())
          {
            ENABLE_THIS_FLIGHT_MODE(CIRCLE_MODE);
            RESET_THIS_FLIGHT_MODE_ONCE(CIRCLE_MODE); //RESETA O MODO CIRCULO PARA OBTER NOVAS COORDENADAS
          }
          WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY] = true;
        }
        //VERIFICAÇÃO DE ESTOURO DE TEMPO
        if (WayPoint_Resources.Mission.OthersParams.PositionHoldTimeToCompare >= ConvertDegreesToDecidegrees(WayPoint_Resources.Mission.OthersParams.PositionHoldTime[WayPoint_Resources.Mission.OthersParams.Number]))
        {
          if (GetAirPlaneEnabled())
          {
            DISABLE_THIS_FLIGHT_MODE(CIRCLE_MODE);
            RESET_THIS_FLIGHT_MODE_ONCE(CRUISE_MODE); //RESETA O MODO CRUISE PARA OBTER NOVAS COORDENADAS
          }
          WayPoint_Resources.Mission.OthersParams.Mode = WAYPOINT_SET_ALTITUDE;
        }
      }

      //LAND
      if (WayPoint_Resources.Mission.OthersParams.FlightMode[WayPoint_Resources.Mission.OthersParams.Number] == WAYPOINT_LAND)
      {
        if (!WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY])
        {
          GPS_Resources.Mode.Navigation = DO_LAND_INIT;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          if (GetMultirotorEnabled())
          {
            ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
            RESET_THIS_FLIGHT_MODE_ONCE(HEADING_HOLD_MODE);
            SetThisPointToPositionHold();
          }
          WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY] = true;
        }
      }

      //RTH
      if (WayPoint_Resources.Mission.OthersParams.FlightMode[WayPoint_Resources.Mission.OthersParams.Number] == WAYPOINT_RTH)
      {
        if (!WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY])
        {
          GPS_Resources.Mode.Navigation = DO_START_RTH;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          if (GetMultirotorEnabled())
          {
            ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
            RESET_THIS_FLIGHT_MODE_ONCE(HEADING_HOLD_MODE);
            //Multirotor_Do_Mode_RTH_Now();
          }
          //else if (GetAirPlaneEnabled())
          {
            ENABLE_THIS_FLIGHT_MODE(RTH_MODE);
          }
          WayPoint_Resources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_XY] = true;
        }
      }
    }
    break;
  }
}