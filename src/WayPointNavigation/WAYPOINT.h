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

#ifndef WAYPOINT_H_
#define WAYPOINT_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
#include "BAR/BAR.h"
typedef struct
{
  int32_t LatitudeOne;
  int32_t LongitudeOne;
  uint8_t AltitudeOne;
  uint8_t FlightModeOne;
  uint8_t GPSHoldTimedOne;
  int32_t LatitudeTwo;
  int32_t LongitudeTwo;
  uint8_t AltitudeTwo;
  uint8_t FlightModeTwo;
  uint8_t GPSHoldTimedTwo;
  int32_t LatitudeThree;
  int32_t LongitudeThree;
  uint8_t AltitudeThree;
  uint8_t FlightModeThree;
  uint8_t GPSHoldTimedThree;
  int32_t LatitudeFour;
  int32_t LongitudeFour;
  uint8_t AltitudeFour;
  uint8_t FlightModeFour;
  uint8_t GPSHoldTimedFour;
  int32_t LatitudeFive;
  int32_t LongitudeFive;
  uint8_t AltitudeFive;
  uint8_t FlightModeFive;
  uint8_t GPSHoldTimedFive;
  void Reset(void)
  {
    LatitudeOne = 0;
    LongitudeOne = 0;
    AltitudeOne = 0;
    FlightModeOne = 0;
    GPSHoldTimedOne = 0;
    LatitudeTwo = 0;
    LongitudeTwo = 0;
    AltitudeTwo = 0;
    FlightModeTwo = 0;
    GPSHoldTimedTwo = 0;
    LatitudeThree = 0;
    LongitudeThree = 0;
    AltitudeThree = 0;
    FlightModeThree = 0;
    GPSHoldTimedThree = 0;
    LatitudeFour = 0;
    LongitudeFour = 0;
    AltitudeFour = 0;
    FlightModeFour = 0;
    GPSHoldTimedFour = 0;
    LatitudeFive = 0;
    LongitudeFive = 0;
    AltitudeFive = 0;
    FlightModeFive = 0;
    GPSHoldTimedFive = 0;
  }
} _GetWayPointPacketOne;

typedef struct
{
  int32_t LatitudeSix;
  int32_t LongitudeSix;
  uint8_t AltitudeSix;
  uint8_t FlightModeSix;
  uint8_t GPSHoldTimedSix;
  int32_t LatitudeSeven;
  int32_t LongitudeSeven;
  uint8_t AltitudeSeven;
  uint8_t FlightModeSeven;
  uint8_t GPSHoldTimedSeven;
  int32_t LatitudeEight;
  int32_t LongitudeEight;
  uint8_t AltitudeEight;
  uint8_t FlightModeEight;
  uint8_t GPSHoldTimedEight;
  int32_t LatitudeNine;
  int32_t LongitudeNine;
  uint8_t AltitudeNine;
  uint8_t FlightModeNine;
  uint8_t GPSHoldTimedNine;
  int32_t LatitudeTen;
  int32_t LongitudeTen;
  uint8_t AltitudeTen;
  uint8_t FlightModeTen;
  uint8_t GPSHoldTimedTen;
  void Reset(void)
  {
    LatitudeSix = 0;
    LongitudeSix = 0;
    AltitudeSix = 0;
    FlightModeSix = 0;
    GPSHoldTimedSix = 0;
    LatitudeSeven = 0;
    LongitudeSeven = 0;
    AltitudeSeven = 0;
    FlightModeSeven = 0;
    GPSHoldTimedSeven = 0;
    LatitudeEight = 0;
    LongitudeEight = 0;
    AltitudeEight = 0;
    FlightModeEight = 0;
    GPSHoldTimedEight = 0;
    LatitudeNine = 0;
    LongitudeNine = 0;
    AltitudeNine = 0;
    FlightModeNine = 0;
    GPSHoldTimedNine = 0;
    LatitudeTen = 0;
    LongitudeTen = 0;
    AltitudeTen = 0;
    FlightModeTen = 0;
    GPSHoldTimedTen = 0;
  }
} _GetWayPointPacketTwo;

#define COORDINATES_ADDR_TO_COMPARE ((INITIAL_ADDR_OF_COORDINATES + FINAL_ADDR_OF_COORDINATES) / 2)
#define GPS_HOLD_TIMED_ADDR_TO_COMPARE (INITIAL_ADDR_OF_OTHERS_PARAMS + WAYPOINTS_MAXIMUM)
#define FLIGHT_MODE_ADDR_TO_COMPARE ((WAYPOINTS_MAXIMUM * (WAYPOINT_OTHERS_PARAMS_MAXIMUM - 1)) + INITIAL_ADDR_OF_OTHERS_PARAMS)
#define ALTITUDE_ADDR_TO_COMPARE ((WAYPOINTS_MAXIMUM * WAYPOINT_OTHERS_PARAMS_MAXIMUM) + INITIAL_ADDR_OF_OTHERS_PARAMS)

enum WayPoint_States_Enum
{
  WAYPOINT_INIT = 0,
  WAYPOINT_RUN_TAKEOFF,
  WAYPOINT_SET_ALTITUDE,
  WAYPOINT_START_MISSION,
  WAYPOINT_MISSION_ENROUTE
};

enum WayPoint_FlightModes_Enum
{
  WAYPOINT_ADVANCE = 1,
  WAYPOINT_TIMED,
  WAYPOINT_LAND,
  WAYPOINT_RTH,
  WAYPOINT_TAKEOFF
};

enum WayPointOnce_Enum
{
  WAYPOINT_PREDICT_POS_Z = 0,
  WAYPOINT_PREDICT_POS_XY,
  WAYPOINT_RESET_POS_Z,
  SIZE_OF_WAYPOINT_ONCE
};

enum WayPointMisc_Enum
{
  WAYPOINT_ENABLE_AUTO_TAKEOFF = 0,
  WAYPOINT_DISABLE_AUTO_TAKEOFF,
  WAYPOINT_NORMALIZE_TAKEOFF,
  WAYPOINT_NORMALIZE_RESET
};

enum WayPointStorage_Enum
{
  WAYPOINT_STORAGE_NONE = 0,
  WAYPOINT_STORAGE_RESET,
  WAYPOINT_STORAGE_SAVE
};

typedef struct
{

  struct AutoTakeOff_Struct
  {

    struct Flags_Struct
    {
      bool State = false;
      bool Normalized = false;
    } Flags;

    struct Throttle_Struct
    {
      uint8_t IncrementCount = 0;
      int16_t Increment = 1000;
    } Throttle;

  } AutoTakeOff;

  struct Mission_Struct
  {

    struct Flags_Struct
    {
      bool Reached = false;
      bool OnceFlight[SIZE_OF_WAYPOINT_ONCE];
    } Flags;

    struct Coordinates_Struct
    {
      int32_t Latitude[WAYPOINTS_MAXIMUM];
      int32_t Longitude[WAYPOINTS_MAXIMUM];
    } Coordinates;

    struct OthersParams_Struct
    {
      uint8_t Altitude[WAYPOINTS_MAXIMUM];
      uint8_t FlightMode[WAYPOINTS_MAXIMUM];
      uint8_t PositionHoldTime[WAYPOINTS_MAXIMUM];
      uint8_t Number = 0;
      uint8_t Mode = WAYPOINT_INIT;
      uint32_t PositionHoldTimeToCompare = 0;
    } OthersParams;

  } Mission;

  struct Storage_Struct
  {
    uint8_t Function = WAYPOINT_STORAGE_NONE;
    uint8_t ArrayCount = 0;
    uint8_t GetArrayCount(void)
    {
      return ArrayCount;
    }
    void IncrementArrayCount(void)
    {
      ArrayCount++;
    }
    void ResetArrayCount(void)
    {
      ArrayCount = 0;
    }
  } Storage;

} WayPoint_Resources_Struct;

extern WayPoint_Resources_Struct WayPoint_Resources;
extern _GetWayPointPacketOne GetWayPointPacketOne;
extern _GetWayPointPacketTwo GetWayPointPacketTwo;

class WayPointClass
{
public:
  void Initialization(void);
  void Erase(void);
  void Update(void);
};
extern WayPointClass WAYPOINT;
#endif
