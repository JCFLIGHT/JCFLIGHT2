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

#include "NAVIGATION.h"
#include "PID/PIDPARAMS.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "Declination/AUTODECLINATION.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "PID/GPSPID.h"
#include "Buzzer/BUZZER.h"
#include "GPS/GPSSTATES.h"
#include "Yaw/HEADINGHOLD.h"
#include "GPS/GPSUBLOX.h"
#include "FlightModes/FLIGHTMODES.h"
#include "InertialNavigation/INS.h"
#include "AHRS/AHRS.h"
#include "Barometer/BAROBACKEND.h"
#include "Param/PARAM.h"
#include "NAVIGATIONGEO.h"
#include "INSNAVIGATION.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "BitArray/BITARRAY.h"

GPS_Resources_Struct GPS_Resources;

#define TIME_TO_INIT_LAND 100 //MS
#define GPS_PID_NAV_MAX_INTEGRAL 2000.0f

void Load_GPS_Navigation_Params(void)
{
  GPS_Resources.Home.Altitude = STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR);
  GPS_Resources.Navigation.LandAfterRTH = STORAGEMANAGER.Read_8Bits(LAND_AFTER_RTH_ADDR) > 0 ? true : false;

  PositionHoldPID.kP = (float)GET_SET[PID_GPS_POSITION].kP / 100.0f;
  PositionHoldPID.kI = (float)GET_SET[PID_GPS_POSITION].kI / 100.0f;
  PositionHoldPID.GPSFilter.IntegralMax = GPS_PID_NAV_MAX_INTEGRAL;

  PositionHoldRatePID.kP = (float)GET_SET[PID_GPS_POSITION_RATE].kP / 10.0f;
  PositionHoldRatePID.kI = (float)GET_SET[PID_GPS_POSITION_RATE].kI / 100.0f;
  PositionHoldRatePID.kD = (float)GET_SET[PID_GPS_POSITION_RATE].kD / 100.0f;
  PositionHoldRatePID.GPSFilter.IntegralMax = GPS_PID_NAV_MAX_INTEGRAL;

  NavigationPID.kP = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kP / 10.0f;
  NavigationPID.kI = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kI / 100.0f;
  NavigationPID.kD = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kD / 1000.0f;
  NavigationPID.GPSFilter.IntegralMax = GPS_PID_NAV_MAX_INTEGRAL;
}

void GPS_Reset_Navigation(void)
{
  GPS_Resources.Mode.Navigation = DO_NONE;
  GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] = 0;
  GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] = 0;
  ResetAllPIDOfGPS();
}

static void UpdateMagneticDeclination(void)
{
  //OBTÉM A DECLINAÇÃO MAGNETICA AUTOMATICAMENTE
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && !GPS_Resources.Declination.Pushed)
  {
    AUTODECLINATION.Set_Initial_Location(GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE], GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE]);
    GPS_Resources.Declination.PushedCount++;
  }

  //SALVA O VALOR DA DECLINAÇÃO MAGNETICA NA EEPROM
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) &&      //CHECA SE ESTÁ DESARMADO
      AUTODECLINATION.GetDeclination() != 0 &&     //CHECA SE O VALOR É DIFERENTE DE ZERO
      !GPS_Resources.Declination.Pushed &&         //CHECA SE A DECLINAÇÃO NÃO FOI PUXADA
      GPS_Resources.Declination.PushedCount > 250) //UTILIZA 250 CICLOS DE MAQUINA PARA CALCULAR O VALOR
  {
    STORAGEMANAGER.Write_Float(MAG_DECLINATION_ADDR, AUTODECLINATION.GetDeclination());
    GPS_Resources.Declination.Pushed = true;
  }
}

void GPS_Process_FlightModes(float DeltaTime)
{
  uint32_t CalculateDistance;
  int32_t CalculateDirection;

  //SAIA DA FUNÇÃO SE O GPS ESTIVER RUIM
  if (Get_GPS_In_Bad_Condition())
  {
    GPS_Resources.Navigation.Misc.Velocity.NEDStatus = false;
    return;
  }

  //SAFE PARA RESETAR O HOME-POINT
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    GPS_Resources.Home.Marked = false;
  }
  else //RESETA O HOME-POINT AO ARMAR
  {
    if (!GPS_Resources.Home.Marked)
    {
      GPS_Resources.Home.Coordinates[COORD_LATITUDE] = GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE];
      GPS_Resources.Home.Coordinates[COORD_LONGITUDE] = GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE];
      GPS_Calcule_Longitude_Scaling(GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE]);
      GPS_Resources.Navigation.Bearing.InitialTarget = Attitude.EulerAngles.Yaw;
      GPS_Resources.Home.Marked = true;
    }
  }

  UpdateMagneticDeclination();

  GPS_Resources.DeltaTime.Navigation = DeltaTime;
  GPS_Resources.DeltaTime.Navigation = MIN(GPS_Resources.DeltaTime.Navigation, 1.0f);
  GPS_Calcule_Bearing(GPS_Resources.Home.Coordinates[COORD_LATITUDE], GPS_Resources.Home.Coordinates[COORD_LONGITUDE], &CalculateDirection);
  GPS_Calcule_Distance_To_Home(&CalculateDistance);
  GPS_Calcule_Velocity();

  if (!GPS_Resources.Home.Marked)
  {
    GPS_Resources.Home.Distance = 0;
    GPS_Resources.Home.Direction = 0;
  }
  else
  {
    GPS_Resources.Home.Direction = CalculateDirection / 100; //FUTURAMENTE PARA O GCS E OSD
    GPS_Resources.Home.Distance = CalculateDistance / 100;
  }

  if (GPS_Resources.Navigation.AutoPilot.Control.Enabled && GetMultirotorEnabled())
  {

    GPS_Calcule_Bearing(GPS_Resources.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPS_Resources.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPS_Resources.Navigation.Bearing.ActualTarget);
    GPS_Calcule_Distance_In_CM(GPS_Resources.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPS_Resources.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPS_Resources.Navigation.Coordinates.Distance);

    int16_t CalculateNavigationSpeed = 0;

    switch (GPS_Resources.Mode.Navigation)
    {

    case DO_NONE:
      break;

    case DO_POSITION_HOLD:
      break;

    case DO_START_RTH:
      if (GPS_Resources.Home.Distance <= JCF_Param.GPS_RTH_Land_Radius)
      {
        if (GPS_Resources.Navigation.LandAfterRTH)
        {
          GPS_Resources.Mode.Navigation = DO_LAND_INIT;
        }
        else
        {
          SetNewAltitudeToHold(Barometer.INS.Altitude.Estimated);
          GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
        }
        GPS_Resources.Navigation.HeadingHoldTarget = GPS_Resources.Navigation.Bearing.InitialTarget;
      }
      else if (GetAltitudeReached())
      {
        Set_Next_Point_To_Navigation(GPS_Resources.Home.Coordinates[COORD_LATITUDE], GPS_Resources.Home.Coordinates[COORD_LONGITUDE]);
        GPS_Resources.Mode.Navigation = DO_RTH_ENROUTE;
      }
      break;

    case DO_RTH_ENROUTE:
      CalculateNavigationSpeed = Calculate_Navigation_Speed(JCF_Param.Navigation_Vel);
      GPSCalculateNavigationRate(CalculateNavigationSpeed);
      GPS_Adjust_Heading();
      if ((GPS_Resources.Navigation.Coordinates.Distance <= ConverMetersToCM(JCF_Param.GPS_WP_Radius)) || GetWaypointMissed())
      {
        if (GPS_Resources.Navigation.LandAfterRTH)
        {
          GPS_Resources.Mode.Navigation = DO_LAND_INIT;
        }
        else
        {
          GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
        }
        GPS_Resources.Navigation.HeadingHoldTarget = GPS_Resources.Navigation.Bearing.InitialTarget;
      }
      break;

    case DO_LAND_INIT:
      Do_RTH_Or_Land_Call_Alt_Hold = true;
      SetNewAltitudeToHold(Barometer.INS.Altitude.Estimated);
      GPS_Resources.DeltaTime.InitLand = SCHEDULERTIME.GetMillis() + TIME_TO_INIT_LAND;
      GPS_Resources.Mode.Navigation = DO_LAND_SETTLE;
      break;

    case DO_LAND_SETTLE:
      if (SCHEDULERTIME.GetMillis() >= GPS_Resources.DeltaTime.InitLand)
      {
        GPS_Resources.Mode.Navigation = DO_LAND_DESCENT;
      }
      break;

    case DO_LAND_DESCENT:
      if (GetLanded())
      {
        GPS_Resources.Mode.Navigation = DO_LANDED;
      }
      else if (GetGroundDetected())
      {
        GPS_Resources.Mode.Navigation = DO_LAND_DETECTED;
      }
      break;

    case DO_LAND_DETECTED:
      if (GetLanded())
      {
        GPS_Resources.Mode.Navigation = DO_LANDED;
      }
      break;

    case DO_LANDED:
      DISABLE_THIS_STATE(PRIMARY_ARM_DISARM);
      Do_RTH_Or_Land_Call_Alt_Hold = false;
      BEEPER.Play(BEEPER_ACTION_SUCCESS);
      GPS_Reset_Navigation();
      break;
    }
  }
}

void Multirotor_Do_Mode_RTH_Now(void)
{
  if (Barometer.INS.Altitude.Estimated < ConverMetersToCM(GPS_Resources.Home.Altitude))
  {
    SetNewAltitudeToHold(ConverMetersToCM(GPS_Resources.Home.Altitude));
  }
  else
  {
    SetNewAltitudeToHold(Barometer.INS.Altitude.Estimated);
  }
  SetThisPointToPositionHold();
}

void Set_Next_Point_To_Navigation(int32_t Latitude_Destiny, int32_t Longitude_Destiny)
{
  GPS_Resources.Navigation.Coordinates.Destiny[COORD_LATITUDE] = Latitude_Destiny;
  GPS_Resources.Navigation.Coordinates.Destiny[COORD_LONGITUDE] = Longitude_Destiny;
  GPS_Calcule_Longitude_Scaling(Latitude_Destiny);
  GPS_Calcule_Bearing(GPS_Resources.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPS_Resources.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPS_Resources.Navigation.Bearing.ActualTarget);
  GPS_Calcule_Distance_In_CM(GPS_Resources.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPS_Resources.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPS_Resources.Navigation.Coordinates.Distance);
  INS.Position.Hold[INS_LATITUDE] = (GPS_Resources.Navigation.Coordinates.Destiny[COORD_LATITUDE] - GPS_Resources.Home.Coordinates[COORD_LATITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
  INS.Position.Hold[INS_LONGITUDE] = (GPS_Resources.Navigation.Coordinates.Destiny[COORD_LONGITUDE] - GPS_Resources.Home.Coordinates[COORD_LONGITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * GPS_Resources.ScaleDownOfLongitude;
  GPS_Resources.Navigation.Bearing.TargetPrev = GPS_Resources.Navigation.Bearing.ActualTarget;
}

bool Get_Safe_State_To_Apply_Position_Hold(void)
{
  return GPS_Resources.Mode.Navigation == DO_POSITION_HOLD ||
         GPS_Resources.Mode.Navigation == DO_START_RTH ||
         GPS_Resources.Mode.Navigation == DO_LAND_INIT ||
         GPS_Resources.Mode.Navigation == DO_LAND_SETTLE ||
         GPS_Resources.Mode.Navigation == DO_LAND_DESCENT;
}
