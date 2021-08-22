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

#include "NAVIGATIONGEO.h"
#include "NAVIGATION.h"
#include "Math/MATHSUPPORT.h"
#include "Yaw/HEADINGHOLD.h"
#include "Param/PARAM.h"
#include "PID/GPSPID.h"
#include "PID/PIDPARAMS.h"

#define MIN_NAVIGATION_SPEED 100 //100CM/S ~ 3.6KM/M -> VELOCIDADE MINIMA SUPORTADA
#define CROSSTRACK_ERROR 0.4     //TESTAR COM 1 FUTURAMENTE

void GPS_Adjust_Heading(void)
{
    GPS_Resources.Navigation.HeadingHoldTarget = WRap_18000(GPS_Resources.Navigation.Bearing.ActualTarget) / 100;
}

void GPS_Calcule_Bearing(int32_t InputLatitude, int32_t InputLongitude, int32_t *Bearing)
{
    int32_t Adjust_OffSet_Lat = (InputLatitude - GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE]) / GPS_Resources.ScaleDownOfLongitude;
    int32_t Adjust_OffSet_Long = InputLongitude - GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE];
    *Bearing = 9000 + Fast_Atan2(-Adjust_OffSet_Lat, Adjust_OffSet_Long) * 5729.57795f;
    if (*Bearing < 0)
    {
        *Bearing += 36000;
    }
}

void GPS_Calcule_Distance_In_CM(int32_t InputLatitude, int32_t InputLongitude, int32_t *CalculateDistance)
{
    float DistanceOfLatitude = (float)(GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE] - InputLatitude);
    float DistanceOfLongitude = (float)(GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE] - InputLongitude) * GPS_Resources.ScaleDownOfLongitude;
    *CalculateDistance = SquareRootU32Bits(SquareFloat(DistanceOfLatitude) + SquareFloat(DistanceOfLongitude)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
}

void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance)
{
    GPS_Resources.Home.INS.Distance[COORD_LATITUDE] = (GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE] - GPS_Resources.Home.Coordinates[COORD_LATITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
    GPS_Resources.Home.INS.Distance[COORD_LONGITUDE] = (GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE] - GPS_Resources.Home.Coordinates[COORD_LONGITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * GPS_Resources.ScaleDownOfLongitude;
    *CalculateDistance = SquareRootU32Bits(Square32Bits(GPS_Resources.Home.INS.Distance[COORD_LATITUDE]) + Square32Bits(GPS_Resources.Home.INS.Distance[COORD_LONGITUDE]));
}

int16_t Calculate_Navigation_Speed(int16_t Maximum_Velocity)
{
    static int16_t Coordinates_Navigation_Speed = MIN_NAVIGATION_SPEED;
    Maximum_Velocity = MIN(Maximum_Velocity, GPS_Resources.Navigation.Coordinates.Distance);
    Maximum_Velocity = MAX(Maximum_Velocity, MIN_NAVIGATION_SPEED);
    if (Maximum_Velocity > Coordinates_Navigation_Speed)
    {
        Coordinates_Navigation_Speed += (int16_t)(100.0f * GPS_Resources.DeltaTime.Navigation);
        Maximum_Velocity = Coordinates_Navigation_Speed;
    }
    return Maximum_Velocity;
}

void GPS_Calcule_Velocity(void)
{
    static int16_t Previous_Velocity[2] = {0, 0};
    static int32_t Last_CoordinatesOfGPS[2] = {0, 0};
    static bool IgnoreFirstPeak = false;
    if (IgnoreFirstPeak)
    {
        float DeltaTimeStored;
        if (GPS_Resources.DeltaTime.Navigation >= 0.07f && GPS_Resources.DeltaTime.Navigation <= 0.13f)
        {
            DeltaTimeStored = 0.1f;
        }
        else if (GPS_Resources.DeltaTime.Navigation >= 0.17f && GPS_Resources.DeltaTime.Navigation <= 0.23f)
        {
            DeltaTimeStored = 0.2f;
        }
        else
        {
            DeltaTimeStored = GPS_Resources.DeltaTime.Navigation;
        }
        DeltaTimeStored = 1.0f / DeltaTimeStored;
        GPS_Resources.Navigation.Speed[COORD_LATITUDE] = (float)(GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE] - Last_CoordinatesOfGPS[COORD_LATITUDE]) * DeltaTimeStored;
        GPS_Resources.Navigation.Speed[COORD_LONGITUDE] = (float)(GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE] - Last_CoordinatesOfGPS[COORD_LONGITUDE]) * GPS_Resources.ScaleDownOfLongitude * DeltaTimeStored;
        GPS_Resources.Navigation.Speed[COORD_LATITUDE] = (GPS_Resources.Navigation.Speed[COORD_LATITUDE] + Previous_Velocity[COORD_LATITUDE]) / 2;
        GPS_Resources.Navigation.Speed[COORD_LONGITUDE] = (GPS_Resources.Navigation.Speed[COORD_LONGITUDE] + Previous_Velocity[COORD_LONGITUDE]) / 2;
        Previous_Velocity[COORD_LATITUDE] = GPS_Resources.Navigation.Speed[COORD_LATITUDE];
        Previous_Velocity[COORD_LONGITUDE] = GPS_Resources.Navigation.Speed[COORD_LONGITUDE];
    }
    IgnoreFirstPeak = true;
    Last_CoordinatesOfGPS[COORD_LATITUDE] = GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE];
    Last_CoordinatesOfGPS[COORD_LONGITUDE] = GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE];
}

bool GetWaypointMissed(void)
{
    int32_t TargetCalculed;
    TargetCalculed = GPS_Resources.Navigation.Bearing.ActualTarget - GPS_Resources.Navigation.Bearing.TargetPrev;
    TargetCalculed = WRap_18000(TargetCalculed);
    return (ABS(TargetCalculed) > 10000); //BEARING PASSOU DE 100°
}

void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput)
{
    GPS_Resources.ScaleDownOfLongitude = Constrain_Float(Fast_Cosine(ConvertToRadians((ConvertCoordinateToFloatingPoint(LatitudeVectorInput)))), 0.01f, 1.0f);
}

int16_t GPS_Update_CrossTrackError(void)
{
    float TargetCalculed = (GPS_Resources.Navigation.Bearing.ActualTarget - GPS_Resources.Navigation.Bearing.TargetPrev) * 0.000174532925f;
    return Fast_Sine(TargetCalculed) * GPS_Resources.Navigation.Coordinates.Distance;
}

void GPSCalculateNavigationRate(uint16_t Maximum_Velocity)
{
    float Trigonometry[2];
    float NavCompensation = 0.0f;
    int32_t Target_Speed[2];
    int16_t Cross_Speed = GPS_Update_CrossTrackError() * CROSSTRACK_ERROR;
    Cross_Speed = Constrain_16Bits(Cross_Speed, -200, 200);
    Cross_Speed = -Cross_Speed;
    float TargetCalculed = (9000L - GPS_Resources.Navigation.Bearing.ActualTarget) * 0.000174532925f;
    Trigonometry[COORD_LATITUDE] = Fast_Sine(TargetCalculed);
    Trigonometry[COORD_LONGITUDE] = Fast_Cosine(TargetCalculed);
    Target_Speed[COORD_LATITUDE] = Cross_Speed * Trigonometry[COORD_LONGITUDE] + Maximum_Velocity * Trigonometry[COORD_LATITUDE];
    Target_Speed[COORD_LONGITUDE] = Maximum_Velocity * Trigonometry[COORD_LONGITUDE] - Cross_Speed * Trigonometry[COORD_LATITUDE];
    for (uint8_t IndexCount = 0; IndexCount < 2; IndexCount++)
    {
        GPS_Resources.Navigation.RateError[IndexCount] = Target_Speed[IndexCount] - GPS_Resources.Navigation.Speed[IndexCount];
        GPS_Resources.Navigation.RateError[IndexCount] = Constrain_16Bits(GPS_Resources.Navigation.RateError[IndexCount], -1000, 1000);
        GPS_Resources.Navigation.AutoPilot.INS.Angle[IndexCount] = GPSGetProportional(GPS_Resources.Navigation.RateError[IndexCount], &NavigationPID) +
                                                                   GPSGetIntegral(GPS_Resources.Navigation.RateError[IndexCount], GPS_Resources.DeltaTime.Navigation, &NavigationPIDArray[IndexCount], &NavigationPID) +
                                                                   GPSGetDerivative(GPS_Resources.Navigation.RateError[IndexCount], GPS_Resources.DeltaTime.Navigation, &NavigationPIDArray[IndexCount], &NavigationPID);

        if (JCF_Param.GPS_TiltCompensation != 0)
        {
            NavCompensation = Target_Speed[IndexCount] * Target_Speed[IndexCount] * ((float)JCF_Param.GPS_TiltCompensation * 0.0001f);
            if (Target_Speed[IndexCount] < 0)
            {
                NavCompensation = -NavCompensation;
            }
        }
        else
        {
            NavCompensation = 0;
        }
        GPS_Resources.Navigation.AutoPilot.INS.Angle[IndexCount] = Constrain_16Bits(GPS_Resources.Navigation.AutoPilot.INS.Angle[IndexCount] + NavCompensation, -ConvertDegreesToDecidegrees(GET_SET[NAV_GPS_BANK_MAX].MaxValue), ConvertDegreesToDecidegrees(GET_SET[NAV_GPS_BANK_MAX].MaxValue));
        PositionHoldRatePIDArray[IndexCount].GPSFilter.IntegralSum = NavigationPIDArray[IndexCount].GPSFilter.IntegralSum;
    }
}