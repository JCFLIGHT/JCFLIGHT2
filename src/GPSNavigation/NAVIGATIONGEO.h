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

#ifndef NAVIGATIONGEO_H_
#define NAVIGATIONGEO_H_
#include <inttypes.h>
//#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR 1.113195f //RETIRADO DA ARDUPILOT ATÉ A VERSÃO 4.0.3
#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR 1.1131884502145034f //RETIRADO DA ARDUPILOT DEPOIS DA VERSÃO 4.0.3
void GPS_Adjust_Heading(void);
void GPS_Calcule_Bearing(int32_t InputLatitude, int32_t InputLongitude, int32_t *Bearing);
void GPS_Calcule_Distance_In_CM(int32_t InputLatitude, int32_t InputLongitude, int32_t *CalculateDistance);
void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance);
int16_t Calculate_Navigation_Speed(int16_t Maximum_Velocity);
void GPS_Calcule_Velocity(void);
bool GetWaypointMissed(void);
void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput);
void GPSCalculateNavigationRate(uint16_t Maximum_Velocity);
#endif