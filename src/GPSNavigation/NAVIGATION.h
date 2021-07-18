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

#ifndef NAVIGATION_H_
#define NAVIGATION_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern GPS_Resources_Struct GPS_Resources;
void Load_GPS_Navigation_Params(void);
void GPS_Reset_Navigation(void);
void GPS_Process_FlightModes(float DeltaTime);
void Multirotor_Do_Mode_RTH_Now(void);
void Set_Next_Point_To_Navigation(int32_t Latitude_Destiny, int32_t Longitude_Destiny);
bool Get_Safe_State_To_Apply_Position_Hold(void);
#endif
