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

#ifndef DJINAZAGPS_H_
#define DJINAZAGPS_H_
#include <inttypes.h>
#include "math.h"
#include "HardwareSerial.h"
#include "wiring_private.h"
//VARIAVEIS DE SAÍDA
extern uint8_t DJINaza_Num_Sat;
extern uint8_t DJINaza_Fix_State;
extern uint16_t DJINaza_HDOP;
extern int16_t DJINaza_Compass_Roll;
extern int16_t DJINaza_Compass_Pitch;
extern int16_t DJINaza_Compass_Yaw;
extern int32_t DJINaza_Latitude;
extern int32_t DJINaza_Longitude;
extern int32_t DJINaza_Altitude;
extern int32_t DJINaza_GroundCourse;
extern int32_t DJINaza_GroundSpeed;
void DjiNazaGpsNewFrame(uint8_t SerialReceiverBuffer);
#endif
