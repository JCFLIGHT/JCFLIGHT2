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

#ifndef BITARRAY_H_
#define BITARRAY_H_
#include <inttypes.h>
bool IS_FLIGHT_MODE_ACTIVE(uint8_t FlightModeName);
bool IS_FLIGHT_MODE_ACTIVE_ONCE(uint8_t FlightModeName);
void RESET_THIS_FLIGHT_MODE_ONCE(uint8_t FlightModeName);
void ENABLE_THIS_FLIGHT_MODE(uint8_t FlightModeName);
void DISABLE_THIS_FLIGHT_MODE(uint8_t FlightModeName);
void ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(uint8_t FlightModeName, bool Dependency);
bool IS_STATE_ACTIVE(uint8_t FlightModeName);
void ENABLE_THIS_STATE(uint8_t FlightModeName);
void DISABLE_THIS_STATE(uint8_t FlightModeName);
void ENABLE_DISABLE_THIS_STATE_WITH_DEPENDENCY(uint8_t FlightModeName, bool Dependency);
#endif