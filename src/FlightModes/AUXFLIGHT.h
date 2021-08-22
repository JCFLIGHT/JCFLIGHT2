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

#ifndef AUXFLIGHT_H_
#define AUXFLIGHT_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/ENUM.h"

class AUXFLIGHTCLASS
{
public:
  bool GetModeState[SIZE_OF_FLIGHT_MODES];
  uint8_t GetModeConfiguration[SIZE_OF_FLIGHT_MODES];
  uint8_t FlightMode = STABILIZE_MODE;
  void Initialization(void);
  void Update(void);

private:
  void FlightModesAuxSelect(void);
};
extern AUXFLIGHTCLASS AUXFLIGHT;
#endif
