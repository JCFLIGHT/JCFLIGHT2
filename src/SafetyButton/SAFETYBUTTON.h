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

#ifndef SAFETYBUTTON_H_
#define SAFETYBUTTON_h_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/ENUM.h"
class SAFETYBUTTONCLASS
{
public:
  uint8_t DispositivesPassives = OFF_ALL_DISP;
  void Initialization(void);
  void UpdateRoutine(void);
  bool SafeButtonEnabled(void);
  bool GetSafeStateToOutput(void);

private:
  bool GetButtonInterval(void);
  bool GetButtonState(void);
  bool WaitToNextProcess = false;
  bool SafeStateToApplyPulse = false;
  uint8_t DetectRise = 0;
  uint8_t Blink_Counter = 0;
  uint32_t LastDebounceTime = 0;
  void UpdateLedStatus(enum Led_Pattern_Enum Instance);
  void SetStateToLed(bool State);
  void FlashButton(void);
};
extern SAFETYBUTTONCLASS SAFETYBUTTON;
#endif