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

#include "REBOOT.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "MotorsControl/MOTORS.h"
#include "BitArray/BITARRAY.h"
#include "Common/ENUM.h"

WatchDogClass WATCHDOG;

#ifdef __AVR_ATmega2560__

static __inline__ __attribute__((__always_inline__)) void WatchDogReset(const uint8_t ResetDelay)
{
    if (_SFR_IO_REG_P(WDTCSR))
    {
        __asm__ __volatile__(
            "in __tmp_reg__,__SREG__"
            "\n\t"
            "cli"
            "\n\t"
            "wdr"
            "\n\t"
            "out %0, %1"
            "\n\t"
            "out __SREG__,__tmp_reg__"
            "\n\t"
            "out %0, %2"
            "\n \t"
            :
            : "I"(_SFR_IO_ADDR(WDTCSR)),
              "r"((uint8_t)(_BV(WDCE) | _BV(WDE))),
              "r"((uint8_t)((ResetDelay & 0x08 ? _BV(WDP3) : 0x00) |
                            _BV(WDE) | (ResetDelay & 0x07)))
            : "r0");
    }
    else
    {
        __asm__ __volatile__(
            "in __tmp_reg__,__SREG__"
            "\n\t"
            "cli"
            "\n\t"
            "wdr"
            "\n\t"
            "sts %0, %1"
            "\n\t"
            "out __SREG__,__tmp_reg__"
            "\n\t"
            "sts %0, %2"
            "\n \t"
            :
            : "n"(_SFR_MEM_ADDR(WDTCSR)),
              "r"((uint8_t)(_BV(WDCE) | _BV(WDE))),
              "r"((uint8_t)((ResetDelay & 0x08 ? _BV(WDP3) : 0x00) |
                            _BV(WDE) | (ResetDelay & 0x07)))
            : "r0");
    }
}

#elif defined __arm__

#endif

void WatchDogClass::Reboot(void)
{
    if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
        return;
    }

    ShutDownAllMotorsAndServos();
    SCHEDULERTIME.Sleep(1000); //ESPERA O MICROCONTROLADOR DOS ESCS REAGIREM AO PULSO EM LOW

#ifdef __AVR_ATmega2560__

    __asm__ __volatile__("cli" ::
                             : "memory");

    WatchDogReset(0); //WATCHDOG 15MS

    while (true)
    {
    }

#elif defined ESP32

    ESP.restart();

#elif defined __arm__

#endif
}