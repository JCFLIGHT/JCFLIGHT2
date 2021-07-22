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

#include "SAFETYBUTTON.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Buzzer/BUZZER.h"
#include "MotorsControl/MOTORS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Build/BOARDDEFS.h"
#include "AnalogDigitalConverter/ADC.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Common/RCDEFINES.h"
#include "BitArray/BITARRAY.h"

SAFETYBUTTONCLASS SAFETYBUTTON;

#define FIRST_CYCLE_COUNT 30  //1 SEGUNDO
#define SECOND_CYCLE_COUNT 60 //1 SEGUNDO
#define RESET_CYCLE_COUNT 150 //3 SEGUNDOS
#define MAX_BUTTON_COUNT 255  //ITERAÇÕES
#define BUTTON_SCHEDULE 33    //33MS - 30HZ

void SAFETYBUTTONCLASS::Initialization(void)
{
    SAFETY_BUTTON_LED_PINOUT;
    SAFETYBUTTON.DispositivesPassives = STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR);
#ifdef ESP32
    AnalogWriteSetSettings(GPIO_NUM_5, 490, 12);
#endif
}

bool SAFETYBUTTONCLASS::GetButtonInterval(void)
{
    if (SCHEDULERTIME.GetMillis() - SAFETYBUTTON.LastDebounceTime < BUTTON_SCHEDULE)
    {
        return false;
    }
    SAFETYBUTTON.LastDebounceTime = SCHEDULERTIME.GetMillis();
    return true;
}

bool SAFETYBUTTONCLASS::GetButtonState(void)
{
    return (ANALOGSOURCE.Read(SAFETY_BUTTON_PIN_READ_STATE) > 0x200 ? false : true);
}

void SAFETYBUTTONCLASS::FlashButton(void)
{
    static Led_Pattern_Enum Pattern;

    if (SAFETYBUTTON.DetectRise < FIRST_CYCLE_COUNT)
    {
        Pattern = Led_Pattern_Enum::FMU_REFUSE_TO_ARM;
    }
    else if (SAFETYBUTTON.DetectRise > FIRST_CYCLE_COUNT && SAFETYBUTTON.DetectRise < SECOND_CYCLE_COUNT) //VERIFICAÇÃO 1
    {
        Pattern = Led_Pattern_Enum::FMU_INIT_ARM;
        if (SAFETYBUTTON.DetectRise == FIRST_CYCLE_COUNT + 1)
        {
            BEEPER.Play(BEEPER_FMU_INIT);
        }
        if (SAFETYBUTTON.GetButtonState())
        {
            SAFETYBUTTON.WaitToNextProcess = true;
            if (GetMultirotorEnabled())
            {
                PulseInAllMotors(MIN_STICKS_PULSE);
            }
            else if (GetAirPlaneEnabled())
            {
                MotorControl[MOTOR1] = MIN_STICKS_PULSE;
                MotorControl[MOTOR2] = MIDDLE_STICKS_PULSE;
                MotorControl[MOTOR3] = MIDDLE_STICKS_PULSE;
                MotorControl[MOTOR4] = MIDDLE_STICKS_PULSE;
                MotorControl[MOTOR5] = MIDDLE_STICKS_PULSE;
                MotorControl[MOTOR6] = MIDDLE_STICKS_PULSE;
                ApplyPWMControlForMotorsAndServos();
            }
        }
        else
        {
            if (!SAFETYBUTTON.WaitToNextProcess)
            {
                SAFETYBUTTON.DetectRise = FIRST_CYCLE_COUNT + 1;
            }
        }
    }
    else if (SAFETYBUTTON.DetectRise > SECOND_CYCLE_COUNT && SAFETYBUTTON.DetectRise < RESET_CYCLE_COUNT) //VERIFICAÇÃO 2
    {
        Pattern = Led_Pattern_Enum::FMU_SAFE_TO_ARM;
        if (SAFETYBUTTON.DetectRise == SECOND_CYCLE_COUNT + 1)
        {
            BEEPER.Play(BEEPER_FMU_SAFE_TO_ARM);
            SAFETYBUTTON.WaitToNextProcess = false;
        }
        if (SAFETYBUTTON.GetButtonState())
        {
            SAFETYBUTTON.WaitToNextProcess = true;
            SAFETYBUTTON.SafeStateToApplyPulse = true;
        }
        else
        {
            if (!SAFETYBUTTON.WaitToNextProcess)
            {
                SAFETYBUTTON.DetectRise = SECOND_CYCLE_COUNT + 1;
            }
        }
    }
    else if (SAFETYBUTTON.DetectRise > RESET_CYCLE_COUNT) //RESET
    {
        if (SAFETYBUTTON.DetectRise == RESET_CYCLE_COUNT + 1)
        {
            BEEPER.Play(BEEPER_ACTION_FAIL);
            SAFETYBUTTON.WaitToNextProcess = false;
        }
        else
        {
            if (SAFETYBUTTON.GetButtonState())
            {
                SAFETYBUTTON.DetectRise = 0;
                if (GetMultirotorEnabled())
                {
                    PulseInAllMotors(DISABLE_IO_PIN);
                }
                else if (GetAirPlaneEnabled())
                {
                    MotorControl[MOTOR1] = DISABLE_IO_PIN;
                    MotorControl[MOTOR2] = MIDDLE_STICKS_PULSE;
                    MotorControl[MOTOR3] = MIDDLE_STICKS_PULSE;
                    MotorControl[MOTOR4] = MIDDLE_STICKS_PULSE;
                    MotorControl[MOTOR5] = MIDDLE_STICKS_PULSE;
                    MotorControl[MOTOR6] = MIDDLE_STICKS_PULSE;
                    ApplyPWMControlForMotorsAndServos();
                }
                SAFETYBUTTON.SafeStateToApplyPulse = false;
            }
            else
            {
                if (SAFETYBUTTON.DetectRise > RESET_CYCLE_COUNT + 1)
                {
                    SAFETYBUTTON.DetectRise = MAX_BUTTON_COUNT - 2;
                }
            }
        }
    }
    SAFETYBUTTON.UpdateLedStatus(Pattern);
}

void SAFETYBUTTONCLASS::UpdateLedStatus(enum Led_Pattern_Enum Instance)
{
    SAFETYBUTTON.SetStateToLed(((uint16_t)Instance & (1 << (SAFETYBUTTON.Blink_Counter++ / 3))));
    if (SAFETYBUTTON.Blink_Counter >= 48)
    {
        SAFETYBUTTON.Blink_Counter = 0;
    }
}

void SAFETYBUTTONCLASS::SetStateToLed(bool State)
{
    if (!State)
    {
        SAFETY_BUTTON_LED_OFF;
    }
    else
    {
        SAFETY_BUTTON_LED_ON;
    }
}

bool SAFETYBUTTONCLASS::GetSafeStateToOutput(void)
{
    if (!SAFETYBUTTON.SafeButtonEnabled())
    {
        return true;
    }
    return SAFETYBUTTON.SafeStateToApplyPulse;
}

bool SAFETYBUTTONCLASS::SafeButtonEnabled(void)
{
    if (SAFETYBUTTON.DispositivesPassives == OFF_ALL_DISP ||
        SAFETYBUTTON.DispositivesPassives == ONLY_BUZZER)
    {
        return false;
    }
    return true;
}

void SAFETYBUTTONCLASS::UpdateRoutine(void)
{
    if (!SAFETYBUTTON.SafeButtonEnabled() || !SAFETYBUTTON.GetButtonInterval() || IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
        return;
    }
    SAFETYBUTTON.FlashButton();
    if (!SAFETYBUTTON.GetButtonState() && SAFETYBUTTON.DetectRise < MAX_BUTTON_COUNT)
    {
        SAFETYBUTTON.DetectRise++;
    }
}
