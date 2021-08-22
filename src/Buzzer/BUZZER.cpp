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

#include "BUZZER.h"
#include "RadioControl/STICKS.h"
#include "EscCalibration/CALIBESC.h"
#include "BatteryMonitor/BATTERY.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Build/BOARDDEFS.h"
#include "SafetyButton/SAFETYBUTTON.h"

BEEPERCLASS BEEPER;

#define BEEPER_COMMAND_STOP 0xFF //INDICA PARA O SISTEMA QUE O BUZZER DEVE PARAR DE TOCAR
#define TIME_TO_OTHERS_BEEPS 2   //SEGUNDOS

bool BuzzerInit = false;
uint8_t SafeToOthersBeeps = 0;
static uint8_t BeeperState = 0;
static uint16_t BeeperPositionArray = 0;
static uint32_t BeeperNextNote = 0;

//PRIMEIRA COLUNA = DURAÇÃO DA NOTA
//SEGUNDA COLUNA = DURAÇÃO DA PAUSA

static const uint8_t AlgorithmInit_Beep[] = {
    19, 5,
    9, 5,
    9, 5,
    19, 5,
    9, 5,
    0, 39,
    19, 5,
    19, 5,
    BEEPER_COMMAND_STOP};

static const uint8_t Arm_Beep[] = {
    245,
    BEEPER_COMMAND_STOP};

static const uint8_t Disarm_Beep[] = {
    15, 5,
    15, 5,
    BEEPER_COMMAND_STOP};

static const uint8_t LowBattery_Beep[] = {
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 5,
    7, 50,
    BEEPER_COMMAND_STOP};

static const uint8_t Success_Beep[] = {
    5, 5,
    5, 5, BEEPER_COMMAND_STOP};

static const uint8_t Fail_Beep[] = {
    20, 15,
    35, 5,
    BEEPER_COMMAND_STOP};

static const uint8_t Calibration_Beep[] = {
    18, 8,
    18, 8,
    18, 8,
    BEEPER_COMMAND_STOP};

static const uint8_t AutoLaunch_Beep[] = {
    5, 5,
    5, 100,
    BEEPER_COMMAND_STOP};

static const uint8_t Launched_Beep[] = {
    245,
    BEEPER_COMMAND_STOP};

static const uint8_t FMU_Init_Beep[] = {
    5, 5,
    5, 5,
    BEEPER_COMMAND_STOP};

static const uint8_t FMU_Safe_Beep[] = {
    5, 5,
    15, 5,
    5, 5,
    15, 30,
    BEEPER_COMMAND_STOP};

static const uint8_t Fail_Safe_Beep[] = {
    50, 50,
    BEEPER_COMMAND_STOP};

static const uint8_t Fail_Safe_Good_Beep[] = {
    30, 8,
    30, 8,
    30, 8,
    90, 50,
    BEEPER_COMMAND_STOP};

static const uint8_t Parachute_Beep[] = {
    7, 2,
    7, 2,
    7, 2,
    7, 2,
    7, 2,
    7, 2,
    7, 2,
    7, 50,
    BEEPER_COMMAND_STOP};

const BeeperEntry_Struct BeeperTable[] = {
    {BEEPER_CALIBRATION_DONE, 0, Calibration_Beep},
    {BEEPER_DISARMING, 1, Disarm_Beep},
    {BEEPER_BATT_CRIT_LOW, 2, LowBattery_Beep},
    {BEEPER_ACTION_SUCCESS, 3, Success_Beep},
    {BEEPER_ACTION_FAIL, 4, Fail_Beep},
    {BEEPER_ARM, 5, Arm_Beep},
    {BEEPER_ALGORITHM_INIT, 6, AlgorithmInit_Beep},
    {BEEPER_AUTO_LAUNCH, 7, AutoLaunch_Beep},
    {BEEPER_LAUNCHED, 8, Launched_Beep},
    {BEEPER_FMU_INIT, 9, FMU_Init_Beep},
    {BEEPER_FMU_SAFE_TO_ARM, 10, FMU_Safe_Beep},
    {BEEPER_FAIL_SAFE, 11, Fail_Safe_Beep},
    {BEEPER_FAIL_SAFE_GOOD, 12, Fail_Safe_Good_Beep},
    {BEEPER_PARACHUTE, 13, Parachute_Beep},
};

static const BeeperEntry_Struct *BeeperEntry = NULL;

#define BEEPER_TABLE_ENTRY_COUNT (sizeof(BeeperTable) / sizeof(BeeperEntry_Struct))

void BEEPERCLASS::Play(Beeper_Mode Mode)
{
  if (SAFETYBUTTON.DispositivesPassives == OFF_ALL_DISP ||
      SAFETYBUTTON.DispositivesPassives == ONLY_SWITCH)
  {
    return;
  }
  const BeeperEntry_Struct *SelectedSong = NULL;
  for (uint8_t IndexCount = 0; IndexCount < BEEPER_TABLE_ENTRY_COUNT; IndexCount++)
  {
    const BeeperEntry_Struct *SelectedSongTable = &BeeperTable[IndexCount];
    if (SelectedSongTable->Mode != Mode)
    {
      continue;
    }
    if (!BeeperEntry)
    {
      SelectedSong = SelectedSongTable;
      break;
    }
    if (SelectedSongTable->Priority < BeeperEntry->Priority)
    {
      SelectedSong = SelectedSongTable;
    }
    break;
  }
  if (!SelectedSong)
  {
    return;
  }
  BeeperEntry = SelectedSong;
  BeeperPositionArray = 0;
  BeeperNextNote = 0;
}

void BEEPERCLASS::Silence(void)
{
  BEEP_OFF;
  BeeperState = 0;
  BeeperNextNote = 0;
  BeeperPositionArray = 0;
  BeeperEntry = NULL;
}

void BEEPERCLASS::Update(void)
{
  if (BeeperEntry == NULL)
  {
    return;
  }
  if (BeeperNextNote > SCHEDULERTIME.GetMicros() / 1000)
  {
    return;
  }
  if (!BeeperState)
  {
    BeeperState = 1;
    if (BeeperEntry->Sequence[BeeperPositionArray] != 0)
    {
      BEEP_ON;
    }
  }
  else
  {
    BeeperState = 0;
    if (BeeperEntry->Sequence[BeeperPositionArray] != 0)
    {
      BEEP_OFF;
    }
  }
  BEEPER.ProcessCommand();
}

void BEEPERCLASS::ProcessCommand(void)
{
  if (BeeperEntry->Sequence[BeeperPositionArray] == BEEPER_COMMAND_STOP)
  {
    BEEPER.Silence();
  }
  else
  {
    BeeperNextNote = SCHEDULERTIME.GetMillis() + 10 * BeeperEntry->Sequence[BeeperPositionArray];
    BeeperPositionArray++;
  }
}

bool BEEPERCLASS::GetSafeToOthersBeeps(void)
{
  return (SafeToOthersBeeps >= (10 * TIME_TO_OTHERS_BEEPS));
}

void BEEPERCLASS::UpdateSafeToOthersBeepsCounter(void)
{
  if (BuzzerInit && SafeToOthersBeeps < (10 * TIME_TO_OTHERS_BEEPS))
  {
    SafeToOthersBeeps++;
  }
}

void BEEPERCLASS::Run(void)
{
  if (SAFETYBUTTON.DispositivesPassives == OFF_ALL_DISP ||
      SAFETYBUTTON.DispositivesPassives == ONLY_SWITCH)
  {
    SafeToOthersBeeps = 0xFF;
    BEEPER.Silence();
    return;
  }

  if (!BuzzerInit)
  {
    BEEP_PINOUT;
#if defined ESP32
    AnalogWriteSetSettings(GPIO_NUM_18, 490, 12);
#endif
    BEEPER.Play(BEEPER_ALGORITHM_INIT);
    BuzzerInit = true;
  }

  BEEPER.Update();
}
