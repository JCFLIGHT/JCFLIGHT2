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

#include "AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "ParamsToGCS/SETFLIGHTMODES.h"
#include "BAR/BAR.h"
#include "FailSafe/FAILSAFE.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Common/ENUM.h"
#include "Common/RCDEFINES.h"
#include "RadioControl/DECODE.h"
#include "BitArray/BITARRAY.h"

//***********************************************************
//CONFIGURAÇÃO DAS CHAVES AUXILIARES PARA OS MODOS DE VOO
//***********************************************************

AUXFLIGHTCLASS AUXFLIGHT;

uint8_t Channel_Low[MAX_AUX_CHANNELS];
uint8_t Channel_Middle[MAX_AUX_CHANNELS];
uint8_t Channel_High[MAX_AUX_CHANNELS];
uint8_t Channel_Levels_Count = 0;

void AUXFLIGHTCLASS::Initialization(void)
{
  AUXFLIGHT.GetModeConfiguration[SIMPLE_MODE] = STORAGEMANAGER.Read_8Bits(SIMPLE_ADDR);
  AUXFLIGHT.GetModeConfiguration[ALTITUDE_HOLD_MODE] = STORAGEMANAGER.Read_8Bits(ALT_HOLD_ADDR);
  AUXFLIGHT.GetModeConfiguration[POS_HOLD_MODE] = STORAGEMANAGER.Read_8Bits(GPS_HOLD_ADDR);
  AUXFLIGHT.GetModeConfiguration[RTH_MODE] = STORAGEMANAGER.Read_8Bits(RTH_ADDR);
  AUXFLIGHT.GetModeConfiguration[STABILIZE_MODE] = STORAGEMANAGER.Read_8Bits(STABLIZE_ADDR);
  AUXFLIGHT.GetModeConfiguration[ATTACK_MODE] = STORAGEMANAGER.Read_8Bits(ATTACK_ADDR);
  AUXFLIGHT.GetModeConfiguration[PARACHUTE_MODE] = STORAGEMANAGER.Read_8Bits(PARACHUTE_ADDR);
  AUXFLIGHT.GetModeConfiguration[FLIP_MODE] = STORAGEMANAGER.Read_8Bits(AUTOFLIP_ADDR);
  AUXFLIGHT.GetModeConfiguration[GIMBAL_MODE] = STORAGEMANAGER.Read_8Bits(GIMBAL_ADDR);
  SetPlatformType(STORAGEMANAGER.Read_8Bits(FRAME_TYPE_ADDR));
  AUXFLIGHT.GetModeConfiguration[WAYPOINT_MODE] = STORAGEMANAGER.Read_8Bits(AUTOMISSION_ADDR);
  AUXFLIGHT.GetModeConfiguration[LAND_MODE] = STORAGEMANAGER.Read_8Bits(AUTOLAND_ADDR);
  AUXFLIGHT.GetModeConfiguration[SECONDARY_ARM_DISARM] = STORAGEMANAGER.Read_8Bits(ARMDISARM_ADDR);
}

bool GetFlightModeState(uint8_t _Channel)
{

  if (_Channel == 0) //NO GCS O BOX EM "NENHUM" É IGUAL A ZERO
  {
    return false;
  }

  uint8_t ConfiguredChannel = 0;

  for (uint8_t IndexCount = 0; IndexCount <= (MAX_AUX_CHANNELS * 3); IndexCount++)
  {
    if (IndexCount == _Channel)
    {
      ConfiguredChannel = IndexCount / 3;
      //VERIFICA AS CASAS DECIMAIS
      if (((float)IndexCount / 3 >= (ConfiguredChannel + .1f)) && ((float)IndexCount / 3 <= (ConfiguredChannel + .9f)))
      {
        ConfiguredChannel += 1;
      }
      break;
    }
  }

  if (Channel_Levels_Count < MAX_AUX_CHANNELS)
  {

    if (Channel_Levels_Count == 0)
    {
      Channel_Low[Channel_Levels_Count] = 1;
      Channel_Middle[Channel_Levels_Count] = 2;
      Channel_High[Channel_Levels_Count] = 3;
    }
    else
    {
      Channel_Low[Channel_Levels_Count] = Channel_Low[Channel_Levels_Count - 1] + NON_AUX_CHANNEL_COUNT;
      Channel_Middle[Channel_Levels_Count] = Channel_Middle[Channel_Levels_Count - 1] + NON_AUX_CHANNEL_COUNT;
      Channel_High[Channel_Levels_Count] = Channel_High[Channel_Levels_Count - 1] + NON_AUX_CHANNEL_COUNT;
    }

    Channel_Levels_Count++;
    return false; //NÃO ATUALIZA NENHUM MODO DE VOO QUANDO A SOMA ESTIVER EM PROCESSAMENTO
  }

  if (_Channel == Channel_Low[ConfiguredChannel - 1])
  {
    return DECODE.GetRxChannelOutput(ConfiguredChannel + NON_AUX_CHANNEL_COUNT) < MIN_STICKS_PULSE + FLIGHT_MODE_PULSE_OFF_SET;
  }
  else if (_Channel == Channel_Middle[ConfiguredChannel - 1])
  {
    return DECODE.GetRxChannelOutput(ConfiguredChannel + NON_AUX_CHANNEL_COUNT) > (MIDDLE_STICKS_PULSE - FLIGHT_MODE_PULSE_OFF_SET) &&
           DECODE.GetRxChannelOutput(ConfiguredChannel + NON_AUX_CHANNEL_COUNT) < (MIDDLE_STICKS_PULSE + FLIGHT_MODE_PULSE_OFF_SET);
  }
  else if (_Channel == Channel_High[ConfiguredChannel - 1])
  {
    return DECODE.GetRxChannelOutput(ConfiguredChannel + NON_AUX_CHANNEL_COUNT) > MAX_STICKS_PULSE - FLIGHT_MODE_PULSE_OFF_SET;
  }

  return false;
}

void AUXFLIGHTCLASS::FlightModesAuxSelect(void)
{
  SetFlightModeToGCS();

  if (SystemInFailSafe())
  {
    return;
  }

  AUXFLIGHT.GetModeState[SECONDARY_ARM_DISARM] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[SECONDARY_ARM_DISARM]);
  AUXFLIGHT.GetModeState[STABILIZE_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[STABILIZE_MODE]);
  AUXFLIGHT.GetModeState[ALTITUDE_HOLD_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[ALTITUDE_HOLD_MODE]);
  AUXFLIGHT.GetModeState[RTH_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[RTH_MODE]);
  AUXFLIGHT.GetModeState[WAYPOINT_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[WAYPOINT_MODE]);
  AUXFLIGHT.GetModeState[SIMPLE_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[SIMPLE_MODE]);
  AUXFLIGHT.GetModeState[POS_HOLD_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[POS_HOLD_MODE]);
  AUXFLIGHT.GetModeState[LAND_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[LAND_MODE]);
  AUXFLIGHT.GetModeState[ATTACK_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[ATTACK_MODE]);
  AUXFLIGHT.GetModeState[FLIP_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[FLIP_MODE]);
  AUXFLIGHT.GetModeState[PARACHUTE_MODE] = GetFlightModeState(AUXFLIGHT.GetModeConfiguration[PARACHUTE_MODE]);

  ENABLE_DISABLE_THIS_STATE_WITH_DEPENDENCY(SECONDARY_ARM_DISARM, AUXFLIGHT.GetModeState[SECONDARY_ARM_DISARM]);
  ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(STABILIZE_MODE, !AUXFLIGHT.GetModeState[STABILIZE_MODE]);
  ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(ALTITUDE_HOLD_MODE, AUXFLIGHT.GetModeState[ALTITUDE_HOLD_MODE]);
  ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(RTH_MODE, AUXFLIGHT.GetModeState[RTH_MODE]);
  ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(WAYPOINT_MODE, AUXFLIGHT.GetModeState[WAYPOINT_MODE]);

  if (GetMultirotorEnabled())
  {
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(SIMPLE_MODE, AUXFLIGHT.GetModeState[SIMPLE_MODE]);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(POS_HOLD_MODE, AUXFLIGHT.GetModeState[POS_HOLD_MODE]);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(LAND_MODE, AUXFLIGHT.GetModeState[LAND_MODE]);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(ATTACK_MODE, AUXFLIGHT.GetModeState[ATTACK_MODE]);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(FLIP_MODE, AUXFLIGHT.GetModeState[FLIP_MODE]);
  }
  else if (GetAirPlaneEnabled())
  {
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(MANUAL_MODE, AUXFLIGHT.GetModeState[SIMPLE_MODE]);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(CIRCLE_MODE, AUXFLIGHT.GetModeState[POS_HOLD_MODE]);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(CRUISE_MODE, AUXFLIGHT.GetModeState[LAND_MODE]);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(LAUNCH_MODE, AUXFLIGHT.GetModeState[ATTACK_MODE]);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(TURN_MODE, AUXFLIGHT.GetModeState[FLIP_MODE]);
  }
}

void AUXFLIGHTCLASS::Update(void)
{
  AUXFLIGHT.FlightModesAuxSelect();
}