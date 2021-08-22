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

#include "DECODE.h"
#include "Common/STRUCTS.h"
#include "FlightModes/AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "SBUS/SBUSREAD.h"
#include "IBUS/IBUSREAD.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "BAR/BAR.h"
#include "HAL/LEARNINGRECEIVER.h"
#include "HAL/HALPPM.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "BitArray/BITARRAY.h"
#include "PID/RCPID.h"
#include "RadioControl/CURVESRC.h"
#include "PID/TPA.h"
#include "FastSerial/UART2MODE.h"
#include "RadioControl/RCCONFIG.h"
#include "RadioControl/RCSMOOTH.h"

DecodeClass DECODE;

void DecodeClass::Initialization(void)
{
  RC_Resources.ReceiverSequency = STORAGEMANAGER.Read_8Bits(RC_SEQUENCY_ADDR);
  RC_Resources.ReceiverTypeEnabled = STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR);

  if (RC_Resources.ReceiverTypeEnabled == PPM_RECEIVER)
  {
    HAL_PPM.Initialization();
  }

  //CANAIS FUNDAMENTAIS
  if (RC_Resources.ReceiverSequency == 0) //FlySky FS-i6, FlySky FS-i6s, FlySky FS-i6x, FlySky FS-iA10B, TGY-I6(OU TGY-I6 OU FS-i6 ATUALIZADO PARA 10 CANAIS)
  {
    DECODE.RadioControlChannelsMap[0] = ROLL;
    DECODE.RadioControlChannelsMap[1] = PITCH;
    DECODE.RadioControlChannelsMap[2] = THROTTLE;
    DECODE.RadioControlChannelsMap[3] = YAW;
  }
  else //FUTABA OU D4R-II
  {
    DECODE.RadioControlChannelsMap[0] = PITCH;
    DECODE.RadioControlChannelsMap[1] = ROLL;
    DECODE.RadioControlChannelsMap[2] = THROTTLE;
    DECODE.RadioControlChannelsMap[3] = YAW;
  }

  //CANAIS AUXILIARES
  for (uint8_t IndexCount = AUX1; IndexCount < TOTAL_MAX_CHANNELS; IndexCount++)
  {
    DECODE.RadioControlChannelsMap[IndexCount] = IndexCount;
  }

  CurvesRC_SetValues();
  TPA_Initialization();
  CurvesRC_CalculeValue();
  UART2Mode_Initialization();
  RCCONFIG.Initialization();
  CHECKSUM.UpdateChannelsReverse();
  RCInterpolationInit();
}

void DecodeClass::Update(void)
{
  bool CheckFailSafeState = true;

  for (uint8_t IndexCount = 0; IndexCount < TOTAL_MAX_CHANNELS; IndexCount++)
  {
    uint16_t RadioControllOutputDecoded = LearningChannelsOfReceiver(IndexCount);

    CheckFailSafeState = RC_Resources.ReceiverTypeEnabled == SBUS_RECEIVER ? SBUSRC.FailSafe : RadioControllOutputDecoded > CHECKSUM.GetFailSafeValue;

    if (CheckFailSafeState || !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
      DECODE.DirectRadioControlRead[IndexCount] = RadioControllOutputDecoded;
    }
  }
}

int16_t DecodeClass::GetRxChannelOutput(uint8_t Channel)
{
  return DECODE.RadioControlOutput[Channel];
}

void DecodeClass::SetRxChannelInput(uint8_t Channel, int16_t Value)
{
  DECODE.RadioControlOutput[Channel] = Value;
}