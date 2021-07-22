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

#include "RCCONFIG.h"
#include "RadioControl/DECODE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "SBUS/SBUSREAD.h"
#include "IBUS/IBUSREAD.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "Common/RCDEFINES.h"
#include "Common/ENUM.h"
#include "PID/RCPID.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//INSTANCIAS
RC_Config Throttle;
RC_Config Yaw;
RC_Config Pitch;
RC_Config Roll;
RC_Config AuxiliarOne;
RC_Config AuxiliarTwo;
RC_Config AuxiliarThree;
RC_Config AuxiliarFour;
RC_Config AuxiliarFive;
RC_Config AuxiliarSix;
RC_Config AuxiliarSeven;
RC_Config AuxiliarEight;

RCConfigClass RCCONFIG;

void RC_Config::Set_Range(int16_t Min, int16_t Max)
{
  RC_Config::_Max_Pulse = Max;
  RC_Config::_Min_Pulse = Min;
}

void RC_Config::Set_Pulse(int16_t ChannelInputValue)
{
  //SMALL FILTERING NOS CANAIS (APENAS UMA MÉDIA ENTRE O VALOR ATUAL E O ANTERIOR)
  RC_Config::Input = (ChannelInputValue + RC_Config::Input) >> 1;

  if (!RC_Config::_FailSafe)
  {
    RC_Config::_Fail_Safe = false;
  }
  else
  {
    if (RC_Config::Input > (int16_t)CHECKSUM.GetFailSafeValue)
    {
      RC_Config::_Fail_Safe = false;
    }
    else if (RC_Config::Input < (int16_t)CHECKSUM.GetFailSafeValue)
    {
      RC_Config::_Fail_Safe = true;
    }
  }
  RC_Config::Output = RC_Config::Get_Channel_Range();
}

void RC_Config::Set_Reverse(bool Reverse)
{
  RC_Config::_Reverse = Reverse;
}

void RC_Config::Set_Dead_Zone(uint8_t DeadZone)
{
  //NÃO APLICA A ZONA MORTA NO SBUS E IBUS
  if (RC_Resources.ReceiverTypeEnabled != PPM_RECEIVER)
  {
    RC_Config::_DeadZone = 0;
  }
  else
  {
    RC_Config::_DeadZone = Constrain_8Bits(DeadZone, 0, 50);
  }
}

void RC_Config::Set_Fail_Safe(bool FailSafe)
{
  RC_Config::_FailSafe = FailSafe;
}

int16_t RC_Config::Get_Channel_Range(void)
{
  if (!RC_Config::_Fail_Safe)
  {
    RC_Config::RcConstrain = Constrain_16Bits(RC_Config::Input, RC_Config::Min_Pulse, RC_Config::Max_Pulse);
  }
  else
  {
    RC_Config::RcConstrain = Constrain_16Bits(RC_Config::Input, RANGE_MIN, RC_Config::Max_Pulse);
  }
  if (RC_Config::_Reverse)
  {
    RC_Config::RcConstrain = RC_Config::Max_Pulse - (RC_Config::RcConstrain - RC_Config::Min_Pulse);
  }
  if ((RC_Config::Input >= 1450 + RC_Config::_DeadZone) && (RC_Config::Input <= 1550 - RC_Config::_DeadZone) && (RC_Config::_DeadZone > 0))
  {
    return MIDDLE_STICKS_PULSE;
  }
  if (RC_Config::RcConstrain > RC_Config::Min_Pulse)
  {
    return (RC_Config::_Min_Pulse + ((int32_t)(RC_Config::_Max_Pulse - RC_Config::_Min_Pulse) * (int32_t)(RC_Config::RcConstrain - RC_Config::Min_Pulse)) / (int32_t)(RC_Config::Max_Pulse - RC_Config::Min_Pulse));
  }
  if (!RC_Config::_Fail_Safe)
  {
    return RC_Config::_Min_Pulse;
  }
  else
  {
    return RC_Config::RcConstrain;
  }
}

void RCConfigClass::Initialization(void)
{
  //PULSO MINIMO E MAXIMO PARA TODOS OS CANAIS RÁDIO
  Throttle.Min_Pulse = STORAGEMANAGER.Read_16Bits(THROTTLE_MIN_ADDR);
  Yaw.Min_Pulse = STORAGEMANAGER.Read_16Bits(YAW_MIN_ADDR);
  Pitch.Min_Pulse = STORAGEMANAGER.Read_16Bits(PITCH_MIN_ADDR);
  Roll.Min_Pulse = STORAGEMANAGER.Read_16Bits(ROLL_MIN_ADDR);
  Throttle.Max_Pulse = STORAGEMANAGER.Read_16Bits(THROTTLE_MAX_ADDR);
  Yaw.Max_Pulse = STORAGEMANAGER.Read_16Bits(YAW_MAX_ADDR);
  Pitch.Max_Pulse = STORAGEMANAGER.Read_16Bits(PITCH_MAX_ADDR);
  Roll.Max_Pulse = STORAGEMANAGER.Read_16Bits(ROLL_MAX_ADDR);
  AuxiliarOne.Min_Pulse = MIN_PULSE;
  AuxiliarOne.Max_Pulse = MAX_PULSE;
  AuxiliarTwo.Min_Pulse = MIN_PULSE;
  AuxiliarTwo.Max_Pulse = MAX_PULSE;
  AuxiliarThree.Min_Pulse = MIN_PULSE;
  AuxiliarThree.Max_Pulse = MAX_PULSE;
  AuxiliarFour.Min_Pulse = MIN_PULSE;
  AuxiliarFour.Max_Pulse = MAX_PULSE;
  AuxiliarFive.Min_Pulse = MIN_PULSE;
  AuxiliarFive.Max_Pulse = MAX_PULSE;
  AuxiliarSix.Min_Pulse = MIN_PULSE;
  AuxiliarSix.Max_Pulse = MAX_PULSE;
  AuxiliarSeven.Min_Pulse = MIN_PULSE;
  AuxiliarSeven.Max_Pulse = MAX_PULSE;
  AuxiliarEight.Min_Pulse = MIN_PULSE;
  AuxiliarEight.Max_Pulse = MAX_PULSE;
  //CONFIGURAÇÃO DE TODOS OS CANAIS DO RÁDIO
  //THROTTLE
  Throttle.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  Throttle.Set_Dead_Zone(STORAGEMANAGER.Read_8Bits(THROTTLE_DZ_ADDR)); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (ALTITUDE-HOLD)
  Throttle.Set_Fail_Safe(true);
  //YAW
  Yaw.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  Yaw.Set_Dead_Zone(STORAGEMANAGER.Read_8Bits(YAW_DZ_ADDR)); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (ATTITUDE)
  Yaw.Set_Fail_Safe(false);
  //PITCH
  Pitch.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  Pitch.Set_Dead_Zone(STORAGEMANAGER.Read_8Bits(PITCH_DZ_ADDR)); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (ATTITUDE)
  Pitch.Set_Fail_Safe(false);
  //ROLL
  Roll.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  Roll.Set_Dead_Zone(STORAGEMANAGER.Read_8Bits(ROLL_DZ_ADDR)); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (ATTITUDE)
  Roll.Set_Fail_Safe(false);
  //AUX1
  AuxiliarOne.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarOne.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarOne.Set_Fail_Safe(false);
  //AUX2
  AuxiliarTwo.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarTwo.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarTwo.Set_Fail_Safe(false);
  //AUX3
  AuxiliarThree.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarThree.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarThree.Set_Fail_Safe(false);
  //AUX4
  AuxiliarFour.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarFour.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarFour.Set_Fail_Safe(false);
  //AUX5
  AuxiliarFive.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarFive.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarFive.Set_Fail_Safe(false);
  //AUX6
  AuxiliarSix.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarSix.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarSix.Set_Fail_Safe(false);
  //AUX7
  AuxiliarSeven.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarSeven.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarSeven.Set_Fail_Safe(false);
  //AUX8
  AuxiliarEight.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarEight.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarEight.Set_Fail_Safe(false);
  //FAZ AS PRIMEIRAS LEITURAS DOS CANAIS PARA A CALIBRAÇÃO DOS ESC'S
  //CORRE AS FUNÇÕES 100 VEZES PARA OBTER OS VALORES ATUAIS DOS CANAIS DO RADIO
  for (uint8_t IndexCount = 0; IndexCount < 100; IndexCount++)
  {
    DECODE.Update();            //FAZ AS PRIMEIRA LEITURAS
    RCCONFIG.Set_Pulse();       //APLICA OS VALORES LIDOS
    RCCONFIG.Update_Channels(); //FAZ A LEITURA DOS CANAIS APÓS A CONFIGURAÇÃO
    SBUSRC.Update();            //FAZ A LEITURA DOS CANAIS DADO PELA COMUNICAÇÃO SBUS
    IBUSRC.Update();            //FAZ A LEITURA DOS CANAIS DADO PELA COMUNICAÇÃO IBUS
  }
}

void RCConfigClass::Set_Pulse(void)
{
  Throttle.Set_Pulse(DECODE.DirectRadioControlRead[THROTTLE]);
  Yaw.Set_Pulse(DECODE.DirectRadioControlRead[YAW]);
  Pitch.Set_Pulse(DECODE.DirectRadioControlRead[PITCH]);
  Roll.Set_Pulse(DECODE.DirectRadioControlRead[ROLL]);
  AuxiliarOne.Set_Pulse(DECODE.DirectRadioControlRead[AUX1]);
  AuxiliarTwo.Set_Pulse(DECODE.DirectRadioControlRead[AUX2]);
  AuxiliarThree.Set_Pulse(DECODE.DirectRadioControlRead[AUX3]);
  AuxiliarFour.Set_Pulse(DECODE.DirectRadioControlRead[AUX4]);
  AuxiliarFive.Set_Pulse(DECODE.DirectRadioControlRead[AUX5]);
  AuxiliarSix.Set_Pulse(DECODE.DirectRadioControlRead[AUX6]);
  AuxiliarSeven.Set_Pulse(DECODE.DirectRadioControlRead[AUX7]);
  AuxiliarEight.Set_Pulse(DECODE.DirectRadioControlRead[AUX8]);
}

void RCConfigClass::Update_Channels(void)
{
  DECODE.SetRxChannelInput(THROTTLE, Throttle.Output);
  DECODE.SetRxChannelInput(YAW, Yaw.Output);
  DECODE.SetRxChannelInput(PITCH, Pitch.Output);
  DECODE.SetRxChannelInput(ROLL, Roll.Output);
  DECODE.SetRxChannelInput(AUX1, AuxiliarOne.Output);
  DECODE.SetRxChannelInput(AUX2, AuxiliarTwo.Output);
  DECODE.SetRxChannelInput(AUX3, AuxiliarThree.Output);
  DECODE.SetRxChannelInput(AUX4, AuxiliarFour.Output);
  DECODE.SetRxChannelInput(AUX5, AuxiliarFive.Output);
  DECODE.SetRxChannelInput(AUX6, AuxiliarSix.Output);
  DECODE.SetRxChannelInput(AUX7, AuxiliarSeven.Output);
  DECODE.SetRxChannelInput(AUX8, AuxiliarEight.Output);
}
