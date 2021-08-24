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

#include "SWITCHFLAG.h"
#include "RadioControl/DECODE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "ServosMaster/SERVOSMASTER.h"
#include "Compass/COMPASSREAD.h"
#include "BitArray/BITARRAY.h"
#include "IMU/ACCGYROREAD.h"
#include "PerformanceCalibration/PERFORMACC.h"

//***********************************************************************************************
//ATIVAÇÃO PARA O CALIBRÇÃO DO MAG & SERVO AUTO-TRIM DOS SERVOS VIA CANAL AUX
//
//O CANAL 5 É USADO PARA ESSE MODO
//
//CALIBRAÇÃO DO MAG IMPLEMENTADO >> 28/06/2020 (8 TOQUES PARA ATIVAR)
//SERVO AUTO-TRIM IMPLEMENTADO   >> 01/02/2021 (4 TOQUES PARA ATIVAR,2 TOQUES PARA DESATIVAR)
//
//OBS:
//CALIB MAG SÓ FUNCIONA COM A CONTROLADORA DESARMADA
//SERVO AUTO-TRIM SÓ FUNCIONA COM A CONTROLADORA ARMADA E EM VOO COM O PERFIL DE AERO
//***********************************************************************************************

uint8_t FlagParameterFunction;
uint8_t GuardValue;

float CloseReset;

uint32_t TimerFunction;
uint32_t CR_Clear;

static void Switch_Flag_Clear(void)
{
  if (GuardValue == 1 && CloseReset == 0)
  {
    GuardValue = 0;
  }
  else if (GuardValue == 2 && CloseReset == 0)
  {
    GuardValue = 0;
  }
  else if (GuardValue == 3 && CloseReset == 0)
  {
    GuardValue = 0;
  }
  else if (GuardValue == 5 && CloseReset == 0)
  {
    GuardValue = 0;
  }
  else if (GuardValue == 7 && CloseReset == 0)
  {
    GuardValue = 0;
  }
  else if (GuardValue == 9 && CloseReset == 0)
  {
    GuardValue = 0;
  }
  else if (GuardValue == 10 && CloseReset == 0)
  {
    GuardValue = 0;
  }
  else if (GuardValue == 11 && CloseReset == 0)
  {
    GuardValue = 0;
  }
  else if (GuardValue == 13 && CloseReset == 0)
  {
    GuardValue = 0;
  }
}

void Switch_Flag_Update(void)
{
  const bool GetAuxChannelFlagState = DECODE.GetRxChannelOutput(AUX1) > 1400;
  const bool GetInAirPlaneMode = GetAirPlaneEnabled();

  //INICIA A CONTAGEM DA FLAG PRINCIPAL
  if (GetAuxChannelFlagState)
  {
    if ((SCHEDULERTIME.GetMillis() - TimerFunction) > 50) //DEBOUNCE
    {
      FlagParameterFunction += 1;
      if (GuardValue >= 4 && GuardValue <= 12)
      {
        GuardValue += 1;
      }
    }
    CloseReset = 5; //5 SEGUNDOS
    TimerFunction = SCHEDULERTIME.GetMillis();
  }

  //DELAY PARA RESETAR A FLAG PRINCIPAL
  if (CloseReset > 0 && (SCHEDULERTIME.GetMillis() - CR_Clear) > 100)
  {
    CloseReset -= 0.10f;
    CR_Clear = SCHEDULERTIME.GetMillis();
  }

  if (CloseReset < 0)
  {
    CloseReset = 0; //EVITA GUARDAR VALORES NEGATIVOS CAUSADO PELA DECREMENTAÇÃO ACIMA
  }

  //RESETA A FLAG SE O VALOR DELA FOR IGUAL A 8,E A CHAVE AUX DO MODO SIMPLES FOR FALSA
  if (FlagParameterFunction == 8 && !GetAuxChannelFlagState)
  {
    FlagParameterFunction = 0;
  }

  //ESPERA A DECREMENTAÇÃO DA VARIAVEL ACABAR E RESETA A FLAG PRINCIPAL
  if (!GetAuxChannelFlagState && CloseReset == 0)
  {
    FlagParameterFunction = 0;
  }

  //FLAG PRINCIPAL IGUAL A 4?CHAVE AUX ATIVADA?CAL DO MAG ACABOU?SIM...GUARDE O VALOR DA FLAG PRINCIPAL NA VARIAVEL "GUARDVALUE"
  if (FlagParameterFunction == 4 && GetAuxChannelFlagState && !Calibration.Magnetometer.Calibrating)
  {
    GuardValue = FlagParameterFunction;
  }

  //FLAG PRINCIPAL IGUAL A 8?CHAVE AUX ATIVADA?SIM...GUARDE O VALOR DA FLAG PRINCIPAL NA VARIAVEL "GUARDVALUE"
  if (FlagParameterFunction == 8 && GetAuxChannelFlagState)
  {
    GuardValue = FlagParameterFunction;
  }

  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM)) //CONTROLADORA ARMADA?SIM...
  {
    //O VALOR GUARDADO É IGUAL A 4?E A DECREMENTAÇÃO ACABOU?SIM...INICIA O SERVO AUTO-TRIM
    if (GuardValue == 4 && CloseReset < 2.51f && GetInAirPlaneMode)
    {
      Servo.AutoTrim.Enabled = true;
    }

    //O VALOR GUARDADO É IGUAL A 6?E A DECREMENTAÇÃO ACABOU?SIM...DESATIVA O SERVO AUTO-TRIM
    if (GuardValue == 6 && CloseReset < 2.51f && GetInAirPlaneMode)
    {
      Servo.AutoTrim.Enabled = false;
      GuardValue = 0;
    }
  }
  else //CONTROLADORA DESARMADA?SIM...
  {
    //O VALOR GUARDADO É IGUAL A 8?E A DECREMENTAÇÃO ACABOU?SIM...INICIA A CALIBRAÇÃO DO COMPASS
    if (GuardValue == 8 && CloseReset > 2.0f && CloseReset < 4.0f)
    {
      Calibration.Magnetometer.Calibrating = true;
    }

    //RESETA O VALOR GUARDADO NA FLAG
    if (GuardValue == 8 && CloseReset == 2.0f)
    {
      GuardValue = 0;
    }

    //LIMPA A FLAG SE O USUARIO REJEITAR OS VALORES DO SERVO AUTO-TRIM
    if (GuardValue == 4 && CloseReset < 2.51f && GetInAirPlaneMode && Servo.AutoTrim.Enabled)
    {
      GuardValue = 0;
    }
  }

  //O VALOR GUARDADO É IGUAL A 12?E A DECREMENTAÇÃO ACABOU?SIM...SE O SERVO AUTO-TRIM ESTIVER ATIVADO NÃO LIMPA A FLAG,CASO CONTRARIO LIMPA
  if (GuardValue == 12 && CloseReset == 0)
  {
    if (Servo.AutoTrim.Enabled)
    {
      GuardValue = 4;
    }
    else
    {
      GuardValue = 0;
    }
  }

  //LIMPA A FLAG QUANDO O VALOR DA MESMA FOR UM QUE NÃO É UTILIZAVEL
  Switch_Flag_Clear();
}
