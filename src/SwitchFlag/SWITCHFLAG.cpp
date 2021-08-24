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
#include "Common/STRUCTS.h"

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

#define FLAG_DEBOUNCE_TIME 50
#define FLAG_RESET_TIME 100
#define CH_5_SAFE_US 1400

Switch_Flag_Struct Switch_Flag;

static void Switch_Flag_Clear(void)
{
  if (Switch_Flag.Flag.GuardValue == 1 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
  else if (Switch_Flag.Flag.GuardValue == 2 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
  else if (Switch_Flag.Flag.GuardValue == 3 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
  else if (Switch_Flag.Flag.GuardValue == 5 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
  else if (Switch_Flag.Flag.GuardValue == 7 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
  else if (Switch_Flag.Flag.GuardValue == 9 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
  else if (Switch_Flag.Flag.GuardValue == 10 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
  else if (Switch_Flag.Flag.GuardValue == 11 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
  else if (Switch_Flag.Flag.GuardValue == 13 && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.GuardValue = 0;
  }
}

void Switch_Flag_Update(void)
{
  const bool GetAuxChannelFlagState = DECODE.GetRxChannelOutput(AUX1) > CH_5_SAFE_US;
  const bool GetInAirPlaneMode = GetAirPlaneEnabled();

  //INICIA A CONTAGEM DA FLAG PRINCIPAL
  if (GetAuxChannelFlagState)
  {
    if ((SCHEDULERTIME.GetMillis() - Switch_Flag.Time.PreviousTimeDebounce) > FLAG_DEBOUNCE_TIME) //DEBOUNCE
    {
      Switch_Flag.Flag.Count += 1;
      if (Switch_Flag.Flag.GuardValue >= 4 && Switch_Flag.Flag.GuardValue <= 12)
      {
        Switch_Flag.Flag.GuardValue += 1;
      }
    }
    Switch_Flag.Time.Reset = 5; //5 SEGUNDOS
    Switch_Flag.Time.PreviousTimeDebounce = SCHEDULERTIME.GetMillis();
  }

  //DELAY PARA RESETAR A FLAG PRINCIPAL
  if (Switch_Flag.Time.Reset > 0 && (SCHEDULERTIME.GetMillis() - Switch_Flag.Time.PreviousTimeReset) > FLAG_RESET_TIME)
  {
    Switch_Flag.Time.Reset -= 0.10f;
    Switch_Flag.Time.PreviousTimeReset = SCHEDULERTIME.GetMillis();
  }

  if (Switch_Flag.Time.Reset < 0)
  {
    Switch_Flag.Time.Reset = 0; //EVITA GUARDAR VALORES NEGATIVOS CAUSADO PELA DECREMENTAÇÃO ACIMA
  }

  //RESETA A FLAG SE O VALOR DELA FOR IGUAL A 8,E A CHAVE AUX DO MODO SIMPLES FOR FALSA
  if (Switch_Flag.Flag.Count == 8 && !GetAuxChannelFlagState)
  {
    Switch_Flag.Flag.Count = 0;
  }

  //ESPERA A DECREMENTAÇÃO DA VARIAVEL ACABAR E RESETA A FLAG PRINCIPAL
  if (!GetAuxChannelFlagState && Switch_Flag.Time.Reset == 0)
  {
    Switch_Flag.Flag.Count = 0;
  }

  //FLAG PRINCIPAL IGUAL A 4?CHAVE AUX ATIVADA?CAL DO MAG ACABOU?SIM...GUARDE O VALOR DA FLAG PRINCIPAL NA VARIAVEL "GUARDVALUE"
  if (Switch_Flag.Flag.Count == 4 && GetAuxChannelFlagState && !Calibration.Magnetometer.Calibrating)
  {
    Switch_Flag.Flag.GuardValue = Switch_Flag.Flag.Count;
  }

  //FLAG PRINCIPAL IGUAL A 8?CHAVE AUX ATIVADA?SIM...GUARDE O VALOR DA FLAG PRINCIPAL NA VARIAVEL "GUARDVALUE"
  if (Switch_Flag.Flag.Count == 8 && GetAuxChannelFlagState)
  {
    Switch_Flag.Flag.GuardValue = Switch_Flag.Flag.Count;
  }

  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM)) //CONTROLADORA ARMADA?SIM...
  {
    //O VALOR GUARDADO É IGUAL A 4?E A DECREMENTAÇÃO ACABOU?SIM...INICIA O SERVO AUTO-TRIM
    if (Switch_Flag.Flag.GuardValue == 4 && Switch_Flag.Time.Reset < 2.51f && GetInAirPlaneMode)
    {
      Servo.AutoTrim.Enabled = true;
    }

    //O VALOR GUARDADO É IGUAL A 6?E A DECREMENTAÇÃO ACABOU?SIM...DESATIVA O SERVO AUTO-TRIM
    if (Switch_Flag.Flag.GuardValue == 6 && Switch_Flag.Time.Reset < 2.51f && GetInAirPlaneMode)
    {
      Servo.AutoTrim.Enabled = false;
      Switch_Flag.Flag.GuardValue = 0;
    }
  }
  else //CONTROLADORA DESARMADA?SIM...
  {
    //O VALOR GUARDADO É IGUAL A 8?E A DECREMENTAÇÃO ACABOU?SIM...INICIA A CALIBRAÇÃO DO COMPASS
    if (Switch_Flag.Flag.GuardValue == 8 && Switch_Flag.Time.Reset > 2.0f && Switch_Flag.Time.Reset < 4.0f)
    {
      Calibration.Magnetometer.Calibrating = true;
    }

    //RESETA O VALOR GUARDADO NA FLAG
    if (Switch_Flag.Flag.GuardValue == 8 && Switch_Flag.Time.Reset == 2.0f)
    {
      Switch_Flag.Flag.GuardValue = 0;
    }

    //LIMPA A FLAG SE O USUARIO REJEITAR OS VALORES DO SERVO AUTO-TRIM
    if (Switch_Flag.Flag.GuardValue == 4 && Switch_Flag.Time.Reset < 2.51f && GetInAirPlaneMode && Servo.AutoTrim.Enabled)
    {
      Switch_Flag.Flag.GuardValue = 0;
    }
  }

  //O VALOR GUARDADO É IGUAL A 12?E A DECREMENTAÇÃO ACABOU?SIM...SE O SERVO AUTO-TRIM ESTIVER ATIVADO NÃO LIMPA A FLAG,CASO CONTRARIO LIMPA
  if (Switch_Flag.Flag.GuardValue == 12 && Switch_Flag.Time.Reset == 0)
  {
    if (Servo.AutoTrim.Enabled)
    {
      Switch_Flag.Flag.GuardValue = 4;
    }
    else
    {
      Switch_Flag.Flag.GuardValue = 0;
    }
  }

  //LIMPA A FLAG QUANDO O VALOR DA MESMA FOR UM QUE NÃO É UTILIZAVEL
  Switch_Flag_Clear();
}
