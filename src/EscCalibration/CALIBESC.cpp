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

#include "CALIBESC.h"
#include "MotorsControl/MOTORS.h"
#include "LedRGB/LEDRGB.h"
#include "RadioControl/DECODE.h"
#include "RadioControl/RCCONFIG.h"
#include "Buzzer/BUZZER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSTATES.h"
#include "RadioControl/DECODE.h"
#include "Common/RCDEFINES.h"
#include "FastSerial/PRINTF.h"

ClassESC ESC;

#define ESC_CALIBRATION_THROTTLE_FAIL DECODE.GetRxChannelOutput(THROTTLE) < 1600 //VALOR MAXIMO TOLERADO PARA NÃO ENTRAR NO MODO CALIBRAÇÃO DOS ESC'S

void ClassESC::Calibration(void)
{
  if (ESC_CALIBRATION_THROTTLE_FAIL)
  {
    LOG("O usuario pulou a etapa de calib dos escs.");
    return; //FAÇA UMA RAPIDA SAIDA DA FUNÇÃO CASO O USUARIO NÃO QUEIRA CALIBRAR OS ESC'S
  }

  LOG("Etapa de calib dos escs inicializada!");
  ConfigureRegisters(true); //INICIA OS REGISTRADORES DE CONFIGURAÇÃO DOS PINOS PWM

  while (true) //FICA TRAVADO AQUI NO WHILE ATÉ QUE O SISTEMA SEJA REINICIADO MANUALMENTE
  {
    BEEPER.Run();                                                                                                //TOCA A MÚSICA DE INICIALIZAÇÃO
    RGB.Function(CALL_LED_CALIBRATION_ESC);                                                                      //RGB EM MODO DE INDICAÇÃO DE CALIBRAÇÃO DOS ESC
    RGB.Update();                                                                                                //ATUALIZA O ESTADO DO RGB
    DECODE.Update();                                                                                             //FAZ A LEITURA DE TODOS OS CANAIS DO RECEPTOR DO RADIO
    RCCONFIG.Set_Pulse();                                                                                        //SETA A SAÍDA PARA CONFIGURAÇÃO PARA O RECEPTOR DO RADIO
    RCCONFIG.Update_Channels();                                                                                  //FAZ A LEITURA DOS CANAIS DO RECEPTOR DO RADIO APÓS A CONFIGURAÇÃO
    PulseInAllMotors(Constrain_16Bits(DECODE.GetRxChannelOutput(THROTTLE), MIN_STICKS_PULSE, MAX_STICKS_PULSE)); //REALIZA O BY-PASS DO THROTTLE
    SCHEDULERTIME.Sleep(20);                                                                                     //50HZ LOOP
  }
}