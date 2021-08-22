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

#ifndef RCDEFINES_H_
#define RCDEFINES_H_
#define MAX_AUX_CHANNELS 8                                            //AUX1 ~ AUX8
#define NON_AUX_CHANNEL_COUNT 3                                       //THROTTLE + YAW + PITCH + ROLL
#define RANGE_MIN 900                                                 //uS
#define MIN_PULSE 1100                                                //uS
#define MAX_PULSE 1900                                                //uS
#define MIN_STICKS_PULSE 1000                                         //uS
#define MAX_STICKS_PULSE 2000                                         //uS
#define MIDDLE_STICKS_PULSE (MIN_STICKS_PULSE + MAX_STICKS_PULSE) / 2 //1500uS
#define DISABLE_IO_PIN 0                                              //OFF
#define FLIGHT_MODE_PULSE_OFF_SET 200                                 //uS
#define MIDDLE_PULSE_OFF_SET 100                                      //uS
#define ALT_HOLD_DEADBAND 50                                          //uS
#define POS_HOLD_DEADBAND 20                                          //uS
#define MAX_MANUAL_SPEED 500                                          //CM/S
#endif