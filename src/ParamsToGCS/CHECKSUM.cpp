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

#include "CHECKSUM.h"
#include "I2C/I2C.h"
#include "FlightModes/AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "AirPlane/AIRPLANE.h"
#include "ServosMaster/SERVOSMASTER.h"
#include "RadioControl/RCCONFIG.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

CheckSumClass CHECKSUM;

#define GET_RC_DIRECTION(Bit) (((Bit) > 0) ? true : false)

uint8_t CheckSumClass::GetDevicesActived(void)
{
    const bool Compass_Detect = I2CResources.Found.Compass;
    const bool Parachute_Detect = ParachuteConfig > 0 ? true : false;
    const bool SD_Card_Detect = STORAGEMANAGER.Read_8Bits(UART_NUMB_3_ADDR) == 1 ? true : false;
    const bool Pitot_Detect = Get_AirSpeed_Enabled();
    uint8_t CheckDevices = Compass_Detect | Parachute_Detect << 1 | SD_Card_Detect << 2 | Pitot_Detect << 3;
    return CheckDevices;
}

void CheckSumClass::UpdateServosReverse(void)
{
    const uint8_t ServosReverse = STORAGEMANAGER.Read_8Bits(SERVOS_REVERSE_ADDR);

    //ASA
    Servo.Direction.GetAndSet[SERVO1] = GET_SERVO_DIRECTION(ServosReverse & 1);

    //ASA
    Servo.Direction.GetAndSet[SERVO2] = GET_SERVO_DIRECTION(ServosReverse & 2);

    //LEME
    Servo.Direction.GetAndSet[SERVO3] = GET_SERVO_DIRECTION(ServosReverse & 4);

    //PROFUNDOR
    Servo.Direction.GetAndSet[SERVO4] = GET_SERVO_DIRECTION(ServosReverse & 8);
}

void CheckSumClass::UpdateChannelsReverse(void)
{
    const uint8_t ChannelsReverse = STORAGEMANAGER.Read_8Bits(CH_REVERSE_ADDR);
    CHECKSUM.GetFailSafeValue = STORAGEMANAGER.Read_16Bits(FAILSAFE_VAL_ADDR);

    //THROTTLE
    Throttle.Set_Reverse(GET_RC_DIRECTION(ChannelsReverse & 1));

    //YAW
    Yaw.Set_Reverse(GET_RC_DIRECTION(ChannelsReverse & 2));

    //PITCH
    Pitch.Set_Reverse(GET_RC_DIRECTION(ChannelsReverse & 4));

    //ROLL
    Roll.Set_Reverse(GET_RC_DIRECTION(ChannelsReverse & 8));
}