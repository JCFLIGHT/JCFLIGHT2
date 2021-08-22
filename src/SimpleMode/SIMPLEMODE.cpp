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

#include "SIMPLEMODE.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Compass/COMPASSREAD.h"
#include "PID/RCPID.h"
#include "Common/ENUM.h"
#include "BitArray/BITARRAY.h"
#include "AHRS/AHRS.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "I2C/I2C.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//DEBUG
//#define PRINTLN_SIMPLE_MODE

void Simple_Mode_Update(void)
{
  if (IS_FLIGHT_MODE_ACTIVE(SIMPLE_MODE) && GetMultirotorEnabled() && I2CResources.Found.Compass)
  {
    const float HeadingDifference = ConvertToRadians(Attitude.EulerAngles.Yaw - Calibration.Magnetometer.SimpleModeHeading);
    const float CosineDifference = Fast_Cosine(HeadingDifference);
    const float SineDifference = Fast_Sine(HeadingDifference);
    const int16_t CalcedRCControllerPITCH = RC_Resources.Attitude.Controller[PITCH] * CosineDifference + RC_Resources.Attitude.Controller[ROLL] * SineDifference;
    RC_Resources.Attitude.Controller[ROLL] = RC_Resources.Attitude.Controller[ROLL] * CosineDifference - RC_Resources.Attitude.Controller[PITCH] * SineDifference;
    RC_Resources.Attitude.Controller[PITCH] = CalcedRCControllerPITCH;
  }
#ifdef PRINTLN_SIMPLE_MODE
  DEBUG("RC_Resources.Attitude.Controller[ROLL]:%d RC_Resources.Attitude.Controller[PITCH]:%d CalcedRCControllerPITCH:%d HeadingDiff:%.3f CosineDiff:%.3f SineDiff:%.3f",
        RC_Resources.Attitude.Controller[ROLL],
        RC_Resources.Attitude.Controller[PITCH],
        CalcedRCControllerPITCH,
        HeadingDifference,
        CosineDifference,
        SineDifference);
#endif
}
