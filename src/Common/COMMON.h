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

#include "STRUCTS.h"
#include "PID/PIDPARAMS.h"
#include "AHRS/AHRS.h"
#include "InertialNavigation/INS.h"
#include "MotorsControl/MOTORS.h"
#include "RadioControl/DECODE.h"
#include "I2C/I2C.h"
#include "FastSerial/FASTSERIAL.h"
#include "GPSNavigation/NAVIGATION.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "FastSerial/PRINTF.h"
#include "FlightModes/AUXFLIGHT.h"
#include "FailSafe/FAILSAFE.h"
#include "FlightModes/FLIGHTMODES.h"
#include "Yaw/HEADINGHOLD.h"
#include "Scheduler/SCHEDULER.h"
#include "PID/RCPID.h"
#include "Gimbal/GIMBAL.h"
#include "PID/PIDXYZ.h"
#include "LedRGB/LEDRGB.h"
#include "RadioControl/STICKS.h"
#include "Buzzer/BUZZER.h"
#include "BatteryMonitor/BATTERY.h"
#include "Parachute/PARACHUTE.h"
#include "CrashCheck/CRASHCHECK.h"
#include "IMU/IMUHEALTH.h"
#include "SwitchFlag/SWITCHFLAG.h"
#include "RadioControl/RCCONFIG.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "RadioControl/DESARMLOWTHR.h"
#include "Filters/KALMANFILTER.h"
#include "EscCalibration/CALIBESC.h"
#include "SBUS/SBUSREAD.h"
#include "FastSerial/UART2MODE.h"
#include "ServosMaster/SERVOSMASTER.h"
#include "RadioControl/CURVESRC.h"
#include "AirPlane/AIRPLANE.h"
#include "AutoLaunch/AUTOLAUNCH.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "PID/TPA.h"
#include "IMU/ACCGYROREAD.h"
#include "IBUS/IBUSREAD.h"
#include "PrecisionLand/PRECISIONLAND.h"
#include "Compass/COMPASSREAD.h"
#include "Barometer/BAROREAD.h"
#include "AirSpeed/AIRSPEED.h"
#include "GPS/GPSSERIALREAD.h"
#include "Barometer/BAROBACKEND.h"
#include "IOMCU/IOMCU.h"
#include "FunctionsLoop/LOOPS.h"
#include "Param/PARAM.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "Build/BOARDDEFS.h"
#include "Build/DEFAULT.h"
#include "TaskSystem/TASKSYSTEM.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "IMU/GFORCE.h"
#include "TECS/TECS.h"
#include "WindEstimator/WINDESTIMATOR.h"
#include "GPSNavigation/INSNAVIGATION.h"
#include "Tunning/TUNNING.h"