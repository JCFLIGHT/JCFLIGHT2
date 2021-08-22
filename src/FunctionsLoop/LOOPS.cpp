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

#include "LOOPS.h"
#include "Common/COMMON.h"

void Slow_Loop()
{
        STICKS.Pre_Arm();
        COMPASS.Constant_Read();
        BEEPER.UpdateSafeToOthersBeepsCounter();
        UpdateValuesOfPID();
}

void Medium_Loop()
{
        uint32_t ThisTaskTimeUs = GetTaskDeltaTime(TASK_MEDIUM_LOOP);
        const float ThisDeltaTime = (float)ThisTaskTimeUs * 1e-6f;

        DECODE.Update();
        RCCONFIG.Set_Pulse();
        RCCONFIG.Update_Channels();
        DESARMLOWTHROTTLE.Update();
        FailSafeCheck();
        STICKS.Update();
        BAROMETER.Update();
        GPS_Serial_Read();
        GPS_Process_FlightModes(ThisDeltaTime);
        AUXFLIGHT.Update();
        FlightModesUpdate();
        INERTIALNAVIGATION.Calculate_AccelerationXY(ThisDeltaTime);
        INERTIALNAVIGATION.Calculate_AccelerationZ(ThisDeltaTime);
        WINDESTIMATOR.Update();
        BATTERY.Update_Voltage();
        BATTERY.Update_Current();
        TUNNING.Update();
}

void Fast_Medium_Loop()
{
        BEEPER.Run();
        STICKS.Pre_Arm_Leds();
        Gimbal_Controll();
        CrashCheck();
        PARACHUTE.Manual_Do_Now();
        IMU_GForce_Update();
        GCS.Serial_Parse_Protocol();
        PARAM.Update();
}

void Fast_Loop()
{
        Update_PrecisionLand();
}

void Super_Fast_Loop()
{
        RGB.Update();
        SAFETYBUTTON.UpdateRoutine();
        SBUSRC.Update();
        IBUSRC.Update();
        INERTIALNAVIGATION.Calculate_AccelerationXYZ_To_EarthFrame();
        AIRSPEED.Update();
        Switch_Flag();
        BATTERY.Calculate_Total_Current_In_Mah();
}

void Integral_Loop()
{
        uint32_t ThisTaskTimeUs = GetTaskDeltaTime(TASK_INTEGRAL_LOOP);
        const float ThisDeltaTime = (float)ThisTaskTimeUs * 1e-6f;

#ifdef __AVR_ATmega2560__
        Fast_Medium_Loop();
        Fast_Loop();
        Super_Fast_Loop();
#endif

        Acc_ReadBufferData();
        Gyro_ReadBufferData();
        AHRS.Update(ThisDeltaTime);
        RC_PID_Update();
        WAYPOINT.Update();
        AUTOLAUNCH.Update();
        TECS.Update(ThisDeltaTime);
        MultirotorUpdateAutoPilotControl();
        PIDXYZ.Update(ThisDeltaTime);
        SERVOSMASTER.Update(ThisDeltaTime);
        ApplyMixingForMotorsAndServos(ThisDeltaTime);
        ApplyPWMControlForMotorsAndServos();
}