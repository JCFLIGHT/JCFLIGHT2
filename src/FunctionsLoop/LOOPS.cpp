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

void Slow_Loop(void)
{
        STICKS.Pre_Arm();
        COMPASS.Constant_Read();
        BEEPER.UpdateSafeToOthersBeepsCounter();
        UpdateValuesOfPID();
}

void Medium_Loop(void)
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
        WINDESTIMATOR.Update();
        BATTERY.Update_Voltage();
        BATTERY.Update_Current();
        TUNNING.Update();
}

void Fast_Medium_Loop(void)
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

void Fast_Loop(void)
{
        Update_PrecisionLand();
}

void Super_Fast_Loop(void)
{
        RGB.Update();
        SAFETYBUTTON.UpdateRoutine();
        SBUSRC.Update();
        IBUSRC.Update();
        INERTIALNAVIGATION.Update();
        AIRSPEED.Update();
        Switch_Flag_Update();
        BATTERY.Calculate_Total_Current_In_Mah();
}

void Integral_Loop(void)
{
        uint32_t ThisTaskTimeUs = GetTaskDeltaTime(TASK_INTEGRAL_LOOP);
        const float ThisDeltaTime = (float)ThisTaskTimeUs * 1e-6f;

#ifdef __AVR_ATmega2560__
        Fast_Medium_Loop();
        Fast_Loop();
        Super_Fast_Loop();
#endif

        Update_Accelerometer();
        Update_Gyroscope();
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