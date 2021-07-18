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

#include "INSNAVIGATION.h"
#include "PID/GPSPID.h"
#include "NAVIGATION.h"
#include "InertialNavigation/INS.h"
#include "Math/MATHSUPPORT.h"
#include "PID/PIDPARAMS.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "Param/PARAM.h"
#include "PID/RCPID.h"
#include "Scheduler/SCHEDULER.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "RadioControl/CURVESRC.h"
#include "BitArray/BITARRAY.h"
#include "Common/RCDEFINES.h"

void SetThisPointToPositionHold(void)
{
    INS.Position.Hold[INS_LATITUDE] = INS.EarthFrame.Position[INS_LATITUDE] + INS.EarthFrame.Velocity[INS_LATITUDE] * PositionHoldPID.kI;
    INS.Position.Hold[INS_LONGITUDE] = INS.EarthFrame.Position[INS_LONGITUDE] + INS.EarthFrame.Velocity[INS_LONGITUDE] * PositionHoldPID.kI;
}

static void MultirotorApplyINSPositionHoldPIDControl(float DeltaTime)
{
    static bool NewPointToPositionHold = false;

    if (!IS_FLIGHT_MODE_ACTIVE(WAYPOINT_MODE) && (GPS_Resources.Navigation.AutoPilot.Control.Mode == AUTOPILOT_MODE_CRUISE))
    {
        if (RC_Resources.Attitude.Controller[PITCH] || RC_Resources.Attitude.Controller[ROLL]) //CHECA SE OS STICKS DO RÁDIO FORAM MANIPULADOS
        {
            const float RadioControllVelocityRoll = RC_Resources.Attitude.Controller[PITCH] * MAX_MANUAL_SPEED / (float)(500 - POS_HOLD_DEADBAND);
            const float RadioControllVelocityPitch = RC_Resources.Attitude.Controller[ROLL] * MAX_MANUAL_SPEED / (float)(500 - POS_HOLD_DEADBAND);

            //ROTACIONA AS VELOCIDADES DO BODY-FRAME PARA EARTH-FRAME
            const float NEUVelocityRoll = RadioControllVelocityRoll * INS.Math.Cosine.Yaw - RadioControllVelocityPitch * INS.Math.Sine.Yaw;
            const float NEUVelocityPitch = RadioControllVelocityRoll * INS.Math.Sine.Yaw + RadioControllVelocityPitch * INS.Math.Cosine.Yaw;

            //CALCULA UMA NOVA POSIÇÃO
            INS.Position.Hold[INS_LATITUDE] = INS.EarthFrame.Position[INS_LATITUDE] + (NEUVelocityRoll * PositionHoldPID.kI);
            INS.Position.Hold[INS_LONGITUDE] = INS.EarthFrame.Position[INS_LONGITUDE] + (NEUVelocityPitch * PositionHoldPID.kI);

            NewPointToPositionHold = true;
        }
        else
        {
            if (NewPointToPositionHold)
            {
                SetThisPointToPositionHold();
                NewPointToPositionHold = false;
            }
        }
    }

    for (uint8_t IndexCount = 0; IndexCount < 2; IndexCount++)
    {
        int32_t PositionError = INS.Position.Hold[IndexCount] - INS.EarthFrame.Position[IndexCount];
        int32_t TargetPositionError = Constrain_32Bits(GPSGetProportional(PositionError, &PositionHoldPID), -1000, 1000);
        int32_t FinalPositionError = Constrain_32Bits(TargetPositionError - INS.EarthFrame.Velocity[IndexCount], -1000, 1000);
        GPS_Resources.Navigation.AutoPilot.INS.Angle[IndexCount] = GPSGetProportional(FinalPositionError, &PositionHoldRatePID) + GPSGetIntegral(FinalPositionError, DeltaTime, &PositionHoldRatePIDArray[IndexCount], &PositionHoldRatePID);
        GPS_Resources.Navigation.AutoPilot.INS.Angle[IndexCount] -= Constrain_16Bits((INS.AccelerationEarthFrame_Filtered[IndexCount] * PositionHoldRatePID.kD), -2000, 2000);
        GPS_Resources.Navigation.AutoPilot.INS.Angle[IndexCount] = Constrain_16Bits(GPS_Resources.Navigation.AutoPilot.INS.Angle[IndexCount], -ConvertDegreesToDecidegrees(GET_SET[NAV_GPS_BANK_MAX].MaxValue), ConvertDegreesToDecidegrees(GET_SET[NAV_GPS_BANK_MAX].MaxValue));
        NavigationPIDArray[IndexCount].GPSFilter.IntegralSum = PositionHoldRatePIDArray[IndexCount].GPSFilter.IntegralSum;
    }
}

static void MultirotorApplyPosHoldPIDControl(float DeltaTime)
{
    if (!GetTakeOffInProgress() && !GetGroundDetectedFor100ms())
    {
        MultirotorApplyINSPositionHoldPIDControl(DeltaTime);
    }
    else
    {
        GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] = 0;
        GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] = 0;
        GPSResetPID(&PositionHoldRatePIDArray[COORD_LATITUDE]);
        GPSResetPID(&PositionHoldRatePIDArray[COORD_LONGITUDE]);
    }
}

void MultirotorUpdateAutoPilotControl(void)
{
    if (!GetMultirotorEnabled())
    {
        return;
    }

    bool AltHoldControlApplied = ApplyAltitudeHoldControl();
    static Scheduler_Struct GPSControlTimer;
    if (!AltHoldControlApplied && Scheduler(&GPSControlTimer, SCHEDULER_SET_FREQUENCY(50, "Hz")))
    {
        if (GPS_Resources.Navigation.AutoPilot.Control.Enabled)
        {
            if (Get_Safe_State_To_Apply_Position_Hold())
            {
                float DeltaTime = GPSControlTimer.ActualTime * 1e-6f;
                MultirotorApplyPosHoldPIDControl(DeltaTime);
            }
            GPS_Resources.Navigation.AutoPilot.Control.Angle[ROLL] = PIDAngleToRcController(GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] * INS.Math.Cosine.Yaw - GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] * INS.Math.Sine.Yaw, ConvertDegreesToDecidegrees(GET_SET[MAX_ROLL_LEVEL].MaxValue));
            GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH] = PIDAngleToRcController(GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] * INS.Math.Sine.Yaw + GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] * INS.Math.Cosine.Yaw, ConvertDegreesToDecidegrees(GET_SET[MAX_PITCH_LEVEL].MaxValue));
        }
        else
        {
            GPS_Resources.Navigation.AutoPilot.Control.Angle[ROLL] = 0;
            GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH] = 0;
            GPS_Resources.Navigation.AutoPilot.Control.Angle[YAW] = 0;
        }
    }
}