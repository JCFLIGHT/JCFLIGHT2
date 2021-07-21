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
    INS_Resources.Position.Hold[COORD_LATITUDE] = INS_Resources.Estimated.Position.Roll + INS_Resources.Estimated.Velocity.Roll * PositionHoldPID.kI;
    INS_Resources.Position.Hold[COORD_LONGITUDE] = INS_Resources.Estimated.Position.Pitch + INS_Resources.Estimated.Velocity.Pitch * PositionHoldPID.kI;
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
            const float NEUVelocityRoll = RadioControllVelocityRoll * INS_Resources.Math.Cosine.Yaw - RadioControllVelocityPitch * INS_Resources.Math.Sine.Yaw;
            const float NEUVelocityPitch = RadioControllVelocityRoll * INS_Resources.Math.Sine.Yaw + RadioControllVelocityPitch * INS_Resources.Math.Cosine.Yaw;

            //CALCULA UMA NOVA POSIÇÃO
            INS_Resources.Position.Hold[COORD_LATITUDE] = INS_Resources.Estimated.Position.Roll + (NEUVelocityRoll * PositionHoldPID.kI);
            INS_Resources.Position.Hold[COORD_LONGITUDE] = INS_Resources.Estimated.Position.Pitch + (NEUVelocityPitch * PositionHoldPID.kI);

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

    //CALCULA A APLICAÇÃO DE CONTROLE DO PILOTO AUTOMATICO PARA O EIXO X
    int32_t PositionErrorX = INS_Resources.Position.Hold[COORD_LATITUDE] - INS_Resources.Estimated.Position.Roll;
    int32_t TargetPositionErrorX = Constrain_32Bits(GPSGetProportional(PositionErrorX, &PositionHoldPID), -1000, 1000);
    int32_t FinalPositionErrorX = Constrain_32Bits(TargetPositionErrorX - INS_Resources.Estimated.Velocity.Roll, -1000, 1000);
    GPS_Resources.Navigation.AutoPilot.INS.Angle[ROLL] = GPSGetProportional(FinalPositionErrorX, &PositionHoldRatePID) + GPSGetIntegral(FinalPositionErrorX, DeltaTime, &PositionHoldRatePIDArray[COORD_LATITUDE], &PositionHoldRatePID);
    GPS_Resources.Navigation.AutoPilot.INS.Angle[ROLL] -= Constrain_16Bits((INS_Resources.IMU.AccelerationNEU.Roll * PositionHoldRatePID.kD), -2000, 2000);
    GPS_Resources.Navigation.AutoPilot.INS.Angle[ROLL] = Constrain_16Bits(GPS_Resources.Navigation.AutoPilot.INS.Angle[ROLL], -ConvertDegreesToDecidegrees(GET_SET[NAV_GPS_BANK_MAX].MaxValue), ConvertDegreesToDecidegrees(GET_SET[NAV_GPS_BANK_MAX].MaxValue));
    NavigationPIDArray[COORD_LATITUDE].GPSFilter.IntegralSum = PositionHoldRatePIDArray[COORD_LATITUDE].GPSFilter.IntegralSum;

    //CALCULA A APLICAÇÃO DE CONTROLE DO PILOTO AUTOMATICO PARA O EIXO Y
    int32_t PositionErrorY = INS_Resources.Position.Hold[COORD_LONGITUDE] - INS_Resources.Estimated.Position.Pitch;
    int32_t TargetPositionErrorY = Constrain_32Bits(GPSGetProportional(PositionErrorY, &PositionHoldPID), -1000, 1000);
    int32_t FinalPositionErrorY = Constrain_32Bits(TargetPositionErrorY - INS_Resources.Estimated.Velocity.Pitch, -1000, 1000);
    GPS_Resources.Navigation.AutoPilot.INS.Angle[PITCH] = GPSGetProportional(FinalPositionErrorY, &PositionHoldRatePID) + GPSGetIntegral(FinalPositionErrorY, DeltaTime, &PositionHoldRatePIDArray[COORD_LONGITUDE], &PositionHoldRatePID);
    GPS_Resources.Navigation.AutoPilot.INS.Angle[PITCH] -= Constrain_16Bits((INS_Resources.IMU.AccelerationNEU.Pitch * PositionHoldRatePID.kD), -2000, 2000);
    GPS_Resources.Navigation.AutoPilot.INS.Angle[PITCH] = Constrain_16Bits(GPS_Resources.Navigation.AutoPilot.INS.Angle[PITCH], -ConvertDegreesToDecidegrees(GET_SET[NAV_GPS_BANK_MAX].MaxValue), ConvertDegreesToDecidegrees(GET_SET[NAV_GPS_BANK_MAX].MaxValue));
    NavigationPIDArray[COORD_LONGITUDE].GPSFilter.IntegralSum = PositionHoldRatePIDArray[COORD_LONGITUDE].GPSFilter.IntegralSum;
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
            GPS_Resources.Navigation.AutoPilot.Control.Angle[ROLL] = PIDAngleToRcController(GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] * INS_Resources.Math.Cosine.Yaw - GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] * INS_Resources.Math.Sine.Yaw, ConvertDegreesToDecidegrees(GET_SET[MAX_ROLL_LEVEL].MaxValue));
            GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH] = PIDAngleToRcController(GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] * INS_Resources.Math.Sine.Yaw + GPS_Resources.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] * INS_Resources.Math.Cosine.Yaw, ConvertDegreesToDecidegrees(GET_SET[MAX_PITCH_LEVEL].MaxValue));
        }
        else
        {
            GPS_Resources.Navigation.AutoPilot.Control.Angle[ROLL] = 0;
            GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH] = 0;
            GPS_Resources.Navigation.AutoPilot.Control.Angle[YAW] = 0;
        }
    }
}