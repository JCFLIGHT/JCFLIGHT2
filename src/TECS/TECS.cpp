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

#include "TECS.h"
#include "AirSpeed/AIRSPEED.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "PID/RCPID.h"
#include "BitArray/BITARRAY.h"
#include "PID/PIDPARAMS.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSTATES.h"
#include "Filters/PT1.h"
#include "Scheduler/SCHEDULER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "RadioControl/CURVESRC.h"
#include "Barometer/BAROBACKEND.h"
#include "AHRS/AHRS.h"
#include "GPSNavigation/NAVIGATION.h"
#include "InertialNavigation/INS.h"
#include "Common/RCDEFINES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "AutoLaunch/AUTOLAUNCH.h"
#include "FastSerial/PRINTF.h"
#include "Param/PARAM.h"

/*
    TOTAL ENERGY CONSERVATION SYSTEM - SISTEMA DE CONSERVAÇÃO TOTAL DE ENERGIA
    APENAS PARA AIRPLANES E FIXED-WING
*/

TecsClass TECS;

//DEBUG
//#define PRINTLN_TECS

#define TECS_TIMER_US SCHEDULER_SET_FREQUENCY(5, "Hz") //ROTINA DO TECS EM HZ
#define MIN_DEGREES_DEACTIVE_TURN 90                   //GRAUS MINIMO PARA DESATIVAR A CURVA DO MODO CIRCULO EM GRAUS
#define MAX_DEGREES_ACTIVE_TURN 170                    //GRAUS MAXIMO PARA FORÇAR A CURVA CONSTANTE NO MODO CIRCULO EM GRAUS
#define MAX_AUTOPILOT_DESCENT_ANGLE 15                 //ANGULO MAXIMO DE SUBIDA DISPONIVEL PARA O PILOTO AUTOMATICO EM GRAUS
#define MAX_AUTOPILOT_CLIMB_ANGLE 20                   //ANGULO MAXIMO DE DESCIDA DISPONIVEL PARA O PILOTO AUTOMATICO EM GRAUS
#define ALTITUDE_DERIVATIVE_CUTOFF 10                  //HZ
#define POSITION_DERIVATIVE_CUTOFF 10                  //HZ
#define HEADING_DERIVATIVE_CUTOFF 2                    //HZ

TECS_PID_Float_Struct TECS_PID_Altitude_Navigation;
TECS_PID_Float_Struct TECS_PID_Position_Navigation;
TECS_PID_Float_Struct TECS_PID_Heading_Navigation;
TECS_Resources_Struct TECS_Resources;

void TecsClass::Initialization(void)
{
    //RECURSOS DESTINADOS PARA O USUARIO
    TECS_Resources.Params.CircleDirectionToRight = STORAGEMANAGER.Read_8Bits(TECS_CIRCLE_DIR_ADDR) == 0 ? false : true;
    TECS_Resources.Params.DoLandAfterRTH = GPS_Resources.Navigation.LandAfterRTH;
    TECS_Resources.Params.LandMinAltitude = 5;                                                            //METROS
    TECS_Resources.Params.FinalLandPitchAngle = 2;                                                        //GRAUS
    TECS_Resources.Params.PitchToThrottleLPFQuality = STORAGEMANAGER.Read_8Bits(TECS_PITCH2THR_LPF_ADDR); //0 A 9 ~ AJUSTE DO FILTRO LPF DA CORREÇÃO DO PITCH FEITO PELO PILOTO AUTOMATICO
    TECS_Resources.Params.PitchToThrottleDifference = ALT_HOLD_DEADBAND;                                  //DIFERENÇA ENTRE O PITCH FILTRADO E O PITCH NÃO FILTRADO,SE O VALOR FOR MAIOR QUE ESSE PARAMETRO,O PITCH NÃO FILTRADO É PASSADO DIRETO
    TECS_Resources.Params.PitchToThrottleFactor = STORAGEMANAGER.Read_8Bits(TECS_PITCH2THR_FACTOR_ADDR);  //GANHO DE PITCH PARA SER APLICADO AO CONTROLE DO THROTTLE
    TECS_Resources.Params.AutoPilotLPFQuality = STORAGEMANAGER.Read_8Bits(TECS_AP_LPF_ADDR);              //0 A 9 ~ AJUSTE DO FILTRO LPF DOS CONTROLES ROLL,PITCH E YAW FEITO PELO PILOTO AUMATICO
    TECS_Resources.Params.Circle_Radius = (int16_t)ConverMetersToCM(Constrain_U8Bits(GET_SET[ATTACK_BANK_MAX].MaxValue, 30, 75));
    TECS_Resources.Params.AutoPilotMaxDescentAngle = ConvertDegreesToDecidegrees(MAX_AUTOPILOT_DESCENT_ANGLE);
    TECS_Resources.Params.AutoPilotMaxClimbAngle = ConvertDegreesToDecidegrees(MAX_AUTOPILOT_CLIMB_ANGLE);
    TECS_Resources.Params.AutoThrottleGain = (float)(GET_SET[PID_GPS_POSITION].kP / 10.0f);
    TECS_Resources.Params.AutoThrottleMinVel = ConvertDegreesToDecidegrees(GET_SET[PID_GPS_POSITION].kI);
    TECS_Resources.Params.PilotManualRollSpeed = MAX_MANUAL_SPEED; //CM/S
    TECS_Resources.Params.PilotManualClimbDescentRate = 200;       //CM/S
    TECS_Resources.Params.MinCruiseThrottle = STORAGEMANAGER.Read_16Bits(TECS_CRUISE_MIN_THR_ADDR);
    TECS_Resources.Params.MaxCruiseThrottle = STORAGEMANAGER.Read_16Bits(TECS_CRUISE_MAX_THR_ADDR);
    TECS_Resources.Params.CruiseThrottle = STORAGEMANAGER.Read_16Bits(TECS_CRUISE_THR_ADDR);

    //RECURSOS NÃO DESTINADOS PARA O USUARIO - APENAS DEV'S
    TECS_Resources.Position.Tracking.Period = (TECS_TIMER_US * 1e-6f) * 2.0f;
    TECS_Resources.Heading.MinToNormalizeTurnDirection = ConvertDegreesToCentiDegrees(MIN_DEGREES_DEACTIVE_TURN);
    TECS_Resources.Heading.MaxToRunTurnDirection = ConvertDegreesToCentiDegrees(MAX_DEGREES_ACTIVE_TURN);
}

float TecsClass::Floating_Point_PID(TECS_PID_Float_Struct *TECS_PID_Pointer, const float SetProportional, const float SetIntegrator, const float SetDerivative, const float PIDSetPoint,
                                    const float RawMeasurement, const float PIDScaler, const float DerivativeScaler, const float OutputMin, const float OutputMax, const uint8_t Flags, const float DeltaTime)
{
    TECS_PID_Pointer->Error = PIDSetPoint - RawMeasurement;

    if (TECS_PID_Pointer->Reset)
    {
        if (Flags == TECS_PID_USE_TRACKING_ERROR)
        {
            TECS_PID_Pointer->PreviousMeasurement = TECS_PID_Pointer->Error;
        }
        else
        {
            TECS_PID_Pointer->PreviousMeasurement = RawMeasurement;
        }
        TECS_PID_Pointer->Reset = false;
    }

    if (Flags == TECS_PID_USE_TRACKING_ERROR)
    {
        TECS_PID_Pointer->Derivative = -(TECS_PID_Pointer->Error - TECS_PID_Pointer->PreviousMeasurement) / DeltaTime;
        TECS_PID_Pointer->PreviousMeasurement = TECS_PID_Pointer->Error;
    }
    else
    {
        TECS_PID_Pointer->Derivative = -(RawMeasurement - TECS_PID_Pointer->PreviousMeasurement) / DeltaTime;
        TECS_PID_Pointer->PreviousMeasurement = RawMeasurement;
    }

    if (TECS_PID_Pointer->DerivativeCutOff > 0)
    {
        TECS_PID_Pointer->Derivative = SetDerivative * PT1FilterApply(&TECS_PID_Pointer->Derivative_Smooth, TECS_PID_Pointer->Derivative, TECS_PID_Pointer->DerivativeCutOff, DeltaTime);
    }
    else
    {
        TECS_PID_Pointer->Derivative = TECS_PID_Pointer->Derivative * SetDerivative;
    }

    TECS_PID_Pointer->Derivative = TECS_PID_Pointer->Derivative * PIDScaler * DerivativeScaler;

    const float PIDControllerValue = (TECS_PID_Pointer->Error * SetProportional * PIDScaler) + (TECS_PID_Pointer->PreviousIntegrator * PIDScaler) + TECS_PID_Pointer->Derivative;
    TECS_PID_Pointer->ValueConstrained = Constrain_Float(PIDControllerValue, OutputMin, OutputMax);

    if (SetProportional > 1e-6f && SetIntegrator > 1e-6f)
    {
        TECS_PID_Pointer->ControlTracking = 2.0f / ((SetProportional / SetIntegrator) + (SetDerivative / SetProportional));
    }
    else
    {
        TECS_PID_Pointer->ControlTracking = 0.0f;
    }

    TECS_PID_Pointer->IntegratorSum = TECS_PID_Pointer->PreviousIntegrator + (TECS_PID_Pointer->Error * SetIntegrator * PIDScaler * DeltaTime) + ((TECS_PID_Pointer->ValueConstrained - PIDControllerValue) * TECS_PID_Pointer->ControlTracking * DeltaTime);

    if (Flags == TECS_PID_USE_SHRINK_INTEGRATOR)
    {
        if (ABS(TECS_PID_Pointer->IntegratorSum) < ABS(TECS_PID_Pointer->PreviousIntegrator))
        {
            TECS_PID_Pointer->PreviousIntegrator = TECS_PID_Pointer->IntegratorSum;
        }
    }
    else
    {
        TECS_PID_Pointer->PreviousIntegrator = TECS_PID_Pointer->IntegratorSum;
    }

    return TECS_PID_Pointer->ValueConstrained;
}

void TecsClass::Reset_PID_Navigation(TECS_PID_Float_Struct *TECS_PID_Pointer, float DerivativeCutOff)
{
    TECS_PID_Pointer->Reset = true;
    TECS_PID_Pointer->Error = 0.0f;
    TECS_PID_Pointer->PreviousMeasurement = 0.0f;
    TECS_PID_Pointer->Derivative = 0.0f;
    TECS_PID_Pointer->IntegratorSum = 0.0f;
    TECS_PID_Pointer->PreviousIntegrator = 0.0f;
    TECS_PID_Pointer->ValueConstrained = 0.0f;
    TECS_PID_Pointer->DerivativeCutOff = DerivativeCutOff;
    TECS_PID_Pointer->ControlTracking = 0.0f;
    TECS_PID_Pointer->Derivative_Smooth.State = 0.0f;
    TECS_PID_Pointer->AutoPilotControl[ROLL] = 0;
    TECS_PID_Pointer->AutoPilotControl[PITCH] = 0;
    TECS_PID_Pointer->AutoPilotControl[YAW] = 0;
}

bool TecsClass::GetNavigationInAutomaticThrottleMode(void)
{
    return GetAirPlaneEnabled() & (IS_FLIGHT_MODE_ACTIVE(LAUNCH_MODE) |
                                   IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE) |
                                   IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) |
                                   IS_FLIGHT_MODE_ACTIVE(ALTITUDE_HOLD_MODE) |
                                   IS_FLIGHT_MODE_ACTIVE(RTH_MODE));
}

float TecsClass::AutoPitchDown(int16_t InMinThrottleDownPitchAngle)
{
    if (!TECS.GetNavigationInAutomaticThrottleMode() && IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
    {
        return ScaleRange16Bits(MAX(0, TECS_Resources.Params.CruiseThrottle - RC_Resources.Attitude.Controller[THROTTLE]), 0, TECS_Resources.Params.CruiseThrottle - MIN_STICKS_PULSE, 0, InMinThrottleDownPitchAngle);
    }
    return 0.0f;
}

int16_t TecsClass::UpdatePitchToThrottle(int16_t PitchInput, float DeltaTime)
{
    int16_t PitchCommandFiltered = (int16_t)PT1FilterApply(&TECS_Resources.PitchToThrottle_Smooth,
                                                           PitchInput,
                                                           0.002f * Power3_Float(10.0f - Constrain_Float(TECS_Resources.Params.PitchToThrottleLPFQuality, 0.0f, 9.0f)) + 0.01f,
                                                           DeltaTime);

    if (ABS(PitchInput - PitchCommandFiltered) > TECS_Resources.Params.PitchToThrottleDifference)
    {
        return ConvertDeciDegreesToDegrees(PitchInput) * TECS_Resources.Params.PitchToThrottleFactor;
    }
    else
    {
        return ConvertDeciDegreesToDegrees(PitchCommandFiltered) * TECS_Resources.Params.PitchToThrottleFactor;
    }
}

void TecsClass::UpdateEnergyAltitudeController(float DeltaTime)
{
    TECS_Resources.Energies.Specific.Demanded.KineticEnergy = 0.0f;
    TECS_Resources.Energies.Specific.Estimated.KineticEnergy = 0.0f;

    TECS_Resources.Energies.Specific.Demanded.PotentialEnergy = ConvertCMToMeters(TECS_Resources.Position.DestinationNEU.Altitude) * GRAVITY_MSS;
    TECS_Resources.Energies.Specific.Estimated.PotentialEnergy = ConvertCMToMeters(TECS_Resources.Position.Altitude) * GRAVITY_MSS;

    //SPE - CONTROLE DE BALANÇO PARA AS ENERGIAS
    TECS_Resources.Energies.Specific.Control = 0.0f;

    TECS_Resources.Energies.Specific.Demanded.EnergyBalance = TECS_Resources.Energies.Specific.Demanded.PotentialEnergy * (1.0f - TECS_Resources.Energies.Specific.Control) - TECS_Resources.Energies.Specific.Demanded.KineticEnergy * TECS_Resources.Energies.Specific.Control;
    TECS_Resources.Energies.Specific.Estimated.EnergyBalance = TECS_Resources.Energies.Specific.Estimated.PotentialEnergy * (1.0f - TECS_Resources.Energies.Specific.Control) - TECS_Resources.Energies.Specific.Estimated.KineticEnergy * TECS_Resources.Energies.Specific.Control;

    //SEB - BALANÇO DE ENERGIA ESPECIFICO
    TECS_Resources.Energies.Specific.Estimated.PitchGainBalance = 1.0f;

    TECS_Resources.Energies.Specific.Estimated.PitchAngle = TECS.Floating_Point_PID(&TECS_PID_Altitude_Navigation,
                                                                                    GET_SET[PID_POSITION_Z].kP / 10.0f, GET_SET[PID_POSITION_Z].kI / 10.0f, GET_SET[PID_POSITION_Z].kD / 10.0f,
                                                                                    TECS_Resources.Energies.Specific.Demanded.EnergyBalance, TECS_Resources.Energies.Specific.Estimated.EnergyBalance,
                                                                                    TECS_Resources.Energies.Specific.Estimated.PitchGainBalance,
                                                                                    1.0f,
                                                                                    -TECS_Resources.Params.AutoPilotMaxDescentAngle, TECS_Resources.Params.AutoPilotMaxClimbAngle,
                                                                                    TECS_PID_USE_NONE,
                                                                                    DeltaTime);

    TECS_Resources.Energies.Specific.Estimated.PitchAngle = PT1FilterApply(&TECS_Resources.PitchController_Smooth,
                                                                           TECS_Resources.Energies.Specific.Estimated.PitchAngle,
                                                                           0.002f * Power3_Float(10.0f - Constrain_Float(TECS_Resources.Params.AutoPilotLPFQuality, 0.0f, 9.0f)) + 0.1f,
                                                                           DeltaTime);

    TECS_PID_Altitude_Navigation.AutoPilotControl[PITCH] = Constrain_Float(TECS_Resources.Energies.Specific.Estimated.PitchAngle,
                                                                           -TECS_Resources.Params.AutoPilotMaxDescentAngle,
                                                                           TECS_Resources.Params.AutoPilotMaxClimbAngle);
}

int16_t TecsClass::GetEnergyMotorSpeedController(float DeltaTime)
{
    TECS_Resources.Throttle.SpeedBoost = (TECS_Resources.Params.AutoThrottleMinVel - TECS_Resources.Position.VelocityXY) * TECS_Resources.Params.AutoThrottleGain * DeltaTime;
    if (ABS(TECS_Resources.Position.VelocityXY - TECS_Resources.Params.AutoThrottleMinVel) > 50)
    {
        TECS_Resources.Throttle.SpeedAdjustment += TECS_Resources.Throttle.SpeedBoost;
    }
    TECS_Resources.Throttle.SpeedAdjustment = Constrain_Float(TECS_Resources.Throttle.SpeedAdjustment, 0.0f, 500.0f);
    return TECS_Resources.Throttle.SpeedAdjustment;
}

void TecsClass::UpdateAutoPilotControl(float DeltaTime)
{
    if (IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) || IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE))
    {
        TECS_Resources.Position.AutoPilot.RollAngle = Constrain_16Bits(TECS_PID_Position_Navigation.AutoPilotControl[ROLL], -ConvertDegreesToDecidegrees(GET_SET[NAV_ROLL_BANK_MAX].MaxValue), ConvertDegreesToDecidegrees(GET_SET[NAV_ROLL_BANK_MAX].MaxValue));
        GPS_Resources.Navigation.AutoPilot.Control.Angle[ROLL] = PIDAngleToRcController(TECS_Resources.Position.AutoPilot.RollAngle, ConvertDegreesToDecidegrees(GET_SET[MAX_ROLL_LEVEL].MaxValue));
    }

    if (IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) || IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE))
    {
        GPS_Resources.Navigation.AutoPilot.Control.Angle[YAW] = TECS_PID_Heading_Navigation.AutoPilotControl[YAW];
    }

    const bool Fixed_Wing_HomePointReached = GPS_Resources.Home.Distance <= ConverMetersToCM(JCF_Param.GPS_WP_Radius);

    if (IS_FLIGHT_MODE_ACTIVE(CLIMBOUT_MODE))
    {
        GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH] = -PIDAngleToRcController(TECS_PID_Altitude_Navigation.AutoPilotControl[PITCH], ConvertDegreesToDecidegrees(GET_SET[MAX_PITCH_LEVEL].MaxValue));
        TECS_Resources.Throttle.Correction = TECS.UpdatePitchToThrottle(TECS_PID_Altitude_Navigation.AutoPilotControl[PITCH], DeltaTime);

        if (TECS_Resources.Params.DoLandAfterRTH && IS_FLIGHT_MODE_ACTIVE(RTH_MODE) && Fixed_Wing_HomePointReached)
        {
            TECS_Resources.Throttle.Correction = Constrain_16Bits(TECS_Resources.Throttle.Correction, TECS_Resources.Params.MinCruiseThrottle - TECS_Resources.Params.CruiseThrottle, 0);
        }
        else
        {
            TECS_Resources.Throttle.Correction = Constrain_16Bits(TECS_Resources.Throttle.Correction, TECS_Resources.Params.MinCruiseThrottle - TECS_Resources.Params.CruiseThrottle, TECS_Resources.Params.MaxCruiseThrottle - TECS_Resources.Params.CruiseThrottle);
        }

        if ((IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) || IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE)) && (!IS_FLIGHT_MODE_ACTIVE(RTH_MODE) && !Fixed_Wing_HomePointReached))
        {
            TECS_Resources.Throttle.Correction += TECS.GetEnergyMotorSpeedController(DeltaTime);
            TECS_Resources.Throttle.Correction = Constrain_16Bits(TECS_Resources.Throttle.Correction, TECS_Resources.Params.MinCruiseThrottle - TECS_Resources.Params.CruiseThrottle, TECS_Resources.Params.MaxCruiseThrottle - TECS_Resources.Params.CruiseThrottle);
        }

        TECS_Resources.Throttle.Cruise = Constrain_16Bits(TECS_Resources.Params.CruiseThrottle + TECS_Resources.Throttle.Correction, TECS_Resources.Params.MinCruiseThrottle, TECS_Resources.Params.MaxCruiseThrottle);
        RC_Resources.Attitude.Controller[THROTTLE] = Constrain_16Bits(TECS_Resources.Throttle.Cruise, RC_Resources.Attitude.ThrottleMin, RC_Resources.Attitude.ThrottleMax);
    }

    if (TECS_Resources.Params.DoLandAfterRTH && IS_FLIGHT_MODE_ACTIVE(RTH_MODE) && Fixed_Wing_HomePointReached)
    {
        if (TECS_Resources.Position.Altitude <= ConverMetersToCM(TECS_Resources.Params.LandMinAltitude))
        {
            RC_Resources.Attitude.Controller[THROTTLE] = RC_Resources.Attitude.ThrottleMin;
            GPS_Resources.Navigation.AutoPilot.Control.Angle[ROLL] = 0;
            GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH] = PIDAngleToRcController(ConvertDegreesToDecidegrees(TECS_Resources.Params.FinalLandPitchAngle),
                                                                                             ConvertDegreesToDecidegrees(GET_SET[MAX_PITCH_LEVEL].MaxValue));
        }
    }
}

void TecsClass::UpdateEnergyPositionController(float DeltaTime)
{
    TECS_Resources.Position.Error.X = TECS_Resources.Position.DestinationNEU.X - INS.EarthFrame.Position[INS_LATITUDE];
    TECS_Resources.Position.Error.Y = TECS_Resources.Position.DestinationNEU.Y - INS.EarthFrame.Position[INS_LONGITUDE];
    TECS_Resources.Position.Target.Distance = sqrtf(SquareFloat(TECS_Resources.Position.Error.X) + SquareFloat(TECS_Resources.Position.Error.Y));
    TECS_Resources.Position.Tracking.Actual = TECS_Resources.Position.Tracking.Period * MAX(TECS_Resources.Position.VelocityXY, 100.0f);

    TECS_Resources.Position.Circle.Flags.OkToRun = (TECS_Resources.Position.Target.Distance <= (TECS_Resources.Params.Circle_Radius / 0.26795f)) &&
                                                   (TECS_Resources.Position.Target.Distance > 50.0f) && !IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE);

    if (TECS_Resources.Position.Circle.Flags.OkToRun)
    {
        TECS_Resources.Position.Circle.Angle = Fast_Atan2(-TECS_Resources.Position.Error.Y, -TECS_Resources.Position.Error.X) + ConvertToRadians(TECS_Resources.Params.CircleDirectionToRight ? 1 : -1 * 45.0f);
        TECS_Resources.Position.Circle.Target.X = TECS_Resources.Position.DestinationNEU.X + TECS_Resources.Params.Circle_Radius * Fast_Cosine(TECS_Resources.Position.Circle.Angle);
        TECS_Resources.Position.Circle.Target.Y = TECS_Resources.Position.DestinationNEU.Y + TECS_Resources.Params.Circle_Radius * Fast_Sine(TECS_Resources.Position.Circle.Angle);
        TECS_Resources.Position.Error.X = TECS_Resources.Position.Circle.Target.X - INS.EarthFrame.Position[INS_LATITUDE];
        TECS_Resources.Position.Error.Y = TECS_Resources.Position.Circle.Target.Y - INS.EarthFrame.Position[INS_LONGITUDE];
        TECS_Resources.Position.Target.Distance = sqrtf(SquareFloat(TECS_Resources.Position.Error.X) + SquareFloat(TECS_Resources.Position.Error.Y));
    }

    TECS_Resources.Position.Virtual.X = INS.EarthFrame.Position[INS_LATITUDE] + TECS_Resources.Position.Error.X * (TECS_Resources.Position.Tracking.Actual / TECS_Resources.Position.Target.Distance);
    TECS_Resources.Position.Virtual.Y = INS.EarthFrame.Position[INS_LONGITUDE] + TECS_Resources.Position.Error.Y * (TECS_Resources.Position.Tracking.Actual / TECS_Resources.Position.Target.Distance);

    if (ABS(RC_Resources.Attitude.Controller[ROLL]) > POS_HOLD_DEADBAND)
    {
        TECS_Resources.Position.PilotManualAddRoll = RC_Resources.Attitude.Controller[ROLL] * TECS_Resources.Params.PilotManualRollSpeed / 500.0f * TECS_Resources.Position.Tracking.Period;
        //CONVERTE DE BODY-FRAME PARA EARTH-FRAME
        TECS_Resources.Position.Virtual.X += -TECS_Resources.Position.PilotManualAddRoll * INS.Math.Sine.Yaw;
        TECS_Resources.Position.Virtual.Y += TECS_Resources.Position.PilotManualAddRoll * INS.Math.Cosine.Yaw;
    }

    TECS_Resources.Heading.AHRSYawInCentiDegress = ConvertDecidegreesToCentiDegrees(Attitude.EulerAngles.YawDecidegrees);

    TECS_Resources.Heading.TargetBearing = WRap_36000(ConvertRadiansToCentiDegrees(Fast_Atan2(TECS_Resources.Position.Virtual.Y - INS.EarthFrame.Position[INS_LONGITUDE], TECS_Resources.Position.Virtual.X - INS.EarthFrame.Position[INS_LATITUDE])));

    TECS_Resources.Heading.Error = WRap_18000(TECS_Resources.Heading.TargetBearing - TECS_Resources.Heading.AHRSYawInCentiDegress);

    if (ABS(TECS_Resources.Heading.Error) > TECS_Resources.Heading.MaxToRunTurnDirection)
    {
        TECS_Resources.Heading.Flags.ForceTurnDirection = true;
    }
    else if (ABS(TECS_Resources.Heading.Error) < TECS_Resources.Heading.MinToNormalizeTurnDirection && TECS_Resources.Heading.Flags.ForceTurnDirection)
    {
        TECS_Resources.Heading.Flags.ForceTurnDirection = false;
    }

    if (TECS_Resources.Heading.Flags.ForceTurnDirection)
    {
        TECS_Resources.Heading.Error = TECS_Resources.Params.CircleDirectionToRight ? 1 : -1 * ABS(TECS_Resources.Heading.Error);
    }

    if (Scheduler(&TECS_Resources.YawScheduler, SCHEDULER_SET_FREQUENCY(2, "Hz")))
    {
        TECS_Resources.Heading.Flags.ErrorTrasborded = (ABS(TECS_Resources.Heading.PreviousError) > ABS(TECS_Resources.Heading.Error));
        TECS_Resources.Heading.PreviousError = TECS_Resources.Heading.Error;
    }

    TECS_Resources.Heading.AutoPilot.Adjust = TECS.Floating_Point_PID(&TECS_PID_Position_Navigation,
                                                                      GET_SET[PID_GPS_POSITION_RATE].kP / 100.0f, GET_SET[PID_GPS_POSITION_RATE].kI / 100.0f, GET_SET[PID_GPS_POSITION_RATE].kD / 100.0f,
                                                                      TECS_Resources.Heading.AHRSYawInCentiDegress + TECS_Resources.Heading.Error,
                                                                      TECS_Resources.Heading.AHRSYawInCentiDegress,
                                                                      1.0f, 1.0f,
                                                                      -ConvertDegreesToCentiDegrees(GET_SET[NAV_ROLL_BANK_MAX].MaxValue),
                                                                      ConvertDegreesToCentiDegrees(GET_SET[NAV_ROLL_BANK_MAX].MaxValue),
                                                                      TECS_PID_USE_TRACKING_ERROR | (TECS_Resources.Heading.Flags.ErrorTrasborded ? TECS_PID_USE_SHRINK_INTEGRATOR : TECS_PID_USE_NONE),
                                                                      DeltaTime);

    TECS_Resources.Heading.AutoPilot.Adjust = PT1FilterApply(&TECS_Resources.PositionController_Smooth,
                                                             TECS_Resources.Heading.AutoPilot.Adjust,
                                                             0.01f * Power3_Float(10.0f - Constrain_Float(TECS_Resources.Params.AutoPilotLPFQuality, 0.0f, 9.0f)) + 0.1f,
                                                             DeltaTime);

    TECS_PID_Position_Navigation.AutoPilotControl[ROLL] = ConvertCentiDegreesToDeciDegrees(TECS_Resources.Heading.AutoPilot.Adjust);

    if (GetActualPlatformEnabledUsingName(AIR_PLANE) || GetActualPlatformEnabledUsingName(AIR_PLANE_VTAIL))
    {
        TECS_PID_Heading_Navigation.AutoPilotControl[YAW] = ConvertCentiDegreesToDegrees(TECS.Floating_Point_PID(&TECS_PID_Heading_Navigation,
                                                                                                                 GET_SET[PID_GPS_NAVIGATION_RATE].kP / 10.0f, GET_SET[PID_GPS_NAVIGATION_RATE].kI / 10.0f, GET_SET[PID_GPS_NAVIGATION_RATE].kD / 100.0f,
                                                                                                                 0,
                                                                                                                 TECS_Resources.Heading.Error,
                                                                                                                 1.0f, 1.0f,
                                                                                                                 -35000.0f,
                                                                                                                 35000.0f,
                                                                                                                 TECS_Resources.Heading.Flags.ErrorTrasborded ? TECS_PID_USE_SHRINK_INTEGRATOR : 0,
                                                                                                                 DeltaTime));
    }
    else
    {
        TECS_PID_Heading_Navigation.AutoPilotControl[YAW] = 0;
    }
}

float TecsClass::GetFuselageVelocity(void)
{
    return sqrtf(SquareFloat(INS.EarthFrame.Velocity[INS_LATITUDE]) + SquareFloat(INS.EarthFrame.Velocity[INS_LONGITUDE]));
}

void TecsClass::Reset_All(void)
{
    TECS.Reset_PID_Navigation(&TECS_PID_Altitude_Navigation, ALTITUDE_DERIVATIVE_CUTOFF);
    TECS.Reset_PID_Navigation(&TECS_PID_Position_Navigation, POSITION_DERIVATIVE_CUTOFF);
    TECS.Reset_PID_Navigation(&TECS_PID_Heading_Navigation, HEADING_DERIVATIVE_CUTOFF);
    TECS_Resources.Position.DestinationNEU.Clear();
    TECS_Resources.Position.Virtual.Clear();
    TECS_Resources.PositionController_Smooth.State = 0.0f;
    TECS_Resources.Throttle.SpeedAdjustment = 0.0f;
    TECS_PID_Altitude_Navigation.AutoPilotControl[PITCH] = 0;
    TECS_PID_Position_Navigation.AutoPilotControl[ROLL] = 0;
    GPS_Resources.Navigation.AutoPilot.Control.Angle[ROLL] = 0;
    GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH] = 0;
    GPS_Resources.Navigation.AutoPilot.Control.Angle[YAW] = 0;
}

bool TecsClass::Scheduler(Scheduler_Struct *SchedulerPointer, uint32_t RefreshTime)
{
    uint32_t StoredTime = SCHEDULERTIME.GetMicros();
    uint32_t ActualTime = StoredTime - SchedulerPointer->StoredTime;
    SchedulerPointer->ActualTime = ActualTime;
    SchedulerPointer->StoredTime = StoredTime;
    if (ActualTime < RefreshTime)
    {
        return true;
    }
    return false;
}

bool TecsClass::GetStateToEnableTurnCoordinator(void)
{
    //INDICA PARA O CONTROLADOR PID QUE O TECS IRÁ UTILIZAR O RECURSO COORDENADOR DE CURVAS
    return (IS_FLIGHT_MODE_ACTIVE(ALTITUDE_HOLD_MODE) ||
            IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) ||
            IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE) ||
            IS_FLIGHT_MODE_ACTIVE(RTH_MODE));
}

void TecsClass::Update(float DeltaTime)
{
    if (!GetAirPlaneEnabled() || !AUTOLAUNCH.GetLaunchFinalized())
    {
        return;
    }

    if (ABS(RC_Resources.Attitude.Controller[PITCH]) > ALT_HOLD_DEADBAND)
    {
        TECS_Resources.Velocity.ClimbRate = -RC_Resources.Attitude.Controller[PITCH] * TECS_Resources.Params.PilotManualClimbDescentRate / (500.0f - ALT_HOLD_DEADBAND);
    }
    else
    {
        TECS_Resources.Velocity.ClimbRate = 0.0f;
    }

    TECS_Resources.Position.VelocityXY = TECS.GetFuselageVelocity();

    if (INERTIALNAVIGATION.WaitForSample() && !TECS_Resources.Position.HomePointOnce)
    {
        TECS_Resources.Position.HomePoint.X = INS.EarthFrame.Position[INS_LATITUDE] + INS.EarthFrame.Velocity[INS_LATITUDE];
        TECS_Resources.Position.HomePoint.Y = INS.EarthFrame.Position[INS_LONGITUDE] + INS.EarthFrame.Velocity[INS_LONGITUDE];
        TECS_Resources.Position.HomePointOnce = true;
    }
    else if (!INERTIALNAVIGATION.WaitForSample()) //SISTEMA DESARMADO
    {
        TECS_Resources.Position.HomePoint.X = 0.0f;
        TECS_Resources.Position.HomePoint.Y = 0.0f;
        TECS_Resources.Position.HomePointOnce = false;
    }

    if (TECS.Scheduler(&TECS_Resources.Scheduler, TECS_TIMER_US))
    {
        const float DeltaTime = TECS_Resources.Scheduler.ActualTime * 1e-6f;

        TECS_Resources.Position.Altitude = Barometer.INS.Altitude.Estimated;

        if (TECS.GetStateToEnableTurnCoordinator())
        {
            //O CONTROLADOR PID VEM DEPOIS DO TECS NO MAIN LOOP,ENTÃO ATIVAR O TURN AQUI FUNCIONA,
            //O "AUXFLIGHT.cpp" NÃO IRÁ ATRAPALHAR POR QUE ELE ESTÁ EM UMA TASK DE 50HZ
            ENABLE_THIS_FLIGHT_MODE(TURN_MODE);
        }

        if (IS_FLIGHT_MODE_ACTIVE_ONCE(ALTITUDE_HOLD_MODE) ||
            IS_FLIGHT_MODE_ACTIVE_ONCE(CIRCLE_MODE) ||
            IS_FLIGHT_MODE_ACTIVE_ONCE(CRUISE_MODE) ||
            IS_FLIGHT_MODE_ACTIVE_ONCE(RTH_MODE))
        {
            //EM MODO RTH,SE A ALTITUDE DO RTH FOR MENOR DO QUE ALTITUDE ESTIMADA,ESSA VARIAVEL IRÁ GANHAR UM NOVO VALOR NA FUNÇÃO SEGUINTE
            TECS_Resources.Position.DestinationNEU.Altitude = TECS_Resources.Position.Altitude;

            if (GPS_Resources.Mode.Navigation == DO_POSITION_HOLD)
            {
                TECS_Resources.Position.DestinationNEU.X = INS.Position.Hold[INS_LATITUDE];
                TECS_Resources.Position.DestinationNEU.Y = INS.Position.Hold[INS_LONGITUDE];
            }
            else if (GPS_Resources.Mode.Navigation == DO_START_RTH)
            {
                TECS_Resources.Position.DestinationNEU.X = TECS_Resources.Position.HomePoint.X;
                TECS_Resources.Position.DestinationNEU.Y = TECS_Resources.Position.HomePoint.Y;
                //ADICIONA UM NOVO VALOR DE ALTITUDE
                if (TECS_Resources.Position.Altitude < ConverMetersToCM(GPS_Resources.Home.Altitude))
                {
                    TECS_Resources.Position.DestinationNEU.Altitude = ConverMetersToCM(GPS_Resources.Home.Altitude);
                }
            }
        }

        //ESPERA O PROXIMO CICLO DE MAQUINA COM AS VERIFICAÇÕES DO AUXFLIGHT.cpp E FLIGHTMODES.cpp
        if (GPS_Resources.Navigation.AutoPilot.Control.Enabled)
        {
            if (IS_FLIGHT_MODE_ACTIVE(CLIMBOUT_MODE))
            {
                if (!GetActualThrottleStatus(THROTTLE_LOW))
                {
                    TECS_Resources.Position.DestinationNEU.Altitude += TECS_Resources.Velocity.ClimbRate * DeltaTime;
                    TECS_Resources.Position.DestinationNEU.Altitude = Constrain_32Bits(TECS_Resources.Position.DestinationNEU.Altitude,
                                                                                       TECS_Resources.Position.Altitude - 500,
                                                                                       TECS_Resources.Position.Altitude + 500);
                    TECS.UpdateEnergyAltitudeController(DeltaTime);
                }
                else
                {
                    TECS.Reset_PID_Navigation(&TECS_PID_Altitude_Navigation, ALTITUDE_DERIVATIVE_CUTOFF);
                    TECS_PID_Altitude_Navigation.AutoPilotControl[PITCH] = 0;
                    GPS_Resources.Navigation.AutoPilot.Control.Angle[PITCH] = 0;
                    TECS_Resources.Throttle.SpeedAdjustment = 0.0f;
                    //O HOME-POINT COMPARTILHA A MESMA VARIAVEL PARA SETAR UMA NOVA ALTITUDE DE NAVEGAÇÃO,
                    //ESSE IF SERVE PARA NÃO DEIXAR A ALTITUDE MUDAR EM RTH COM O THROTTLE A BAIXO NIVEL.
                    if (GPS_Resources.Mode.Navigation == DO_POSITION_HOLD)
                    {
                        TECS_Resources.Position.DestinationNEU.Altitude = TECS_Resources.Position.Altitude;
                    }
                }
            }

            if (IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) || IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE))
            {
                TECS.UpdateEnergyPositionController(DeltaTime);
            }

            if (IS_FLIGHT_MODE_ACTIVE(CLIMBOUT_MODE) || IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) || IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE))
            {
                TECS.UpdateAutoPilotControl(DeltaTime);
                RC_Resources.Attitude.Controller[YAW] = GPS_Resources.Navigation.AutoPilot.Control.Angle[YAW];
            }
        }
        else
        {
            TECS.Reset_All();
        }
    }

#ifdef PRINTLN_TECS

    DEBUG("%.f %.f %.f %.f %.f %.f",
          TECS_Resources.Position.DestinationNEU.X,
          TECS_Resources.Position.DestinationNEU.Y,
          TECS_Resources.Position.HomePoint.X,
          TECS_Resources.Position.HomePoint.Y,
          INS.EarthFrame.Position[INS_LATITUDE],
          INS.EarthFrame.Position[INS_LONGITUDE]);

    /*
    DEBUG("%ld %.2f %.2f ",
          TECS_Resources.Position.DestinationNEU.Altitude,
          TECS_Resources.Position.Virtual.X,
          TECS_Resources.Position.Virtual.Y);
*/
/*
    DEBUG("%d %d",
          (int16_t)TECS.GetFuselageVelocity(),
          GPS_Resources.Navigation.Misc.Get.GroundSpeed);
*/
#endif
}