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

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "ENUM.h"
#include <inttypes.h>

typedef struct
{
  struct Accelerometer_Struct
  {
    uint8_t Temperature = 0;
    int16_t Read[3] = {0, 0, 0};
    float ReadFloat[3] = {0.0f, 0.0f, 0.0f};

    struct Gravity_Struct
    {
      bool Initialization = false;
      int16_t OneG = 256;
      float Value = 0;
    } GravityForce;

  } Accelerometer;

  struct Gyroscope_Struct
  {
    float Scale = 1.0f;
    int16_t Read[3] = {0, 0, 0};
    float ReadFloat[3] = {0.0f, 0.0f, 0.0f};
  } Gyroscope;

  struct Compass_Struct
  {
    uint8_t Type = 0;
    int16_t Read[3] = {0, 0, 0};
  } Compass;

} IMU_Struct;

typedef struct
{
  //LPF
  float AccelerationEarthFrame_LPF[3] = {0, 0, 0};

  //AVERAGE
  uint8_t AccelerationEarthFrame_Sum_Count[3] = {0, 0, 0};
  float AccelerationEarthFrame_Filtered[3] = {0, 0, 0};
  float AccelerationEarthFrame_Sum[3] = {0, 0, 0};

  struct Math_Struct
  {
    struct Cosine_Struct
    {
      float Yaw = 0.0f;
    } Cosine;

    struct Sine_Struct
    {
      float Yaw = 0.0f;
    } Sine;

  } Math;

  struct History_Struct
  {
    uint8_t XYCount = 0;
    uint8_t ZCount = 0;
    int32_t XYPosition[2][10] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    int32_t ZPosition[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  } History;

  struct EarthFrame_Struct
  {
    float AccelerationNEU[3] = {0, 0, 0};
    float Velocity[3] = {0, 0, 0};
    float Position[3] = {0, 0, 0};
  } EarthFrame;

  struct Bias_Struct
  {
    float Adjust[3] = {0, 0, 0};
    float Difference[3] = {0, 0, 0};
  } Bias;

  struct Position_Struct
  {
    int32_t Hold[2] = {0, 0};
  } Position;

} INS_Struct;

typedef struct
{
  struct Calibration_Struct
  {
    float GroundPressure = 0;
    float GroundTemperature = 0;
  } Calibration;

  struct Raw_Struct
  {
    int16_t Temperature = 0;
    int32_t Pressure = 0;
    int32_t PressureFiltered = 0;
  } Raw;

  struct Altitude_Struct
  {
    int32_t Actual = 0;
    int32_t GroundOffSet = 0;
  } Altitude;

  struct INS_Struct
  {
    struct Velocity_Struct
    {
      int16_t Vertical = 0;
    } Velocity;

    struct Altitude_Struct
    {
      int32_t Estimated = 0;
    } Altitude;

  } INS;

} Barometer_Struct;

typedef union
{
  int16_t Raw[3] = {0, 0, 0};

  struct EulerAngles_Struct
  {
    int16_t Roll;
    int16_t Pitch;
    int16_t Yaw;
    int16_t YawDecidegrees;
  } EulerAngles;

} Attitude_Struct;

typedef struct
{
  struct Accelerometer_Struct
  {

    struct Flags_Struct
    {
      bool Fail = false;
      bool InCalibration = false;
      bool CalibratedPosition[6] = {false, false, false, false, false, false};
      uint8_t State = 0;
    } Flags;

    struct Time_Struct
    {
      uint32_t Start = 0;
    } Time;

    struct Samples_Struct
    {
      float Sum[3] = {0, 0, 0};
      uint16_t Count = 0;
      int32_t Window[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    } Samples;

    int16_t OffSet[3] = {0, 0, 0};
    int16_t Scale[3] = {0, 0, 0};

    struct Gravity_Struct
    {
      struct Flags_Struct
      {
        bool Restart = true;
        bool Calibrated = false;
      } Flags;

      struct Time_Struct
      {
        uint32_t Previous = 0;
      } Time;

      struct Samples_Struct
      {
        int16_t Count = 0;
        float Sum = 0.0f;
      } Samples;

    } Gravity;

  } Accelerometer;

  struct Gyroscope_Struct
  {

    struct Flags_Struct
    {
      bool Calibrated = false;
      bool Restart = true;
    } Flags;

    struct Time_Struct
    {
      uint32_t Previous = 0;
    } Time;

    float Deviation[3] = {0.0f, 0.0f, 0.0f};

    struct Samples_Struct
    {
      int16_t Count = 0;
      int32_t Sum[3] = {0, 0, 0};
    } Samples;

  } Gyroscope;

  struct Magnetometer_Struct
  {
    bool Calibrating = false;
    float Difference = 0.0f;
    float Average = 0.0f;
    float GaussNewtonOffSet[3] = {0.0f, 0.0f, 0.0f};
    int16_t Previous[3] = {0, 0, 0};
    int16_t Deviation[3] = {0, 0, 0};
    int16_t Gain[3] = {1024, 1024, 1024};
    int16_t OffSet[3] = {0, 0, 0};
    int16_t Count = 0;
    int16_t SimpleModeHeading = 0;
  } Magnetometer;

} Calibration_Struct;

typedef struct
{
  bool UpdateRequired = false;
  float Factor = 0;
  float CalcedValue = 0;
  int16_t PreviousThrottle;
  int16_t BreakPointer = 1500;
  uint16_t ThrottlePercent = 0;
  uint16_t FixedWingTauMS = 0;
} TPA_Parameters_Struct;

typedef struct
{
  float ScaleDownOfLongitude = 1.0f;

  struct Declination_Struct
  {
    bool Pushed = false;
    uint8_t PushedCount = 0;
  } Declination;

  struct DeltaTime_Struct
  {
    float Navigation = 0.0f;
    uint32_t InitLand = 0;
  } DeltaTime;

  struct Mode_Struct
  {
    uint8_t Navigation = 0;
  } Mode;

  struct Home_Struct
  {
    bool Marked = false;
    uint8_t Altitude = 0;
    int16_t Direction = 0;
    uint16_t Distance = 0;
    int32_t Coordinates[2] = {0, 0};

    struct INS_Struct
    {
      int32_t Distance[2] = {0, 0};
    } INS;

  } Home;

  struct Navigation_Struct
  {
    bool LandAfterRTH = true;
    float HeadingHoldLimit = 0.0f;
    int16_t Speed[2] = {0, 0};
    int16_t RateError[2] = {0, 0};
    int16_t HeadingHoldTarget = 0;

    struct AutoPilot_Struct
    {
      struct INS_Struct
      {
        int16_t Angle[2] = {0, 0};
      } INS;

      struct Control_Struct
      {
        bool Enabled = false;
        uint8_t Mode = AUTOPILOT_MODE_ATTI;
        int16_t Angle[3] = {0, 0, 0};
      } Control;

    } AutoPilot;

    struct Coordinates_Struct
    {
      int32_t Actual[2] = {0, 0};
      int32_t Destiny[2] = {0, 0};
      int32_t Distance = 0;
    } Coordinates;

    struct Misc_Struct
    {
      struct Velocity_Struct
      {
        bool NEDStatus = false;
        int16_t Get[3] = {0, 0, 0};
      } Velocity;

      struct Get_Struct
      {
        bool Marked3DFix = false;
        bool HeadingInitialized = false;
        uint8_t Satellites = 0;
        uint16_t GroundCourse = 0;
        uint16_t Altitude = 0;
        uint16_t GroundSpeed = 0;
        uint16_t HDOP = 0;
        uint32_t EstimatedPositionHorizontal;
        uint32_t EstimatedPositionVertical;
      } Get;

    } Misc;

    struct Bearing_Struct
    {
      int16_t InitialTarget = 0;
      int32_t ActualTarget = 0;
      int32_t TargetPrev = 0;
    } Bearing;

  } Navigation;

} GPS_Resources_Struct;

typedef struct
{
  float OldMeasure = 0.0f;
  float NewMeasure = 0.0f;
  float OldValue = 0.0f;
  float NewValue = 0.0f;
  int16_t MeasureCount = 0;
} Device_Struct;

typedef struct
{
  float Roll;
  float Pitch;
  float Yaw;
} PID_Mixer_Struct;

typedef struct
{
  uint8_t FrameMotorsCount;
} Motors_Count_Struct;

typedef struct
{
  uint8_t Mode;
  uint8_t Priority;
  const uint8_t *Sequence;
} BeeperEntry_Struct;

typedef struct
{
  float kP_Accelerometer = 0.25f;
  float kI_Accelerometer = 0.0050f;
  float kP_Magnetometer = 1.0f;
  float kI_Magnetometer = 0.0f;
  float Cosine_Z = 25.0f;
} AHRS_Configuration_Struct;

typedef struct
{
  float q0 = 0.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
} Quaternion_Struct;

typedef union
{
  float Vector[3];

  struct
  {
    float Roll;
    float Pitch;
    float Yaw;
  };

} Vector3x3_Struct;

typedef struct
{
  float Matrix3x3[3][3] = {{0, 0, 0}, {0, 0, 0}};
} Matrix3x3_Struct;

typedef struct
{
  bool State = false;
  uint8_t kP = 0;
  uint8_t kI = 0;
  uint8_t kD = 0;
  uint8_t kFF = 0;
  uint8_t MaxValue = 0;
} PID_Terms_Struct;

typedef struct
{
  float kP = 0.0f;
  float kI = 0.0f;
  float kD = 0.0f;

  struct GPS_Struct
  {
    float IntegralSum = 0.0f;
    float IntegralMax = 0.0f;
    float LastDerivative = 0.0f;
    float DerivativeCalced = 0.0f;
    int32_t LastInput = 0;
  } GPSFilter;

} PID_Terms_Float_Struct;

typedef struct
{
  uint32_t ActualTime = 0;
  uint32_t StoredTime = 0;
} Scheduler_Struct;

typedef struct
{
  float State = 0.0f;
  float RC = 0.0f;
  float DeltaTime = 0.0f;
} PT1_Filter_Struct;

typedef struct
{
  const char *TaskName;
  void (*TaskFunction)();
  int32_t DesiredPeriod;
  const uint8_t StaticPriority;
  uint16_t DynamicPriority;
  uint16_t TaskAgeCycles;
  uint32_t LastExecuted;
  int32_t TaskLatestDeltaTime;
} Task_Resources_Struct;

typedef struct
{
  //MATRIX JACOBIANA
  //http://en.wikipedia.org/wiki/Jacobian_matrix
  float Matrix_JtR[4];
  float Matrix_JtJ[4][4];
} Jacobian_Struct;

typedef struct
{
  bool Healthy = false;

  struct Raw_Stuct
  {
    float Pressure = 0.0f;
    float IASPressure = 0.0f;
    float DifferentialPressure = 0.0f;
    uint16_t IASPressureInCM = 0;
  } Raw;

  struct Calibration_Stuct
  {
    bool Initialized = false;
    float OffSet = 0;
    float Sum = 0;
    uint16_t Count = 0;
    uint16_t Read_Count = 0;
    uint32_t Start_MS = 0;
  } Calibration;

  struct Param_Stuct
  {
    float Factor = 0.0f;
  } Param;

} AirSpeed_Struct;

typedef struct
{
  bool ValidWindEstimated = false;
  struct Ground_Struct
  {
    float Velocity[3] = {0, 0, 0};
    float VelocityDifference[3] = {0, 0, 0};
    float VelocitySum[3] = {0, 0, 0};
    float LastVelocity[3] = {0, 0, 0};
  } Ground;

  struct Fuselage_Struct
  {
    float Direction[3] = {0, 0, 0};
    float DirectionDifference[3] = {0, 0, 0};
    float DirectionSum[3] = {0, 0, 0};
    float LastDirection[3] = {0, 0, 0};
  } Fuselage;

  struct EarthFrame_Struct
  {
    float EstimatedWindVelocity[3] = {0, 0, 0};
  } EarthFrame;

  struct Time_Struct
  {
    uint32_t Now = 0;
    uint32_t LastUpdate = 0;
  } Time;

} WindEstimator_Struct;

typedef struct
{
  struct Pulse_Struct
  {
    int16_t Min[4] = {0, 0, 0, 0};
    int16_t Middle[4] = {0, 0, 0, 0};
    int16_t Max[4] = {0, 0, 0, 0};
  } Pulse;

  struct Direction_Struct
  {
    int8_t GetAndSet[4] = {1, 1, 1, 1};
  } Direction;

  struct Rate_Struct
  {
    int8_t GetAndSet[4] = {0, 0, 0, 0};
  } Rate;

  struct Weight_Struct
  {
    int8_t GetAndSet[4] = {0, 0, 0, 0};
  } Weight;

  struct Filter_Struct
  {
    int16_t CutOff = 0;
  } Filter;

  struct Signal_Struct
  {
    int16_t UnFiltered[4] = {0, 0, 0, 0};
    int16_t Filtered[4] = {0, 0, 0, 0};
  } Signal;

  struct AutoTrim_Struct
  {
    bool Enabled = false;
    int16_t ActualPulse[4] = {0, 0, 0, 0};
    int16_t MiddleBackup[4] = {0, 0, 0, 0};
    int32_t MiddleAccum[4] = {0, 0, 0, 0};
    int32_t MiddleAccumCount = 0;
    uint32_t PreviousTime = 0;
    uint32_t SavePreviousTime = 0;
  } AutoTrim;

  struct ContinousTrim_Struct
  {
    bool Enabled = false;
  } ContinousTrim;

} Servo_Struct;

typedef struct
{
  struct Auto_Struct
  {
    uint8_t MinVoltageType = 0;
    uint8_t MaxVoltageType = 0;
    uint16_t MinCount = 0;
    uint16_t MaxCount = 0;
  } Auto;

  struct Calced_Struct
  {
    float Voltage = 0.0f;
    float Current = 0.0f;
    float CurrentInMah = 0.0f;
    float Percentage = 0.0f;
  } Calced;

  struct Exhausted_Struct
  {
    bool LowPercentPreventArm = false;
    uint8_t LowBatteryCount = 0;
  } Exhausted;

  struct Param_Struct
  {
    bool Do_RTH = false;
    uint8_t NumberOfCells = 0;
    uint8_t CriticPercent = 0;
    float Voltage_Factor = 0.0f;
    float Amps_Per_Volt = 0.0f;
    float Amps_OffSet = 0.0f;
    float Min_Cell_Volt = 0.0f;
    float Max_Cell_Volt = 0.0f;
  } Param;

} Battery_Struct;

typedef struct
{
  const char *Param_Name;
  const uint16_t Address;
  const uint8_t Variable_Type;
  void *Ptr;
  const int32_t Value_Min;
  const int32_t Value_Max;
  const float DefaultValue;
} Resources_Of_Param;

typedef struct
{
#ifndef __AVR_ATmega2560__
  uint8_t kP_Acc_AHRS;
  uint8_t kI_Acc_AHRS;
  uint8_t kP_Mag_AHRS;
  uint8_t kI_Mag_AHRS;
  uint8_t AngleLevelBlockArm;
  uint8_t AutoLaunch_AHRS_BankAngle;
  uint8_t AutoLaunch_Velocity_Thresh;
  uint16_t AutoLaunch_Trigger_Motor_Delay;
  uint8_t AutoLaunch_Elevator;
  uint16_t AutoLaunch_SpinUp;
  uint16_t AutoLaunch_SpinUp_Time;
  uint16_t AutoLaunch_MaxThrottle;
  uint16_t AutoLaunch_Exit;
  uint8_t AutoLaunch_Altitude;
  uint8_t CrashCheck_BankAngle;
  uint8_t CrashCheck_Time;
  uint16_t GimbalMinValue;
  uint16_t GimbalMaxValue;
  uint8_t Land_Check_Acc;
  float Throttle_Mix_Gain;
  uint8_t AutoDisarm_Time;
  uint16_t AutoDisarm_Throttle_Min;
  uint16_t AutoDisarm_YPR_Min;
  uint16_t AutoDisarm_YPR_Max;
  uint8_t GPS_Baud_Rate;
#endif
  uint16_t Navigation_Vel;
  uint8_t GPS_WP_Radius;
  uint8_t GPS_RTH_Land_Radius;
  uint8_t GPS_TiltCompensation;
#ifndef __AVR_ATmega2560__
  uint8_t AirSpeed_Samples;
#endif
  uint8_t Arm_Time_Safety;
  uint8_t Disarm_Time_Safety;
  uint8_t Compass_Cal_Timer;
  uint8_t Continuous_Servo_Trim_Rot_Limit;
} JCF_Param_Adjustable_Struct;

typedef struct
{
  struct Buffer_Struct
  {
    uint8_t Data[6] = {0, 0, 0, 0, 0, 0};
  } Buffer;

  struct Found_Struct
  {
    bool Junk = false; //SE TIRAR ESSA BOOL,A BOOL DO COMPASS (A PROXIMA) PASSA A NÃO FUNCIONAR,POR QUE ISSO ACONTECE??GCC??
    bool Compass = false;
    bool Barometer = false;
  } Found;

  struct Error_Struct
  {
    int16_t Count = 0;
  } Error;

} I2C_Resources_Struct;

typedef struct
{
  struct Filter_Struct
  {
    int16_t DerivativeCutOff = 0;
    int16_t IntegralRelaxCutOff = 0;
    int16_t ControlDerivativeCutOff = 0;
  } Filter;

  struct RcRateTarget_Struct
  {
    float Roll = 0.0f;
    float Pitch = 0.0f;
    float Yaw = 0.0f;

    struct GCS_Struct
    {
      int16_t Roll = 0;
      int16_t Pitch = 0;
      int16_t Yaw = 0;
    } GCS;

  } RcRateTarget;

  struct Controller_Struct
  {

    struct Output_Struct
    {
      int16_t Calced[3] = {0, 0, 0};
    } Output;

    struct Integral_Struct
    {
      float ErrorGyro[3] = {0.0f, 0.0f, 0.0f};
      float ErrorGyroLimit[3] = {0.0f, 0.0f, 0.0f};
    } Integral;

  } Controller;

  struct Param_Struct
  {
    uint8_t IntegralTermWindUpPercent = 0;
    float PitchLevelTrim = 0.0f;
    int16_t ReferenceAirSpeed = 1500;
  } Param;

} PID_Resources_Struct;

typedef struct
{
  struct Flags_Struct
  {
    bool TakeOffInProgress = false;
    bool GroundAltitudeSet = false;
    bool Hovering = false;
  } Flags;

  struct Time_Struct
  {
    uint32_t LandDetectorStart = 0;
    uint32_t OnLand = 0;
  } Time;

  struct Target_Struct
  {

    struct Velocity_Struct
    {
      int32_t Z = 0;
    } Velocity;

    struct Position_Struct
    {
      int32_t Z = 0;
    } Position;

    int32_t Altitude = 0;

  } Target;

  struct Throttle_Struct
  {
    int16_t Hover = 1500;
    int16_t Hovering = 0;
    int16_t Difference = 0;
  } Throttle;

  struct PID_Struct
  {
    int16_t Control = 0;
    int16_t IntegratorError = 0;
    int32_t IntegratorSum = 0;
  } PID;

} AltitudeHold_Controller_Struct;

typedef struct
{
  bool Reset = false;
  float Error = 0.0f;
  float PreviousMeasurement = 0.0f;
  float Derivative = 0.0f;
  float DerivativeCutOff = 0.0f;
  float ControlTracking = 0.0f;
  float IntegratorSum = 0.0f;
  float PreviousIntegrator = 0.0f;
  float ValueConstrained = 0.0f;
  int16_t AutoPilotControl[3] = {0, 0, 0};
  PT1_Filter_Struct Derivative_Smooth;
} TECS_PID_Float_Struct;

typedef struct
{
  float X = 0.0f;
  float Y = 0.0f;
  int32_t Altitude = 0;
  void Clear(void)
  {
    X = 0.0f;
    Y = 0.0f;
    Altitude = 0;
  }
} Frame3D_Struct;

typedef struct
{
  Scheduler_Struct Scheduler;
  Scheduler_Struct YawScheduler;

  PT1_Filter_Struct PitchToThrottle_Smooth;
  PT1_Filter_Struct PositionController_Smooth;
  PT1_Filter_Struct PitchController_Smooth;

  struct Position_Struct
  {
    Frame3D_Struct DestinationNEU;
    Frame3D_Struct Virtual;
    Frame3D_Struct HomePoint;

    struct AutoPilot_Struct
    {
      int16_t RollAngle = 0;
    } AutoPilot;

    struct Target_Struct
    {
      float Distance = 0.0f;
    } Target;

    struct Error_Struct
    {
      float X = 0.0f;
      float Y = 0.0f;
    } Error;

    struct Circle_Struct
    {

      struct Flags_Struct
      {
        bool OkToRun = false;
      } Flags;

      float Angle = 0.0f;

      struct Target_Struct
      {
        float X = 0.0f;
        float Y = 0.0f;
      } Target;

    } Circle;

    struct Tracking_Struct
    {
      float Actual = 0.0f;
      float Period = 0.0f;
    } Tracking;

    bool HomePointOnce = false;
    float PilotManualAddRoll = 0.0f;
    float VelocityXY = 0.0f;
    int32_t Altitude = 0;
  } Position;

  struct Velocity_Struct
  {
    float ClimbRate = 0.0f;
  } Velocity;

  struct Heading_Struct
  {

    struct Flags_Struct
    {
      bool ErrorTrasborded = false;
      bool ForceTurnDirection = false;
    } Flags;

    struct AutoPilot_Struct
    {
      float Adjust = 0.0f;
    } AutoPilot;

    float AHRSYawInCentiDegress = 0.0f;
    float MinToNormalizeTurnDirection = 0.0f;
    float MaxToRunTurnDirection = 0.0f;
    int32_t TargetBearing = 0;
    int32_t Error = 0;
    int32_t PreviousError = 0;
  } Heading;

  struct Energies_Struct
  {
    struct Specific_Struct
    {
      struct Demanded_Struct
      {
        float PotentialEnergy = 0.0f;
        float KineticEnergy = 0.0f;
        float EnergyBalance = 0.0f;
      } Demanded;

      struct Estimated_Struct
      {
        float PotentialEnergy = 0.0f;
        float KineticEnergy = 0.0f;
        float EnergyBalance = 0.0f;
        float PitchGainBalance = 0.0f;
        float PitchAngle = 0.0f;
      } Estimated;

      float Control = 0.0f;
    } Specific;

  } Energies;

  struct Throttle_Struct
  {
    float SpeedBoost = 0.0f;
    float SpeedAdjustment = 0.0f;
    int16_t Correction = 0;
    int16_t Cruise = 0;
  } Throttle;

  struct Params_Struct
  {
    bool CircleDirectionToRight = false;
    bool DoLandAfterRTH = false;
    uint8_t LandMinAltitude = 0;
    uint8_t FinalLandPitchAngle = 0;
    uint8_t PitchToThrottleLPFQuality = 0;
    uint8_t PitchToThrottleDifference = 0;
    uint8_t PitchToThrottleFactor = 0;
    uint8_t AutoPilotLPFQuality = 0;
    float AutoPilotMaxDescentAngle = 0.0f;
    float AutoPilotMaxClimbAngle = 0.0f;
    float AutoThrottleGain = 0.0f;
    float AutoThrottleMinVel = 0.0f;
    int16_t PilotManualRollSpeed = 0;
    int16_t PilotManualClimbDescentRate = 0;
    int16_t MinCruiseThrottle = 0;
    int16_t MaxCruiseThrottle = 0;
    int16_t CruiseThrottle = 0;
    int16_t Circle_Radius = 0;
  } Params;

} TECS_Resources_Struct;

typedef struct
{

  struct Rate_Struct
  {
    uint8_t PitchRoll = 0;
    uint8_t Yaw = 0;
  } Rate;

  struct Expo_Struct
  {
    uint8_t Throttle = 0;
    uint8_t Yaw = 0;
    uint8_t PitchRoll = 0;
  } Expo;

  struct Middle_Struct
  {
    uint8_t Throttle = 0;
  } Middle;

  struct Attitude_Struct
  {
    int16_t ThrottleMin = 0;
    int16_t ThrottleMax = 0;
    int16_t Controller[4] = {0, 0, 0, 0};
  } Attitude;

  uint8_t ReceiverTypeEnabled = PPM_RECEIVER;
  uint8_t ReceiverSequency = 0;
} RC_Resources_Struct;
#endif
