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

#include "IOMCU.h"
#include "FastSerial/FASTSERIAL.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "I2C/I2C.h"
#include "AHRS/AHRS.h"
#include "FlightModes/AUXFLIGHT.h"
#include "RadioControl/RCCONFIG.h"
#include "Barometer/BAROREAD.h"
#include "Barometer/BAROBACKEND.h"
#include "BatteryMonitor/BATTERY.h"
#include "LedRGB/LEDRGB.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "Buzzer/BUZZER.h"
#include "GPSNavigation/NAVIGATION.h"
#include "ParamsToGCS/IMUCALGCS.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "MemoryCheck/FREERAM.h"
#include "AirSpeed/AIRSPEED.h"
#include "Build/VERSION.h"
#include "ProgMem/PROGMEM.h"
#include "WatchDog/REBOOT.h"
#include "Arming/ARMING.h"
#include "PID/PIDXYZ.h"
#include "Scheduler/SCHEDULER.h"
#include "TaskSystem/TASKSYSTEM.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Compass/COMPASSREAD.h"
#include "FailSafe/FAILSAFE.h"
#include "GPS/GPSUBLOX.h"
#include "PID/RCPID.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "IMU/ACCGYROREAD.h"
#include "PID/PIDPARAMS.h"
#include "BitArray/BITARRAY.h"
#include "RadioControl/DECODE.h"
#include "Build/BOARDDEFS.h"

GCSClass GCS;

//#define MACHINE_CYCLE

uint8_t SerialCheckSum;
uint8_t ProtocolCommand;
uint8_t SerialInputBuffer[64];
uint8_t SerialOutputBuffer[128];
uint8_t SerialOutputBufferSizeCount;
uint8_t OutputVectorCount;
uint8_t VectorCount;
uint8_t SerialBuffer;
uint8_t SerialAvailableGuard;
uint8_t ProtocolTaskOrder;
uint8_t SerialOffSet;
uint8_t SerialDataSize;
uint8_t PreviousProtocolTaskOrder;

struct _Essential_First_Packet_Parameters
{
    int16_t SendAttitudePitch;
    int16_t SendAttitudeRoll;
    int16_t SendAttitudeYaw;
    uint8_t DevicesOnBoard;
    uint16_t SendThrottleValue;
    uint16_t SendYawValue;
    uint16_t SendPitchValue;
    uint16_t SendRollValue;
    uint16_t SendAuxOneValue;
    uint16_t SendAuxTwoValue;
    uint16_t SendAuxThreeValue;
    uint16_t SendAuxFourValue;
    uint16_t SendAuxFiveValue;
    uint16_t SendAuxSixValue;
    uint16_t SendAuxSevenValue;
    uint16_t SendAuxEightValue;
    uint8_t SendGPSNumberOfSat;
    int32_t SendGPSLatitude;
    int32_t SendGPSLongitude;
    int32_t SendHomePointLatitude;
    int32_t SendHomePointLongitude;
    int32_t SendBarometerValue;
    uint8_t SendFailSafeState;
    uint16_t SendBatteryVoltageValue;
    uint8_t SendBatteryPercentageValue;
    uint8_t SendArmDisarmState;
    uint16_t SendHDOPValue;
    uint16_t SendCurrentValue;
    uint32_t SendWattsValue;
    int16_t SendDeclinationValue;
    uint8_t SendActualFlightMode;
    uint8_t SendFrameType;
    uint8_t SendHomePointState;
    uint8_t SendTemperature;
    uint16_t SendHomePointDistance;
    uint16_t SendCurrentInMah;
    uint16_t SendCourseOverGround;
    int16_t SendBearing;
    int16_t SendAccGForce;
    uint8_t SendAccImageBitMap;
    int16_t SendCompassRoll;
    int16_t SendCompassPitch;
    int16_t SendCompassYaw;
} Essential_First_Packet_Parameters;

struct _Essential_Second_Packet_Parameters
{
    uint16_t SendActualThrottleValue;
    uint16_t SendActualYawValue;
    uint16_t SendActualPitchValue;
    uint16_t SendActualRollValue;
    uint16_t SendActualAuxOneValue;
    uint16_t SendActualAuxTwoValue;
    uint16_t SendActualAuxThreeValue;
    uint16_t SendActualAuxFourValue;
    uint16_t SendActualAuxFiveValue;
    uint16_t SendActualAuxSixValue;
    uint16_t SendActualAuxSevenValue;
    uint16_t SendActualAuxEightValue;
    int16_t SendAttitudeThrottleValue;
    int16_t SendAttitudeYawValue;
    int16_t SendAttitudePitchValue;
    int16_t SendAttitudeRollValue;
    uint16_t SendMemoryRamUsed;
    uint8_t SendMemoryRamUsedPercent;
    int16_t SendAccX;
    int16_t SendAccY;
    int16_t SendAccZ;
    int16_t SendGyroX;
    int16_t SendGyroY;
    int16_t SendGyroZ;
    uint16_t SendGPSGroundSpeed;
    int16_t SendI2CError;
    uint16_t SendAirSpeedValue;
    uint8_t SendCPULoad;
} Essential_Second_Packet_Parameters;

struct _Send_User_Basic_Parameters
{
    uint8_t SendFrameType;
    uint8_t SendRcChSequency;
    uint8_t SendGimbalType;
    uint8_t SendParachuteType;
    uint8_t SendUartNumb1Type;
    uint8_t SendUartNumb2Type;
    uint8_t SendUartNumb3Type;
    uint8_t SendCompassRotationType;
    uint8_t SendRTHAltitude;
    uint8_t SendAcroType;
    uint8_t SendAltitudeHoldType;
    uint8_t SendPositionHoldType;
    uint8_t SendSimpleControlType;
    uint8_t SendReturnToHomeType;
    uint8_t SendAtackType;
    uint8_t SendAutomaticFlipType;
    uint8_t SendAutomaticMissonType;
    uint8_t SendArmDisarmType;
    uint8_t SendAutoLandType;
    uint8_t SendSafeBtnState;
    uint8_t SendAirSpeedState;
    int16_t SendPitchLevelTrim;
    int16_t SendBatteryVoltageScale;
    int16_t SendBatteryCurrentScale;
    int16_t SendBatteryCurrentOffSet;
    int16_t SendBattMinVoltage;
    int16_t SendBattMaxVoltage;
    uint8_t SendNumberOfCells;
    uint8_t SendCriticBattPercent;
    uint8_t SendRTHLowBatt;
} Send_User_Basic_Parameters;

struct _Get_User_Basic_Parameters
{
    uint8_t GetFrameType;
    uint8_t GetRcChSequency;
    uint8_t GetGimbalType;
    uint8_t GetParachuteType;
    uint8_t GetUartNumb1Type;
    uint8_t GetUartNumb2Type;
    uint8_t GetUartNumb3Type;
    uint8_t GetCompassRotationType;
    uint8_t GetRTHAltitude;
    uint8_t GetAcroType;
    uint8_t GetAltitudeHoldType;
    uint8_t GetPositionHoldType;
    uint8_t GetSimpleControlType;
    uint8_t GetReturnToHomeType;
    uint8_t GetAtackType;
    uint8_t GetAutomaticFlipType;
    uint8_t GetAutomaticMissonType;
    uint8_t GetArmDisarmType;
    uint8_t GetAutoLandType;
    uint8_t GetSafeBtnState;
    uint8_t GetAirSpeedState;
    int16_t GetPitchLevelTrim;
    int16_t GetBatteryVoltageScale;
    int16_t GetBatteryCurrentScale;
    int16_t GetBatteryCurrentOffSet;
    int16_t GetBattMinVoltage;
    int16_t GetBattMaxVoltage;
    uint8_t GetNumberOfCells;
    uint8_t GetCriticBattPercent;
    uint8_t GetRTHLowBatt;
} Get_User_Basic_Parameters;

struct _Send_Radio_Control_Parameters
{
    uint8_t SendThrottleMiddle;
    uint8_t SendThrottleExpo;
    uint8_t SendPRRate;
    uint8_t SendPRExpo;
    uint8_t SendYawRate;
    uint8_t SendYawExpo;
    int16_t SendRCPulseMin;
    int16_t SendRCPulseMax;
    int16_t SendThrottleMin;
    int16_t SendYawMin;
    int16_t SendPitchMin;
    int16_t SendRollMin;
    int16_t SendThrottleMax;
    int16_t SendYawMax;
    int16_t SendPitchMax;
    int16_t SendRollMax;
    uint8_t SendThrottleDeadZone;
    uint8_t SendYawDeadZone;
    uint8_t SendPitchDeadZone;
    uint8_t SendRollDeadZone;
    uint8_t SendChannelsReverse;
    int16_t SendServo1Rate;
    int16_t SendServo2Rate;
    int16_t SendServo3Rate;
    int16_t SendServo4Rate;
    uint8_t SendServosReverse;
    int16_t SendServo1Min;
    int16_t SendServo2Min;
    int16_t SendServo3Min;
    int16_t SendServo4Min;
    int16_t SendServo1Med;
    int16_t SendServo2Med;
    int16_t SendServo3Med;
    int16_t SendServo4Med;
    int16_t SendServo1Max;
    int16_t SendServo2Max;
    int16_t SendServo3Max;
    int16_t SendServo4Max;
    int16_t SendFailSafeValue;
    uint8_t SendMaxBankPitch;
    uint8_t SendMaxBankRoll;
    uint8_t SendAutoPilotMode;
    int32_t SendAirSpeedScale;
    uint8_t SendTunningChannel;
    uint8_t SendTunning;
    uint8_t SendLandAfterRTH;
    int16_t SendHoverThrottle;
    int16_t SendAirSpeedReference;
    uint8_t SendTECSPitch2ThrFactor;
    uint8_t SendTECSPitch2ThrLPF;
    uint8_t SendTECSAutoPilotLPF;
    int16_t SendTECSCruiseMinThrottle;
    int16_t SendTECSCruiseMaxThrottle;
    int16_t SendTECSCruiseThrottle;
    uint8_t SendTECSCircleDirection;
    uint8_t SendContServosTrimState;
} Send_Radio_Control_Parameters;

struct _Get_Radio_Control_Parameters
{
    uint8_t GetThrottleMiddle;
    uint8_t GetThrottleExpo;
    uint8_t GetPRRate;
    uint8_t GetPRExpo;
    uint8_t GetYawRate;
    uint8_t GetYawExpo;
    int16_t GetRCPulseMin;
    int16_t GetRCPulseMax;
    int16_t GetThrottleMin;
    int16_t GetYawMin;
    int16_t GetPitchMin;
    int16_t GetRollMin;
    int16_t GetThrottleMax;
    int16_t GetYawMax;
    int16_t GetPitchMax;
    int16_t GetRollMax;
    uint8_t GetThrottleDeadZone;
    uint8_t GetYawDeadZone;
    uint8_t GetPitchDeadZone;
    uint8_t GetRollDeadZone;
    uint8_t GetChannelsReverse;
    int16_t GetFailSafeValue;
    uint8_t GetMaxBankPitch;
    uint8_t GetMaxBankRoll;
    uint8_t GetdAutoPilotMode;
    int32_t GetAirSpeedScale;
    uint8_t GetTunningChannel;
    uint8_t GetTunning;
    uint8_t GetLandAfterRTH;
    int16_t GetHoverThrottle;
    int16_t GetAirSpeedReference;
    uint8_t GetContServosTrimState;
} Get_Radio_Control_Parameters;

struct _Get_Servos_Parameters
{
    int16_t GetServo1Rate;
    int16_t GetServo2Rate;
    int16_t GetServo3Rate;
    int16_t GetServo4Rate;
    uint8_t GetServosReverse;
    int16_t GetServo1Min;
    int16_t GetServo2Min;
    int16_t GetServo3Min;
    int16_t GetServo4Min;
    int16_t GetServo1Med;
    int16_t GetServo2Med;
    int16_t GetServo3Med;
    int16_t GetServo4Med;
    int16_t GetServo1Max;
    int16_t GetServo2Max;
    int16_t GetServo3Max;
    int16_t GetServo4Max;
    uint8_t GetTECSPitch2ThrFactor;
    uint8_t GetTECSPitch2ThrLPF;
    uint8_t GetTECSAutoPilotLPF;
    int16_t GetTECSCruiseMinThrottle;
    int16_t GetTECSCruiseMaxThrottle;
    int16_t GetTECSCruiseThrottle;
    uint8_t GetTECSCircleDirection;
} Get_Servos_Parameters;

struct _Send_User_Medium_Parameters
{
    uint8_t SendTPAInPercent;
    int16_t SendBreakPointValue;
    uint8_t SendGyroLPF;
    int16_t SendDerivativeLPF;
    int16_t SendRCLPF;
    uint8_t SendKalmanState;
    int16_t SendBiQuadAccLPF;
    int16_t SendBiQuadGyroLPF;
    int16_t SendBiQuadAccNotch;
    int16_t SendBiQuadGyroNotch;
    uint8_t SendMotorCompensationState;
    uint8_t SendProportionalPitch;
    uint8_t SendIntegralPitch;
    uint8_t SendDerivativePitch;
    uint8_t SendProportionalRoll;
    uint8_t SendIntegralRoll;
    uint8_t SendDerivativeRoll;
    uint8_t SendProportionalYaw;
    uint8_t SendIntegralYaw;
    uint8_t SendDerivativeYaw;
    uint8_t SendProportionalVelZ;
    uint8_t SendProportionalGPSHold;
    uint8_t SendIntegralGPSHold;
    int16_t SendServosLPF;
    uint8_t SendCDOrFFRoll;
    uint8_t SendCDOrFFPitch;
    uint8_t SendCDOrFFYaw;
    uint8_t SendAutoLevelProportional;
    uint8_t SendAutoLevelIntegral;
    uint8_t SendHeadingHoldRate;
    uint8_t SendHeadingHoldRateLimit;
    uint8_t SendRollBankMax;
    uint8_t SendPitchBankMin;
    uint8_t SendPitchBankMax;
    uint8_t SendAttackBank;
    uint8_t SendGPSBank;
    uint16_t SendIntegralLPF;
    uint16_t SendkCDLPF;
    uint8_t SendIntegralVelZ;
    uint8_t SendDerivativeVelZ;
    uint8_t SendProportionalAltitudeHold;
    uint8_t SendIntegralAltitudeHold;
    uint8_t SendDerivativeAltitudeHold;
    uint8_t SendProportionalPositionRate;
    uint8_t SendIntegralPositionRate;
    uint8_t SendDerivativePositionRate;
    uint8_t SendProportionalNavigationRate;
    uint8_t SendIntegralNavigationRate;
    uint8_t SendDerivativeNavigationRate;
    uint8_t SendIntegralWindUp;
} Send_User_Medium_Parameters;

struct _Get_User_Medium_Parameters
{
    uint8_t GetTPAInPercent;
    int16_t GetBreakPointValue;
    uint8_t GetGyroLPF;
    int16_t GetDerivativeLPF;
    int16_t GetRCLPF;
    uint8_t GetKalmanState;
    int16_t GetBiquadAccLPF;
    int16_t GetBiquadGyroLPF;
    int16_t GetBiquadAccNotch;
    int16_t GetBiquadGyroNotch;
    uint8_t GetMotorCompensationState;
    uint8_t GetProportionalPitch;
    uint8_t GetIntegralPitch;
    uint8_t GetDerivativePitch;
    uint8_t GetProportionalRoll;
    uint8_t GetIntegralRoll;
    uint8_t GetDerivativeRoll;
    uint8_t GetProportionalYaw;
    uint8_t GetIntegralYaw;
    uint8_t GetDerivativeYaw;
    uint8_t GetProportionalVelZ;
    uint8_t GetProportionalGPSHold;
    uint8_t GetIntegralGPSHold;
    int16_t GetServosLPF;
    uint8_t GetCDOrFFRoll;
    uint8_t GetCDOrFFPitch;
    uint8_t GetCDOrFFYaw;
    uint8_t GetAutoLevelProportional;
    uint8_t GetAutoLevelIntegral;
    uint8_t GetHeadingHoldRate;
    uint8_t GetHeadingHoldRateLimit;
    uint8_t GetRollBankMax;
    uint8_t GetPitchBankMin;
    uint8_t GetPitchBankMax;
    uint8_t GetAttackBank;
    uint8_t GetGPSBank;
    int16_t GetIntegralLPF;
    int16_t GetkCDLPF;
    uint8_t GetIntegralVelZ;
    uint8_t GetDerivativeVelZ;
    uint8_t GetProportionalAltitudeHold;
    uint8_t GetIntegralAltitudeHold;
    uint8_t GetDerivativeAltitudeHold;
    uint8_t GetProportionalPositionRate;
    uint8_t GetIntegralPositionRate;
    uint8_t GetDerivativePositionRate;
    uint8_t GetProportionalNavigationRate;
    uint8_t GetIntegralNavigationRate;
    uint8_t GetDerivativeNavigationRate;
    uint8_t GetIntegralWindUp;
} Get_User_Medium_Parameters;

struct _Send_WayPoint_Coordinates
{
    int32_t SendLatitudeOne;
    int32_t SendLatitudeTwo;
    int32_t SendLatitudeThree;
    int32_t SendLatitudeFour;
    int32_t SendLatitudeFive;
    int32_t SendLatitudeSix;
    int32_t SendLatitudeSeven;
    int32_t SendLatitudeEight;
    int32_t SendLatitudeNine;
    int32_t SendLatitudeTen;
    int32_t SendLongitudeOne;
    int32_t SendLongitudeTwo;
    int32_t SendLongitudeThree;
    int32_t SendLongitudeFour;
    int32_t SendLongitudeFive;
    int32_t SendLongitudeSix;
    int32_t SendLongitudeSeven;
    int32_t SendLongitudeEight;
    int32_t SendLongitudeNine;
    int32_t SendLongitudeTen;
} Send_WayPoint_Coordinates;

struct _Send_WayPoint_Misc_Parameters
{
    uint8_t SendAltitudeOne;
    uint8_t SendAltitudeTwo;
    uint8_t SendAltitudeThree;
    uint8_t SendAltitudeFour;
    uint8_t SendAltitudeFive;
    uint8_t SendAltitudeSix;
    uint8_t SendAltitudeSeven;
    uint8_t SendAltitudeEight;
    uint8_t SendAltitudeNine;
    uint8_t SendAltitudeTen;
    uint8_t SendFlightModeOne;
    uint8_t SendFlightModeTwo;
    uint8_t SendFlightModeThree;
    uint8_t SendFlightModeFour;
    uint8_t SendFlightModeFive;
    uint8_t SendFlightModeSix;
    uint8_t SendFlightModeSeven;
    uint8_t SendFlightModeEight;
    uint8_t SendFlightModeNine;
    uint8_t SendFlightModeTen;
    uint8_t SendGPSHoldTimedOne;
    uint8_t SendGPSHoldTimedTwo;
    uint8_t SendGPSHoldTimedThree;
    uint8_t SendGPSHoldTimedFour;
    uint8_t SendGPSHoldTimedFive;
    uint8_t SendGPSHoldTimedSix;
    uint8_t SendGPSHoldTimedSeven;
    uint8_t SendGPSHoldTimedEight;
    uint8_t SendGPSHoldTimedNine;
    uint8_t SendGPSHoldTimedTen;
} Send_WayPoint_Misc_Parameters;

#ifdef __AVR_ATmega2560__

static void Send_Data_To_GCS(uint8_t Buffer)
{
    FASTSERIAL.StoreTX(UART_NUMB_0, Buffer);
    SerialCheckSum ^= Buffer;

#elif defined ESP32 || defined __arm__

static void Send_Data_To_GCS(int32_t Buffer, uint8_t Length)
{
    switch (Length)
    {
    case VAR_8BITS:
        uint8_t MemGet8BitsBuffer;
        memcpy(&MemGet8BitsBuffer, &Buffer, sizeof(uint8_t));
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = MemGet8BitsBuffer & 0xff;
        SerialCheckSum ^= MemGet8BitsBuffer & 0xff;
        break;

    case VAR_16BITS:
        int16_t MemGet16BitsBuffer;
        memcpy(&MemGet16BitsBuffer, &Buffer, sizeof(int16_t));
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = MemGet16BitsBuffer & 0xff;
        SerialCheckSum ^= MemGet16BitsBuffer & 0xff;
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = (MemGet16BitsBuffer >> 8) & 0xff;
        SerialCheckSum ^= (MemGet16BitsBuffer >> 8) & 0xff;
        break;

    case VAR_32BITS:
        int32_t MemGet32BitsBuffer;
        memcpy(&MemGet32BitsBuffer, &Buffer, sizeof(int32_t));
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = MemGet32BitsBuffer & 0xff;
        SerialCheckSum ^= MemGet32BitsBuffer & 0xff;
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = (MemGet32BitsBuffer >> 8) & 0xff;
        SerialCheckSum ^= (MemGet32BitsBuffer >> 8) & 0xff;
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = (MemGet32BitsBuffer >> 16) & 0xff;
        SerialCheckSum ^= (MemGet32BitsBuffer >> 16) & 0xff;
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = (MemGet32BitsBuffer >> 24) & 0xff;
        SerialCheckSum ^= (MemGet32BitsBuffer >> 24) & 0xff;
        break;
    }

#endif
}

static void Communication_Passed(bool Error, uint8_t Buffer)
{
#ifdef __AVR_ATmega2560__

    FASTSERIAL.StoreTX(UART_NUMB_0, 0x4A);
    SerialCheckSum ^= 0x4A;
    FASTSERIAL.StoreTX(UART_NUMB_0, 0x43);
    SerialCheckSum ^= 0x43;
    FASTSERIAL.StoreTX(UART_NUMB_0, Error ? 0x21 : 0x46);
    SerialCheckSum ^= Error ? 0x21 : 0x46;
    SerialCheckSum = 0;
    FASTSERIAL.StoreTX(UART_NUMB_0, Buffer);
    SerialCheckSum ^= Buffer;
    FASTSERIAL.StoreTX(UART_NUMB_0, ProtocolCommand);
    SerialCheckSum ^= ProtocolCommand;

#elif defined ESP32 || defined __arm__

    SerialOutputBuffer[SerialOutputBufferSizeCount++] = 0x4A;
    SerialCheckSum ^= 0x4A;
    SerialOutputBuffer[SerialOutputBufferSizeCount++] = 0x43;
    SerialCheckSum ^= 0x43;
    SerialOutputBuffer[SerialOutputBufferSizeCount++] = Error ? 0x21 : 0x46;
    SerialCheckSum ^= Error ? 0x21 : 0x46;
    SerialCheckSum = 0;
    SerialOutputBuffer[SerialOutputBufferSizeCount++] = Buffer;
    SerialCheckSum ^= Buffer;
    SerialOutputBuffer[SerialOutputBufferSizeCount++] = ProtocolCommand;
    SerialCheckSum ^= ProtocolCommand;

#endif
}

#ifdef __AVR_ATmega2560__

static void Send_Struct_Params_To_GCS(uint8_t *CheckBuffer, uint8_t SizeOfBuffer)
{
    Communication_Passed(false, SizeOfBuffer);
    while (SizeOfBuffer--)
    {
        Send_Data_To_GCS(*CheckBuffer++);
    }
    Send_Data_To_GCS(SerialCheckSum);
    FASTSERIAL.UartSendData(UART_NUMB_0);
}

#endif

static void __attribute__((noinline)) Get_Struct_Params_To_GCS(uint8_t *CheckBuffer, uint8_t SizeOfBuffer)
{
    while (SizeOfBuffer--)
    {
        *CheckBuffer++ = SerialInputBuffer[VectorCount++] & 0xff;
    }
}

void GCSClass::Send_String_To_GCS(const char *String)
{
    Communication_Passed(false, strlen_P(String));
    for (const char *StringCount = String; ProgMemReadByte(StringCount); StringCount++)
    {

#ifdef __AVR_ATmega2560__

        Send_Data_To_GCS(ProgMemReadByte(StringCount));
    }
    Send_Data_To_GCS(SerialCheckSum);
    FASTSERIAL.UartSendData(UART_NUMB_0);

#elif defined ESP32 || defined __arm__

        Send_Data_To_GCS(ProgMemReadByte(StringCount), VAR_8BITS);
    }
    Send_Data_To_GCS(SerialCheckSum, VAR_8BITS);

#endif
}

void GCSClass::Serial_Parse_Protocol(void)
{
#ifdef USE_CLI

    if (GCS.CliMode)
    {
        return;
    }

#endif

    SerialAvailableGuard = FASTSERIAL.Available(UART_NUMB_0);
    while (SerialAvailableGuard--)
    {
        SerialBuffer = FASTSERIAL.Read(UART_NUMB_0);
        ProtocolTaskOrder = PreviousProtocolTaskOrder;

        switch (ProtocolTaskOrder)
        {

        case 0:
            if (SerialBuffer == 0x4A)
            {
                ProtocolTaskOrder = 1;
            }

#ifdef USE_CLI

            if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && SerialBuffer == 0x23)
            {
                GCS.CliMode = true;
            }

#endif
            break;

        case 1:
            ProtocolTaskOrder = (SerialBuffer == 0x43) ? 2 : 0;
            break;

        case 2:
            ProtocolTaskOrder = (SerialBuffer == 0x3C) ? 3 : 0;
            break;

        case 3:
            SerialDataSize = SerialBuffer;
            SerialCheckSum = SerialBuffer;
            SerialOffSet = 0;
            VectorCount = 0;
            ProtocolTaskOrder = 4;
            break;

        case 4:
            ProtocolCommand = SerialBuffer;
            SerialCheckSum ^= SerialBuffer;
            ProtocolTaskOrder = 5;
            break;

        case 5:
            if (SerialOffSet < SerialDataSize)
            {
                SerialCheckSum ^= SerialBuffer;
                SerialInputBuffer[SerialOffSet++] = SerialBuffer;
            }
            else
            {
                if (SerialCheckSum == SerialBuffer)
                {
                    GCS.Update_BiDirect_Protocol(ProtocolCommand);
                }
                ProtocolTaskOrder = 0;
                SerialAvailableGuard = 0;
            }
            break;
        }
        PreviousProtocolTaskOrder = ProtocolTaskOrder;
    }
#if defined(ESP32) || (defined __arm__)

    while (SerialOutputBufferSizeCount > 0)
    {
        SerialOutputBufferSizeCount--;
        FASTSERIAL.Write(UART_NUMB_0, SerialOutputBuffer[OutputVectorCount++]);
    }

#endif
}

void GCSClass::Update_BiDirect_Protocol(uint8_t TaskOrderGCS)
{
#ifdef __AVR_ATmega2560__

    switch (TaskOrderGCS)
    {

    case 1:
        GCS.WayPoint_Request_Coordinates_Parameters();
        Send_Struct_Params_To_GCS((uint8_t *)&Send_WayPoint_Coordinates, sizeof(_Send_WayPoint_Coordinates));
        break;

    case 2:
        GCS.WayPoint_Request_Misc_Parameters();
        Send_Struct_Params_To_GCS((uint8_t *)&Send_WayPoint_Misc_Parameters, sizeof(_Send_WayPoint_Misc_Parameters));
        break;

    case 3:
        WayPoint_Resources.Storage.Function = WAYPOINT_STORAGE_RESET;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 4:
        WayPoint_Resources.Storage.Function = WAYPOINT_STORAGE_SAVE;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 5:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&GetWayPointPacketOne, sizeof(_GetWayPointPacketOne));
        break;

    case 6:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&GetWayPointPacketTwo, sizeof(_GetWayPointPacketTwo));
        break;

    case 7:
        GCS.First_Packet_Request_Parameters();
        Send_Struct_Params_To_GCS((uint8_t *)&Essential_First_Packet_Parameters, sizeof(_Essential_First_Packet_Parameters));
        break;

    case 8:
        Send_Struct_Params_To_GCS((uint8_t *)&Send_User_Basic_Parameters, sizeof(_Send_User_Basic_Parameters));
        break;

    case 9:
        Send_Struct_Params_To_GCS((uint8_t *)&Send_User_Medium_Parameters, sizeof(_Send_User_Medium_Parameters));
        break;

    case 10:
        GCS.Second_Packet_Request_Parameters();
        Send_Struct_Params_To_GCS((uint8_t *)&Essential_Second_Packet_Parameters, sizeof(_Essential_Second_Packet_Parameters));
        break;

    case 11:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            ACCCALIBRATION.Start();
        }
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 12:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && I2CResources.Found.Compass)
        {
            Calibration.Magnetometer.Calibrating = true;
        }
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 13:
        GCS.ConfigFlight = true;
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 14:
        GCS.ConfigFlight = false;
        Communication_Passed(0, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 15:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&Get_User_Basic_Parameters, sizeof(_Get_User_Basic_Parameters));
        break;

    case 16:
        GCS.Save_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(0, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 17:
        GCS.Default_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 18:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&Get_User_Medium_Parameters, sizeof(_Get_User_Medium_Parameters));
        break;

    case 19:
        GCS.Save_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 20:
        GCS.Default_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 21:
        GCS.Send_String_To_GCS(PlatformName);
        break;

    case 22:
        GCS.Send_String_To_GCS(FirwareName);
        break;

    case 23:
        GCS.Send_String_To_GCS(FirmwareVersion);
        break;

    case 24:
        GCS.Send_String_To_GCS(CompilerVersion);
        break;

    case 25:
        GCS.Send_String_To_GCS(BuildDate);
        break;

    case 26:
        GCS.Send_String_To_GCS(BuildTime);
        break;

    case 27:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            PREARM.UpdateGCSErrorText(PREARM.Checking());
        }
        break;

    case 28:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            WATCHDOG.Reboot();
        }
        break;

    case 29:
        GCS.Default_RadioControl_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 30:
        Send_Struct_Params_To_GCS((uint8_t *)&Send_Radio_Control_Parameters, sizeof(_Send_Radio_Control_Parameters));
        break;

    case 31:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&Get_Radio_Control_Parameters, sizeof(_Get_Radio_Control_Parameters));
        break;

    case 32:
        GCS.Save_Radio_Control_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 33:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&Get_Servos_Parameters, sizeof(_Get_Servos_Parameters));
        break;

    default:
        Communication_Passed(true, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;
    }

#elif defined ESP32 || defined __arm__

    switch (TaskOrderGCS)
    {

    case 3:
        WayPoint_Resources.Storage.Function = WAYPOINT_STORAGE_RESET;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 4:
        WayPoint_Resources.Storage.Function = WAYPOINT_STORAGE_SAVE;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 5:
        Get_Struct_Params_To_GCS((uint8_t *)&GetWayPointPacketOne, sizeof(_GetWayPointPacketOne));
        break;

    case 6:
        Get_Struct_Params_To_GCS((uint8_t *)&GetWayPointPacketTwo, sizeof(_GetWayPointPacketTwo));
        break;

    case 7:
        GCS.First_Packet_Request_Parameters();
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        OutputVectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 10) +     //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 27) + //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
                                        (sizeof(int32_t) * 6));  //NÚMERO TOTAL DE VARIAVEIS DE 32 BITS CONTIDO AQUI
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAttitudePitch, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAttitudeRoll, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAttitudeYaw, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.DevicesOnBoard, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendThrottleValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendYawValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendPitchValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendRollValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxOneValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxTwoValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxThreeValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxFourValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxFiveValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxSixValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxSevenValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxEightValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendGPSNumberOfSat, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendGPSLatitude, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendGPSLongitude, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHomePointLatitude, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHomePointLongitude, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendBarometerValue, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendFailSafeState, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendBatteryVoltageValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendBatteryPercentageValue, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendArmDisarmState, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHDOPValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCurrentValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendWattsValue, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendDeclinationValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendActualFlightMode, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendFrameType, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHomePointState, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendTemperature, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHomePointDistance, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCurrentInMah, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCourseOverGround, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendBearing, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAccGForce, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAccImageBitMap, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCompassRoll, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCompassPitch, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCompassYaw, VAR_16BITS);

        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 8:
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        OutputVectorCount = 0;
        Communication_Passed(false, (sizeof(uint16_t) * 6) +     //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(uint8_t) * 24)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendFrameType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendRcChSequency, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendGimbalType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendParachuteType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendUartNumb1Type, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendUartNumb2Type, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendUartNumb3Type, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendCompassRotationType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendRTHAltitude, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAcroType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAltitudeHoldType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendPositionHoldType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendSimpleControlType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendReturnToHomeType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAtackType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAutomaticFlipType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAutomaticMissonType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendArmDisarmType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAutoLandType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendSafeBtnState, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAirSpeedState, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendPitchLevelTrim, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendBatteryVoltageScale, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendBatteryCurrentScale, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendBatteryCurrentOffSet, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendBattMinVoltage, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendBattMaxVoltage, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendNumberOfCells, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendCriticBattPercent, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendRTHLowBatt, VAR_8BITS);

        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 9:
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        OutputVectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 40) +    //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 8)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendTPAInPercent, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBreakPointValue, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendGyroLPF, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendRCLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendKalmanState, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBiQuadAccLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBiQuadGyroLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBiQuadAccNotch, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBiQuadGyroNotch, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendMotorCompensationState, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalPitch, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralPitch, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativePitch, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalYaw, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralYaw, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeYaw, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalVelZ, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalGPSHold, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralGPSHold, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendServosLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendCDOrFFRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendCDOrFFPitch, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendCDOrFFYaw, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendAutoLevelProportional, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendAutoLevelIntegral, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendHeadingHoldRate, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendHeadingHoldRateLimit, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendRollBankMax, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendPitchBankMin, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendPitchBankMax, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendAttackBank, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendGPSBank, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralVelZ, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeVelZ, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalAltitudeHold, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralAltitudeHold, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeAltitudeHold, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalPositionRate, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralPositionRate, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativePositionRate, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalNavigationRate, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralNavigationRate, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeNavigationRate, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralWindUp, VAR_8BITS);

        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 10:
        GCS.Second_Packet_Request_Parameters();
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        OutputVectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 2) +      //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 26)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualThrottleValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualYawValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualPitchValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualRollValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxOneValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxTwoValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxThreeValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxFourValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxFiveValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxSixValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxSevenValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxEightValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAttitudeThrottleValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAttitudeYawValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAttitudePitchValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAttitudeRollValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendMemoryRamUsed, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendMemoryRamUsedPercent, VAR_8BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccX, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccY, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccZ, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroX, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroY, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroZ, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGPSGroundSpeed, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendI2CError, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAirSpeedValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendCPULoad, VAR_8BITS);

        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 11:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            ACCCALIBRATION.Start();
        }
        break;

    case 12:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && I2CResources.Found.Compass)
        {
            Calibration.Magnetometer.Calibrating = true;
        }
        break;

    case 13:
        GCS.ConfigFlight = true;
        break;

    case 14:
        GCS.ConfigFlight = false;
        break;

    case 15:
        Get_Struct_Params_To_GCS((uint8_t *)&Get_User_Basic_Parameters, sizeof(_Get_User_Basic_Parameters));
        break;

    case 16:
        GCS.Save_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 17:
        GCS.Default_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 18:
        Get_Struct_Params_To_GCS((uint8_t *)&Get_User_Medium_Parameters, sizeof(_Get_User_Medium_Parameters));
        break;

    case 19:
        GCS.Save_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 20:
        GCS.Default_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 21:
        GCS.Send_String_To_GCS(PlatformName);
        break;

    case 22:
        GCS.Send_String_To_GCS(FirwareName);
        break;

    case 23:
        GCS.Send_String_To_GCS(FirmwareVersion);
        break;

    case 24:
        GCS.Send_String_To_GCS(CompilerVersion);
        break;

    case 25:
        GCS.Send_String_To_GCS(BuildDate);
        break;

    case 26:
        GCS.Send_String_To_GCS(BuildTime);
        break;

    case 27:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            PREARM.UpdateGCSErrorText(PREARM.Checking());
        }
        break;

    case 28:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            WATCHDOG.Reboot();
        }
        break;

    case 29:
        GCS.Default_RadioControl_Configuration();
        break;

    case 30:
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        OutputVectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 17) +     //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 29) + //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
                                        (sizeof(int32_t) * 1));  //NÚMERO TOTAL DE VARIAVEIS DE 32 BITS CONTIDO AQUI
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleMiddle, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleExpo, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRCRate, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRCExpo, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendYawRate, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRCPulseMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRCPulseMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendYawMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendPitchMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRollMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendYawMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendPitchMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRollMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendYawDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendPitchDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRollDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendChannelsReverse, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo1Rate, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo2Rate, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo3Rate, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo4Rate, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServosReverse, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo1Min, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo2Min, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo3Min, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo4Min, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo1Med, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo2Med, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo3Med, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo4Med, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo1Max, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo2Max, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo3Max, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo4Max, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendFailSafeValue, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendMaxBankPitch, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendMaxBankRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendAutoPilotMode, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendAirSpeedScale, VAR_32BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTunningChannel, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTunning, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendLandAfterRTH, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendHoverThrottle, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendAirSpeedReference, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTECSPitch2ThrFactor, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTECSPitch2ThrLPF, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTECSAutoPilotLPF, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTECSCruiseMinThrottle, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTECSCruiseMaxThrottle, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTECSCruiseThrottle, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendTECSCircleDirection, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendContServosTrimState, VAR_8BITS);

        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 31:
        Get_Struct_Params_To_GCS((uint8_t *)&Get_Radio_Control_Parameters, sizeof(_Get_Radio_Control_Parameters));
        break;

    case 32:
        GCS.Save_Radio_Control_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 33:
        Get_Struct_Params_To_GCS((uint8_t *)&Get_Servos_Parameters, sizeof(_Get_Servos_Parameters));
        break;
    }

#endif
}

void GCSClass::First_Packet_Request_Parameters(void)
{
    //ENVIA OS PARAMETROS FUNDAMENTAIS PARA O GCS
    Essential_First_Packet_Parameters.SendAttitudePitch = Constrain_16Bits(Attitude.Raw[PITCH], -900, 900);
    Essential_First_Packet_Parameters.SendAttitudeRoll = Constrain_16Bits(Attitude.Raw[ROLL], -900, 900);
    if (I2CResources.Found.Compass)
    {
        Essential_First_Packet_Parameters.SendAttitudeYaw = Attitude.EulerAngles.Yaw;
    }
    else
    {
        Essential_First_Packet_Parameters.SendAttitudeYaw = WRap_18000(GPS_Resources.Navigation.Misc.Get.GroundCourse * 10) / 10;
    }
    Essential_First_Packet_Parameters.DevicesOnBoard = CHECKSUM.GetDevicesActived();
    Essential_First_Packet_Parameters.SendThrottleValue = Throttle.Output;
    Essential_First_Packet_Parameters.SendYawValue = Yaw.Output;
    Essential_First_Packet_Parameters.SendPitchValue = Pitch.Output;
    Essential_First_Packet_Parameters.SendRollValue = Roll.Output;
    Essential_First_Packet_Parameters.SendAuxOneValue = AuxiliarOne.Output;
    Essential_First_Packet_Parameters.SendAuxTwoValue = AuxiliarTwo.Output;
    Essential_First_Packet_Parameters.SendAuxThreeValue = AuxiliarThree.Output;
    Essential_First_Packet_Parameters.SendAuxFourValue = AuxiliarFour.Output;
    Essential_First_Packet_Parameters.SendAuxFiveValue = AuxiliarFive.Output;
    Essential_First_Packet_Parameters.SendAuxSixValue = AuxiliarSix.Output;
    Essential_First_Packet_Parameters.SendAuxSevenValue = AuxiliarSeven.Output;
    Essential_First_Packet_Parameters.SendAuxEightValue = AuxiliarEight.Output;
    Essential_First_Packet_Parameters.SendGPSNumberOfSat = GPS_Resources.Navigation.Misc.Get.Satellites;
    Essential_First_Packet_Parameters.SendGPSLatitude = GPS_Resources.Navigation.Coordinates.Actual[COORD_LATITUDE];
    Essential_First_Packet_Parameters.SendGPSLongitude = GPS_Resources.Navigation.Coordinates.Actual[COORD_LONGITUDE];
    Essential_First_Packet_Parameters.SendHomePointLatitude = GPS_Resources.Home.Coordinates[COORD_LATITUDE];
    Essential_First_Packet_Parameters.SendHomePointLongitude = GPS_Resources.Home.Coordinates[COORD_LONGITUDE];
    if (I2CResources.Found.Barometer)
    {
        Essential_First_Packet_Parameters.SendBarometerValue = Barometer.Altitude.Actual;
    }
    else
    {
        Essential_First_Packet_Parameters.SendBarometerValue = 0;
    }
    Essential_First_Packet_Parameters.SendFailSafeState = SystemInFailSafe();
    Essential_First_Packet_Parameters.SendBatteryVoltageValue = BATTERY.Get_Actual_Voltage() * 100;
    Essential_First_Packet_Parameters.SendBatteryPercentageValue = BATTERY.GetPercentage();
    Essential_First_Packet_Parameters.SendArmDisarmState = IS_STATE_ACTIVE(PRIMARY_ARM_DISARM);
    Essential_First_Packet_Parameters.SendHDOPValue = GPS_Resources.Navigation.Misc.Get.HDOP;
    Essential_First_Packet_Parameters.SendCurrentValue = BATTERY.Get_Actual_Current();
    Essential_First_Packet_Parameters.SendWattsValue = BATTERY.GetWatts();
    Essential_First_Packet_Parameters.SendDeclinationValue = (int16_t)(STORAGEMANAGER.Read_Float(MAG_DECLINATION_ADDR) * 100);
    Essential_First_Packet_Parameters.SendActualFlightMode = AUXFLIGHT.FlightMode;
    Essential_First_Packet_Parameters.SendFrameType = GetActualPlatformType();
    Essential_First_Packet_Parameters.SendHomePointState = GPS_Resources.Home.Marked;
    Essential_First_Packet_Parameters.SendTemperature = IMU.Accelerometer.Temperature;
    Essential_First_Packet_Parameters.SendHomePointDistance = GPS_Resources.Home.Distance;
    Essential_First_Packet_Parameters.SendCurrentInMah = BATTERY.Get_Current_In_Mah();
#ifndef MACHINE_CYCLE
    Essential_First_Packet_Parameters.SendCourseOverGround = GPS_Resources.Navigation.Misc.Get.GroundCourse;
#else
    Essential_First_Packet_Parameters.SendCourseOverGround = GetTaskDeltaTime(TASK_INTEGRAL_LOOP);
#endif
    Essential_First_Packet_Parameters.SendBearing = GPS_Resources.Navigation.Bearing.ActualTarget;
    Essential_First_Packet_Parameters.SendAccGForce = IMU.Accelerometer.GravityForce.Value;
    Essential_First_Packet_Parameters.SendAccImageBitMap = GetImageToGCS();
    Essential_First_Packet_Parameters.SendCompassRoll = IMU.Compass.Read[ROLL];
    Essential_First_Packet_Parameters.SendCompassPitch = IMU.Compass.Read[PITCH];
    Essential_First_Packet_Parameters.SendCompassYaw = IMU.Compass.Read[YAW];
}

void GCSClass::Second_Packet_Request_Parameters(void)
{
    Essential_Second_Packet_Parameters.SendActualThrottleValue = DECODE.DirectRadioControlRead[THROTTLE];
    Essential_Second_Packet_Parameters.SendActualYawValue = DECODE.DirectRadioControlRead[YAW];
    Essential_Second_Packet_Parameters.SendActualPitchValue = DECODE.DirectRadioControlRead[PITCH];
    Essential_Second_Packet_Parameters.SendActualRollValue = DECODE.DirectRadioControlRead[ROLL];
    Essential_Second_Packet_Parameters.SendActualAuxOneValue = DECODE.DirectRadioControlRead[AUX1];
    Essential_Second_Packet_Parameters.SendActualAuxTwoValue = DECODE.DirectRadioControlRead[AUX2];
    Essential_Second_Packet_Parameters.SendActualAuxThreeValue = DECODE.DirectRadioControlRead[AUX3];
    Essential_Second_Packet_Parameters.SendActualAuxFourValue = DECODE.DirectRadioControlRead[AUX4];
    Essential_Second_Packet_Parameters.SendActualAuxFiveValue = DECODE.DirectRadioControlRead[AUX5];
    Essential_Second_Packet_Parameters.SendActualAuxSixValue = DECODE.DirectRadioControlRead[AUX6];
    Essential_Second_Packet_Parameters.SendActualAuxSevenValue = DECODE.DirectRadioControlRead[AUX7];
    Essential_Second_Packet_Parameters.SendActualAuxEightValue = DECODE.DirectRadioControlRead[AUX8];
    Essential_Second_Packet_Parameters.SendAttitudeThrottleValue = RC_Resources.Attitude.Controller[THROTTLE];
    Essential_Second_Packet_Parameters.SendAttitudeYawValue = PID_Resources.RcRateTarget.GCS.Yaw;
    Essential_Second_Packet_Parameters.SendAttitudePitchValue = PID_Resources.RcRateTarget.GCS.Pitch;
    Essential_Second_Packet_Parameters.SendAttitudeRollValue = PID_Resources.RcRateTarget.GCS.Roll;
    Essential_Second_Packet_Parameters.SendMemoryRamUsed = MEMORY.Check();
    Essential_Second_Packet_Parameters.SendMemoryRamUsedPercent = MEMORY.GetPercentageRAMUsed();
    Essential_Second_Packet_Parameters.SendAccX = IMU.Accelerometer.ReadFloat[ROLL] * 100;
    Essential_Second_Packet_Parameters.SendAccY = IMU.Accelerometer.ReadFloat[PITCH] * 100;
    Essential_Second_Packet_Parameters.SendAccZ = IMU.Accelerometer.ReadFloat[YAW] * 100;
    Essential_Second_Packet_Parameters.SendGyroX = IMU.Gyroscope.ReadFloat[ROLL] * 100;
    Essential_Second_Packet_Parameters.SendGyroY = IMU.Gyroscope.ReadFloat[PITCH] * 100;
    Essential_Second_Packet_Parameters.SendGyroZ = IMU.Gyroscope.ReadFloat[YAW] * 100;
    Essential_Second_Packet_Parameters.SendGPSGroundSpeed = GPS_Resources.Navigation.Misc.Get.GroundSpeed;
    Essential_Second_Packet_Parameters.SendI2CError = I2CResources.Error.Count;
    Essential_Second_Packet_Parameters.SendAirSpeedValue = AirSpeed.Raw.IASPressureInCM;
    Essential_Second_Packet_Parameters.SendCPULoad = SystemLoadPercent;
}

void GCSClass::WayPoint_Request_Coordinates_Parameters(void)
{
    int16_t AddressCount = 0;

    Send_WayPoint_Coordinates.SendLatitudeOne = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeTwo = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeThree = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeFour = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeFive = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeSix = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeSeven = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeEight = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeNine = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLatitudeTen = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);

    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeOne = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeTwo = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeThree = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeFour = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeFive = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeSix = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeSeven = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeEight = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeNine = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
    AddressCount += sizeof(int32_t);
    Send_WayPoint_Coordinates.SendLongitudeTen = STORAGEMANAGER.Read_32Bits(INITIAL_ADDR_OF_COORDINATES + AddressCount);
}

void GCSClass::WayPoint_Request_Misc_Parameters(void)
{
    int16_t AddressCount = 0;

    //TEMPO DO POS-HOLD
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedOne = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedTwo = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedThree = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedFour = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedFive = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedSix = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedSeven = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedEight = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedNine = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedTen = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);

    //MODO DE VOO
    Send_WayPoint_Misc_Parameters.SendFlightModeOne = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeTwo = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeThree = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeFour = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeFive = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeSix = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeSeven = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeEight = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeNine = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendFlightModeTen = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);

    //ALTITUDE DE NAVEGAÇÃO
    Send_WayPoint_Misc_Parameters.SendAltitudeOne = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeTwo = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeThree = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeFour = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeFive = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeSix = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeSeven = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeEight = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeNine = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
    AddressCount += sizeof(uint8_t);
    Send_WayPoint_Misc_Parameters.SendAltitudeTen = STORAGEMANAGER.Read_8Bits(INITIAL_ADDR_OF_OTHERS_PARAMS + AddressCount);
}

void GCSClass::Save_Basic_Configuration(void)
{
    STORAGEMANAGER.Write_8Bits(FRAME_TYPE_ADDR, Get_User_Basic_Parameters.GetFrameType);
    STORAGEMANAGER.Write_8Bits(RC_SEQUENCY_ADDR, Get_User_Basic_Parameters.GetRcChSequency);
    STORAGEMANAGER.Write_8Bits(GIMBAL_ADDR, Get_User_Basic_Parameters.GetGimbalType);
    STORAGEMANAGER.Write_8Bits(PARACHUTE_ADDR, Get_User_Basic_Parameters.GetParachuteType);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_1_ADDR, Get_User_Basic_Parameters.GetUartNumb1Type);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_2_ADDR, Get_User_Basic_Parameters.GetUartNumb2Type);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_3_ADDR, Get_User_Basic_Parameters.GetUartNumb3Type);
    STORAGEMANAGER.Write_8Bits(COMPASS_ROTATION_ADDR, Get_User_Basic_Parameters.GetCompassRotationType);
    STORAGEMANAGER.Write_8Bits(RTH_ALTITUDE_ADDR, Get_User_Basic_Parameters.GetRTHAltitude);
    STORAGEMANAGER.Write_8Bits(STABLIZE_ADDR, Get_User_Basic_Parameters.GetAcroType);
    STORAGEMANAGER.Write_8Bits(ALT_HOLD_ADDR, Get_User_Basic_Parameters.GetAltitudeHoldType);
    STORAGEMANAGER.Write_8Bits(GPS_HOLD_ADDR, Get_User_Basic_Parameters.GetPositionHoldType);
    STORAGEMANAGER.Write_8Bits(SIMPLE_ADDR, Get_User_Basic_Parameters.GetSimpleControlType);
    STORAGEMANAGER.Write_8Bits(RTH_ADDR, Get_User_Basic_Parameters.GetReturnToHomeType);
    STORAGEMANAGER.Write_8Bits(ATTACK_ADDR, Get_User_Basic_Parameters.GetAtackType);
    STORAGEMANAGER.Write_8Bits(AUTOFLIP_ADDR, Get_User_Basic_Parameters.GetAutomaticFlipType);
    STORAGEMANAGER.Write_8Bits(AUTOMISSION_ADDR, Get_User_Basic_Parameters.GetAutomaticMissonType);
    STORAGEMANAGER.Write_8Bits(ARMDISARM_ADDR, Get_User_Basic_Parameters.GetArmDisarmType);
    STORAGEMANAGER.Write_8Bits(AUTOLAND_ADDR, Get_User_Basic_Parameters.GetAutoLandType);
    STORAGEMANAGER.Write_8Bits(DISP_PASSIVES_ADDR, Get_User_Basic_Parameters.GetSafeBtnState);
    STORAGEMANAGER.Write_8Bits(AIRSPEED_TYPE_ADDR, Get_User_Basic_Parameters.GetAirSpeedState);
    STORAGEMANAGER.Write_16Bits(PITCH_LEVEL_TRIM_ADDR, Get_User_Basic_Parameters.GetPitchLevelTrim);
    STORAGEMANAGER.Write_16Bits(BATT_VOLTAGE_FACTOR_ADDR, Get_User_Basic_Parameters.GetBatteryVoltageScale);
    STORAGEMANAGER.Write_16Bits(BATT_AMPS_VOLT_ADDR, Get_User_Basic_Parameters.GetBatteryCurrentScale);
    STORAGEMANAGER.Write_16Bits(BATT_AMPS_OFFSET_ADDR, Get_User_Basic_Parameters.GetBatteryCurrentOffSet);
    STORAGEMANAGER.Write_16Bits(BATT_MIN_VOLTAGE_ADDR, Get_User_Basic_Parameters.GetBattMinVoltage);
    STORAGEMANAGER.Write_16Bits(BATT_MAX_VOLTAGE_ADDR, Get_User_Basic_Parameters.GetBattMaxVoltage);
    STORAGEMANAGER.Write_8Bits(BATT_NUMBER_OF_CELLS_ADDR, Get_User_Basic_Parameters.GetNumberOfCells);
    STORAGEMANAGER.Write_8Bits(BATT_CRIT_PERCENT_ADDR, Get_User_Basic_Parameters.GetCriticBattPercent);
    STORAGEMANAGER.Write_8Bits(BATT_RTH_LOW_BATT_ADDR, Get_User_Basic_Parameters.GetRTHLowBatt);

    //ATUALIZA OS PARAMETROS DO PID
    GET_SET[PID_UPDATED].State = false;
}

void GCSClass::Save_Radio_Control_Configuration(void)
{
    STORAGEMANAGER.Write_8Bits(THROTTLE_MIDDLE_ADDR, Get_Radio_Control_Parameters.GetThrottleMiddle);
    STORAGEMANAGER.Write_8Bits(THROTTLE_EXPO_ADDR, Get_Radio_Control_Parameters.GetThrottleExpo);
    STORAGEMANAGER.Write_8Bits(PR_RATE_ADDR, Get_Radio_Control_Parameters.GetPRRate);
    STORAGEMANAGER.Write_8Bits(PR_EXPO_ADDR, Get_Radio_Control_Parameters.GetPRExpo);
    STORAGEMANAGER.Write_8Bits(YAW_RATE_ADDR, Get_Radio_Control_Parameters.GetYawRate);
    STORAGEMANAGER.Write_8Bits(YAW_EXPO_ADDR, Get_Radio_Control_Parameters.GetYawExpo);
    STORAGEMANAGER.Write_16Bits(THR_ATTITUDE_MIN_ADDR, Get_Radio_Control_Parameters.GetRCPulseMin);
    STORAGEMANAGER.Write_16Bits(THR_ATTITUDE_MAX_ADDR, Get_Radio_Control_Parameters.GetRCPulseMax);
    STORAGEMANAGER.Write_16Bits(THROTTLE_MIN_ADDR, Get_Radio_Control_Parameters.GetThrottleMin);
    STORAGEMANAGER.Write_16Bits(YAW_MIN_ADDR, Get_Radio_Control_Parameters.GetYawMin);
    STORAGEMANAGER.Write_16Bits(PITCH_MIN_ADDR, Get_Radio_Control_Parameters.GetPitchMin);
    STORAGEMANAGER.Write_16Bits(ROLL_MIN_ADDR, Get_Radio_Control_Parameters.GetRollMin);
    STORAGEMANAGER.Write_16Bits(THROTTLE_MAX_ADDR, Get_Radio_Control_Parameters.GetThrottleMax);
    STORAGEMANAGER.Write_16Bits(YAW_MAX_ADDR, Get_Radio_Control_Parameters.GetYawMax);
    STORAGEMANAGER.Write_16Bits(PITCH_MAX_ADDR, Get_Radio_Control_Parameters.GetPitchMax);
    STORAGEMANAGER.Write_16Bits(ROLL_MAX_ADDR, Get_Radio_Control_Parameters.GetRollMax);
    STORAGEMANAGER.Write_8Bits(THROTTLE_DZ_ADDR, Get_Radio_Control_Parameters.GetThrottleDeadZone);
    STORAGEMANAGER.Write_8Bits(YAW_DZ_ADDR, Get_Radio_Control_Parameters.GetYawDeadZone);
    STORAGEMANAGER.Write_8Bits(PITCH_DZ_ADDR, Get_Radio_Control_Parameters.GetPitchDeadZone);
    STORAGEMANAGER.Write_8Bits(ROLL_DZ_ADDR, Get_Radio_Control_Parameters.GetRollDeadZone);
    STORAGEMANAGER.Write_8Bits(CH_REVERSE_ADDR, Get_Radio_Control_Parameters.GetChannelsReverse);
    STORAGEMANAGER.Write_16Bits(SERVO1_RATE_ADDR, Get_Servos_Parameters.GetServo1Rate);
    STORAGEMANAGER.Write_16Bits(SERVO2_RATE_ADDR, Get_Servos_Parameters.GetServo2Rate);
    STORAGEMANAGER.Write_16Bits(SERVO3_RATE_ADDR, Get_Servos_Parameters.GetServo3Rate);
    STORAGEMANAGER.Write_16Bits(SERVO4_RATE_ADDR, Get_Servos_Parameters.GetServo4Rate);
    STORAGEMANAGER.Write_8Bits(SERVOS_REVERSE_ADDR, Get_Servos_Parameters.GetServosReverse);
    STORAGEMANAGER.Write_16Bits(SERVO1_MIN_ADDR, Get_Servos_Parameters.GetServo1Min);
    STORAGEMANAGER.Write_16Bits(SERVO2_MIN_ADDR, Get_Servos_Parameters.GetServo2Min);
    STORAGEMANAGER.Write_16Bits(SERVO3_MIN_ADDR, Get_Servos_Parameters.GetServo3Min);
    STORAGEMANAGER.Write_16Bits(SERVO4_MIN_ADDR, Get_Servos_Parameters.GetServo4Min);
    STORAGEMANAGER.Write_16Bits(SERVO1_MID_ADDR, Get_Servos_Parameters.GetServo1Med);
    STORAGEMANAGER.Write_16Bits(SERVO2_MID_ADDR, Get_Servos_Parameters.GetServo2Med);
    STORAGEMANAGER.Write_16Bits(SERVO3_MID_ADDR, Get_Servos_Parameters.GetServo3Med);
    STORAGEMANAGER.Write_16Bits(SERVO4_MID_ADDR, Get_Servos_Parameters.GetServo4Med);
    STORAGEMANAGER.Write_16Bits(SERVO1_MAX_ADDR, Get_Servos_Parameters.GetServo1Max);
    STORAGEMANAGER.Write_16Bits(SERVO2_MAX_ADDR, Get_Servos_Parameters.GetServo2Max);
    STORAGEMANAGER.Write_16Bits(SERVO3_MAX_ADDR, Get_Servos_Parameters.GetServo3Max);
    STORAGEMANAGER.Write_16Bits(SERVO4_MAX_ADDR, Get_Servos_Parameters.GetServo4Max);
    STORAGEMANAGER.Write_16Bits(FAILSAFE_VAL_ADDR, Get_Radio_Control_Parameters.GetFailSafeValue);
    STORAGEMANAGER.Write_8Bits(MAX_PITCH_LEVEL_ADDR, Get_Radio_Control_Parameters.GetMaxBankPitch);
    STORAGEMANAGER.Write_8Bits(MAX_ROLL_LEVEL_ADDR, Get_Radio_Control_Parameters.GetMaxBankRoll);
    STORAGEMANAGER.Write_8Bits(AUTO_PILOT_MODE_ADDR, Get_Radio_Control_Parameters.GetdAutoPilotMode);
    STORAGEMANAGER.Write_32Bits(AIRSPEED_FACTOR_ADDR, Get_Radio_Control_Parameters.GetAirSpeedScale);
    STORAGEMANAGER.Write_8Bits(CH_TUNNING_ADDR, Get_Radio_Control_Parameters.GetTunningChannel);
    STORAGEMANAGER.Write_8Bits(TUNNING_ADDR, Get_Radio_Control_Parameters.GetTunning);
    STORAGEMANAGER.Write_8Bits(LAND_AFTER_RTH_ADDR, Get_Radio_Control_Parameters.GetLandAfterRTH);
    STORAGEMANAGER.Write_16Bits(HOVER_THROTTLE_ADDR, Get_Radio_Control_Parameters.GetHoverThrottle);
    STORAGEMANAGER.Write_16Bits(AIR_SPEED_REFERENCE_ADDR, Get_Radio_Control_Parameters.GetAirSpeedReference);
    STORAGEMANAGER.Write_8Bits(TECS_PITCH2THR_FACTOR_ADDR, Get_Servos_Parameters.GetTECSPitch2ThrFactor);
    STORAGEMANAGER.Write_8Bits(TECS_PITCH2THR_LPF_ADDR, Get_Servos_Parameters.GetTECSPitch2ThrLPF);
    STORAGEMANAGER.Write_8Bits(TECS_AP_LPF_ADDR, Get_Servos_Parameters.GetTECSAutoPilotLPF);
    STORAGEMANAGER.Write_16Bits(TECS_CRUISE_MIN_THR_ADDR, Get_Servos_Parameters.GetTECSCruiseMinThrottle);
    STORAGEMANAGER.Write_16Bits(TECS_CRUISE_MAX_THR_ADDR, Get_Servos_Parameters.GetTECSCruiseMaxThrottle);
    STORAGEMANAGER.Write_16Bits(TECS_CRUISE_THR_ADDR, Get_Servos_Parameters.GetTECSCruiseThrottle);
    STORAGEMANAGER.Write_8Bits(TECS_CIRCLE_DIR_ADDR, Get_Servos_Parameters.GetTECSCircleDirection);
    STORAGEMANAGER.Write_8Bits(CONT_SERVO_TRIM_STATE_ADDR, Get_Radio_Control_Parameters.GetContServosTrimState);
}

void GCSClass::Save_Medium_Configuration(void)
{
    STORAGEMANAGER.Write_8Bits(TPA_PERCENT_ADDR, Get_User_Medium_Parameters.GetTPAInPercent);
    STORAGEMANAGER.Write_16Bits(BREAKPOINT_ADDR, Get_User_Medium_Parameters.GetBreakPointValue);
    STORAGEMANAGER.Write_8Bits(HW_GYRO_LPF_ADDR, Get_User_Medium_Parameters.GetGyroLPF);
    STORAGEMANAGER.Write_16Bits(DERIVATIVE_LPF_ADDR, Get_User_Medium_Parameters.GetDerivativeLPF);
    STORAGEMANAGER.Write_16Bits(RC_LPF_ADDR, Get_User_Medium_Parameters.GetRCLPF);
    STORAGEMANAGER.Write_8Bits(KALMAN_ADDR, Get_User_Medium_Parameters.GetKalmanState);
    STORAGEMANAGER.Write_16Bits(BI_ACC_LPF_ADDR, Get_User_Medium_Parameters.GetBiquadAccLPF);
    STORAGEMANAGER.Write_16Bits(BI_GYRO_LPF_ADDR, Get_User_Medium_Parameters.GetBiquadGyroLPF);
    STORAGEMANAGER.Write_16Bits(BI_ACC_NOTCH_ADDR, Get_User_Medium_Parameters.GetBiquadAccNotch);
    STORAGEMANAGER.Write_16Bits(BI_GYRO_NOTCH_ADDR, Get_User_Medium_Parameters.GetBiquadGyroNotch);
    STORAGEMANAGER.Write_8Bits(MOT_COMP_STATE_ADDR, Get_User_Medium_Parameters.GetMotorCompensationState);
    STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, Get_User_Medium_Parameters.GetProportionalPitch);
    STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, Get_User_Medium_Parameters.GetIntegralPitch);
    STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, Get_User_Medium_Parameters.GetDerivativePitch);
    STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, Get_User_Medium_Parameters.GetProportionalRoll);
    STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, Get_User_Medium_Parameters.GetIntegralRoll);
    STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, Get_User_Medium_Parameters.GetDerivativeRoll);
    STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, Get_User_Medium_Parameters.GetProportionalYaw);
    STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, Get_User_Medium_Parameters.GetIntegralYaw);
    STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, Get_User_Medium_Parameters.GetDerivativeYaw);
    STORAGEMANAGER.Write_8Bits(KP_VEL_Z_ADDR, Get_User_Medium_Parameters.GetProportionalVelZ);
    STORAGEMANAGER.Write_8Bits(KI_VEL_Z_ADDR, Get_User_Medium_Parameters.GetIntegralVelZ);
    STORAGEMANAGER.Write_8Bits(KD_VEL_Z_ADDR, Get_User_Medium_Parameters.GetDerivativeVelZ);
    STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, Get_User_Medium_Parameters.GetProportionalGPSHold);
    STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, Get_User_Medium_Parameters.GetIntegralGPSHold);
    STORAGEMANAGER.Write_16Bits(SERVOS_LPF_ADDR, Get_User_Medium_Parameters.GetServosLPF);
    STORAGEMANAGER.Write_8Bits(FF_OR_CD_ROLL_ADDR, Get_User_Medium_Parameters.GetCDOrFFRoll);
    STORAGEMANAGER.Write_8Bits(FF_OR_CD_PITCH_ADDR, Get_User_Medium_Parameters.GetCDOrFFPitch);
    STORAGEMANAGER.Write_8Bits(FF_OR_CD_YAW_ADDR, Get_User_Medium_Parameters.GetCDOrFFYaw);
    STORAGEMANAGER.Write_8Bits(KP_AUTOLEVEL_ADDR, Get_User_Medium_Parameters.GetAutoLevelProportional);
    STORAGEMANAGER.Write_8Bits(KI_AUTOLEVEL_ADDR, Get_User_Medium_Parameters.GetAutoLevelIntegral);
    STORAGEMANAGER.Write_8Bits(KP_HEADING_HOLD_ADDR, Get_User_Medium_Parameters.GetHeadingHoldRate);
    STORAGEMANAGER.Write_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR, Get_User_Medium_Parameters.GetHeadingHoldRateLimit);
    STORAGEMANAGER.Write_8Bits(ROLL_BANK_ADDR, Get_User_Medium_Parameters.GetRollBankMax);
    STORAGEMANAGER.Write_8Bits(PITCH_BANK_MIN_ADDR, Get_User_Medium_Parameters.GetPitchBankMin);
    STORAGEMANAGER.Write_8Bits(PITCH_BANK_MAX_ADDR, Get_User_Medium_Parameters.GetPitchBankMax);
    STORAGEMANAGER.Write_8Bits(ATTACK_BANK_ADDR, Get_User_Medium_Parameters.GetAttackBank);
    STORAGEMANAGER.Write_8Bits(GPS_BANK_ADDR, Get_User_Medium_Parameters.GetGPSBank);
    STORAGEMANAGER.Write_16Bits(INTEGRAL_RELAX_LPF_ADDR, Get_User_Medium_Parameters.GetIntegralLPF);
    STORAGEMANAGER.Write_16Bits(KCD_OR_FF_LPF_ADDR, Get_User_Medium_Parameters.GetkCDLPF);
    STORAGEMANAGER.Write_8Bits(KP_POS_Z_ADDR, Get_User_Medium_Parameters.GetProportionalAltitudeHold);
    STORAGEMANAGER.Write_8Bits(KI_POS_Z_ADDR, Get_User_Medium_Parameters.GetIntegralAltitudeHold);
    STORAGEMANAGER.Write_8Bits(KD_POS_Z_ADDR, Get_User_Medium_Parameters.GetDerivativeAltitudeHold);
    STORAGEMANAGER.Write_8Bits(KP_POS_RATE_ADDR, Get_User_Medium_Parameters.GetProportionalPositionRate);
    STORAGEMANAGER.Write_8Bits(KI_POS_RATE_ADDR, Get_User_Medium_Parameters.GetIntegralPositionRate);
    STORAGEMANAGER.Write_8Bits(KD_POS_RATE_ADDR, Get_User_Medium_Parameters.GetDerivativePositionRate);
    STORAGEMANAGER.Write_8Bits(KP_NAV_RATE_ADDR, Get_User_Medium_Parameters.GetProportionalNavigationRate);
    STORAGEMANAGER.Write_8Bits(KI_NAV_RATE_ADDR, Get_User_Medium_Parameters.GetIntegralNavigationRate);
    STORAGEMANAGER.Write_8Bits(KD_NAV_RATE_ADDR, Get_User_Medium_Parameters.GetDerivativeNavigationRate);
    STORAGEMANAGER.Write_8Bits(INTEGRAL_WINDUP_ADDR, Get_User_Medium_Parameters.GetIntegralWindUp);

    //ATUALIZA OS PARAMETROS DO PID
    GET_SET[PID_UPDATED].State = false;
}

void GCSClass::Default_Basic_Configuration(void)
{
    //LIMPA TODAS AS CONFIGURAÇÕES SALVAS
    STORAGEMANAGER.Write_8Bits(SIMPLE_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(ALT_HOLD_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(GPS_HOLD_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(RTH_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(PARACHUTE_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(GIMBAL_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(FRAME_TYPE_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(RC_SEQUENCY_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(COMPASS_ROTATION_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_1_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_2_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_3_ADDR, 0);
    if (GetMultirotorEnabled())
    {
        STORAGEMANAGER.Write_8Bits(RTH_ALTITUDE_ADDR, 10);
    }
    else if (GetAirPlaneEnabled())
    {
        STORAGEMANAGER.Write_8Bits(RTH_ALTITUDE_ADDR, 40);
    }
    STORAGEMANAGER.Write_8Bits(STABLIZE_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(ATTACK_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(AUTOFLIP_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(AUTOMISSION_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(ARMDISARM_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(AUTOLAND_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(DISP_PASSIVES_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(AIRSPEED_TYPE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(PITCH_LEVEL_TRIM_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(BATT_VOLTAGE_FACTOR_ADDR, 1010);
    STORAGEMANAGER.Write_16Bits(BATT_AMPS_VOLT_ADDR, 6200);
    STORAGEMANAGER.Write_16Bits(BATT_AMPS_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(BATT_MIN_VOLTAGE_ADDR, 360);
    STORAGEMANAGER.Write_16Bits(BATT_MAX_VOLTAGE_ADDR, 420);
    STORAGEMANAGER.Write_8Bits(BATT_NUMBER_OF_CELLS_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(BATT_CRIT_PERCENT_ADDR, 20);
    STORAGEMANAGER.Write_8Bits(BATT_RTH_LOW_BATT_ADDR, 0);
}

void GCSClass::Default_RadioControl_Configuration(void)
{
    //LIMPA TODAS AS CONFIGURAÇÕES SALVAS DO RÁDIO E DOS SERVOS
    STORAGEMANAGER.Write_8Bits(THROTTLE_MIDDLE_ADDR, 50);
    STORAGEMANAGER.Write_8Bits(THROTTLE_EXPO_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(PR_RATE_ADDR, 20);
    STORAGEMANAGER.Write_8Bits(PR_EXPO_ADDR, 70);
    STORAGEMANAGER.Write_8Bits(YAW_RATE_ADDR, 20);
    STORAGEMANAGER.Write_8Bits(YAW_EXPO_ADDR, 20);
    STORAGEMANAGER.Write_16Bits(THR_ATTITUDE_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(THR_ATTITUDE_MAX_ADDR, 1850);
    STORAGEMANAGER.Write_16Bits(THROTTLE_MIN_ADDR, 1050);
    STORAGEMANAGER.Write_16Bits(YAW_MIN_ADDR, 1050);
    STORAGEMANAGER.Write_16Bits(PITCH_MIN_ADDR, 1050);
    STORAGEMANAGER.Write_16Bits(ROLL_MIN_ADDR, 1050);
    STORAGEMANAGER.Write_16Bits(THROTTLE_MAX_ADDR, 1950);
    STORAGEMANAGER.Write_16Bits(YAW_MAX_ADDR, 1950);
    STORAGEMANAGER.Write_16Bits(PITCH_MAX_ADDR, 1950);
    STORAGEMANAGER.Write_16Bits(ROLL_MAX_ADDR, 1950);
    STORAGEMANAGER.Write_8Bits(THROTTLE_DZ_ADDR, 45);
    STORAGEMANAGER.Write_8Bits(YAW_DZ_ADDR, 45);
    STORAGEMANAGER.Write_8Bits(PITCH_DZ_ADDR, 45);
    STORAGEMANAGER.Write_8Bits(ROLL_DZ_ADDR, 45);
    STORAGEMANAGER.Write_8Bits(CH_REVERSE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(SERVO1_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(SERVO2_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(SERVO3_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(SERVO4_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(SERVO1_MID_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(SERVO2_MID_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(SERVO3_MID_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(SERVO4_MID_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(SERVO1_MAX_ADDR, 2000);
    STORAGEMANAGER.Write_16Bits(SERVO2_MAX_ADDR, 2000);
    STORAGEMANAGER.Write_16Bits(SERVO3_MAX_ADDR, 2000);
    STORAGEMANAGER.Write_16Bits(SERVO4_MAX_ADDR, 2000);
    STORAGEMANAGER.Write_8Bits(SERVOS_REVERSE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(SERVO1_RATE_ADDR, 100);
    STORAGEMANAGER.Write_16Bits(SERVO2_RATE_ADDR, 100);
    STORAGEMANAGER.Write_16Bits(SERVO3_RATE_ADDR, 100);
    STORAGEMANAGER.Write_16Bits(SERVO4_RATE_ADDR, 100);
    STORAGEMANAGER.Write_16Bits(FAILSAFE_VAL_ADDR, 975);
    STORAGEMANAGER.Write_8Bits(MAX_PITCH_LEVEL_ADDR, 30);
    STORAGEMANAGER.Write_8Bits(MAX_ROLL_LEVEL_ADDR, 30);
    STORAGEMANAGER.Write_8Bits(AUTO_PILOT_MODE_ADDR, 0);
    STORAGEMANAGER.Write_32Bits(AIRSPEED_FACTOR_ADDR, 10000);
    STORAGEMANAGER.Write_8Bits(CH_TUNNING_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(TUNNING_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(LAND_AFTER_RTH_ADDR, 1);
    STORAGEMANAGER.Write_16Bits(HOVER_THROTTLE_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(AIR_SPEED_REFERENCE_ADDR, 1500);
    STORAGEMANAGER.Write_8Bits(TECS_PITCH2THR_FACTOR_ADDR, 10);
    STORAGEMANAGER.Write_8Bits(TECS_PITCH2THR_LPF_ADDR, 6);
    STORAGEMANAGER.Write_8Bits(TECS_AP_LPF_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(TECS_CRUISE_MIN_THR_ADDR, 1200);
    STORAGEMANAGER.Write_16Bits(TECS_CRUISE_MAX_THR_ADDR, 1700);
    STORAGEMANAGER.Write_16Bits(TECS_CRUISE_THR_ADDR, 1400);
    STORAGEMANAGER.Write_8Bits(TECS_CIRCLE_DIR_ADDR, 1);
    STORAGEMANAGER.Write_8Bits(CONT_SERVO_TRIM_STATE_ADDR, 0);
}

void GCSClass::Default_Medium_Configuration(void)
{
    //LIMPA AS CONFIGURAÇÕES SALVAS
    STORAGEMANAGER.Write_16Bits(BREAKPOINT_ADDR, 1500);
    STORAGEMANAGER.Write_8Bits(TPA_PERCENT_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(HW_GYRO_LPF_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(DERIVATIVE_LPF_ADDR, 40);
    STORAGEMANAGER.Write_16Bits(RC_LPF_ADDR, 50);
    STORAGEMANAGER.Write_8Bits(KALMAN_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(BI_ACC_LPF_ADDR, 15);
    STORAGEMANAGER.Write_16Bits(BI_GYRO_LPF_ADDR, 60);
    STORAGEMANAGER.Write_16Bits(BI_ACC_NOTCH_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(BI_GYRO_NOTCH_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(MOT_COMP_STATE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(SERVOS_LPF_ADDR, 50);
    STORAGEMANAGER.Write_16Bits(INTEGRAL_RELAX_LPF_ADDR, 15);
    STORAGEMANAGER.Write_16Bits(KCD_OR_FF_LPF_ADDR, 30);
    STORAGEMANAGER.Write_8Bits(INTEGRAL_WINDUP_ADDR, 50);

    if (GetMultirotorEnabled())
    {
        DefaultForMultirotorPlatform();
    }
    else if (GetAirPlaneEnabled())
    {
        DefaultForAirPlanePlatform();
    }

    DefaultForAllPlatforms();

    //ATUALIZA OS PARAMETROS DO PID
    GET_SET[PID_UPDATED].State = false;
}

void GCSClass::Default_All_Configs(void)
{
    GCS.Default_Basic_Configuration();
    GCS.Default_Medium_Configuration();
    GCS.Default_RadioControl_Configuration();
}

void GCSClass::LoadAllParameters(void)
{
    //ATUALIZA OS PARAMETROS BASICOS AJUSTAVEIS PELO USUARIO
    Send_User_Basic_Parameters.SendFrameType = STORAGEMANAGER.Read_8Bits(FRAME_TYPE_ADDR);
    Send_User_Basic_Parameters.SendRcChSequency = STORAGEMANAGER.Read_8Bits(RC_SEQUENCY_ADDR);
    Send_User_Basic_Parameters.SendGimbalType = STORAGEMANAGER.Read_8Bits(GIMBAL_ADDR);
    Send_User_Basic_Parameters.SendParachuteType = STORAGEMANAGER.Read_8Bits(PARACHUTE_ADDR);
    Send_User_Basic_Parameters.SendUartNumb1Type = STORAGEMANAGER.Read_8Bits(UART_NUMB_1_ADDR);
    Send_User_Basic_Parameters.SendUartNumb2Type = STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR);
    Send_User_Basic_Parameters.SendUartNumb3Type = STORAGEMANAGER.Read_8Bits(UART_NUMB_3_ADDR);
    Send_User_Basic_Parameters.SendCompassRotationType = STORAGEMANAGER.Read_8Bits(COMPASS_ROTATION_ADDR);
    Send_User_Basic_Parameters.SendRTHAltitude = STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR);
    Send_User_Basic_Parameters.SendAcroType = STORAGEMANAGER.Read_8Bits(STABLIZE_ADDR);
    Send_User_Basic_Parameters.SendAltitudeHoldType = STORAGEMANAGER.Read_8Bits(ALT_HOLD_ADDR);
    Send_User_Basic_Parameters.SendPositionHoldType = STORAGEMANAGER.Read_8Bits(GPS_HOLD_ADDR);
    Send_User_Basic_Parameters.SendSimpleControlType = STORAGEMANAGER.Read_8Bits(SIMPLE_ADDR);
    Send_User_Basic_Parameters.SendReturnToHomeType = STORAGEMANAGER.Read_8Bits(RTH_ADDR);
    Send_User_Basic_Parameters.SendAtackType = STORAGEMANAGER.Read_8Bits(ATTACK_ADDR);
    Send_User_Basic_Parameters.SendAutomaticFlipType = STORAGEMANAGER.Read_8Bits(AUTOFLIP_ADDR);
    Send_User_Basic_Parameters.SendAutomaticMissonType = STORAGEMANAGER.Read_8Bits(AUTOMISSION_ADDR);
    Send_User_Basic_Parameters.SendArmDisarmType = STORAGEMANAGER.Read_8Bits(ARMDISARM_ADDR);
    Send_User_Basic_Parameters.SendAutoLandType = STORAGEMANAGER.Read_8Bits(AUTOLAND_ADDR);
    Send_User_Basic_Parameters.SendSafeBtnState = STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR);
    Send_User_Basic_Parameters.SendAirSpeedState = STORAGEMANAGER.Read_8Bits(AIRSPEED_TYPE_ADDR);
    Send_User_Basic_Parameters.SendPitchLevelTrim = STORAGEMANAGER.Read_16Bits(PITCH_LEVEL_TRIM_ADDR);
    Send_User_Basic_Parameters.SendBatteryVoltageScale = STORAGEMANAGER.Read_16Bits(BATT_VOLTAGE_FACTOR_ADDR);
    Send_User_Basic_Parameters.SendBatteryCurrentScale = STORAGEMANAGER.Read_16Bits(BATT_AMPS_VOLT_ADDR);
    Send_User_Basic_Parameters.SendBatteryCurrentOffSet = STORAGEMANAGER.Read_16Bits(BATT_AMPS_OFFSET_ADDR);
    Send_User_Basic_Parameters.SendBattMinVoltage = STORAGEMANAGER.Read_16Bits(BATT_MIN_VOLTAGE_ADDR);
    Send_User_Basic_Parameters.SendBattMaxVoltage = STORAGEMANAGER.Read_16Bits(BATT_MAX_VOLTAGE_ADDR);
    Send_User_Basic_Parameters.SendNumberOfCells = STORAGEMANAGER.Read_8Bits(BATT_NUMBER_OF_CELLS_ADDR);
    Send_User_Basic_Parameters.SendCriticBattPercent = STORAGEMANAGER.Read_8Bits(BATT_CRIT_PERCENT_ADDR);
    Send_User_Basic_Parameters.SendRTHLowBatt = STORAGEMANAGER.Read_8Bits(BATT_RTH_LOW_BATT_ADDR);

    //ATUALIZA OS PARAMETROS DO RADIO CONTROLE
    Send_Radio_Control_Parameters.SendThrottleMiddle = STORAGEMANAGER.Read_8Bits(THROTTLE_MIDDLE_ADDR);
    Send_Radio_Control_Parameters.SendThrottleExpo = STORAGEMANAGER.Read_8Bits(THROTTLE_EXPO_ADDR);
    Send_Radio_Control_Parameters.SendPRRate = STORAGEMANAGER.Read_8Bits(PR_RATE_ADDR);
    Send_Radio_Control_Parameters.SendPRExpo = STORAGEMANAGER.Read_8Bits(PR_EXPO_ADDR);
    Send_Radio_Control_Parameters.SendYawRate = STORAGEMANAGER.Read_8Bits(YAW_RATE_ADDR);
    Send_Radio_Control_Parameters.SendYawExpo = STORAGEMANAGER.Read_8Bits(YAW_EXPO_ADDR);
    Send_Radio_Control_Parameters.SendRCPulseMin = STORAGEMANAGER.Read_16Bits(THR_ATTITUDE_MIN_ADDR);
    Send_Radio_Control_Parameters.SendRCPulseMax = STORAGEMANAGER.Read_16Bits(THR_ATTITUDE_MAX_ADDR);
    Send_Radio_Control_Parameters.SendThrottleMin = STORAGEMANAGER.Read_16Bits(THROTTLE_MIN_ADDR);
    Send_Radio_Control_Parameters.SendYawMin = STORAGEMANAGER.Read_16Bits(YAW_MIN_ADDR);
    Send_Radio_Control_Parameters.SendPitchMin = STORAGEMANAGER.Read_16Bits(PITCH_MIN_ADDR);
    Send_Radio_Control_Parameters.SendRollMin = STORAGEMANAGER.Read_16Bits(ROLL_MIN_ADDR);
    Send_Radio_Control_Parameters.SendThrottleMax = STORAGEMANAGER.Read_16Bits(THROTTLE_MAX_ADDR);
    Send_Radio_Control_Parameters.SendYawMax = STORAGEMANAGER.Read_16Bits(YAW_MAX_ADDR);
    Send_Radio_Control_Parameters.SendPitchMax = STORAGEMANAGER.Read_16Bits(PITCH_MAX_ADDR);
    Send_Radio_Control_Parameters.SendRollMax = STORAGEMANAGER.Read_16Bits(ROLL_MAX_ADDR);
    Send_Radio_Control_Parameters.SendThrottleDeadZone = STORAGEMANAGER.Read_8Bits(THROTTLE_DZ_ADDR);
    Send_Radio_Control_Parameters.SendYawDeadZone = STORAGEMANAGER.Read_8Bits(YAW_DZ_ADDR);
    Send_Radio_Control_Parameters.SendPitchDeadZone = STORAGEMANAGER.Read_8Bits(PITCH_DZ_ADDR);
    Send_Radio_Control_Parameters.SendRollDeadZone = STORAGEMANAGER.Read_8Bits(ROLL_DZ_ADDR);
    Send_Radio_Control_Parameters.SendChannelsReverse = STORAGEMANAGER.Read_8Bits(CH_REVERSE_ADDR);
    Send_Radio_Control_Parameters.SendServo1Rate = STORAGEMANAGER.Read_16Bits(SERVO1_RATE_ADDR);
    Send_Radio_Control_Parameters.SendServo2Rate = STORAGEMANAGER.Read_16Bits(SERVO2_RATE_ADDR);
    Send_Radio_Control_Parameters.SendServo3Rate = STORAGEMANAGER.Read_16Bits(SERVO3_RATE_ADDR);
    Send_Radio_Control_Parameters.SendServo4Rate = STORAGEMANAGER.Read_16Bits(SERVO4_RATE_ADDR);
    Send_Radio_Control_Parameters.SendServosReverse = STORAGEMANAGER.Read_8Bits(SERVOS_REVERSE_ADDR);
    Send_Radio_Control_Parameters.SendServo1Min = STORAGEMANAGER.Read_16Bits(SERVO1_MIN_ADDR);
    Send_Radio_Control_Parameters.SendServo2Min = STORAGEMANAGER.Read_16Bits(SERVO2_MIN_ADDR);
    Send_Radio_Control_Parameters.SendServo3Min = STORAGEMANAGER.Read_16Bits(SERVO3_MIN_ADDR);
    Send_Radio_Control_Parameters.SendServo4Min = STORAGEMANAGER.Read_16Bits(SERVO4_MIN_ADDR);
    Send_Radio_Control_Parameters.SendServo1Med = STORAGEMANAGER.Read_16Bits(SERVO1_MID_ADDR);
    Send_Radio_Control_Parameters.SendServo2Med = STORAGEMANAGER.Read_16Bits(SERVO2_MID_ADDR);
    Send_Radio_Control_Parameters.SendServo3Med = STORAGEMANAGER.Read_16Bits(SERVO3_MID_ADDR);
    Send_Radio_Control_Parameters.SendServo4Med = STORAGEMANAGER.Read_16Bits(SERVO4_MID_ADDR);
    Send_Radio_Control_Parameters.SendServo1Max = STORAGEMANAGER.Read_16Bits(SERVO1_MAX_ADDR);
    Send_Radio_Control_Parameters.SendServo2Max = STORAGEMANAGER.Read_16Bits(SERVO2_MAX_ADDR);
    Send_Radio_Control_Parameters.SendServo3Max = STORAGEMANAGER.Read_16Bits(SERVO3_MAX_ADDR);
    Send_Radio_Control_Parameters.SendServo4Max = STORAGEMANAGER.Read_16Bits(SERVO4_MAX_ADDR);
    Send_Radio_Control_Parameters.SendFailSafeValue = STORAGEMANAGER.Read_16Bits(FAILSAFE_VAL_ADDR);
    Send_Radio_Control_Parameters.SendMaxBankPitch = STORAGEMANAGER.Read_8Bits(MAX_PITCH_LEVEL_ADDR);
    Send_Radio_Control_Parameters.SendMaxBankRoll = STORAGEMANAGER.Read_8Bits(MAX_ROLL_LEVEL_ADDR);
    Send_Radio_Control_Parameters.SendAutoPilotMode = STORAGEMANAGER.Read_8Bits(AUTO_PILOT_MODE_ADDR);
    Send_Radio_Control_Parameters.SendAirSpeedScale = STORAGEMANAGER.Read_32Bits(AIRSPEED_FACTOR_ADDR);
    Send_Radio_Control_Parameters.SendTunningChannel = STORAGEMANAGER.Read_8Bits(CH_TUNNING_ADDR);
    Send_Radio_Control_Parameters.SendTunning = STORAGEMANAGER.Read_8Bits(TUNNING_ADDR);
    Send_Radio_Control_Parameters.SendLandAfterRTH = STORAGEMANAGER.Read_8Bits(LAND_AFTER_RTH_ADDR);
    Send_Radio_Control_Parameters.SendHoverThrottle = STORAGEMANAGER.Read_16Bits(HOVER_THROTTLE_ADDR);
    Send_Radio_Control_Parameters.SendAirSpeedReference = STORAGEMANAGER.Read_16Bits(AIR_SPEED_REFERENCE_ADDR);
    Send_Radio_Control_Parameters.SendTECSPitch2ThrFactor = STORAGEMANAGER.Read_8Bits(TECS_PITCH2THR_FACTOR_ADDR);
    Send_Radio_Control_Parameters.SendTECSPitch2ThrLPF = STORAGEMANAGER.Read_8Bits(TECS_PITCH2THR_LPF_ADDR);
    Send_Radio_Control_Parameters.SendTECSAutoPilotLPF = STORAGEMANAGER.Read_8Bits(TECS_AP_LPF_ADDR);
    Send_Radio_Control_Parameters.SendTECSCruiseMinThrottle = STORAGEMANAGER.Read_16Bits(TECS_CRUISE_MIN_THR_ADDR);
    Send_Radio_Control_Parameters.SendTECSCruiseMaxThrottle = STORAGEMANAGER.Read_16Bits(TECS_CRUISE_MAX_THR_ADDR);
    Send_Radio_Control_Parameters.SendTECSCruiseThrottle = STORAGEMANAGER.Read_16Bits(TECS_CRUISE_THR_ADDR);
    Send_Radio_Control_Parameters.SendTECSCircleDirection = STORAGEMANAGER.Read_8Bits(TECS_CIRCLE_DIR_ADDR);
    Send_Radio_Control_Parameters.SendContServosTrimState = STORAGEMANAGER.Read_8Bits(CONT_SERVO_TRIM_STATE_ADDR);

    //ATUALIZA OS PARAMETROS MEDIOS AJUSTAVEIS PELO USUARIO
    Send_User_Medium_Parameters.SendTPAInPercent = STORAGEMANAGER.Read_8Bits(TPA_PERCENT_ADDR);
    Send_User_Medium_Parameters.SendBreakPointValue = STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR);
    Send_User_Medium_Parameters.SendGyroLPF = STORAGEMANAGER.Read_8Bits(HW_GYRO_LPF_ADDR);
    Send_User_Medium_Parameters.SendDerivativeLPF = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
    Send_User_Medium_Parameters.SendRCLPF = STORAGEMANAGER.Read_16Bits(RC_LPF_ADDR);
    Send_User_Medium_Parameters.SendKalmanState = STORAGEMANAGER.Read_8Bits(KALMAN_ADDR);
    Send_User_Medium_Parameters.SendBiQuadAccLPF = STORAGEMANAGER.Read_16Bits(BI_ACC_LPF_ADDR);
    Send_User_Medium_Parameters.SendBiQuadGyroLPF = STORAGEMANAGER.Read_16Bits(BI_GYRO_LPF_ADDR);
    Send_User_Medium_Parameters.SendBiQuadAccNotch = STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR);
    Send_User_Medium_Parameters.SendBiQuadGyroNotch = STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR);
    Send_User_Medium_Parameters.SendMotorCompensationState = STORAGEMANAGER.Read_8Bits(MOT_COMP_STATE_ADDR);
    Send_User_Medium_Parameters.SendProportionalPitch = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    Send_User_Medium_Parameters.SendIntegralPitch = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    Send_User_Medium_Parameters.SendDerivativePitch = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    Send_User_Medium_Parameters.SendProportionalRoll = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    Send_User_Medium_Parameters.SendIntegralRoll = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    Send_User_Medium_Parameters.SendDerivativeRoll = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    Send_User_Medium_Parameters.SendProportionalYaw = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    Send_User_Medium_Parameters.SendIntegralYaw = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    Send_User_Medium_Parameters.SendDerivativeYaw = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    Send_User_Medium_Parameters.SendProportionalVelZ = STORAGEMANAGER.Read_8Bits(KP_VEL_Z_ADDR);
    Send_User_Medium_Parameters.SendIntegralVelZ = STORAGEMANAGER.Read_8Bits(KI_VEL_Z_ADDR);
    Send_User_Medium_Parameters.SendDerivativeVelZ = STORAGEMANAGER.Read_8Bits(KD_VEL_Z_ADDR);
    Send_User_Medium_Parameters.SendProportionalGPSHold = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    Send_User_Medium_Parameters.SendIntegralGPSHold = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);
    Send_User_Medium_Parameters.SendServosLPF = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);
    Send_User_Medium_Parameters.SendCDOrFFRoll = STORAGEMANAGER.Read_8Bits(FF_OR_CD_ROLL_ADDR);
    Send_User_Medium_Parameters.SendCDOrFFPitch = STORAGEMANAGER.Read_8Bits(FF_OR_CD_PITCH_ADDR);
    Send_User_Medium_Parameters.SendCDOrFFYaw = STORAGEMANAGER.Read_8Bits(FF_OR_CD_YAW_ADDR);
    Send_User_Medium_Parameters.SendAutoLevelProportional = STORAGEMANAGER.Read_8Bits(KP_AUTOLEVEL_ADDR);
    Send_User_Medium_Parameters.SendAutoLevelIntegral = STORAGEMANAGER.Read_8Bits(KI_AUTOLEVEL_ADDR);
    Send_User_Medium_Parameters.SendHeadingHoldRate = STORAGEMANAGER.Read_8Bits(KP_HEADING_HOLD_ADDR);
    Send_User_Medium_Parameters.SendHeadingHoldRateLimit = STORAGEMANAGER.Read_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR);
    Send_User_Medium_Parameters.SendRollBankMax = STORAGEMANAGER.Read_8Bits(ROLL_BANK_ADDR);
    Send_User_Medium_Parameters.SendPitchBankMin = STORAGEMANAGER.Read_8Bits(PITCH_BANK_MIN_ADDR);
    Send_User_Medium_Parameters.SendPitchBankMax = STORAGEMANAGER.Read_8Bits(PITCH_BANK_MAX_ADDR);
    Send_User_Medium_Parameters.SendAttackBank = STORAGEMANAGER.Read_8Bits(ATTACK_BANK_ADDR);
    Send_User_Medium_Parameters.SendGPSBank = STORAGEMANAGER.Read_8Bits(GPS_BANK_ADDR);
    Send_User_Medium_Parameters.SendIntegralLPF = STORAGEMANAGER.Read_16Bits(INTEGRAL_RELAX_LPF_ADDR);
    Send_User_Medium_Parameters.SendkCDLPF = STORAGEMANAGER.Read_16Bits(KCD_OR_FF_LPF_ADDR);
    Send_User_Medium_Parameters.SendProportionalAltitudeHold = STORAGEMANAGER.Read_8Bits(KP_POS_Z_ADDR);
    Send_User_Medium_Parameters.SendIntegralAltitudeHold = STORAGEMANAGER.Read_8Bits(KI_POS_Z_ADDR);
    Send_User_Medium_Parameters.SendDerivativeAltitudeHold = STORAGEMANAGER.Read_8Bits(KD_POS_Z_ADDR);
    Send_User_Medium_Parameters.SendProportionalPositionRate = STORAGEMANAGER.Read_8Bits(KP_POS_RATE_ADDR);
    Send_User_Medium_Parameters.SendIntegralPositionRate = STORAGEMANAGER.Read_8Bits(KI_POS_RATE_ADDR);
    Send_User_Medium_Parameters.SendDerivativePositionRate = STORAGEMANAGER.Read_8Bits(KD_POS_RATE_ADDR);
    Send_User_Medium_Parameters.SendProportionalNavigationRate = STORAGEMANAGER.Read_8Bits(KP_NAV_RATE_ADDR);
    Send_User_Medium_Parameters.SendIntegralNavigationRate = STORAGEMANAGER.Read_8Bits(KI_NAV_RATE_ADDR);
    Send_User_Medium_Parameters.SendDerivativeNavigationRate = STORAGEMANAGER.Read_8Bits(KD_NAV_RATE_ADDR);
    Send_User_Medium_Parameters.SendIntegralWindUp = STORAGEMANAGER.Read_8Bits(INTEGRAL_WINDUP_ADDR);
}