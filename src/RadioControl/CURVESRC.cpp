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

#include "CURVESRC.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Math/MATHSUPPORT.h"
#include "PID/TPA.h"
#include "Common/RCDEFINES.h"
#include "PID/RCPID.h"
#include "RadioControl/DECODE.h"

#define THROTTLE_LOOKUP_LENGTH 11

uint16_t CalculeLookUpThrottle[THROTTLE_LOOKUP_LENGTH];

void CurvesRC_SetValues()
{
  RC_Resources.Middle.Throttle = STORAGEMANAGER.Read_8Bits(THROTTLE_MIDDLE_ADDR);
  RC_Resources.Expo.Throttle = STORAGEMANAGER.Read_8Bits(THROTTLE_EXPO_ADDR);
  RC_Resources.Rate.PitchRoll = STORAGEMANAGER.Read_8Bits(PR_RATE_ADDR);
  RC_Resources.Expo.PitchRoll = STORAGEMANAGER.Read_8Bits(PR_EXPO_ADDR);
  RC_Resources.Rate.Yaw = STORAGEMANAGER.Read_8Bits(YAW_RATE_ADDR);
  RC_Resources.Expo.Yaw = STORAGEMANAGER.Read_8Bits(YAW_EXPO_ADDR);
  RC_Resources.Attitude.ThrottleMin = STORAGEMANAGER.Read_16Bits(THR_ATTITUDE_MIN_ADDR);
  RC_Resources.Attitude.ThrottleMax = STORAGEMANAGER.Read_16Bits(THR_ATTITUDE_MAX_ADDR);
}

void CurvesRC_CalculeValue()
{
  if (RC_Resources.Middle.Throttle == 0) //EVITA DE OCORRER UMA DIVISÃO POR ZERO
  {
    return;
  }

  int8_t NewValueCalculed;
  uint8_t ThrottleMiddlePoint;

  for (uint8_t IndexOfLookUpThrottle = 0; IndexOfLookUpThrottle < THROTTLE_LOOKUP_LENGTH; IndexOfLookUpThrottle++)
  {
    NewValueCalculed = 10 * IndexOfLookUpThrottle - RC_Resources.Middle.Throttle;
    ThrottleMiddlePoint = RC_Resources.Middle.Throttle;
    if (NewValueCalculed > 0)
    {
      ThrottleMiddlePoint = 100 - ThrottleMiddlePoint;
    }
    CalculeLookUpThrottle[IndexOfLookUpThrottle] = 100 * RC_Resources.Middle.Throttle + NewValueCalculed * ((int32_t)RC_Resources.Expo.Throttle * (NewValueCalculed * NewValueCalculed) / ((uint16_t)ThrottleMiddlePoint * ThrottleMiddlePoint) + 100 - RC_Resources.Expo.Throttle);
    CalculeLookUpThrottle[IndexOfLookUpThrottle] = RC_Resources.Attitude.ThrottleMin + (uint32_t)((uint16_t)(RC_Resources.Attitude.ThrottleMax - RC_Resources.Attitude.ThrottleMin)) * CalculeLookUpThrottle[IndexOfLookUpThrottle] / 10000;
  }
}

float RCControllerToRate(int16_t StickData, uint8_t Rate)
{
  const int16_t MaximumRateDPS = ConvertDegreesToDecidegrees(Rate);
  return ScaleRangeFloat((int16_t)StickData, -500, 500, -MaximumRateDPS, MaximumRateDPS);
}

int16_t CalcedAttitudeRC(int16_t Data, int16_t RcExpo)
{
  int16_t RCValueDeflection;
  RCValueDeflection = Constrain_16Bits(DECODE.GetRxChannelOutput(Data) - MIDDLE_STICKS_PULSE, -500, 500);
  float ConvertValueToFloat = RCValueDeflection / 100.0f;
  return lrint((2500.0f + (float)RcExpo * (ConvertValueToFloat * ConvertValueToFloat - 25.0f)) * ConvertValueToFloat / 25.0f);
}

float RcControllerToAngle(int16_t RcControllerInput, int16_t MaxInclination)
{
  RcControllerInput = Constrain_Float(RcControllerInput, -500, 500);
  return ScaleRangeFloat((float)RcControllerInput, -500.0f, 500.0f, (float)-MaxInclination, (float)MaxInclination);
}

int16_t PIDAngleToRcController(float AngleDeciDegrees, int16_t MaxInclination)
{
  AngleDeciDegrees = Constrain_Float(AngleDeciDegrees, (float)-MaxInclination, (float)MaxInclination);
  return ScaleRangeFloat((float)AngleDeciDegrees, (float)-MaxInclination, (float)MaxInclination, -500.0f, 500.0f);
}

float RcControllerToAngleWithMinMax(int16_t RcControllerInput, int16_t MinInclination, int16_t MaxInclination)
{
  RcControllerInput = Constrain_Float(RcControllerInput, -500, 500);
  return ScaleRangeFloat((float)RcControllerInput, -500.0f, 500.0f, (float)-MinInclination, (float)MaxInclination);
}

uint16_t CalcedLookupThrottle(uint16_t CalcedDeflection)
{
  if (CalcedDeflection > 999)
  {
    return RC_Resources.Attitude.ThrottleMax;
  }

  const uint8_t CalcedLookUpStep = CalcedDeflection / 100;
  return CalculeLookUpThrottle[CalcedLookUpStep] + (CalcedDeflection - CalcedLookUpStep * 100) * (CalculeLookUpThrottle[CalcedLookUpStep + 1] - CalculeLookUpThrottle[CalcedLookUpStep]) / 100;
}