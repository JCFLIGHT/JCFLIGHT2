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

#include "AIRSPEED.h"
#include "Math/MATHSUPPORT.h"
#include "AIRSPEEDANALOG.h"
#include "AIRSPEEDI2C.h"
#include "AIRSPEEDVIRTUAL.h"
#include "AIRSPEEDBACKEND.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "CALIBRATION.h"
#include "Filters/PT1.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "Param/PARAM.h"
#include "Barometer/BAROFRONTEND.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

AirSpeedClass AIRSPEED;

AirSpeed_Struct AirSpeed;
PT1_Filter_Struct Pitot_Smooth;

#define PITOT_LPF_CUTOFF 350 / 1000.0f   //EM MILLI-HZ
#define AIR_DENSITY_SEA_LEVEL_15C 1.225f //DENSIDADE DO AR ACIMADO NO NIVEL DO MAR A 15 GRAUS CELSIUS

void AirSpeedClass::Initialization(void)
{
  if (GetMultirotorEnabled() || Get_AirSpeed_Type() == AIR_SPEED_DISABLED)
  {
    return;
  }

  AirSpeed.Healthy = true;

  AirSpeed.Param.Factor = (float)STORAGEMANAGER.Read_32Bits(AIRSPEED_FACTOR_ADDR) / 10000.0f;

  PT1FilterInit(&Pitot_Smooth, PITOT_LPF_CUTOFF, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);

  AIRSPEEDCALIBRATION.Initialization();
}

float AirSpeedClass::Get_Raw_Pressure(void)
{
  float RetValue = 0;

  switch (Get_AirSpeed_Type())
  {

  case ANALOG_AIR_SPEED:
    RetValue = AirSpeed_Analog_Get_Actual_Value();
    break;

  case DIGITAL_AIR_SPEED:
    RetValue = AirSpeed_I2C_Get_Actual_Value();
    break;

  case VIRTUAL_AIR_SPEED:
    RetValue = AirSpeed_Virtual_Get_Actual_Value();
    break;
  }

  return RetValue;
}

void AirSpeedClass::Parse_IAS_Pressure(float &Pressure)
{
  //https://en.wikipedia.org/wiki/Indicated_airspeed
  AirSpeed.Raw.DifferentialPressure = 2.0f * ABS(AirSpeed.Raw.Pressure - AirSpeed.Calibration.OffSet) / AIR_DENSITY_SEA_LEVEL_15C;
  Pressure = AirSpeed.Param.Factor * Fast_SquareRoot(AirSpeed.Raw.DifferentialPressure);
  Pressure = PT1FilterApply3(&Pitot_Smooth, Pressure);
}

void AirSpeedClass::Update(void)
{
  if (!AirSpeed.Healthy)
  {
    return;
  }

  AirSpeed.Raw.Pressure = AIRSPEED.Get_Raw_Pressure();

  if (!AIRSPEEDCALIBRATION.Calibrate())
  {
    return;
  }

  AIRSPEEDCALIBRATION.Scale_Update();

  AirSpeed.Raw.IASPressure = 0.0f;

  AIRSPEED.Parse_IAS_Pressure(AirSpeed.Raw.IASPressure);

  AirSpeed.Raw.IASPressureInCM = AirSpeed.Raw.IASPressure * 100; //EM CM/S
}

float AirSpeedClass::Get_True_Value(const char *Type)
{
  if (!AirSpeed.Healthy)
  {
    return 1.0f;
  }

  float EAS2TAS = Get_EAS2TAS();

  if (strncasecmp(Type, "In Centimeters", 14) == 0)
  {
    return AirSpeed.Raw.IASPressureInCM * EAS2TAS;
  }
  else if (strncasecmp(Type, "In Meters", 9) == 0)
  {
    return AirSpeed.Raw.IASPressure * EAS2TAS;
  }

  return 0.0f;
}