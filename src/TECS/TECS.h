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

#ifndef TECS_H_
#define TECS_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern TECS_Resources_Struct TECS_Resources;
class TecsClass
{
public:
  void Initialization(void);
  void Update(float DeltaTime);
  float AutoPitchDown(int16_t InMinThrottleDownPitchAngle);
  float GetFuselageVelocity(void);

private:
  float Floating_Point_PID(TECS_PID_Float_Struct *TECS_PID_Pointer, const float SetProportional, const float SetIntegrator, const float SetDerivative, const float PIDSetPoint,
                           const float RawMeasurement, const float PIDScaler, const float DerivativeScaler, const float OutputMin, const float OutputMax, const uint8_t Flags, const float DeltaTime);
  void Reset_PID_Navigation(TECS_PID_Float_Struct *TECS_PID_Pointer, float DerivativeCutOff);
  bool GetNavigationInAutomaticThrottleMode(void);
  int16_t UpdatePitchToThrottle(int16_t PitchInput, float DeltaTime);
  void UpdateEnergyAltitudeController(float DeltaTime);
  int16_t GetEnergyMotorSpeedController(float DeltaTime);
  void UpdateAutoPilotControl(float DeltaTime);
  void UpdateEnergyPositionController(float DeltaTime);
  void Reset_All(void);
  bool Scheduler(Scheduler_Struct *SchedulerPointer, uint32_t RefreshTime);
  bool GetStateToEnableTurnCoordinator(void);
};
extern TecsClass TECS;
#endif