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

#ifndef INS_H_
#define INS_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern INS_Resources_Struct INS_Resources;
class InertialNavigationClass
{
public:
  void Initialization(void);
  void Update(void);
  bool WaitForSample(void);

private:
  void UpdateIMU(uint32_t ActualTimeInUs);
  void UpdateBarometer(uint32_t ActualTimeInUs);
  void UpdateGPS(void);
  void UpdatePredictXYZ(INS_Context_Struct *Context);
  bool CorrectXYStateWithGPS(INS_Context_Struct *Context);
  bool CorrectZStateWithBaroOrGPS(INS_Context_Struct *Context);
  void UpdateEstimationPredictXYZ(uint32_t ActualTimeInUs);
};
extern InertialNavigationClass INERTIALNAVIGATION;
#endif
