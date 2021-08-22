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

#ifndef CALIBRATION_H_
#define CALIBRATION_H_
#include <inttypes.h>
class AirSpeedCalibrationClass
{
public:
  void Initialization(void);
  bool Calibrate(void);
  void Scale_Update(void);

private:
  uint8_t Scale_Counter = 0;
  float Previous_Scale = 0;
};
extern AirSpeedCalibrationClass AIRSPEEDCALIBRATION;
#endif