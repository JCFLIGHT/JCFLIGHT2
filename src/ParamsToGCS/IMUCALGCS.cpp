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

#include "IMUCALGCS.h"
#include "Common/STRUCTS.h"
#include "Math/MATHSUPPORT.h"
#include "PerformanceCalibration/PERFORMACC.h"

int8_t GetAxisInclinedToCalibration(int16_t AccSample[3])
{
  if ((ABS(AccSample[ROLL]) / 1.5f) > ABS(AccSample[PITCH]) &&
      (ABS(AccSample[ROLL]) / 1.5f) > ABS(AccSample[YAW]))
  {
    //ROLL
    return (AccSample[ROLL] > 0) ? 2 : 3;
  }
  else if ((ABS(AccSample[PITCH]) / 1.5f) > ABS(AccSample[ROLL]) &&
           (ABS(AccSample[PITCH]) / 1.5f) > ABS(AccSample[YAW]))
  {
    //PITCH
    return (AccSample[PITCH] > 0) ? 4 : 5;
  }
  else if ((ABS(AccSample[YAW]) / 1.5f) > ABS(AccSample[ROLL]) &&
           (ABS(AccSample[YAW]) / 1.5f) > ABS(AccSample[PITCH]))
  {
    //YAW
    return (AccSample[YAW] > 0) ? 0 : 1;
  }
  else
  {
    return -1;
  }
}

uint8_t GetImageToGCS(void)
{
  static const uint8_t ImageBitMap[6] = {0, 1, 2, 3, 4, 5};
  uint8_t FlagCheck = 0;

  if (Calibration.Accelerometer.OffSet[ROLL] != 0 &&
      Calibration.Accelerometer.OffSet[PITCH] != 0 &&
      Calibration.Accelerometer.OffSet[YAW] != 0 &&
      Calibration.Accelerometer.Scale[ROLL] != 0 &&
      Calibration.Accelerometer.Scale[PITCH] != 0 &&
      Calibration.Accelerometer.Scale[YAW] != 0)
  {
    return 0x3F;
  }

  if (Calibration.Accelerometer.Flags.CalibratedPosition[0])
  {
    FlagCheck |= (1 << ImageBitMap[0]);
  }

  if (Calibration.Accelerometer.Flags.CalibratedPosition[1])
  {
    FlagCheck |= (1 << ImageBitMap[1]);
  }

  if (Calibration.Accelerometer.Flags.CalibratedPosition[2])
  {
    FlagCheck |= (1 << ImageBitMap[2]);
  }

  if (Calibration.Accelerometer.Flags.CalibratedPosition[3])
  {
    FlagCheck |= (1 << ImageBitMap[3]);
  }

  if (Calibration.Accelerometer.Flags.CalibratedPosition[4])
  {
    FlagCheck |= (1 << ImageBitMap[4]);
  }

  if (Calibration.Accelerometer.Flags.CalibratedPosition[5])
  {
    FlagCheck |= (1 << ImageBitMap[5]);
  }

  return FlagCheck;
}