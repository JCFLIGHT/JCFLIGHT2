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

#include "BAROFRONTEND.h"
#include "BAROBACKEND.h"
#include "BAROREAD.h"
#include "Math/MATHSUPPORT.h"
#include "I2C/I2C.h"
#include "BitArray/BITARRAY.h"

//https://en.wikipedia.org/wiki/Equivalent_airspeed

float EAS2TAS;
float Last_EAS2TAS;

static bool SafeToApplyEAS2TAS(void)
{
    return IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && I2CResources.Found.Barometer;
}

//FATOR DE ESCALA PARA CONVERTER A VELOCIDADE EQUIVALENTE DO TUBO DE PITOT EM VELOCIDADE REAL
float Get_EAS2TAS(void)
{
    if (!SafeToApplyEAS2TAS()) //EVITA DE OCORRER OVERFLOW
    {
        return 1.0f;
    }

    float ActualBaroAltitude = (float)Barometer.Altitude.Actual;

    if ((ABS(ActualBaroAltitude - Last_EAS2TAS) < 100.0f) && (EAS2TAS != 0.0f))
    {
        return EAS2TAS;
    }

    float TempKelvin = ((float)Barometer.Calibration.GroundTemperature) + 273.15f - 0.0065f * ActualBaroAltitude;
    EAS2TAS = Fast_SquareRoot(1.225f / ((float)Barometer.Raw.Pressure / (287.26f * TempKelvin)));
    Last_EAS2TAS = ActualBaroAltitude;

    return EAS2TAS;
}