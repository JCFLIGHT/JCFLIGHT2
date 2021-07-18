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

#include <inttypes.h>
#include "math.h"
#include "HardwareSerial.h"
#include "wiring_private.h"
#include "BLACKBOX.h"

/*
  MOSI - PINO 51
  MISO - PINO 50
  SCLK - PINO 52
  CS   - PINO 53
*/

BlackBox BlackBoxWrite(52, 51, 50, 53, "LOG_", " - "); //SCLK,MOSI,MISO,CHIPSELECT,NOME,SEPARADOR

bool BlackBoxEnabled = true; //O VALOR DESSA BOOL É DATA PELO VALOR GUARDADO NA EEPROM DA JCFLIGHT

void SetDateTime(uint16_t *Date, uint16_t *Time)
{
  //DATA DO REGISTRO
  *Date = FAT_DATE(1, 12, 2020);

  //HORARIO DO REGISTRO
  *Time = FAT_TIME(23, 13, 10);
}

void setup()
{
  Serial.begin(115200);
  if (BlackBoxEnabled)
  {
    BlackBoxWrite.Init();
  }
}

void loop()
{
  if (BlackBoxEnabled)
  {

    SdFile::dateTimeCallback(SetDateTime);

    BlackBoxWrite.IMU(1, 1, 180);

    BlackBoxWrite.RC(1501, 1548, 1456, 1200);

    BlackBoxWrite.GPS(-123456789, 123456789, 10);
  }
  delay(20); //50Hz LOOP
}