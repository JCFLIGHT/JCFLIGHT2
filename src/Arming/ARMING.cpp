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

#include "ARMING.h"
#include "RadioControl/RCSTATES.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "BatteryMonitor/BATTERY.h"
#include "ProgMem/PROGMEM.h"
#include "Glitch/GLITCH.h"
#include "IOMCU/IOMCU.h"
#include "FailSafe/FAILSAFE.h"
#include "GPSNavigation/NAVIGATION.h"
#include "Common/STRUCTS.h"
#include "PerformanceCalibration/PERFORMGYRO.h"
#include "ParamsToGCS/IMUCALGCS.h"
#include "GPS/GPSSTATES.h"
#include "BitArray/BITARRAY.h"

PreArmClass PREARM;

#ifdef __AVR_ATmega2560__

const char Message_0[] FLASH_MEMORY_ATTRIBUTE = "Erro:Acelerometro ruim;";
const char Message_1[] FLASH_MEMORY_ATTRIBUTE = "Erro:Piloto Automatico ativo;";
const char Message_2[] FLASH_MEMORY_ATTRIBUTE = "Erro:GPS Glitch;";
const char Message_3[] FLASH_MEMORY_ATTRIBUTE = "Erro:Fail-Safe ativo;";
const char Message_4[] FLASH_MEMORY_ATTRIBUTE = "Erro:Giroscopio ruim;";
const char Message_5[] FLASH_MEMORY_ATTRIBUTE = "Erro:Controladora muito inclinada;";
const char Message_6[] FLASH_MEMORY_ATTRIBUTE = "Erro:O switch nao foi ativado para o modo safe;";
const char Message_7[] FLASH_MEMORY_ATTRIBUTE = "Erro:Bateria ruim;";
const char Message_8[] FLASH_MEMORY_ATTRIBUTE = "Nenhum erro,seguro para armar;";
const char Message_9[] FLASH_MEMORY_ATTRIBUTE = "Erro:Compass ruim;";
const char Message_10[] FLASH_MEMORY_ATTRIBUTE = "Erro:Barometro ruim;";

#elif defined __arm__ || defined ESP32

const char *const Message_0 = "Erro:Acelerometro ruim;";
const char *const Message_1 = "Erro:Piloto Automatico ativo;";
const char *const Message_2 = "Erro:GPS Glitch;";
const char *const Message_3 = "Erro:Fail-Safe ativo;";
const char *const Message_4 = "Erro:Giroscopio ruim;";
const char *const Message_5 = "Erro:Controladora muito inclinada;";
const char *const Message_6 = "Erro:O switch nao foi ativado para o modo safe;";
const char *const Message_7 = "Erro:Bateria ruim;";
const char *const Message_8 = "Nenhum erro,seguro para armar;";
const char *const Message_9 = "Erro:Compass ruim;";
const char *const Message_10 = "Erro:Barometro ruim;";

#endif

void PreArmClass::UpdateGCSErrorText(uint8_t GCSErrorType)
{
    if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
        return;
    }

    switch (GCSErrorType)
    {

    case IMU_ERROR:
        GCS.Send_String_To_GCS(Message_0);
        break;

    case AUTO_PILOT_MODE_ERROR:
        GCS.Send_String_To_GCS(Message_1);
        break;

    case GPS_ERROR:
        GCS.Send_String_To_GCS(Message_2);
        break;

    case FAIL_SAFE_ERROR:
        GCS.Send_String_To_GCS(Message_3);
        break;

    case GYRO_EEROR:
        GCS.Send_String_To_GCS(Message_4);
        break;

    case INCLINATION_ERROR:
        GCS.Send_String_To_GCS(Message_5);
        break;

    case BUTTON_ERROR:
        GCS.Send_String_To_GCS(Message_6);
        break;

    case BATTERY_ERROR:
        GCS.Send_String_To_GCS(Message_7);
        break;

    case COMPASS_ERROR:
        GCS.Send_String_To_GCS(Message_9);
        break;

    case BAROMETER_ERROR:
        GCS.Send_String_To_GCS(Message_10);
        break;

    case NONE_ERROR:
        GCS.Send_String_To_GCS(Message_8);
        break;
    }
}

uint8_t PreArmClass::Checking(void)
{
    if (GYROCALIBRATION.GetRunning()) //GYROSCOPIO EM CALIBRAÇÃO
    {
        return GYRO_EEROR;
    }

    if (GetImageToGCS() != 63) //IMU NÃO CALIBRADA
    {
        return IMU_ERROR;
    }

    if (SystemInFailSafe()) //MODO FAIL-SAFE ATIVO
    {
        return FAIL_SAFE_ERROR;
    }

    if (GPS_Resources.Navigation.AutoPilot.Control.Enabled) //MODOS DE VOO POR GPS ATIVO
    {
        return AUTO_PILOT_MODE_ERROR;
    }

    if (GetCheckInclinationForArm()) //INCLINAÇÃO DE 'N' GRAUS DETECTADA
    {
        return INCLINATION_ERROR;
    }

    if (!SAFETYBUTTON.GetSafeStateToOutput()) //SAFETY-BUTTON EMBARCADO,PORÉM NÃO ESTÁ NO MODE "SAFE"
    {
        return BUTTON_ERROR;
    }

    if (BATTERY.GetExhausted()) //BATERIA COM BAIXA TENSÃO
    {
        return BATTERY_ERROR;
    }

    if (!GLITCH.CheckGPS()) //CHECA O GPS
    {
        return GPS_ERROR;
    }

    if (!GLITCH.CheckCompass()) //CHECA O COMPASS
    {
        return COMPASS_ERROR;
    }

    if (!GLITCH.CheckBarometer()) //CHECA O BAROMETRO
    {
        return BAROMETER_ERROR;
    }

    //TUDO ESTÁ OK,O SISTEMA ESTÁ PRONTO PARA ARMAR
    return NONE_ERROR;
}

bool PreArmClass::CheckSafeState(void)
{
    const uint8_t CheckingResult = PREARM.Checking();

    if (CheckingResult == NONE_ERROR ||    //NENHUM DISPOSITVO ESTÁ RUIM
        CheckingResult == GPS_ERROR ||     //NOTIFIQUE QUE O GPS ESTÁ RUIM,MAS NÃO IMPEÇA DE ARMAR
        CheckingResult == COMPASS_ERROR || //NOTIFIQUE QUE O COMPASS ESTÁ RUIM,MAS NÃO IMPEÇA DE ARMAR
        CheckingResult == BAROMETER_ERROR) //NOTIFIQUE QUE O BAROMETRO ESTÁ RUIM,MAS NÃO IMPEÇA DE ARMAR
    {
        return true;
    }
    return false;
}