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

#include "CRASHCHECK.h"
#include "Parachute/PARACHUTE.h"
#include "FlightModes/AUXFLIGHT.h"
#include "PrecisionLand/PRECISIONLAND.h"
#include "CHECK2D.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Common/RCDEFINES.h"
#include "AHRS/AHRS.h"
#include "Barometer/BAROBACKEND.h"
#include "BitArray/BITARRAY.h"
#include "Param/PARAM.h"
#include "FastSerial/PRINTF.h"

#ifdef __AVR_ATmega2560__

#define THIS_LOOP_RATE 60                   //HZ
#define CRASH_CHECK_TIMER 2                 //TEMPO MAXIMO DE CRASH EM SEGUNDOS
#define ATTITUDE_CHECK_THRESH_ROLL_PITCH 30 //VALOR CONVERTIDO PARA RADIANOS E FATORADO POR 1000

#else

#define THIS_LOOP_RATE 100                                              //HZ
#define CRASH_CHECK_TIMER JCF_Param.CrashCheck_Time                     //TEMPO MAXIMO DE CRASH EM SEGUNDOS
#define ATTITUDE_CHECK_THRESH_ROLL_PITCH JCF_Param.CrashCheck_BankAngle //VALOR CONVERTIDO PARA RADIANOS E FATORADO POR 1000

#endif

//DEBUG
//#define PRINTLN_CRASHCHECK

void CrashCheck()
{

  static uint16_t Crash_Counter;        //NÚMERO DE ITERAÇÕES PARA VERIFICAR SE O VEICULO CAPOTOU
  static int32_t AltitudeBaroToCompare; //COMPARA OS VALORES DO BAROMETRO PARA SABER SE ESTAMOS CAINDO

#ifdef PRINTLN_CRASHCHECK

  if (GetLandSuccess())
  {
    LOG("!!!Solo Detectado!!!");
  }

#endif

#ifndef PRINTLN_CRASHCHECK

  //VERIFICA SE OS MOTORES ESTÃO DESARMADOS
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    Crash_Counter = 0;
    return;
  }

  //VERIFICA SE A CONTROLADORA ESTÁ NO SOLO
  if (GetLandSuccess())
  {
    Crash_Counter = 0;
    return;
  }

  //VERIFICA SE ESTÁ EM QUALQUER MODO DE VOO,MENOS NO MODO ACRO OU MANUAL
  //SE ESTIVER NO MODO ACRO,O DETECTOR DE CRASH NÃO IRÁ FUNCIONAR
  if (!IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE) || (GetAirPlaneEnabled() && IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)))
  {
    Crash_Counter = 0;
    return;
  }

#endif

  //VERIFICA SE HÁ CRASH NO CONTROLADOR DE ATTITUDE
  if (!CrashCheck2D(Attitude.EulerAngles.Roll, Attitude.EulerAngles.Pitch, ATTITUDE_CHECK_THRESH_ROLL_PITCH))
  {
    Crash_Counter = 0;
    return;
  }

#ifndef PRINTLN_CRASHCHECK

  if (Crash_Counter == 1) //OK,PROVAVELMENTE ESTAMOS CAINDO
  {
    AltitudeBaroToCompare = Barometer.Altitude.Actual;
  }
  else if (Barometer.Altitude.Actual >= AltitudeBaroToCompare)
  {
    Crash_Counter = 0;
    return;
  }

#endif

  Crash_Counter++; //CONTROLADOR DE ATTITUDE DETECTOU CRASH

  if (Crash_Counter >= (CRASH_CHECK_TIMER * THIS_LOOP_RATE))
  {

#ifdef PRINTLN_CRASHCHECK

    LOG("!!!Crash Detectado!!!");

#endif

    //DESARMA OS MOTORES
    if (PARACHUTE.GetSafeStateToDisarmMotors())
    {
      DISABLE_THIS_STATE(PRIMARY_ARM_DISARM);
    }

    //CHAMA O PARACHUTE SE ESTIVER EQUIPADO
    PARACHUTE.Auto_Do_Now(AUXFLIGHT.GetModeConfiguration[PARACHUTE_MODE] > NONE);
  }
}
