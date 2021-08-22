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

#pragma once

#include "Common/ENUM.h"
#include "Targets/PASCAL.h"
#include "Targets/EXTREME.h"
#include "Targets/CLASSIC.h"

#define INITIAL_ADDRESS_EEPROM_TO_CLEAR 0  //ENDEREÇO INICIAL PARA ERASE
#define FINAL_ADDRESS_EEPROM_TO_CLEAR 2000 //ENDEREÇO FINAL PARA ERASE
#define SIZE_OF_EEPROM 2000                //TAMANHO TOTAL DA EEPROM

#ifdef __AVR_ATmega2560__

#define ADC_VOLTAGE_SCALER 1.0f
#define ADC_VOLTAGE_ZERO 0.0f
#define ADC_VOLTAGE_OPERATION 5.0f
#define ADC_MAX_SAMPLES 1023.0f
#define TOTAL_MAX_CHANNELS 12
#define SBUS_MAX_CHANNELS 12
#define IBUS_MAX_CHANNELS 12
#define IBUS_MAX_SLOTS 12

//NÃO USA ALGUNS RECURSOS NA VERSÃO CLASSIC POR MOTIVO DE FALTA DE MEMORIA RAM E CICLO DE MAQUINA

#undef USE_IMU_FILTERS
#undef USE_NAZA_GPS
#undef USE_DERIVATIVE_BOOST_PID
#undef USE_AIRSPEED_AUTO_SCALE_CALIBRATION
#undef USE_WIND_ESTIMATOR
#undef USE_THROTTLE_COMPENSATION
#undef USE_GYRO_FILTER_RATE_TABLE
#undef USE_BARO_PRECISE_MATH
#undef USE_SBUS_EXTENDED
#undef USE_IBUS_EXTENDED
#undef USE_CLI
#undef USE_TUNNING_MODE

#else //STM32

#define ADC_VOLTAGE_SCALER 2.0f
#define ADC_VOLTAGE_ZERO 2.5f
#define ADC_VOLTAGE_OPERATION 3.3f
#define ADC_MAX_SAMPLES 4095.0f
#define TOTAL_MAX_CHANNELS 18
#define SBUS_MAX_CHANNELS 18
#define IBUS_MAX_CHANNELS 18
#define IBUS_MAX_SLOTS 14

#define USE_IMU_FILTERS
#define USE_NAZA_GPS
#define USE_DERIVATIVE_BOOST_PID
#define USE_AIRSPEED_AUTO_SCALE_CALIBRATION
#define USE_WIND_ESTIMATOR
#define USE_THROTTLE_COMPENSATION
#define USE_GYRO_FILTER_RATE_TABLE
#define USE_BARO_PRECISE_MATH
#define USE_SBUS_EXTENDED
#define USE_IBUS_EXTENDED
#define USE_CLI
#define USE_TUNNING_MODE

#endif