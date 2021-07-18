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

#include "MOTORS.h"
#include "EscCalibration/CALIBESC.h"
#include "Common/STRUCTS.h"
#include "AirPlane/AIRPLANE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "MIXING.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "WatchDog/REBOOT.h"
#include "RadioControl/RCSTATES.h"
#ifdef ESP32
#include "HAL_ESP32/ESP32PWM.h"
#endif
#include "THRCLIPPING.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Math/MATHSUPPORT.h"
#include "THRCOMPENSATION.h"
#include "PID/RCPID.h"
#include "PID/PIDXYZ.h"
#include "MIXTABLE.h"
#include "ProgMem/PROGMEM.h"
#include "Build/BOARDDEFS.h"
#include "BitArray/BITARRAY.h"
#include "Param/PARAM.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//DEBUG
//#define PRINTLN_MOTORS

int16_t MotorControl[8];

void ConfigureRegisters(bool Run_Calibrate_ESC)
{
#ifdef __AVR_ATmega2560__

  //CONFIGURAÇÃO DAS PORTAS DE SAÍDA
  DDRE |= (1 << DDD4); //DEFINE A PORTA DIGITAL 2 COMO SAIDA
  DDRE |= (1 << DDD5); //DEFINE A PORTA DIGITAL 3 COMO SAIDA
  DDRE |= (1 << DDD3); //DEFINE A PORTA DIGITAL 5 COMO SAIDA
  DDRH |= (1 << DDD3); //DEFINE A PORTA DIGITAL 6 COMO SAIDA
  DDRH |= (1 << DDD4); //DEFINE A PORTA DIGITAL 7 COMO SAIDA
  DDRH |= (1 << DDD5); //DEFINE A PORTA DIGITAL 8 COMO SAIDA
  DDRL |= (1 << DDD3); //DEFINE A PORTA DIGITAL 46 COMO SAIDA
  DDRL |= (1 << DDD4); //DEFINE A PORTA DIGITAL 45 COMO SAIDA

  //CONFIGURA O TIMER1 CANAIS A E B E TAMBÉM O TIMER2 CANAL A PARA O CONTROLE DO LED RGB
  TCCR1A |= _BV(COM1A1); //CONECTA O PINO 12 AO TIMER1 CANAL A
  TCCR1A |= _BV(COM1B1); //CONECTA O PINO 11 AO TIMER1 CANAL B
  TCCR2A |= _BV(COM2A1); //CONECTA O PINO 10 AO TIMER2 CANAL A

  //CONFIGURA O TIMER2 CANAL B PARA O CONTROLE DO BUZZER
  TCCR2A |= _BV(COM2B1); //CONECTA O PINO 9 AO TIMER2 CANAL B

  //CONFIGURA O TIMER 3 PARA OPERAR INICIALMENTE EM 490Hz
  TCCR3A |= (1 << WGM31);
  TCCR3A &= ~(1 << WGM30);
  TCCR3B |= (1 << WGM33);
  TCCR3B &= ~(1 << CS31);
  if (GetAirPlaneEnabled())
  {
    ICR3 |= 40000; //50Hz
  }
  else
  {
    ICR3 |= 16383; //490Hz
  }

  //CONFIGURA O TIMER 3
  TCCR3A |= _BV(COM3A1); //CONECTA O PINO 5 AO TIMER1 CANAL A
  TCCR3A |= _BV(COM3B1); //CONECTA O PINO 2 AO TIMER3 CANAL B
  TCCR3A |= _BV(COM3C1); //CONECTA O PINO 3 AO TIMER1 CANAL C

  //CONFIGURA O TIMER 4 PARA OPERAR INICIALMENTE EM 490Hz
  TCCR4A |= (1 << WGM41);
  TCCR4A &= ~(1 << WGM40);
  TCCR4B |= (1 << WGM43);
  TCCR4B &= ~(1 << CS41);
  if (GetAirPlaneEnabled())
  {
    ICR4 |= 40000; //50Hz
  }
  else
  {
    ICR4 |= 16383; //490Hz
  }

  //CONFIGURA O TIMER 4
  TCCR4A |= _BV(COM4A1); //CONECTA O PINO 6 AO TIMER4 CANAL A
  TCCR4A |= _BV(COM4B1); //CONECTA O PINO 7 AO TIMER4 CANAL B
  TCCR4A |= _BV(COM4C1); //CONECTA O PINO 8 AO TIMER4 CANAL C

  //CONFIGURA O TIMER 5 PARA OPERAR EM 50Hz
  TCCR5A |= (1 << WGM51);
  TCCR5A &= ~(1 << WGM50);
  TCCR5B |= (1 << WGM53);
  TCCR5B &= ~(1 << CS41);
  ICR5 |= 40000; //50Hz

  //CONFIGURA O TIMER 5
  TCCR5A |= _BV(COM4A1); //CONECTA O PINO 46 AO TIMER4 CANAL A
  TCCR5A |= _BV(COM4B1); //CONECTA O PINO 45 AO TIMER4 CANAL B
  //TCCR5A |= _BV(COM4C1); //CONECTA O PINO 44 AO TIMER4 CANAL C

#elif defined ESP32

  //6 MOTORES
  pinMode(GPIO_NUM_25, OUTPUT);
  pinMode(GPIO_NUM_26, OUTPUT);
  pinMode(GPIO_NUM_27, OUTPUT);
  pinMode(GPIO_NUM_14, OUTPUT);
  pinMode(GPIO_NUM_12, OUTPUT);
  pinMode(GPIO_NUM_13, OUTPUT);
  //GIMBAL
  pinMode(GPIO_NUM_23, OUTPUT);
  AnalogWriteSetSettings(GPIO_NUM_23, 50, 12);
  //PARACHUTE
  pinMode(GPIO_NUM_19, OUTPUT);
  AnalogWriteSetSettings(GPIO_NUM_19, 50, 12);
  //BUZZER
  //A DECLARAÇÃO DE SAÍDA É FEITA NA EXTENSÃO DO BUZZER
  AnalogWriteSetSettings(GPIO_NUM_18, 490, 12);

  if (GetAirPlaneEnabled())
  {
    AnalogWriteSetSettings(GPIO_NUM_25, 490, 12);
    AnalogWriteSetSettings(GPIO_NUM_26, 50, 12);
    AnalogWriteSetSettings(GPIO_NUM_27, 50, 12);
    AnalogWriteSetSettings(GPIO_NUM_14, 50, 12);
    AnalogWriteSetSettings(GPIO_NUM_12, 50, 12);
    AnalogWriteSetSettings(GPIO_NUM_13, 50, 12);
  }
  else
  {
    AnalogWriteSetSettings(GPIO_NUM_25, 490, 12);
    AnalogWriteSetSettings(GPIO_NUM_26, 490, 12);
    AnalogWriteSetSettings(GPIO_NUM_27, 490, 12);
    AnalogWriteSetSettings(GPIO_NUM_14, 490, 12);
    AnalogWriteSetSettings(GPIO_NUM_12, 490, 12);
    AnalogWriteSetSettings(GPIO_NUM_13, 490, 12);
  }

#elif defined __arm__

#endif

  if (!Run_Calibrate_ESC && !SAFETYBUTTON.SafeButtonEnabled())
  {
    PulseInAllMotors(1000);
    SCHEDULERTIME.Sleep(300);
  }
}

void ApplyMixingForMotorsAndServos(float DeltaTime)
{
  if (!SAFETYBUTTON.GetSafeStateToOutput() || WATCHDOG.InShutDown)
  {
    return;
  }

  const uint8_t PlatformTypeEnabled = GetActualPlatformType();

  const uint8_t NumberOfMotors = ProgMemReadByte(&Motors_Count[PlatformTypeEnabled].FrameMotorsCount);

  int16_t MixerThrottleController = RC_Resources.Attitude.Controller[THROTTLE];

#ifdef __AVR_ATmega2560__

  MixerThrottleController = (MixerThrottleController - RC_Resources.Attitude.ThrottleMin) + RC_Resources.Attitude.ThrottleMin;

#else

  JCF_Param.Throttle_Mix_Gain = 1.0f; //MOVER PARA AS CONFIGURAÇÕES INTERMEDIARIAS - RANGE:0.0f A 1.0f

  MixerThrottleController = ((MixerThrottleController - RC_Resources.Attitude.ThrottleMin) * JCF_Param.Throttle_Mix_Gain) + RC_Resources.Attitude.ThrottleMin;

#endif

#ifdef USE_THROTTLE_COMPENSATION

  if (STORAGEMANAGER.Read_8Bits(MOT_COMP_STATE_ADDR) > 0)
  {
    MixerThrottleController = MIN(RC_Resources.Attitude.ThrottleMin + (MixerThrottleController - RC_Resources.Attitude.ThrottleMin) * CalculateThrottleCompensationFactor(DeltaTime), RC_Resources.Attitude.ThrottleMax);
  }

#endif

  //ATUALIZA O MIX DO PID
  Mixing_Update(PlatformTypeEnabled);

  //ATUALIZA O THROTTLE CLIPPING
  Throttle_Clipping_Update(NumberOfMotors, MixerThrottleController);

  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    if (GetMultirotorEnabled())
    {
      for (uint8_t MotorsCount = 0; MotorsCount < NumberOfMotors; MotorsCount++)
      {
        MotorControl[MotorsCount] = Constrain_16Bits(MotorControl[MotorsCount], RC_Resources.Attitude.ThrottleMin, RC_Resources.Attitude.ThrottleMax);
        if (GetActualThrottleStatus(THROTTLE_LOW))
        {
          MotorControl[MotorsCount] = RC_Resources.Attitude.ThrottleMin;
        }
      }
    }
  }
  else
  {
    if (GetMultirotorEnabled())
    {
      for (uint8_t MotorsCount = 0; MotorsCount < NumberOfMotors; MotorsCount++)
      {
        MotorControl[MotorsCount] = 1000;
      }
    }
    else if (GetAirPlaneEnabled())
    {
      MotorControl[MOTOR1] = 1000;
    }
  }

#ifdef PRINTLN_MOTORS

  DEBUG("Motor1:%d Motor2:%d Motor3:%d Motor4:%d MixerIsOutputSaturated:%d",
        MotorControl[MOTOR1],
        MotorControl[MOTOR2],
        MotorControl[MOTOR3],
        MotorControl[MOTOR4],
        MixerIsOutputSaturated());

#endif
}

void PulseInAllMotors(int16_t Pulse)
{
  MotorControl[MOTOR4] = Pulse;
  MotorControl[MOTOR3] = Pulse;
  MotorControl[MOTOR2] = Pulse;
  MotorControl[MOTOR1] = Pulse;
  MotorControl[MOTOR6] = Pulse;
  MotorControl[MOTOR5] = Pulse;
  ApplyPWMControlForMotorsAndServos();
}

void ShutDownAllMotorsAndServos()
{
  WATCHDOG.InShutDown = true;

#ifdef __AVR_ATmega2560__

  OCR3A = 0;
  OCR3B = 0;
  OCR3C = 0;
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;

#elif defined ESP32

  AnalogWriteApplyPulse(GPIO_NUM_25, 0);
  AnalogWriteApplyPulse(GPIO_NUM_26, 0);
  AnalogWriteApplyPulse(GPIO_NUM_27, 0);
  AnalogWriteApplyPulse(GPIO_NUM_14, 0);
  AnalogWriteApplyPulse(GPIO_NUM_12, 0);
  AnalogWriteApplyPulse(GPIO_NUM_13, 0);

#elif defined __arm__

#endif
}

void ApplyPWMControlForMotorsAndServos()
{
  if (WATCHDOG.InShutDown)
  {
    return;
  }

#ifdef __AVR_ATmega2560__

  OCR3B = MotorControl[MOTOR1] << 3; //PINO DIGITAL 2 (MOTOR 1 NO FRAME)
  OCR3C = MotorControl[MOTOR2] << 3; //PINO DIGITAL 3 (MOTOR 2 NO FRAME)
  OCR3A = MotorControl[MOTOR3] << 3; //PINO DIGITAL 5 (MOTOR 3 NO FRAME)
  OCR4A = MotorControl[MOTOR4] << 3; //PINO DIGITAL 6 (MOTOR 4 NO FRAME)
  OCR4B = MotorControl[MOTOR5] << 3; //PINO DIGITAL 7 (MOTOR 5 NO FRAME)
  OCR4C = MotorControl[MOTOR6] << 3; //PINO DIGITAL 8 (MOTOR 6 NO FRAME)

  OCR5A = MotorControl[GIMBAL] << 3;         //PINO DIGITAL 46
  OCR5B = MotorControl[PARACHUTESERVO] << 3; //PINO DIGITAL 45

#elif defined ESP32

  AnalogWriteApplyPulse(GPIO_NUM_25, MotorControl[MOTOR1]);
  AnalogWriteApplyPulse(GPIO_NUM_26, MotorControl[MOTOR2]);
  AnalogWriteApplyPulse(GPIO_NUM_27, MotorControl[MOTOR3]);
  AnalogWriteApplyPulse(GPIO_NUM_14, MotorControl[MOTOR4]);
  AnalogWriteApplyPulse(GPIO_NUM_12, MotorControl[MOTOR5]);
  AnalogWriteApplyPulse(GPIO_NUM_13, MotorControl[MOTOR6]);
  AnalogWriteApplyPulse(GPIO_NUM_23, MotorControl[GIMBAL]);
  AnalogWriteApplyPulse(GPIO_NUM_19, MotorControl[PARACHUTESERVO]);

#elif defined __arm__

#endif
}
