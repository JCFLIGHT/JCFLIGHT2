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

#include <inttypes.h>
#include "math.h"
#include "HardwareSerial.h"
#include "wiring_private.h"

#define SCHEDULER_SET_FREQUENCY(Frequecy, Unidad) (1000000 / (Frequecy))
#define ACTIVE_THIS_TASK true
#define DESACTIVE_THIS_TASK false

enum TaskPriority_Enum
{
  TASK_PRIORITY_LOW = 1,
  TASK_PRIORITY_MEDIUM = 3,
  TASK_PRIORITY_MEDIUM_HIGH = 4,
  TASK_PRIORITY_HIGH = 5,
  TASK_PRIORITY_REALTIME = 6
};

typedef struct
{
  const char *TaskName;
  void (*TaskFunction)();
  int32_t DesiredPeriod;
  const uint8_t StaticPriority;
  uint16_t DynamicPriority;
  uint16_t TaskAgeCycles;
  uint32_t LastExecuted;
  int32_t TaskLatestDeltaTime;
} Task_Resources_Struct;
