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

#ifndef MATHSUPPORT_H_
#define MATHSUPPORT_H_
#include <inttypes.h>
#include "math.h"
#include "HardwareSerial.h"
#include "wiring_private.h"
#define MIN(a, b) \
  __extension__({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a, b) \
  __extension__({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define ABS(x) \
  __extension__({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })
float Fast_SquareRoot(float ValueInput);
float Fast_Sine(float X);
float Fast_Cosine(float X);
float Fast_Atan2(float Y, float X);
float Fast_AtanCosine(float X);
float Fast_Tangent(float InputValue);
#endif