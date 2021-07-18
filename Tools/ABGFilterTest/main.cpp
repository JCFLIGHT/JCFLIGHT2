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

/*
  AUTOR:QUICK-FLASH - EMUFLIGHT
  https://github.com/emuflight/EmuFlight/blob/master/src/main/common/filter.c
*/

//http://yadda.icm.edu.pl/yadda/element/bwmeta1.element.baztech-922ff6cb-e991-417f-93f0-77448f1ef4ec/c/A_Study_Jeong_1_2017.pdf

typedef struct
{
  float a, b, g, e;
  float aK, vK, xK, jK;
  float DeltaTime, DeltaTime2, DeltaTime3;
  float HalfLife, Boost;
} AlphaBetaGammaFilter_Struct;

void ABG_Initialization(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Alpha, int16_t BoostGain, int16_t HalfLife, float DeltaTime)
{
  const float Alpha2 = Alpha * 0.001f;
  const float xi = powf(-Alpha2 + 1.0f, 0.25);
  Filter_Pointer->xK = 0.0f;
  Filter_Pointer->vK = 0.0f;
  Filter_Pointer->aK = 0.0f;
  Filter_Pointer->jK = 0.0f;
  Filter_Pointer->a = Alpha2;
  Filter_Pointer->b = (1.0f / 6.0f) * powf(1.0f - xi, 2) * (11.0f + 14.0f * xi + 11 * xi * xi);
  Filter_Pointer->g = 2 * powf(1.0f - xi, 3) * (1 + xi);
  Filter_Pointer->e = (1.0f / 6.0f) * powf(1 - xi, 4);
  Filter_Pointer->DeltaTime = DeltaTime;
  Filter_Pointer->DeltaTime2 = DeltaTime * DeltaTime;
  Filter_Pointer->DeltaTime3 = DeltaTime * DeltaTime * DeltaTime;
  Filter_Pointer->Boost = (BoostGain * BoostGain / 1000000) * 0.003;
  Filter_Pointer->HalfLife = HalfLife != 0 ? powf(0.5f, DeltaTime / HalfLife / 100.0f) : 1.0f;
}

float AlphaBetaGammaApply(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Input)
{
  float rK; //ERRO RESIDUAL

  Filter_Pointer->xK *= Filter_Pointer->HalfLife;
  Filter_Pointer->vK *= Filter_Pointer->HalfLife;
  Filter_Pointer->aK *= Filter_Pointer->HalfLife;
  Filter_Pointer->jK *= Filter_Pointer->HalfLife;

  //ATUALZIA O ESTADO ESTIMADO DO SISTEMA
  Filter_Pointer->xK += Filter_Pointer->DeltaTime * Filter_Pointer->vK + (1.0f / 2.0f) * Filter_Pointer->DeltaTime2 * Filter_Pointer->aK + (1.0f / 6.0f) * Filter_Pointer->DeltaTime3 * Filter_Pointer->jK;
  //ATUALIZA A VELOCIDADE ESTIMADA
  Filter_Pointer->vK += Filter_Pointer->DeltaTime * Filter_Pointer->aK + 0.5f * Filter_Pointer->DeltaTime2 * Filter_Pointer->jK;
  Filter_Pointer->aK += Filter_Pointer->DeltaTime * Filter_Pointer->jK;
  //CALCULA O ERRO RESIDUAL
  rK = Input - Filter_Pointer->xK;
  //AUMENTA ARTIFICIALMENTE O ERRO PARA AUMENTAR A RESPOSTA DO FILTRO
  rK += (fabsf(rK) * rK * Filter_Pointer->Boost);
  //ATUALIZA A ESTIMATIVA DE ERRO RESIDUAL
  Filter_Pointer->xK += Filter_Pointer->a * rK;
  Filter_Pointer->vK += Filter_Pointer->b / Filter_Pointer->DeltaTime * rK;
  Filter_Pointer->aK += Filter_Pointer->g / (2.0f * Filter_Pointer->DeltaTime2) * rK;
  Filter_Pointer->jK += Filter_Pointer->e / (6.0f * Filter_Pointer->DeltaTime3) * rK;

  return Filter_Pointer->xK;
}

AlphaBetaGammaFilter_Struct ABGFilter_Test;

#define THIS_LOOP_RATE 1000 //1KHZ

const float AlphaDefault = 10; //VALOR PEQUENO
const int16_t BoostDefault = 275;
const int16_t HalfLifeDefault = 50;

void setup()
{
  Serial.begin(115200);
  ABG_Initialization(&ABGFilter_Test, AlphaDefault, BoostDefault, HalfLifeDefault, THIS_LOOP_RATE * 1e-6f);
}

void loop()
{
  int16_t GetAnalogRead = analogRead(0);
  float ABG_AnalogRead = AlphaBetaGammaApply(&ABGFilter_Test, GetAnalogRead);
  Serial.print(GetAnalogRead);
  Serial.print("   ");
  Serial.println(ABG_AnalogRead);
  delay(1); //1KHZ LOOP
}