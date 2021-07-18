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

#include "GAUSSNEWTON.h"
#include "Math/MATHSUPPORT.h"

//http://en.wikipedia.org/wiki/Jacobian_matrix

//https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm

//https://en.wikipedia.org/wiki/Gaussian_elimination

void ClearGaussNewtonMatrices(Jacobian_Struct *JacobianPointer)
{
  for (int16_t FirstIndex = 0; FirstIndex < 4; FirstIndex++)
  {
    for (int16_t SecondIndex = 0; SecondIndex < 4; SecondIndex++)
    {
      JacobianPointer->Matrix_JtJ[FirstIndex][SecondIndex] = 0;
    }

    JacobianPointer->Matrix_JtR[FirstIndex] = 0;
  }
}

void GaussNewtonPushSampleForOffSetCalculation(Jacobian_Struct *JacobianPointer, int32_t SensorSample[3])
{
  JacobianPointer->Matrix_JtJ[0][0] += (float)SensorSample[0] * SensorSample[0];
  JacobianPointer->Matrix_JtJ[0][1] += (float)SensorSample[0] * SensorSample[1];
  JacobianPointer->Matrix_JtJ[0][2] += (float)SensorSample[0] * SensorSample[2];
  JacobianPointer->Matrix_JtJ[0][3] += (float)SensorSample[0];

  JacobianPointer->Matrix_JtJ[1][0] += (float)SensorSample[1] * SensorSample[0];
  JacobianPointer->Matrix_JtJ[1][1] += (float)SensorSample[1] * SensorSample[1];
  JacobianPointer->Matrix_JtJ[1][2] += (float)SensorSample[1] * SensorSample[2];
  JacobianPointer->Matrix_JtJ[1][3] += (float)SensorSample[1];

  JacobianPointer->Matrix_JtJ[2][0] += (float)SensorSample[2] * SensorSample[0];
  JacobianPointer->Matrix_JtJ[2][1] += (float)SensorSample[2] * SensorSample[1];
  JacobianPointer->Matrix_JtJ[2][2] += (float)SensorSample[2] * SensorSample[2];
  JacobianPointer->Matrix_JtJ[2][3] += (float)SensorSample[2];

  JacobianPointer->Matrix_JtJ[3][0] += (float)SensorSample[0];
  JacobianPointer->Matrix_JtJ[3][1] += (float)SensorSample[1];
  JacobianPointer->Matrix_JtJ[3][2] += (float)SensorSample[2];
  JacobianPointer->Matrix_JtJ[3][3] += 1;

  float SquareObservation = ((float)SensorSample[0] * SensorSample[0]) + ((float)SensorSample[1] * SensorSample[1]) + ((float)SensorSample[2] * SensorSample[2]);
  JacobianPointer->Matrix_JtR[0] += SensorSample[0] * SquareObservation;
  JacobianPointer->Matrix_JtR[1] += SensorSample[1] * SquareObservation;
  JacobianPointer->Matrix_JtR[2] += SensorSample[2] * SquareObservation;
  JacobianPointer->Matrix_JtR[3] += SquareObservation;
}

void GaussNewtonPushSampleForScaleCalculation(Jacobian_Struct *JacobianPointer, int16_t AxisIndex, int32_t SensorSample[3], int16_t Target)
{
  for (int16_t IndexCount = 0; IndexCount < 3; IndexCount++)
  {
    float ScaledSample = (float)SensorSample[IndexCount] / (float)Target;
    JacobianPointer->Matrix_JtJ[AxisIndex][IndexCount] += ScaledSample * ScaledSample;
    JacobianPointer->Matrix_JtJ[3][IndexCount] += ScaledSample * ScaledSample;
  }

  JacobianPointer->Matrix_JtJ[AxisIndex][3] += 1;
  JacobianPointer->Matrix_JtR[AxisIndex] += 1;
  JacobianPointer->Matrix_JtR[3] += 1;
}

static void GaussNewton_Left_Right(float Math[4][4])
{
  int16_t N = 4; //NÚMERO DE AMOSTRAS
  int16_t FirstIndex;
  int16_t SecondIndex;
  int16_t ThirdIndex;

  for (FirstIndex = 0; FirstIndex < 4; FirstIndex++)
  {
    //CALCULA O LADO DIREITO DA EQUAÇÃO
    for (SecondIndex = FirstIndex; SecondIndex < 4; SecondIndex++)
    {
      for (ThirdIndex = 0; ThirdIndex < FirstIndex; ThirdIndex++)
      {
        Math[FirstIndex][SecondIndex] -= Math[FirstIndex][ThirdIndex] * Math[ThirdIndex][SecondIndex];
      }
    }
    //CALCULA O LADO ESQUERDO DA EQUAÇÃO
    for (SecondIndex = FirstIndex + 1; SecondIndex < N; SecondIndex++)
    {
      for (ThirdIndex = 0; ThirdIndex < FirstIndex; ThirdIndex++)
      {
        Math[SecondIndex][FirstIndex] -= Math[SecondIndex][ThirdIndex] * Math[ThirdIndex][FirstIndex];
      }
      Math[SecondIndex][FirstIndex] /= Math[FirstIndex][FirstIndex];
    }
  }
}

static void GaussNewton_ForwardSubstitution(float Left_Right[4][4], float JacobObservationY[4], float JacobObservationB[4])
{
  int16_t RowCount;
  int16_t ColumnCount;
  for (RowCount = 0; RowCount < 4; ++RowCount)
  {
    JacobObservationY[RowCount] = JacobObservationB[RowCount];
    for (ColumnCount = 0; ColumnCount < RowCount; ++ColumnCount)
    {
      JacobObservationY[RowCount] -= Left_Right[RowCount][ColumnCount] * JacobObservationY[ColumnCount];
    }
  }
}

static void GaussNewton_BackwardSubstitution(float Left_Right[4][4], float JacobObservationX[4], float JacobObservationY[4])
{
  int16_t RowCount;
  int16_t ColumnCount;
  for (RowCount = 3; RowCount >= 0; --RowCount)
  {
    JacobObservationX[RowCount] = JacobObservationY[RowCount];
    for (ColumnCount = RowCount + 1; ColumnCount < 4; ++ColumnCount)
    {
      JacobObservationX[RowCount] -= Left_Right[RowCount][ColumnCount] * JacobObservationX[ColumnCount];
    }
    JacobObservationX[RowCount] /= Left_Right[RowCount][RowCount];
  }
}

static void GaussNewton_SolveLGS(float JacobObservationA[4][4], float JacobObservationX[4], float JacobObservationB[4])
{
  float JacobObservationY[4];

  GaussNewton_Left_Right(JacobObservationA);

  for (int16_t IndexCount = 0; IndexCount < 4; ++IndexCount)
  {
    JacobObservationY[IndexCount] = 0;
  }

  GaussNewton_ForwardSubstitution(JacobObservationA, JacobObservationY, JacobObservationB);
  GaussNewton_BackwardSubstitution(JacobObservationA, JacobObservationX, JacobObservationY);
}

static bool GaussNewtonValidateResult(const float Result[3])
{
  for (uint8_t IndexCount = 0; IndexCount < 3; IndexCount++)
  {
    if (isnan(Result[IndexCount]) && isinf(Result[IndexCount]))
    {
      return false;
    }
  }
  return true;
}

bool GaussNewtonSolveForOffSet(Jacobian_Struct *JacobianPointer, float Result[3])
{
  float Beta[4];

  GaussNewton_SolveLGS(JacobianPointer->Matrix_JtJ, Beta, JacobianPointer->Matrix_JtR);

  for (int16_t IndexCount = 0; IndexCount < 3; IndexCount++)
  {
    Result[IndexCount] = Beta[IndexCount] / 2;
  }

  return GaussNewtonValidateResult(Result);
}

bool GaussNewtonSolveForScale(Jacobian_Struct *JacobianPointer, float Result[3])
{
  float Beta[4];

  GaussNewton_SolveLGS(JacobianPointer->Matrix_JtJ, Beta, JacobianPointer->Matrix_JtR);

  for (int16_t IndexCount = 0; IndexCount < 3; IndexCount++)
  {
    Result[IndexCount] = Fast_SquareRoot(Beta[IndexCount]);
  }

  return GaussNewtonValidateResult(Result);
}