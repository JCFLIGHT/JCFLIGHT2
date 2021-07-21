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

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "VECTOR.h"

static inline Quaternion_Struct *QuaternionInit(Quaternion_Struct *Result)
{
    Result->q0 = 1.0f;
    Result->q1 = 0.0f;
    Result->q2 = 0.0f;
    Result->q3 = 0.0f;
    return Result;
}

static inline float QuaternionNormalizedSquared(const Quaternion_Struct *Quaternion)
{
    return SquareFloat(Quaternion->q0) + SquareFloat(Quaternion->q1) + SquareFloat(Quaternion->q2) + SquareFloat(Quaternion->q3);
}

static inline Quaternion_Struct *QuaternionNormalize(Quaternion_Struct *Result, const Quaternion_Struct *Quaternion)
{
    float CheckSquare = Fast_SquareRoot(QuaternionNormalizedSquared(Quaternion));
    if (CheckSquare < 1e-6f)
    {
        Result->q0 = 1;
        Result->q1 = 0;
        Result->q2 = 0;
        Result->q3 = 0;
    }
    else
    {
        Result->q0 = Quaternion->q0 / CheckSquare;
        Result->q1 = Quaternion->q1 / CheckSquare;
        Result->q2 = Quaternion->q2 / CheckSquare;
        Result->q3 = Quaternion->q3 / CheckSquare;
    }
    return Result;
}

static inline Quaternion_Struct *QuaternionMultiply(Quaternion_Struct *Result, const Quaternion_Struct *VectorA, const Quaternion_Struct *VectorB)
{
    Quaternion_Struct CalcedResult;
    CalcedResult.q0 = VectorA->q0 * VectorB->q0 - VectorA->q1 * VectorB->q1 - VectorA->q2 * VectorB->q2 - VectorA->q3 * VectorB->q3;
    CalcedResult.q1 = VectorA->q0 * VectorB->q1 + VectorA->q1 * VectorB->q0 + VectorA->q2 * VectorB->q3 - VectorA->q3 * VectorB->q2;
    CalcedResult.q2 = VectorA->q0 * VectorB->q2 - VectorA->q1 * VectorB->q3 + VectorA->q2 * VectorB->q0 + VectorA->q3 * VectorB->q1;
    CalcedResult.q3 = VectorA->q0 * VectorB->q3 + VectorA->q1 * VectorB->q2 - VectorA->q2 * VectorB->q1 + VectorA->q3 * VectorB->q0;
    *Result = CalcedResult;
    return Result;
}

static inline Quaternion_Struct *QuaternionConjugate(Quaternion_Struct *Result, const Quaternion_Struct *Quaternion)
{
    Result->q0 = Quaternion->q0;
    Result->q1 = -Quaternion->q1;
    Result->q2 = -Quaternion->q2;
    Result->q3 = -Quaternion->q3;
    return Result;
}

static inline Vector3x3_Struct *QuaternionRotateVector(Vector3x3_Struct *Result, const Vector3x3_Struct *Vector, const Quaternion_Struct *Reference)
{
    Quaternion_Struct QuaternionVector, ReferenceConjugate;
    QuaternionVector.q0 = 0;
    QuaternionVector.q1 = Vector->X;
    QuaternionVector.q2 = Vector->Y;
    QuaternionVector.q3 = Vector->Z;
    QuaternionConjugate(&ReferenceConjugate, Reference);
    QuaternionMultiply(&QuaternionVector, &ReferenceConjugate, &QuaternionVector);
    QuaternionMultiply(&QuaternionVector, &QuaternionVector, Reference);
    Result->X = QuaternionVector.q1;
    Result->Y = QuaternionVector.q2;
    Result->Z = QuaternionVector.q3;
    return Result;
}

static inline Vector3x3_Struct *QuaternionRotateVectorInverse(Vector3x3_Struct *Result, const Vector3x3_Struct *Vector, const Quaternion_Struct *Reference)
{
    Quaternion_Struct QuaternionVector, ReferenceConjugate;
    QuaternionVector.q0 = 0;
    QuaternionVector.q1 = Vector->X;
    QuaternionVector.q2 = Vector->Y;
    QuaternionVector.q3 = Vector->Z;
    QuaternionConjugate(&ReferenceConjugate, Reference);
    QuaternionMultiply(&QuaternionVector, Reference, &QuaternionVector);
    QuaternionMultiply(&QuaternionVector, &QuaternionVector, &ReferenceConjugate);
    Result->X = QuaternionVector.q1;
    Result->Y = QuaternionVector.q2;
    Result->Z = QuaternionVector.q3;
    return Result;
}

static inline Quaternion_Struct *QuaternionInitFromVector(Quaternion_Struct *Result, const Vector3x3_Struct *Vector)
{
    Result->q0 = 0.0f;
    Result->q1 = Vector->X;
    Result->q2 = Vector->Y;
    Result->q3 = Vector->Z;
    return Result;
}

static inline Quaternion_Struct *QuaternionScale(Quaternion_Struct *Result, const Quaternion_Struct *VectorA, const float VectorB)
{
    Quaternion_Struct CalcedResult;
    CalcedResult.q0 = VectorA->q0 * VectorB;
    CalcedResult.q1 = VectorA->q1 * VectorB;
    CalcedResult.q2 = VectorA->q2 * VectorB;
    CalcedResult.q3 = VectorA->q3 * VectorB;
    *Result = CalcedResult;
    return Result;
}
#endif