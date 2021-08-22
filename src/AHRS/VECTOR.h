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

#ifndef VECTOR_H_
#define VECTOR_H_

#include <stdint.h>
#include "Common/STRUCTS.h"
#include "Math/MATHSUPPORT.h"

static inline float VectorNormSquared(const Vector3x3_Struct *Vector)
{
    return SquareFloat(Vector->X) + SquareFloat(Vector->Y) + SquareFloat(Vector->Z);
}

static inline Vector3x3_Struct *VectorNormalize(Vector3x3_Struct *Result, const Vector3x3_Struct *Vector)
{
    float Length = Fast_SquareRoot(VectorNormSquared(Vector));
    if (Length != 0)
    {
        Result->X = Vector->X / Length;
        Result->Y = Vector->Y / Length;
        Result->Z = Vector->Z / Length;
    }
    else
    {
        Result->X = 0;
        Result->Y = 0;
        Result->Z = 0;
    }
    return Result;
}

static inline Vector3x3_Struct *VectorCrossProduct(Vector3x3_Struct *Result, const Vector3x3_Struct *VectorA, const Vector3x3_Struct *VectorB)
{
    Vector3x3_Struct CalcedVector;
    CalcedVector.X = VectorA->Y * VectorB->Z - VectorA->Z * VectorB->Y;
    CalcedVector.Y = VectorA->Z * VectorB->X - VectorA->X * VectorB->Z;
    CalcedVector.Z = VectorA->X * VectorB->Y - VectorA->Y * VectorB->X;
    *Result = CalcedVector;
    return Result;
}

static inline Vector3x3_Struct *VectorAdd(Vector3x3_Struct *Result, const Vector3x3_Struct *VectorA, const Vector3x3_Struct *VectorB)
{
    Vector3x3_Struct CalcedVector;
    CalcedVector.X = VectorA->X + VectorB->X;
    CalcedVector.Y = VectorA->Y + VectorB->Y;
    CalcedVector.Z = VectorA->Z + VectorB->Z;
    *Result = CalcedVector;
    return Result;
}

static inline Vector3x3_Struct *VectorScale(Vector3x3_Struct *Result, const Vector3x3_Struct *VectorA, const float VectorB)
{
    Vector3x3_Struct CalcedVector;
    CalcedVector.X = VectorA->X * VectorB;
    CalcedVector.Y = VectorA->Y * VectorB;
    CalcedVector.Z = VectorA->Z * VectorB;
    *Result = CalcedVector;
    return Result;
}

static inline void VectorZero(Vector3x3_Struct *VectorPointer)
{
    VectorPointer->X = 0.0f;
    VectorPointer->Y = 0.0f;
    VectorPointer->Z = 0.0f;
}

#endif