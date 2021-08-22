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

#ifndef AHRS_H_
#define AHRS_H_
#include "Build/LIBDEPENDENCIES.h"
#include "VECTOR.h"
#include "Common/STRUCTS.h"
extern Attitude_Struct Attitude;
extern Vector3x3_Struct BodyFrameAcceleration;
extern Vector3x3_Struct BodyFrameRotation;
extern Quaternion_Struct Orientation;
extern Matrix3x3_Struct Rotation;
class AHRSClass
{
public:
  void Initialization(void);
  void Update(float DeltaTime);
  float CosineTiltAngle(void);
  bool CheckAnglesInclination(int16_t Angle);
  void TransformVectorEarthFrameToBodyFrame(Vector3x3_Struct *VectorPointer);
  void TransformVectorBodyFrameToEarthFrame(Vector3x3_Struct *VectorPointer);
  bool Get_Cosine_Z_Overflowed(void);
  float GetSineRoll(void);
  float GetCosineRoll(void);
  float GetSinePitch(void);
  float GetCosinePitch(void);
  float GetSineYaw(void);
  float GetCosineYaw(void);
};
extern AHRSClass AHRS;
#endif
