#ifndef BIQUADFILTER_h
#define BIQUADFILTER_h
#include <inttypes.h>
#include "math.h"
#include "HardwareSerial.h"
#include "wiring_private.h"
enum {
  LPF,
  NOTCH
};
typedef struct
{
  float Beta0, Beta1, Beta2, Alpha1, Alpha2;
  float SampleX1, SampleX2, SampleY1, SampleY2;
} BiquadFilter_Struct;
class BiQuadFilter
{
  public:
    void Settings(BiquadFilter_Struct *Filter, int16_t FilterFreq, int16_t CutOffFreq, int16_t SampleInterval, uint8_t FilterType);
    float ApplyAndGet(BiquadFilter_Struct *Filter, float DeviceToFilter);
};
extern BiQuadFilter BIQUADFILTER;
#endif
