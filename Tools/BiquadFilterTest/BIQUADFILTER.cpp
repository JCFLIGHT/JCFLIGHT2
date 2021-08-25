#include "BIQUAD.h"

BiQuadFilter BIQUADFILTER;

float Fast_Sine(float X)
{
  int32_t X32BITS = X;
  if (X32BITS < -32 || X32BITS > 32)
  {
    return 0.0f;
  }
  while (X > 3.14159265358979323846f)
  {
    X -= (2.0f * 3.14159265358979323846f);
  }
  while (X < -3.14159265358979323846f)
  {
    X += (2.0f * 3.14159265358979323846f);
  }
  if (X > (0.5f * 3.14159265358979323846f))
  {
    X = (0.5f * 3.14159265358979323846f) - (X - (0.5f * 3.14159265358979323846f));
  }
  else if (X < -(0.5f * 3.14159265358979323846f))
  {
    X = -(0.5f * 3.14159265358979323846f) - ((0.5f * 3.14159265358979323846f) + X);
  }
  float XFLOAT = X * X;
  return X + X * XFLOAT * (-1.666665710e-1f + XFLOAT * (8.333017292e-3f + XFLOAT * (-1.980661520e-4f + XFLOAT * 2.600054768e-6f)));
}

float Fast_Cosine(float X)
{
  return Fast_Sine(X + (0.5f * 3.14159265358979323846f));
}

float FilterGetLPF_Quality()
{
  return 1.0f / sqrtf(2.0f);
}

float FilterGetNotch_Quality(float CenterFrequencyHz, float CutOffFrequencyHz)
{
  return CenterFrequencyHz * CutOffFrequencyHz / (CenterFrequencyHz * CenterFrequencyHz - CutOffFrequencyHz * CutOffFrequencyHz);
}

void BiQuadFilterSetupPassthrough(BiquadFilter_Struct *Filter)
{
  Filter->Beta0 = 1.0f;
  Filter->Beta1 = 0.0f;
  Filter->Beta2 = 0.0f;
  Filter->Alpha1 = 0.0f;
  Filter->Alpha2 = 0.0f;
}

void BiQuadFilter::Settings(BiquadFilter_Struct *Filter, int16_t FilterFreq, int16_t CutOffFreq, int16_t SampleInterval, uint8_t FilterType)
{

  float Q_Quality = 0;

  if (FilterType == LPF)
  {
    Q_Quality = FilterGetLPF_Quality();
  }
  else //NOTCH
  {
    Q_Quality = FilterGetNotch_Quality(FilterFreq, CutOffFreq);
  }

  if (FilterFreq < (1000000 / SampleInterval / 2))
  {

    float Beta0, Beta1, Beta2;
    const float SampleRate = 1.0f / ((float)SampleInterval * 0.000001f);
    const float Omega = 6.283185482f * ((float)FilterFreq) / SampleRate;
    const float CalcedSine = Fast_Sine(Omega);
    const float CalcedCosine = Fast_Cosine(Omega);
    const float Alpha = CalcedSine / (2 * Q_Quality);

    switch (FilterType)
    {

      case LPF:
        Beta0 = (1 - CalcedCosine) / 2;
        Beta1 = 1 - CalcedCosine;
        Beta2 = (1 - CalcedCosine) / 2;
        break;

      case NOTCH:
        Beta0 = 1;
        Beta1 = -2 * CalcedCosine;
        Beta2 = 1;
        break;

      default: //FALHA
        BiQuadFilterSetupPassthrough(Filter);
        return;
    }
    const float Alpha0 = 1 + Alpha;
    const float Alpha1 = -2 * CalcedCosine;
    const float Alpha2 = 1 - Alpha;
    Filter->Beta0 = Beta0 / Alpha0;
    Filter->Beta1 = Beta1 / Alpha0;
    Filter->Beta2 = Beta2 / Alpha0;
    Filter->Alpha1 = Alpha1 / Alpha0;
    Filter->Alpha2 = Alpha2 / Alpha0;
  }
  else //FALHA
  {
    BiQuadFilterSetupPassthrough(Filter);
  }
  Filter->SampleX1 = Filter->SampleX2 = 0;
  Filter->SampleY1 = Filter->SampleY2 = 0;
}

float BiQuadFilter::ApplyAndGet(BiquadFilter_Struct *Filter, float DeviceToFilter)
{
  const float Result = Filter->Beta0 * DeviceToFilter + Filter->SampleX1;
  Filter->SampleX1 = Filter->Beta1 * DeviceToFilter - Filter->Alpha1 * Result + Filter->SampleX2;
  Filter->SampleX2 = Filter->Beta2 * DeviceToFilter - Filter->Alpha2 * Result;
  return Result;
}
