#ifndef LINEAR_RESAMPLER_H
#define LINEAR_RESAMPLER_H

#include <cmath>
#include <vector>

// linear interpolating linear resampler for rate matching
// assumes rates to be close
template <typename T, typename U>struct LinearResampler
: std::vector<T>
{
  LinearResampler(int samples, int channels)
  : std::vector<T>(samples * channels),
    mCarry(channels, 0)
  {
  }
  int process(const T* data, int insamples, float log_ratio)
  {
    int channels = mCarry.size();
    int outsamples = insamples * (1.0 + log_ratio);
    outsamples = std::max(outsamples, insamples/2);
    outsamples = std::min(outsamples, 3*insamples/2);
    for(int i = 0; i < outsamples; ++i)
    {
      int inpos = 1024 * i * insamples / outsamples;
      int insample = inpos / 1024;
      int fracpart = inpos % 1024;
      for(int ch = 0; ch < channels; ++ch)
      {
        U s1 = mCarry[ch], s2 = data[insample * channels + ch];
        mCarry[ch] = s2;
        U value = s1 * (1024 - fracpart) + s2 * fracpart;
        value /= 1024;
        this->data()[i * channels + ch] = value;
      }
    }
    return outsamples;
  }
private:
  std::vector<T> mCarry;
};

#endif // LINEAR_RESAMPLER_H
