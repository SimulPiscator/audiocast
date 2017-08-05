#ifndef ALSA_AUDIO_H
#define ALSA_AUDIO_H

#include <string>
#include <vector>
#include <poll.h>
#include <alsa/asoundlib.h>
#include "util.h"

struct AlsaRetval : Retval
{
    explicit AlsaRetval(const std::string& desc = "") : Retval(desc) {}
    AlsaRetval(int val) : Retval(val) {}
    using Retval::operator=;
    using Retval::operator+=;
    using Retval::operator-=;
    using Retval::operator int;
    std::string getString(int err) const override { return snd_strerror(err); }
    int getError(int val) const override { return val < 0 ? val : 0; }
};

class AlsaStream
{
public:
    enum Direction { in, out };
    AlsaStream(Direction dir, const std::string& name, int rate, int channels,
      float frame_ms, int frames_buffered, float frames_low = -1);
    ~AlsaStream();
    snd_pcm_t* pcm() const;
    const std::vector<struct pollfd>& pfds() const;
    unsigned short revents(struct pollfd*, unsigned int nfds) const;
    void setNonblock(bool b);
    int periodSamples() const { return mPeriodSize; }
    int periods() const { return mPeriods; }
    int rate() const { return mRate; }

    void updateStatus(struct timespec* = nullptr);
    bool running() const;
    int sampleDelay() const;
    int samplesAvailable() const;
    int samplesFromOverflow() const;
    int samplesFromUnderrun() const;

    void start(int zerosamples = 0);
    void stop();
    int reset();
    int read(void* data, int samples);
    int write(const void* data, int samples);

private:
    Direction mDirection;
    unsigned int mPeriods;
    int mBytesPerSample, mPeriodSize;
    snd_pcm_t* mPcm;
    mutable snd_pcm_status_t* mStatus;
    int mSamplesAvailable, mSampleDelay, mRate;
    mutable std::vector<struct pollfd> mFds;
    mutable AlsaRetval mAlsaRetval;
};

#endif // ALSA_AUDIO_H
