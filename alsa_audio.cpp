#include "alsa_audio.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <algorithm>

//#define LOG Log(std::clog) << __func__
#define LOG NoLog()

static const char sZerodata[4096] = { 0 };

AlsaStream::AlsaStream(
        Direction dir,
        const std::string& name,
        int rate, int channels, float frame_ms,
        int frames_buffered, float frames_low)
    : mBytesPerSample(channels * sizeof(float)),
      mDirection(dir), mPcm(nullptr), mAlsaRetval(name + " alsa device"),
      mPeriods(0), mPeriodSize(0), mStatus(nullptr), mSamplesAvailable(0),
      mRate(rate)
{
    snd_pcm_status_malloc(&mStatus);
    if(frames_low < 0)
      frames_low = 1;
    LOG << name << rate << channels << frame_ms << frames_buffered << frames_low;
    assert(frames_low > 0 && frames_low <= frames_buffered);
    if(name == "-")
    {
        struct pollfd pfd = { 0 };
        pfd.fd = fileno(dir == in ? stdin : stdout);
        pfd.events = dir == in ? POLLIN : POLLOUT;
        mFds.push_back(pfd);
    }
    else
    {
        AlsaRetval r("setting up " + name);
        snd_pcm_stream_t d;
        switch(dir)
        {
        case in:
            d = SND_PCM_STREAM_CAPTURE;
            break;
        case out:
            d = SND_PCM_STREAM_PLAYBACK;
            break;
        default:
            throw std::runtime_error("invalid direction");
        }

        r = snd_pcm_open(&mPcm, name.c_str(), d, 0);
        snd_pcm_hw_params_t* hw;
        snd_pcm_hw_params_alloca(&hw);
        r = snd_pcm_hw_params_any(mPcm, hw);
        r = snd_pcm_hw_params_set_access(mPcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED);
        r = snd_pcm_hw_params_set_format(mPcm, hw, SND_PCM_FORMAT_S16_LE);
        r = snd_pcm_hw_params_set_rate(mPcm, hw, mRate, 0);
        r = snd_pcm_hw_params_set_channels(mPcm, hw, channels);
        r = snd_pcm_hw_params_set_period_size_integer(mPcm, hw);
        snd_pcm_uframes_t periodsize = floor(rate * frame_ms * 1e-3);
        r = snd_pcm_hw_params_set_period_size_near(mPcm, hw, &periodsize, 0);
        mPeriodSize = periodsize;
        r = snd_pcm_hw_params_set_periods_integer(mPcm, hw);
        mPeriods = frames_buffered;
        r = snd_pcm_hw_params_set_periods_near(mPcm, hw, &mPeriods, 0);
        r = snd_pcm_hw_params(mPcm, hw);

        snd_pcm_sw_params_t* swparams;
        snd_pcm_sw_params_alloca(&swparams);
        r = snd_pcm_sw_params_current(mPcm, swparams);
        snd_pcm_uframes_t buffersize = mPeriods * periodsize,
          availmin = buffersize - frames_low * periodsize;
        r = snd_pcm_sw_params_set_avail_min(mPcm, swparams, availmin);
        r = snd_pcm_sw_params_set_start_threshold(mPcm, swparams, 2*buffersize);
        r = snd_pcm_sw_params_set_stop_threshold(mPcm, swparams, 2*buffersize);
        r = snd_pcm_sw_params_set_silence_threshold(mPcm, swparams, 0);
        r = snd_pcm_sw_params_set_tstamp_mode(mPcm, swparams, SND_PCM_TSTAMP_ENABLE);
        r = snd_pcm_sw_params(mPcm, swparams);
        r = snd_pcm_prepare(mPcm);
    }
}

AlsaStream::~AlsaStream()
{
    LOG;
    if(mPcm)
        snd_pcm_close(mPcm);
    snd_pcm_status_free(mStatus);
}

snd_pcm_t*
AlsaStream::pcm() const
{
    return mPcm;
}

const std::vector<struct pollfd>&
AlsaStream::pfds() const
{
    if(mPcm)
    {
        mAlsaRetval = snd_pcm_poll_descriptors_count(mPcm);
        mFds.resize(mAlsaRetval);
        mAlsaRetval = snd_pcm_poll_descriptors(mPcm, mFds.data(), mFds.size());
    }
    return mFds;
}

unsigned short
AlsaStream::revents(struct pollfd* pfd, unsigned int nfds) const
{
  unsigned short revents = 0;
  if(mPcm)
  {
    if(snd_pcm_poll_descriptors_revents(mPcm, pfd, nfds, &revents) < 0)
      revents = POLLERR;
  }
  else
  {
    for(unsigned int i = 0; i < nfds; ++i)
      revents |= pfd[i].revents;
  }
  return revents;
}

void
AlsaStream::setNonblock(bool b)
{
    LOG;
    if(!mPcm)
    {
        Retval r("fcntl");
        r = fcntl(mFds[0].fd, F_GETFL, 0);
        int flags = r;
        flags &= ~O_NONBLOCK;
        if(b)
            flags |= O_NONBLOCK;
        r = fcntl(mFds[0].fd, F_SETFL, flags);
    }
    else
    {
        mAlsaRetval = snd_pcm_nonblock(mPcm, b ? 1 : 0);
    }
}

void
AlsaStream::updateStatus(struct timespec* ts)
{
  mSampleDelay = 0;
  mAlsaRetval = snd_pcm_status(mPcm, mStatus);
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  if(ts)
    *ts = now;
  if(snd_pcm_status_get_state(mStatus) == SND_PCM_STATE_RUNNING)
  {
    mSamplesAvailable = snd_pcm_status_get_avail(mStatus);
    mSampleDelay = snd_pcm_status_get_delay(mStatus);
#if 0
    snd_htimestamp_t t;
    snd_pcm_status_get_htstamp(mStatus, &t);
    int delta_usec = (now.tv_nsec - t.tv_nsec)/1000 + 1000*1000*(now.tv_sec-t.tv_sec);
#else
    snd_timestamp_t t;
    snd_pcm_status_get_tstamp(mStatus, &t);
    int delta_usec = now.tv_nsec/1000 - t.tv_usec + 1000*1000*(now.tv_sec-t.tv_sec);
#endif
    int delta_samples = delta_usec * mRate / 1000000;
    mSamplesAvailable += delta_samples;
    mSampleDelay -= delta_samples;
  }
  else if(mDirection == out)
    mSamplesAvailable = mPeriods * mPeriodSize;
  else
    mSamplesAvailable = 0;
}

bool
AlsaStream::running() const
{
  return snd_pcm_status_get_state(mStatus) == SND_PCM_STATE_RUNNING;
}

int
AlsaStream::samplesFromOverflow() const
{
  switch(mDirection)
  {
  case in:
    return mPeriods * mPeriodSize - samplesAvailable();
  case out:
    return samplesAvailable();
  }
  return Retval(-1);
}

int
AlsaStream::samplesFromUnderrun() const
{
  switch(mDirection)
  {
  case in:
    return samplesAvailable();
  case out:
    return mPeriods * mPeriodSize - samplesAvailable();
  }
  return Retval(-1);
}

int
AlsaStream::samplesAvailable() const
{
    if(!mPcm)
        return std::numeric_limits<int>::max();
    return mSamplesAvailable;
}

int
AlsaStream::sampleDelay() const
{
  return mSampleDelay;
}

void
AlsaStream::start(int zerosamples)
{
    LOG;
    if(mPcm)
    {
      while(mDirection == out && zerosamples > 0)
      {
        mAlsaRetval = snd_pcm_writei(mPcm, sZerodata, std::min<int>(zerosamples, sizeof(sZerodata)/mBytesPerSample));
        zerosamples -= mAlsaRetval;
      }
      mAlsaRetval = snd_pcm_start(mPcm);
    }
}

void
AlsaStream::stop()
{
  LOG;
  if(mPcm)
    mAlsaRetval = snd_pcm_drop(mPcm);
}

int
AlsaStream::reset()
{
    LOG;
    if(!mPcm)
        return -1;
    snd_pcm_drop(mPcm);
    mAlsaRetval = snd_pcm_prepare(mPcm);
    return 0;
}

int
AlsaStream::read(void* data, int samples)
{
    LOG << data << samples;
    char* begin = static_cast<char*>(data), *p = begin;
    if(mPcm)
    {
        while(samples > 0)
        {
            int read = snd_pcm_readi(mPcm, p, samples);
            if(read < 0)
            {
              std::clog << __func__ << ": Recovering from error " << read << std::endl;
              read = snd_pcm_recover(mPcm, read, 0);
            }
            if(read < 0)
                mAlsaRetval = read;
            samples -= read;
            p += read * mBytesPerSample;
        }
    }
    else
    {
        int bytes = samples * mBytesPerSample;
        while(bytes > 0)
        {
            Retval read("read");
            read = ::read(mFds.front().fd, p, bytes);
            p += read;
            bytes -= read;
        }
    }
    samples = (p - begin)/mBytesPerSample;
    mSamplesAvailable = std::max(mSamplesAvailable - samples, 0);
    mSampleDelay = std::max(mSampleDelay - samples, 0);
    return samples;
}

int
AlsaStream::write(const void* data, int samples)
{
    const char* p = static_cast<const char*>(data);
    int written = 0;
    if(mPcm)
    {
        while(written < samples)
        {
            int r = snd_pcm_writei(mPcm, p, samples - written);
            if(r < 0)
            {
              std::clog << __func__ << ": Recovering from error " << r << std::endl;
              r = snd_pcm_recover(mPcm, r, 0);
            }
            if(r < 0)
                mAlsaRetval = r;
            written += r;
            p += r * mBytesPerSample;
        }
    }
    else
    {
        int bytes = samples * mBytesPerSample;
        while(bytes > 0)
        {
            Retval r("write");
            r = ::write(mFds.front().fd, p, bytes);
            p += r;
            bytes -= r;
        }
        written = samples;
    }
    mSamplesAvailable = std::max(mSamplesAvailable - written, 0);
    mSampleDelay += samples;
    return written;
}
