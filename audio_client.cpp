#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <opus/opus.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <alsa/asoundlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>
#include <errno.h>
#include <cstring>
#include <limits>
#include <cassert>
#include <sstream>
#include <stdexcept>

#include "packet.h"
#include "util.h"
#include "alsa_audio.h"
#include "linear_resampler.h"

template<int N, typename T=float> struct Statistics
{
  T decay, p[N+1];
  Statistics(T d) : decay(exp(-1/d)) { reset(); }
  void reset() { memset(p, 0, sizeof(p)); }
  void observe(T d)
  {
    T v = 1;
    for(int i = 0; i < N; ++i)
    {
      p[i] *= decay;
      p[i] += v;
      v *= d;
    }
    p[N] *= decay;
    p[N] += v;
  }
  T mean() const { return p[1]/p[0]; }
  T var() const { return (p[2] - p[1]*p[1]/p[0])/p[0]; }
};

struct Config
{
    float input_frame_ms = 10, output_frame_ms = 0, delay_ms = 60;
    int max_transport_delay_ms = 40;
    int channels = 2;
    float rate = 48000;
    std::string input, control_port;
    uint16_t client_id = 0;
};

template<typename T> class SampleBuffer
{
  int mSamples, mChannels, mReadPos;
  T* mpData;

public:
  SampleBuffer(int nsamples, int nchannels)
  : mSamples(0), mChannels(nchannels), mReadPos(0)
  { mpData = new T[nsamples * nchannels]; }
  ~SampleBuffer() { delete[] mpData; }
  T* writePtr(int samples) { mSamples = samples; mReadPos = 0; return mpData; }
  const T* readPtr(int samples = 0) { auto p = mpData + mReadPos * mChannels; mReadPos += samples; return p; }
  int samples() const { return mSamples - mReadPos; }
  int bytesPerSample() const { return mChannels * sizeof(T); }
};

template<typename T> struct BufferSet
{
  int mChannels, mReadIdx, mWriteIdx;
  std::vector<SampleBuffer<T>*> mBuffers;
public:
  BufferSet(int nsamples, int nchannels, int nbufs)
  : mChannels(nchannels), mReadIdx(0), mWriteIdx(0)
  {
    for(int i = 0; i < nbufs; ++i)
      mBuffers.push_back(new SampleBuffer<T>(nsamples, nchannels));
  }
  ~BufferSet()
  {
    for(auto p : mBuffers)
      delete p;
  }
  SampleBuffer<T>* nextWriteBuffer()
  {
    if((mWriteIdx+1) % mBuffers.size() == mReadIdx)
      return nullptr;
    ++mWriteIdx;
    mWriteIdx %= mBuffers.size();
    return mBuffers[mWriteIdx];
  }
  SampleBuffer<T>* currentReadBuffer()
  {
    return mBuffers[mReadIdx];
  }
  SampleBuffer<T>* nextReadBuffer()
  {
    if(mReadIdx == mWriteIdx)
      return nullptr;
    ++mReadIdx;
    mReadIdx %= mBuffers.size();
    return mBuffers[mReadIdx];
  }
};

class Input : std::vector<unsigned char>
{
    typedef union { unsigned char* p; PacketHeader* h; } Header;
    Header mRpos, mWpos;
    int mBytes, mPacketsAvail;
    Retval mRetval;
    struct pollfd mPfd;

   public:
    Input(const std::string& name, size_t size)
      : std::vector<unsigned char>(size),
      mRetval(name)
    {
      clear();
      mPfd.fd = -1;
      if(name.find(':') == std::string::npos)
        mPfd.fd = open(name.c_str(), O_RDONLY);
      else
        mPfd.fd = udp_bind(name);
      mRetval = mPfd.fd;
      mPfd.events = POLLIN | POLLERR;
    }
    const PacketHeader* getNextPacket(unsigned char** data)
    {
      assert(mPacketsAvail > 0);
      const PacketHeader* p = mRpos.h;
      if(data)
        *data = mRpos.p + sizeof(PacketHeader);
      mRpos.p += sizeof(PacketHeader) + mRpos.h->size;
      if(--mPacketsAvail == 0)
        clear();
      return p;
    }
    void clear()
    {
      mRpos.p = this->data();
      mWpos.p = this->data();
      mBytes = 0;
      mPacketsAvail = 0;
    }

    int packetsAvailable() const
    {
      return mPacketsAvail;
    }
    struct pollfd pfd() const
    {
      return mPfd;
    }
    int read()
    {
      bool eof = false;
      int totalbytes = 0, bytesread = 1, canread = 1;
      while(mBytes <= size() && bytesread > 0 && canread)
      {
        if(mBytes == size())
            throw std::runtime_error("Input buffer overflow");
        bytesread = mRetval = ::read(mPfd.fd, data() + mBytes, size() - mBytes);
        if(bytesread == 0)
        {
          eof = (totalbytes == 0);
        }
        else
        {
          mBytes += bytesread;
          totalbytes += bytesread;
          mRetval = poll(&mPfd, 1, 0);
          canread = (mRetval > 0 && mPfd.revents & POLLIN);
          eof = (mRetval > 0 && mPfd.revents & POLLERR);
        }
      }
      Header h = mWpos;
      int count = 0;
      while(h.p < mWpos.p + totalbytes && h.h->ntoh())
      {
        ++count;
        h.p += sizeof(PacketHeader) + h.h->size;
      }
      if(h.p == mWpos.p + totalbytes)
      {
        mPacketsAvail += count;
        mWpos = h;
      }
      else
      {
        std::clog << "ignoring packet(s) with invalid size" << std::endl;
        clear();
      }
      return eof ? -1 : count;
    }
};

class Clocksync
{
  int mSocket;
  uint16_t mClientId, mOffset;
  bool mValid;
  float mTimeout;
  struct timespec mRequestTs, mResponseTs;

public:
  Clocksync(const std::string& server, const Config& config)
  : mClientId(config.client_id), mOffset(0), mValid(false), mTimeout(config.max_transport_delay_ms*1e-3)
  {
    mRequestTs.tv_sec = 0;
    mResponseTs = mRequestTs;
    Retval r(server);
    r = udp_connect(server);
    mSocket = r;
  }

  uint16_t offset() const { return mOffset; }
  bool valid() const { return mValid; }
  const struct timespec& timestamp() const { return mResponseTs; }
  void invalidate()
  {
    mValid = false;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if(mRequestTs.tv_sec != 0 && timediff_s(now, mRequestTs) < mTimeout)
      return;
    mRequestTs = now;
    PacketHeader packet;
    packet.type = PacketHeader::Ping;
    packet.index = mClientId;
    packet.ts_ms = get_ms<uint16_t>(mRequestTs);
    packet.size = 0;
    packet.hton();
    Retval r = ::write(mSocket, &packet, sizeof(packet));
  }
  bool handleResponse(const PacketHeader& packet)
  {
    if(packet.type != PacketHeader::Ping || packet.index != mClientId)
      return false;
    clock_gettime(CLOCK_MONOTONIC, &mResponseTs);
    mOffset = packet.ts_ms;
    mValid = true;
    return true;
  }
};

static void connectToServer(const std::string& serverAddr, Config& config)
{
    Retval socket(serverAddr);
    socket = tcp_connect(serverAddr);
    std::string line;
    std::ostringstream oss;
    oss << "client " << config.input;
    fd_putline(socket, oss.str());
    while(fd_getline(socket, line) && !line.empty())
    {
        std::istringstream iss(line);
        std::string name;
        iss >> name;
        if(name=="input")
            iss >> config.input;
        else if(name=="rate")
            iss >> config.rate;
        else if(name=="channels")
            iss >> config.channels;
        else if(name=="frame-ms")
            iss >> config.input_frame_ms;
        else if(name=="delay-ms")
            iss >> config.delay_ms;
        else if(name=="max-transport-delay-ms")
            iss >> config.max_transport_delay_ms;
        else if(name=="client-id")
            iss >> config.client_id;
        else if(name=="control-port")
            iss >> config.control_port;
        else
            std::cerr << serverAddr << "ignoring unknown setting: " << line << std::endl;
    }
    fd_putline(socket, "");
    ::close(socket);
}

int main(int argc, char** argv)
{
  std::string outdev = "default", server;
  const char* default_port = "1784";
  float rate_adapt = 0.1, rate_observations = 10, max_correction_cents = 9;
  bool quiet = false, stdin_control = false;
  int input_timeout_ms = 0, clocksync_interval_ms = 1500;
  Config config;

  for(int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    size_t eq = arg.find('=');
    if(eq < arg.length())
      ++eq;
    else
      eq = arg.length();

    if(arg == "--no-rate-adapt")
      rate_adapt = 0.0;
    else if(arg.find("--rate-adapt=") == 0)
      rate_adapt = atof(arg.c_str() + eq);
    else if(arg.find("--delay-ms=") == 0)
      config.delay_ms = atof(arg.c_str() + eq);
    else if(arg.find("--max-transport-delay-ms=") == 0)
      config.max_transport_delay_ms = atof(arg.c_str() + eq);
    else if(arg.find("--quiet") == 0)
      quiet = true;
    else if(arg.find("--stdin-control") == 0)
     stdin_control = true;
    else if(arg.find("--frame-ms=") == 0)
      config.output_frame_ms = atof(arg.c_str() + eq);
    else if(arg.find("--input-timeout-ms=") == 0)
      input_timeout_ms = atoi(arg.c_str() + eq);
    else if(arg.find("--clocksync-interval-ms=") == 0)
      clocksync_interval_ms = atoi(arg.c_str() + eq);
    else if(arg.find("--rate-observations=") == 0)
      rate_observations = atof(arg.c_str() + eq);
    else if(arg.find("--max-correction-cents=") == 0)
      max_correction_cents = atof(arg.c_str() + eq);
    else if(arg.find("--channels=")==0)
      config.channels = atoi(arg.c_str() + eq);
    else if(arg.find("--server=") == 0)
      server = arg.substr(eq);
    else if(arg.find("--input=") == 0)
      config.input = arg.substr(eq);
    else if(arg.find("--output=") == 0)
      outdev = arg.substr(eq);
    else
      std::cerr << "ignoring unexpected argument: " << arg << std::endl;
  }
  if(config.input.empty() && server.empty() || outdev.empty())
  {
    std::cerr << "usage: " << argv[0] <<
                 " --server=<server ip>:<server port> | --input=<local ip>:<local port>"
                 " [--output=<alsa output device>]"
                 " [--frame-ms=<auto>]"
                 " [--channels=2]"
                 " [--input-timeout-ms=0]"
                 " [--delay-ms=60]"
                 " [--rate-adapt=0.1]"
                 " [--rate-observations=10]"
                 " [--clocksync-interval-ms=1500]"
                 " [--max-transport-delay-ms=40]"
                 " [--max-correction-cents=9]"
                 " [--quiet]"
                 " [--stdin-control]"
              << std::endl;
    return -1;
  }
  const char* sep = quiet ? "" : "\n";
  if(quiet)
    std::clog.rdbuf(nullptr);

  struct Status
  {
    bool running = false;
    uint32_t packets_received = 0, packets_lost = 0;
    uint32_t input_events = 0;
    uint16_t packet_index = 0;
    uint16_t packet_ts = 0;
    uint16_t ts_offset = 0;
    bool packet_ts_valid = false;
    float rate_correction = 0.0;
  } status;

  try {

    Clocksync* pClocksync = nullptr;
    if(!server.empty())
    {
        size_t pos = server.find(':');
        if(pos == std::string::npos)
        {
          pos = server.length();
          server += ":";
          server += default_port;
        }
        std::clog << "Connecting to " << server << std::endl;
        connectToServer(server, config);
        if(!config.control_port.empty())
          pClocksync = new Clocksync(server.substr(0, pos + 1) + config.control_port, config);
    }
    if(config.output_frame_ms <= 0)
      config.output_frame_ms = config.input_frame_ms;
    Input input(config.input, 65536);
    std::clog << "client-id=" << config.client_id
              << "\ninput=" << config.input
              << "\ncontrol-port=" << config.control_port
              << "\nframe-ms=" << config.input_frame_ms << "," << config.output_frame_ms
              << "\ndelay-ms=" << config.delay_ms
              << "\nmax-transport-delay-ms=" << config.max_transport_delay_ms
              << "\nrate=" << config.rate
              << "\nchannels=" << config.channels
              << std::endl;
    AlsaStream output(
                AlsaStream::out, outdev, config.rate,
                config.channels, config.output_frame_ms,
                (config.delay_ms + 100)/config.output_frame_ms);
    if(!output.pcm())
      rate_adapt = 0;
    rate_adapt = fabs(rate_adapt);
    if(rate_observations < 0)
      rate_observations = config.delay_ms/config.output_frame_ms;
    else if(rate_observations == 0)
      rate_observations = std::numeric_limits<float>::min();
    max_correction_cents = fabs(max_correction_cents);
    max_correction_cents = std::min(max_correction_cents, 100.0f); // avoid resampling buffer overflow
    std::clog << "rate-adapt=" << rate_adapt
              << "\nrate-observations=" << rate_observations
              << "\nmax-correction-cents=" << max_correction_cents
              << std::endl;
    output.setNonblock(rate_adapt);
    struct pollfd stdinPfd = { fileno(stdin), POLLIN, 0 };

    int err = 0;
    OpusDecoder* pDecoder = opus_decoder_create(config.rate, config.channels, &err);
    OpusRetval opuserr = err;

    int inframesamples = ceil(config.input_frame_ms * config.rate * 1e-3);
    SampleBuffer<opus_int16> inbuf(inframesamples, config.channels);
    LinearResampler<opus_int16, int32_t> resampler(3*inframesamples/2+1, config.channels);
    BufferSet<opus_int16> outbufs(3*inframesamples/2+1, config.channels, ceil(1e-3*config.delay_ms*config.rate/inframesamples) + 16);

    struct timespec ts_last_update = {0}, ts_last_input = {0}, ts_now = {0};
    clock_gettime(CLOCK_MONOTONIC, &ts_now);
    ts_last_update = ts_now;
    ts_last_update.tv_sec -= 1;
    ts_last_input = ts_now;
    Statistics<2> delaystat(rate_observations);
    delaystat.observe(config.delay_ms*1e-3);
    Statistics<1> ratestat(rate_observations);
    ratestat.observe(0);

    int poll_timeout_ms = ceil(config.input_frame_ms);
    struct pollfd pfds[] = { input.pfd(), stdinPfd };
    int npfds = stdin_control ? 2 : 1;

    mlockall(MCL_FUTURE);
    while(true)
    {
      if(!quiet)
      {
        static const int update_ms = 750;
        if(timediff_s(ts_now, ts_last_update) > update_ms*1e-3)
        {
            ts_last_update = ts_now;
            int pitch = -status.rate_correction / log(2.0) * 1200 + 0.5; // cents
            std::clog << "i/o: " << std::setw(6) << status.input_events
                      << " recv: " << std::setw(6) << status.packets_received
                      << " lost: " << std::setw(1) << status.packets_lost
                      << " delay: ";
            if(status.packet_ts_valid)
              std::clog << std::fixed << std::setprecision(0) << delaystat.mean()*1e3 << "ms";
            else
              std::clog << "unknown";
            std::clog << " pitch: " << std::showpos << std::setw(3) << pitch << std::noshowpos << "¢"
                      << " t_ms: " << status.packet_ts << "-" << status.ts_offset
                      << "      \r" << std::flush;
        }
      }

      if(pClocksync && timediff_s(ts_now, pClocksync->timestamp()) > clocksync_interval_ms*1e-3)
        pClocksync->invalidate();

      Retval r = poll(pfds, npfds, poll_timeout_ms);
      clock_gettime(CLOCK_MONOTONIC, &ts_now);
      bool is_input_event = (pfds[0].revents & POLLIN),
           is_stdin_event = (pfds[1].revents & POLLIN);
      if(pfds[0].revents & POLLERR)
      {
        std::cerr << sep << "Input error";
        exit(-1);
      }
      if(is_input_event)
      {
        ++status.input_events;
      }
      else if(input_timeout_ms > 0 && timediff_s(ts_now, ts_last_input) >= input_timeout_ms*1e-3)
      {
        std::clog << "\nInput timeout after " << input_timeout_ms << "ms" << std::endl;
        exit(-1);
      }

      if(is_stdin_event)
      {
          std::string command;
          std::getline(std::cin, command);
          if(command.empty() || command == "get_statistics")
          {
            std::cout << "packets_lost=" << status.packets_lost
                      << "\nbuffer_delay=" << (status.packet_ts_valid ? delaystat.mean() : std::numeric_limits<float>::quiet_NaN())
                      << "\nrate_correction=" << ratestat.mean()
                      << std::endl;
          }
          else
          {
            std::cerr << "unknown command: " << command << std::endl;
          }
      }

      if(is_input_event)
      {
        ts_last_input = ts_now;
        output.updateStatus(&ts_now);
        if(input.read() < 0)
          exit(0);
        int packets_processed = 0;

        SampleBuffer<opus_int16>* bufptr = nullptr;
        while(input.packetsAvailable())
        {
          unsigned char* data;
          auto hdr = input.getNextPacket(&data);
          if(hdr->type == PacketHeader::Data)
          {
            status.packet_ts = hdr->ts_ms;
            uint16_t delta_ms = get_ms<uint16_t>(ts_now) + status.ts_offset - hdr->ts_ms;
            status.packet_ts_valid = status.packet_ts_valid && (delta_ms <= config.max_transport_delay_ms); // a valid timestamp must be close to now
            if(!status.packet_ts_valid && pClocksync)
              pClocksync->invalidate();
            int16_t d = hdr->index - status.packet_index;
            if(!status.running)
            {
              status.running = true;
              int zero_samples = config.rate*config.delay_ms*1e-3 - input.packetsAvailable()*inframesamples;
              zero_samples = std::max(zero_samples, inframesamples/2);
              output.start(zero_samples);
              d = 1;
            }
            if(d <= 0)
            {
              ++status.packets_lost;
            }
            else if(d >= 1 && (bufptr = outbufs.nextWriteBuffer()))
            {
              ++status.packets_received;
              ++packets_processed;
              status.packet_index = hdr->index;

              int samples = opus_decode(pDecoder, data, hdr->size, inbuf.writePtr(inframesamples), inframesamples, 0);
              if(samples < 0)
                opuserr = samples;
              assert(samples == inframesamples);
              samples = resampler.process(inbuf.readPtr(), inbuf.samples(), status.rate_correction);
              memcpy(bufptr->writePtr(samples), resampler.data(), samples * bufptr->bytesPerSample());
            }
          }
          else if(pClocksync && pClocksync->handleResponse(*hdr))
          {
              status.packet_ts_valid = true;
              status.ts_offset = pClocksync->offset();
          }
          if(packets_processed > 0 && pClocksync && pClocksync->valid())
          {
            int16_t delta_ms = get_ms<uint16_t>(ts_now) + pClocksync->offset() - status.packet_ts;
            float delay = 1.0f*(output.sampleDelay() - (packets_processed-1)*inframesamples)/config.rate + 1e-3f*delta_ms;
            if(delay > 0)
            {
              delaystat.observe(delay);
              status.rate_correction = 0; // percentage of samples to remove if <0, to add if >0
              if(rate_adapt)
              {
                float delaydelta = delaystat.mean() - config.delay_ms*1e-3;
                status.rate_correction = -rate_adapt * delaydelta;
                static const float one_semitone = log(2.0)/12;
                static const float max_correction = one_semitone/100*max_correction_cents; // must be < log(3/2)
                status.rate_correction = std::max(status.rate_correction, -max_correction);
                status.rate_correction = std::min(status.rate_correction, max_correction);
              }
            }
            ratestat.observe(status.rate_correction);
          }
        }
      }

      if(status.running)
      {
        output.updateStatus();
        auto bufptr = outbufs.currentReadBuffer();
        while(bufptr && output.samplesFromOverflow() > 0)
        {
          int samples = std::min(bufptr->samples(), output.samplesFromOverflow());
          output.write(bufptr->readPtr(samples), samples);
          if(bufptr->samples() < 1)
            bufptr = outbufs.nextReadBuffer();
        }
        if(output.samplesFromUnderrun() < inframesamples) // underflow imminent, insert a packet
        {
          int samples = opus_decode(pDecoder, nullptr, 0, inbuf.writePtr(inframesamples), inframesamples, 0);
          if(samples < 0)
            opuserr = samples;
          assert(samples == inframesamples);
          samples = resampler.process(inbuf.readPtr(), inbuf.samples(), status.rate_correction);
          output.write(resampler.data(), samples);
          ++status.packet_index;
        }
      }
    }
    opus_decoder_destroy(pDecoder);
  }
  catch(const std::exception& e)
  {
    std::cerr << sep << e.what() << std::endl;
    return -1;
  }
  std::cerr << sep;
  return 0;
}

