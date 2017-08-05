#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>
#include <opus/opus.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/mman.h>

#include "util.h"
#include "packet.h"
#include "alsa_audio.h"

struct Bitrate
{
    Bitrate(float obs_time_s)
        : mObsTime_s(obs_time_s),
          mBits(0), mSeconds(0)
    {}
    float update(int bytes, float duration_s)
    {
        float decay = exp(-duration_s/mObsTime_s);
        mBits *= decay;
        mBits += 8 * bytes;
        mSeconds *= decay;
        mSeconds += duration_s;
        return mBits/mSeconds;
    }
    float mObsTime_s, mBits, mSeconds;
};

struct Client
{
  std::string address;
  uint16_t id;
  int socket;
  struct timespec last_active;
  std::vector<PacketHeader> pending_replies;
};

struct Config
{
    std::mutex mutex;
    std::string serverAddr, controlAddr;
    bool quiet = false;
    int channels = 2;
    float rate = 48000;
    float frame_ms = 10;
    int frames_buffered = 10;
    int complexity = 8;
    int control_port = 0;
    std::vector<Client> clients;
    float session_timeout_ms = 2000;
};

static void controlServer(Config* s)
{
  try
  {
    Retval r(s->controlAddr);
    r = udp_bind(s->controlAddr);
    int socket = r;
    char buf[1024];
    while(true)
    {
      r = recv(socket, buf, sizeof(buf), 0);
      union { char* c; PacketHeader* h; } p = { buf };
      if(p.h->ntoh())
      {
        std::lock_guard<std::mutex> lock(s->mutex);
        for(auto& c : s->clients)
        {
          if(c.id == p.h->index)
          {
            switch(p.h->type)
            {
              case PacketHeader::Ping:
              {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                p.h->ts_ms = get_ms<uint16_t>(ts) - p.h->ts_ms;
                p.h->size = 0;
//                std::clog << "Responding to ping from client " << c.id << " with offset " << p.h->ts_ms << std::endl;
              } break;
            }
            p.h->hton();
            c.pending_replies.push_back(*p.h);
            break;
          }
        }
      }
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    exit(-1);
  }
}

static void subscriptionServer(Config* s)
{
    uint16_t nextId = 1;
    try
    {
        Retval r(s->serverAddr);
        r = tcp_bind(s->serverAddr);
        int server = r;
        r = listen(server, 1024);
        while(true)
        {
            struct pollfd pfd = {0};
            pfd.fd = server;
            pfd.events = POLLIN;
            r = poll(&pfd, 1, -1);
            if(r)
            {
                try
                {
                    Retval socket("accept");
                    union { struct sockaddr a; struct sockaddr_in in; struct sockaddr_in6 in6; } addr;
                    socklen_t addrsize = sizeof(addr);
                    socket = accept(server, &addr.a, &addrsize);
                    const void* ipaddr = nullptr;
                    switch(addr.a.sa_family)
                    {
                    case AF_INET6:
                      ipaddr = &addr.in6.sin6_addr;
                      break;
                    case AF_INET:
                      ipaddr = &addr.in.sin_addr;
                      break;
                    }
                    char buf[64];
                    if(!inet_ntop(addr.a.sa_family, ipaddr, buf, sizeof(buf)))
                      *buf = 0;
                    std::string clientAddr = buf + s->serverAddr.substr(s->serverAddr.find(':'));
                    std::string line;
                    if(fd_getline(socket, line))
                    {
                        std::istringstream iss(line);
                        std::string name;
                        iss >> name;
                        if(name == "client")
                        {
                            std::string addr;
                            iss >> addr;
                            if(addr.empty())
                              addr = clientAddr;
                            uint16_t id = nextId++;
                            std::ostringstream oss;
                            oss << "client-id " << id
                                << "\ncontrol-port " << s->control_port
                                << "\ninput " << addr
                                << "\nframe-ms " << s->frame_ms
                                << "\nchannels " << s->channels
                                << std::endl;
                            if(fd_putline(socket, oss.str())
                                && fd_getline(socket, line))
                            {
                                Retval c(addr);
                                c = udp_connect(addr);
                                Client client;
                                client.address = addr;
                                client.socket = c;
                                client.id = id;
                                clock_gettime(CLOCK_MONOTONIC, &client.last_active);
                                std::lock_guard<std::mutex> lock(s->mutex);
                                s->clients.push_back(client);
                                std::clog << "new client session #" << id << " from " << addr
                                          << std::endl;
                            }
                        }
                    }
                    close(socket);
                }
                catch(const std::exception& e)
                {
                    std::cerr << "could not serve client: " << e.what() << std::endl;
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        exit(-1);
    }
}

int main(int argc, char** argv)
{
  std::string input;
  Config config;
  float compression = 0.5;

  for(int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    size_t eq = arg.find('=');
    if(eq < arg.length())
      ++eq;
    else
     eq = arg.length();
    if(arg.find("--frame-ms=") == 0)
      config.frame_ms = atof(arg.c_str() + eq);
    if(arg.find("--session-timeout-ms=") == 0)
      config.session_timeout_ms = atof(arg.c_str() + eq);
    else if(arg.find("--channels=") == 0)
      config.channels = atoi(arg.c_str() + eq);
    else if(arg.find("--compression-ratio=") == 0)
      compression = atof(arg.c_str() + eq);
    else if(arg.find("--subscription-address=") == 0)
        config.serverAddr = arg.substr(eq);
    else if(arg.find("--control-port=") == 0)
        config.control_port = atoi(arg.c_str() + eq);
    else if(arg.find("--complexity=") == 0)
        config.complexity = atoi(arg.c_str() + eq);
    else if(arg.find("--input=") == 0)
        input = arg.substr(eq);
    else if(arg == "--quiet")
        config.quiet = true;
    else
      std::cerr << "ignoring unexpected argument: " << arg << std::endl;
  }
  if(input.empty())
  {
    std::cerr << "usage: " << argv[0]
              << " --input=<audio device>|-"
              << " [--subscription-address=0.0.0.0:1784]"
              << " [--control-port=1785]"
              << " [--session-timeout-ms=2000]"
                 " [--frame-ms=10]"
                 " [--channels=2]"
                 " [--compression-ratio=0.5]"
                 " [--complexity=8]"
                 " [--quiet]"
              << std::endl;
    return -1;
  }
  if(config.quiet)
      std::clog.rdbuf(nullptr);

  if(config.serverAddr.empty())
    config.serverAddr = "0.0.0.0";
  size_t pos = config.serverAddr.find(':');
  if(pos == std::string::npos)
    config.serverAddr += ":1784";
  pos = config.serverAddr.find(':');
  if(config.control_port == 0)
    config.control_port = ::atoi(config.serverAddr.c_str() + pos + 1) + 1;
  std::ostringstream controlAddr;
  controlAddr << config.serverAddr.substr(0, pos + 1) << config.control_port;
  config.controlAddr = controlAddr.str();

  int framesize = floor(1e-3 * config.frame_ms * config.rate + 0.5);
  std::clog << "frame duration " << config.frame_ms
            << "ms = " << framesize << " samples"
            << std::endl;
  int bytespersample = 2 * config.channels;

  std::vector<opus_int16> inputbuf(framesize * config.channels);
  std::vector<unsigned char> outputbuf(sizeof(PacketHeader) + ::ceil(inputbuf.size() * compression));

  try {

    AlsaStream inputStream(
                AlsaStream::in, input,
                config.rate, config.channels,
                config.frame_ms, config.frames_buffered);

    int err = 0;
    OpusEncoder* pEncoder = opus_encoder_create(
      config.rate, config.channels, OPUS_APPLICATION_AUDIO, &err);
    OpusRetval opuserr = err;
    opuserr = opus_encoder_ctl(pEncoder, OPUS_SET_PACKET_LOSS_PERC(0));
    opuserr = opus_encoder_ctl(pEncoder, OPUS_SET_COMPLEXITY(config.complexity));
    opuserr = opus_encoder_ctl(pEncoder, OPUS_SET_BITRATE(OPUS_BITRATE_MAX));
    Bitrate bitrate(1);

    std::thread subscriptionServerThread(subscriptionServer, &config);
    std::thread controlServerThread(controlServer, &config);
    mlockall(MCL_FUTURE);
    int packetIndex = 0;
    inputStream.start();
    while(true)
    {
      inputStream.read(inputbuf.data(), framesize);
      struct timespec ts;
      inputStream.updateStatus(&ts);
      ts.tv_nsec -= 1e9f * (framesize + inputStream.sampleDelay()) / inputStream.rate();
      OpusRetval bytes = opus_encode(pEncoder,
        inputbuf.data(), framesize,
        outputbuf.data() + sizeof(PacketHeader),
        outputbuf.size() - sizeof(PacketHeader));
      union { unsigned char* p; PacketHeader* h; } hdr = { outputbuf.data() };
      hdr.h->index = packetIndex++;
      hdr.h->type = PacketHeader::Data;
      hdr.h->ts_ms = get_ms<uint16_t>(ts);
      hdr.h->size = bytes;
      hdr.h->hton();
      bytes += sizeof(PacketHeader);
      if(!config.quiet && (packetIndex % 10 == 0))
      {
        float b = bitrate.update(bytes, framesize / config.rate),
              kbit = b / 1024, kbytes = kbit / 8,
              ratio = b / config.rate / 16 / config.channels;
        std::clog << "\r"
                  << std::fixed << std::setprecision(1)
                  << kbit << "kBit/s = "
                  << kbytes << "kB/s = "
                  << std::setprecision(2)
                  << ratio << "*raw  t_ms = "
                  << ntohs(hdr.h->ts_ms) << "    \t"
                  << std::flush;
      }
      std::lock_guard<std::mutex> lock(config.mutex);
      for(auto& c : config.clients)
        write(c.socket, outputbuf.data(), bytes);
      for(auto& c : config.clients)
      {
        if(!c.pending_replies.empty())
        {
          for(auto& p : c.pending_replies)
            write(c.socket, &p, sizeof(p));
          c.pending_replies.clear();
          c.last_active = ts;
        }
      }
      auto i = config.clients.begin();
      while(i != config.clients.end())
      {
        if(timediff_s(ts, i->last_active) > config.session_timeout_ms*1e-3)
        {
          std::clog << "session #" << i->id << " from " << i->address
                    << " inactive for " << config.session_timeout_ms*1e-3 << "s, closing"
                    << std::endl;
          ::close(i->socket);
          i = config.clients.erase(i);
        }
        else
          ++i;
      }
    }
    opus_encoder_destroy(pEncoder);
  }
  catch(const std::exception& e)
  {
    std::cerr << "\n" << e.what() << std::endl;
    exit(-1);
  }
  return 0;
}

