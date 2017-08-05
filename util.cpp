#include "util.h"

#include <opus/opus.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <poll.h>

float timediff_s(const struct timespec& t1, const struct timespec& t2)
{
  float delta = t1.tv_nsec/1000 - t2.tv_nsec/1000;
  delta /= 1000*1000;
  delta += t1.tv_sec - t2.tv_sec;
  return delta;
}

bool fd_putline(int fd, const std::string& line, int eolchar)
{
    int written = 0;
    Retval r("fd_putline");
    while(written < line.length())
    {
      r = write(fd, line.data() + written, line.length() - written);
      if(r == 0)
          return false;
      written += r;
    }
    char c = eolchar;
    r = write(fd, &c, 1);
    return r == 1;
}

bool fd_getline(int fd, std::string& line, int eolchar)
{
    line.clear();
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    while(poll(&pfd, 1, -1))
    {
        char c;
        Retval r("fd_getline");
        r = read(fd, &c, 1);
        if(r < 1)
            return !line.empty();
        if(c == eolchar)
            return true;
        line += c;
    }
    return false;
}

static int socket_open(const std::string& address, int socktype, int(*func)(int, const sockaddr*, socklen_t))
{
  size_t pos = address.find(':');
  if(pos < 1 || pos > address.size())
  {
    errno = EINVAL;
    return -1;
  }
  std::string ip = address.substr(0, pos),
    port = address.substr(pos + 1);
  addrinfo hints = { 0 };
  hints.ai_family = AF_INET;
  hints.ai_socktype = socktype;
  addrinfo* info = nullptr;
  int err = getaddrinfo(ip.c_str(), port.c_str(), &hints, &info);
  if(err)
  {
    std::cerr << gai_strerror(err) << std::endl;
    return -1;
  }
  int s = -1;
  for(addrinfo* p = info; p && s < 0; p = p->ai_next)
  {
    s = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    int enable = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
#if SO_REUSEPORT
    setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &enable, sizeof(enable));
#endif
    if(s >= 0 && func(s, p->ai_addr, p->ai_addrlen))
    {
      err = errno;
      close(s);
      s = -1;
    }
  }
  freeaddrinfo(info);
  if(err)
    errno = err;
  return s;
}

int tcp_connect(const std::string& s)
{
  return socket_open(s, SOCK_STREAM, &connect);
}

int tcp_bind(const std::string& s)
{
  return socket_open(s, SOCK_STREAM, &bind);
}

int udp_connect(const std::string& s)
{
  return socket_open(s, SOCK_DGRAM, &connect);
}

int udp_bind(const std::string& s)
{
  return socket_open(s, SOCK_DGRAM, &bind);
}

int Retval::ThrowIfError(int val) const
{
  int err = getError(val);
  if(err)
  {
    std::ostringstream oss;
    oss << (mDesc.empty() ? "" : mDesc + ": ")
        << getString(err)
        << " (" << err << ")";
    throw std::runtime_error(oss.str());
  }
  return val;
}

