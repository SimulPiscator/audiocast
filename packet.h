#ifndef PACKET_H
#define PACKET_H

#include <cstdint>
#include <arpa/inet.h>

struct PacketHeader
{
  static const uint16_t magicval = 'a' << 8 | 'd';
  uint16_t magic;
  enum { Data = 0, Ping = 1, numTypes };
  uint16_t type = numTypes;
  uint16_t index;
  uint16_t ts_ms;
  uint16_t size;
  void hton()
  {
    magic = htons(magicval);
    type = htons(type);
    index = htons(index);
    ts_ms = htons(ts_ms);
    size = htons(size);
  }
  bool ntoh()
  {
    if(magic != htons(magicval))
      return false;
    type = ntohs(type);
    if(type >= numTypes)
      return false;
    index = ntohs(index);
    ts_ms = ntohs(ts_ms);
    size = ntohs(size);
    return true;
  }
};

#endif // PACKET_H
