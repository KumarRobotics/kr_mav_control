/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#ifndef KBOT_PACKET_PARSER
#define KBOT_PACKET_PARSER

#include "kBotPacket2.h"
#include <stdint.h>

class kBotPacketParser
{
  public:     kBotPacketParser();
  public:    ~kBotPacketParser();
  public: int ProcessBuffer(kBotPacket2 * packet);
  public: int PushData(uint8_t * data, int size);


  protected: kBotPacket2 packet;
  protected: int nBuffered;
  protected: uint8_t * bufPos;
  protected: uint8_t * buf;
};

#endif
