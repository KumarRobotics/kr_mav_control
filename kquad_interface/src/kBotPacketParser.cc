/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#include "kBotPacketParser.hh"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

kBotPacketParser::kBotPacketParser()
{

  kBotPacket2Init(&this->packet);
  
  this->buf       = new uint8_t[1024];
  this->bufPos    = this->buf;
  this->nBuffered = 0;
}

kBotPacketParser::~kBotPacketParser()
{
  
  delete [] this->buf;
}

int kBotPacketParser::ProcessBuffer(kBotPacket2 * ppacket)
{
  int ret = 0;
  while(this->nBuffered > 0)
  {
    ret = kBotPacket2ProcessChar(*this->bufPos++,&this->packet);
    this->nBuffered--;
  
    if (ret > 0)
    {
      memcpy(ppacket, &this->packet, sizeof(this->packet));
      return ret;
    }
    //else if (ret < 0) printf("ERROR while processing buffer \n");
  }

  return ret;
}

int kBotPacketParser::PushData(uint8_t * data, int size)
{
  if (this->nBuffered > 0)
  {
    printf("kBotPacketParser::PushData: ERROR: process the stored data before adding more\n");
    return -1;
  }
  
  if (size > 1024)
  {
    printf("kBotPacketParser::PushData: ERROR: can take in no more than 1024 bytes\n");
    return -1;
  }
  
  this->bufPos    = this->buf;
  this->nBuffered = size;
  memcpy(this->buf,data,size);
  
  return 0;
}
