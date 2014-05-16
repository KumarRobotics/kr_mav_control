/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#include "kBotPacket2.h"
#include "crc.h"
#include <string.h>

//initialize the packet
void kBotPacket2Init(kBotPacket2 * packet)
{
  packet->lenReceived = 0;
  packet->lenExpected = 0;
  packet->bp          = NULL;
}

//feed in a character and see if we got a complete packet
int16_t   kBotPacket2ProcessChar(uint8_t c, kBotPacket2 * packet)
{
  int16_t ret = 0;
  uint32_t checksum = 0;
  uint32_t * pchecksum;

  switch (packet->lenReceived)
  {
    case 0:
      packet->bp = packet->buffer;    //reset the pointer for storing data

    //fall through, since first two bytes should be 0xFA
    case 1:
      if (c != KBOT_PACKET2_HEADER)       //check the packet header (0xFA)
      {
        packet->lenReceived = 0;
        ret = -1;
        break;
      }

    //fall through, since we are just storing ID and length
    case 2:                           //ID
    case 3:                           //LENGTH
      packet->lenReceived++;
      *(packet->bp)++ = c;
      break;

    case 4:                           //by now we've got 0xFF, 0xFF, ID, LENGTH
      packet->lenExpected = packet->buffer[3] + 4;  //add 4 to get the full length

      //verify the expected length
      if ( (packet->lenExpected < KBOT_PACKET2_MIN_SIZE)
        || (packet->lenExpected > KBOT_PACKET2_MAX_SIZE) )
      {
        packet->lenReceived = 0;
        packet->lenExpected = 0;
        ret = -2;
        break;
      }

    //read off the rest of the packet
    default:
      packet->lenReceived++;
      *(packet->bp)++ = c;

      if (packet->lenReceived < packet->lenExpected)
        break;  //have not received enough yet

      //calculate expected checksum
      //skip first two 0xFA and the actual checksum
      //checksum =  CRC16_ANSI_Block(packet->buffer+2,
      //                             packet->lenReceived-4);
      checksum =  kmel_crc32(packet->buffer+2,packet->lenReceived-6);
      //printf("checksum is %X\n",checksum);


      pchecksum = (uint32_t *)(packet->bp-4);

      if (checksum == *pchecksum)
        ret  = packet->lenReceived;
      else
        ret = -3;

      //reset the counter
      packet->lenReceived = 0;
  }

  return ret;
}


//wrap arbitrary data into the kBot packet format
int16_t kBotPacket2WrapData(uint8_t id, uint8_t type,
                            uint8_t * data, uint16_t dataSize,
                            uint8_t * outBuf, uint16_t outSize)
{
  uint16_t packetSize = dataSize + KBOT_PACKET2_OVERHEAD;

  //make sure enough memory is externally allocated
  if (outSize < packetSize)
    return -1;

  uint8_t checkSize   = dataSize + 3;     //data + id + length + type
  uint8_t payloadSize = dataSize + 5;     //length includes packet, type and checksum
  uint8_t * obuf      = outBuf;
  uint8_t * ibuf      = data;
  *obuf++             = KBOT_PACKET2_HEADER;           //two header bytes
  *obuf++             = KBOT_PACKET2_HEADER;
  *obuf++             = id;
  *obuf++             = payloadSize;
  *obuf++             = type;

  //copy data and calculate the checksum
  memcpy(obuf,ibuf,dataSize);
  obuf               += dataSize;

  //uint16_t checksum =  CRC16_ANSI_Block(outBuf+2, payloadSize);  //dont count first two bytes and checksum
  uint32_t checksum = kmel_crc32(outBuf+2, checkSize);

  uint32_t * obuf2 = (uint32_t *)obuf;
  *obuf2 = checksum;

  return packetSize;
}

