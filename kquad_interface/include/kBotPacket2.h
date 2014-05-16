/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
// [OxFF][0xFF][ID][LENGTH][TYPE][DATA1]..[DATAN][CHECKSUM0][CHECKSUM1][CHECKSUM2][CHECKSUM3]
// CHECKSUM = CRC-16 (x^16+x^15+x^2+1)
//
// Alex Kushleyev, KMel Robotics, 2012

#ifndef KBOT_PACKET2_H
#define KBOT_PACKET2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

#define KBOT_PACKET2_HEADER 0xFA
#define KBOT_PACKET2_OVERHEAD 9
#define KBOT_PACKET2_MIN_SIZE KBOT_PACKET2_OVERHEAD
#define KBOT_PACKET2_MAX_SIZE 247
#define KBOT_PACKET2_MAX_PAYLOAD_SIZE (KBOT_PACKET2_MAX_SIZE-KBOT_PACKET2_MIN_SIZE)

enum { KBOT_PACKET2_POS_HEADER1 = 0,
       KBOT_PACKET2_POS_HEADER2,
       KBOT_PACKET2_POS_ID,
       KBOT_PACKET2_POS_SIZE,
       KBOT_PACKET2_POS_TYPE,
       KBOT_PACKET2_POS_DATA };


//buffer packet definition
typedef struct
{
  uint8_t buffer[KBOT_PACKET2_MAX_SIZE];  //buffer for data
  uint8_t lenReceived; //number of chars received so far
  uint8_t lenExpected; //expected number of chars based on header
  uint8_t * bp;        //pointer to the next write position in the buffer
} kBotPacket2;

//initialze the fields in the dynamixel packet buffer
void      kBotPacket2Init(kBotPacket2 * packet);

//feed one char and see if we have accumulated a complete packet
int16_t   kBotPacket2ProcessChar(uint8_t c, kBotPacket2 * packet);

//get id of the sender
static inline uint8_t   kBotPacket2GetId(kBotPacket2 * packet)
{
  return packet->buffer[KBOT_PACKET2_POS_ID];
}

static inline uint8_t   kBotPacket2RawGetId(uint8_t * packet)
{
  return packet[KBOT_PACKET2_POS_ID];
}

//get size of the packet (as it appears in dynamixel packet)
static inline uint8_t   kBotPacket2GetSize(kBotPacket2 * packet)
{
  return packet->buffer[KBOT_PACKET2_POS_SIZE];
}

static inline uint8_t   kBotPacket2RawGetSize(uint8_t * packet)
{
  return packet[KBOT_PACKET2_POS_SIZE];
}

//get the size of payload (without message type or checksum)
static inline uint8_t   kBotPacket2GetPayloadSize(kBotPacket2 * packet)
{
  return packet->buffer[KBOT_PACKET2_POS_SIZE] - 5;  //subtract the instruction and checksum
}

static inline uint8_t   kBotPacket2RawGetPayloadSize(uint8_t * packet)
{
  return packet[KBOT_PACKET2_POS_SIZE] - 5;
}

//get a pointer to the packet type
static inline uint8_t   kBotPacket2GetType(kBotPacket2 * packet)
{
  return packet->buffer[KBOT_PACKET2_POS_TYPE];
}

static inline uint8_t   kBotPacket2RawGetType(uint8_t * packet)
{
  return packet[KBOT_PACKET2_POS_TYPE];
}

//get a pointer to the packet payload
static inline uint8_t * kBotPacket2GetData(kBotPacket2 * packet)
{
  return &(packet->buffer[KBOT_PACKET2_POS_DATA]);
}

static inline uint8_t * kBotPacket2RawGetData(uint8_t * packet)
{
  return &(packet[KBOT_PACKET2_POS_DATA]);
}

//wrap arbitrary data into the kBot packet format
int16_t kBotPacket2WrapData(uint8_t id, uint8_t type,
                            uint8_t * data, uint16_t dataSize,
                            uint8_t * outBuf, uint16_t outSize);



#ifdef __cplusplus
}
#endif

#endif //KBOT_PACKET2_H

