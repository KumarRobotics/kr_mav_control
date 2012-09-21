#include "quad_serial_comm/serial_interface.h"

/* Packet format:
 * Packet: | > | * | > |countL|countH|type|---data---|crcL|crcH| < | # | < |
 * State:    0   1   2    3      4     5       6       7    8    9  10  11
 */

enum PacketState
{
  kPacketStart1,
  kPacketStart2,
  kPacketStart3,
  kPacketCountL,
  kPacketCountH,
  kPacketType,
  kPacketData,
  kPacketCRCL,
  kPacketCRCH,
  kPacketEnd1,
  kPacketEnd2,
  kPacketEnd3
};

static const unsigned int kPacketOverhead = 11;
static const char kPacketStartString[] = {'>', '*', '>'};
static const char kPacketStopString[] = {'<', '#', '<'};

static uint16_t crc16(const std::vector<uint8_t> &data);

void encode_serial_msg(const quadrotor_msgs::Serial &msg,
                       std::vector<uint8_t> &serial_data)
{
  const uint16_t data_length = msg.data.size();

  serial_data.resize(kPacketOverhead + data_length);
  serial_data[0] = kPacketStartString[0];
  serial_data[1] = kPacketStartString[1];
  serial_data[2] = kPacketStartString[2];
  serial_data[3] = data_length & 0xFF;
  serial_data[4] = data_length >> 8;
  serial_data[5] = msg.type;

  memcpy(&(serial_data[6]), &(msg.data[0]), data_length);
  const uint16_t crc = crc16(msg.data);

  serial_data[5+data_length + 1] = crc & 0xFF;
  serial_data[5+data_length + 2] = crc >> 8;
  serial_data[5+data_length + 3] = kPacketStopString[0];
  serial_data[5+data_length + 4] = kPacketStopString[1];
  serial_data[5+data_length + 5] = kPacketStopString[2];
}

void process_serial_data(const unsigned char *data, const size_t count,
                         boost::function<void (quadrotor_msgs::Serial)> callback)
{
  static quadrotor_msgs::Serial serial_msg;
  static enum PacketState state = kPacketStart1;
  static uint16_t received_length = 0, data_count = 0;;
  static uint16_t expected_crc = 0, received_crc = 0;

  for(size_t i = 0; i < count; i++)
  {
    const uint8_t c = data[i];
    if(state == kPacketStart1 && c == '>')
      state = kPacketStart2;
    else if(state == kPacketStart2 && c == '*')
      state = kPacketStart3;
    else if(state == kPacketStart3 && c == '>')
      state = kPacketCountL;
    else if(state == kPacketCountL)
    {
      received_length = c;
      state = kPacketCountH;
    }
    else if(state == kPacketCountH)
    {
      received_length = received_length + 256*c;
      serial_msg.data.resize(received_length);
      data_count = 0;
      state = kPacketType;
    }
    else if(state == kPacketType)
    {
      serial_msg.type = c;
      state = kPacketData;
    }
    else if(state == kPacketData)
    {
      serial_msg.data[data_count] = c;
      data_count++;
      if(data_count == received_length)
      {
        expected_crc = crc16(serial_msg.data);
        state = kPacketCRCL;
      }
    }
    else if(state == kPacketCRCL)
    {
      received_crc = c;
      state = kPacketCRCH;
    }
    else if(state == kPacketCRCH)
    {
      received_crc = received_crc + 256*c;
      if(expected_crc == received_crc)
        state = kPacketEnd1;
      else
        state = kPacketStart1;
    }
    else if(state == kPacketEnd1 && c == '<')
      state = kPacketEnd2;
    else if(state == kPacketEnd2 && c == '#')
      state = kPacketEnd3;
    else if(state == kPacketEnd3 && c == '<')
    {
      // Received complete packet
      callback(serial_msg);
      state = kPacketStart1;
    }
    else
      state = kPacketStart1;
  }
}

static inline uint16_t crc_update(uint16_t crc, uint8_t data)
{
  data ^= (crc & 0xff);
  data ^= data << 4;

  return ((((uint16_t)data << 8) | ((crc>>8)&0xff)) ^ (uint8_t)(data >> 4)
          ^ ((uint16_t)data << 3));
}

static uint16_t crc16(const std::vector<uint8_t> &data)
{
  uint16_t crc = 0xffff;

  for(uint16_t i = 0; i < data.size(); i++)
  {
    crc = crc_update(crc, data[i]);
  }
  return crc;
}
