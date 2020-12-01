#include <kr_serial_interface/serial_interface.h>

/* Packet format:
 * Packet: | 0x55 | 0x55 |count|type|---data---|crcL|crcH|
 * State:     0      1      2    3       4       5    6
 */

enum PacketState
{
  kPacketStart1,
  kPacketStart2,
  kPacketCount,
  kPacketType,
  kPacketData,
  kPacketCRCL,
  kPacketCRCH,
};

static const unsigned int kPacketOverhead = 6;
static const unsigned int kPacketDataMaxSize = 100;
static const uint8_t kPacketStartString[] = {0x55, 0x55};

static uint16_t crc16(const uint8_t *data, int count);
static uint16_t crc_update(uint16_t crc, uint8_t data);

void encode_serial_msg(const kr_mav_msgs::Serial &msg, std::vector<uint8_t> &serial_data)
{
  const uint8_t data_length = msg.data.size();

  if(data_length > kPacketDataMaxSize)
  {
    std::cerr << "encode_serial_msg: Message too large: " << msg.data.size() << ", max: " << kPacketDataMaxSize
              << std::endl;
    return;
  }

  serial_data.resize(kPacketOverhead + data_length);
  serial_data[0] = kPacketStartString[0];
  serial_data[1] = kPacketStartString[1];
  serial_data[2] = data_length;
  serial_data[3] = msg.type;

  uint16_t crc = crc16(&data_length, 1);
  crc = crc_update(crc, msg.type);

  memcpy(&(serial_data[4]), &(msg.data[0]), data_length);
  for(int i = 0; i < data_length; i++)
    crc = crc_update(crc, msg.data[i]);

  serial_data[4 + data_length + 0] = crc & 0xFF;
  serial_data[4 + data_length + 1] = crc >> 8;
}

void process_serial_data(const uint8_t *data, const size_t count, boost::function<void(kr_mav_msgs::Serial &)> callback)
{
  static kr_mav_msgs::Serial serial_msg;
  static enum PacketState state = kPacketStart1;
  static uint16_t received_length = 0, data_count = 0;
  ;
  static uint16_t expected_crc = 0, received_crc = 0;

  for(size_t i = 0; i < count; i++)
  {
    const uint8_t c = data[i];
    // printf("%d, %X\n", state, c);
    if(state == kPacketStart1 && c == kPacketStartString[0])
      state = kPacketStart2;
    else if(state == kPacketStart2 && c == kPacketStartString[1])
      state = kPacketCount;
    else if(state == kPacketCount)
    {
      received_length = c;
      if(received_length > kPacketDataMaxSize)
        state = kPacketStart1;
      else
      {
        expected_crc = crc16(&c, 1);
        serial_msg.data.resize(received_length);
        data_count = 0;
        state = kPacketType;
      }
    }
    else if(state == kPacketType)
    {
      serial_msg.type = c;
      expected_crc = crc_update(expected_crc, c);
      if(received_length > 0)
        state = kPacketData;
      else
        state = kPacketCRCL;
    }
    else if(state == kPacketData)
    {
      serial_msg.data[data_count] = c;
      expected_crc = crc_update(expected_crc, c);
      data_count++;
      if(data_count == received_length)
        state = kPacketCRCL;
    }
    else if(state == kPacketCRCL)
    {
      received_crc = c;
      state = kPacketCRCH;
    }
    else if(state == kPacketCRCH)
    {
      received_crc = received_crc + 256 * c;
      if(expected_crc == received_crc)
      {
        // Received complete packet
        callback(serial_msg);
      }
      state = kPacketStart1;
    }
    else
      state = kPacketStart1;
  }
}

// CRC-16-ITU-T
static uint16_t crc_update(uint16_t crc, uint8_t data)
{
  uint16_t x = ((crc >> 8) ^ data);
  x ^= x >> 4;

  crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;

  return crc;
}

static uint16_t crc16(const uint8_t *data, int count)
{
  uint16_t crc = 0xffff;

  for(uint16_t i = 0; i < count; i++)
  {
    crc = crc_update(crc, data[i]);
  }
  return crc;
}
