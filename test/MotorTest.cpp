#include <gtest/gtest.h>

#include "../src/CRC.h"
#include "../src/MotorMessage.h"

#include <unistd.h>
#include <stdint.h>

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(crc_check, crc_check_pass) {
  ur::CRC crc;

  uint8_t msg[9] = { 0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x00, 0x64, 0xD7 };
  ASSERT_EQ(0xD7, crc.checksum(msg, 1, 7));
}

TEST(crc_check, crc_check_fail) {
  ur::CRC crc;

  uint8_t msg[9] = { 0x7E, 0x02, 0xBB, 0x08, 0x00, 0x00, 0x00, 0x64, 0xD7 };
  ASSERT_NE(0xD7, crc.checksum(msg, 1, 7));
}

TEST(motormessage_check, motormessage_get_delimiter_pass)
{

  ur::MotorMessage msg(ur::MOTOR_MSG_TYPE_WRITE, ur::REG_ADDR_SET_LEFT_SPEED, 0x0F0F);
  ASSERT_EQ(0x7E, msg.getDelimiter());
}
/*
uint8_t getVersion(void);
uint8_t getType(void);
uint8_t getAddr(void);
message_value getValue(void);
uint8_t getChecksum(void);

// Converts m_Value from the motor controller board (bigendian) to
// Host format
void btoh(void);

// Converts m_Value from host format to the motor controller board (bigendian)
void htob(void);

// Sets the checksum (m_crc8) for the MotorMessage
void setChecksum(void);

// Tests the checksum, generated in this routine against the parameter 'ck'
// If true, routine sets m_crc8
bool checkChecksum(uint8_t ck);

// Tests the checksum, generated in this routing against m_crc8
bool checkChecksum(void);
*/
