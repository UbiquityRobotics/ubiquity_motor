/*
 * MotorMessage.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: kurt
 */

#include <endian.h>

#include "CRC.h"
#include "MotorMessage.h"

namespace ur {

MotorMessage::MotorMessage(const uint8_t type, const uint8_t addr, ur::message_value val) :
    m_Delimiter(MOTOR_MSG_DELIM), m_Version(MOTOR_MSG_PROTOCOL_VER),
    m_Type(type), m_Addr(addr), m_Value(val), m_crc8(0)
{
  // TODO Auto-generated constructor stub

}

MotorMessage::~MotorMessage() {
  // TODO Auto-generated destructor stub
}

uint8_t MotorMessage::getDelimiter(void)
{
  return m_Delimiter;
}

uint8_t MotorMessage::getVersion(void)
{
  return m_Version;
}

uint8_t MotorMessage::getType(void)
{
  return m_Type;
}

uint8_t MotorMessage::getAddr(void)
{
  return m_Addr;
}

message_value MotorMessage::getValue(void)
{
  return m_Value;
}

uint8_t MotorMessage::getChecksum(void)
{
  return m_crc8;
}

void MotorMessage::btoh(void)
{
  m_Value.ui = be32toh(m_Value.ui);
}

void MotorMessage::htob(void)
{
  m_Value.ui = htobe32(m_Value.ui);
}

void MotorMessage::setChecksum(void)
{
  CRC crc;
  uint8_t data[9];
  ur::message_value v = m_Value;

  data[0] = m_Delimiter;
  data[1] = m_Version;
  data[2] = m_Type;
  data[3] = m_Addr;
  data[4] = v.varray[0];
  data[5] = v.varray[1];
  data[6] = v.varray[2];
  data[7] = v.varray[3];
  m_crc8 = crc.checksum(data, 1,7);
}

bool MotorMessage::checkChecksum(uint8_t ck)
{
  CRC crc;
  bool ret = false;

  if ( crc.checksum(this, 1, 7) == ck )
  {
    m_crc8 = ck;
    ret = true;
  }

  return (ret);
}

bool MotorMessage::checkChecksum(void)
{
  CRC crc;
  return ( crc.checksum(this, 1, 7) == m_crc8 ? true : false );
}

} /* namespace ur */
