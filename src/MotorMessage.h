/*
 * MotorMessage.h
 *
 *  Created on: Apr 28, 2015
 *      Author: kurt
 */

#ifndef MOTORMESSAGE_H_
#define MOTORMESSAGE_H_

namespace ur {
const uint32_t MOTOR_LEFT = 0x00;
const uint32_t MOTOR_RIGHT = 0x01;

// Start of message delimiter (NOT ESCAPED IN MSG DATA)
const uint8_t MOTOR_MSG_DELIM = 0x7E;
const uint8_t MOTOR_MSG_PROTOCOL_VER = 0x02;

const uint8_t MOTOR_MSG_TYPE_READ = 0xAA;
const uint8_t MOTOR_MSG_TYPE_WRITE = 0xBB;
const uint8_t MOTOR_MSG_TYPE_RESPONSE = 0xCC;

// Register Addresses
// Stop or Start Register
const uint8_t REG_ADDR_ON_OFF = 0x00;
// Hard stop the motor
const uint8_t REG_ADDR_BRAKE = 0x01;
// Cruise to stop
const uint8_t REG_ADDR_HALT = 0x02;
// Left Motor PWM
const uint8_t REG_ADDR_LEFT_PWM = 0x03;
// Right Motor PWM
const uint8_t REG_ADDR_RIGHT_PWM = 0x04;
// Left Motor Direction
const uint8_t REG_ADDR_LEFT_DIRECTION = 0x05;
// Right Motor Direction
const uint8_t REG_ADDR_RIGHT_DIRECTION = 0x06;
// Left Motor Speed
const uint8_t REG_ADDR_SET_LEFT_SPEED = 0x07;
// Right Motor Speed
const uint8_t REG_ADDR_SET_RIGHT_SPEED = 0x08;
// Left Motor Ramp (ms till full power)
const uint8_t REG_ADDR_LEFT_RAMP = 0x09;
// Right Motor Ramp (ms till full power)
const uint8_t REG_ADDR_RIGHT_RAMP = 0x0A;
// Left Motor Tics (Clears on reading, Signed)
const uint8_t REG_ADDR_LEFT_TICS = 0x0B;
// Right Motor Tics (Clears on reading, Signed)
const uint8_t REG_ADDR_RIGHT_TICS = 0x0C;
// Deadman Timer (in ms)
const uint8_t REG_ADDR_DEADMAN_TIMER = 0x0D;
// Left Motor Current Sense
const uint8_t REG_ADDR_LEFT_CURRENT_SENSE = 0x0E;
// Right Motor Current Sense
const uint8_t REG_ADDR_RIGHT_RIGHT_CURRENT_SENSE = 0x0F;
// Error Count
const uint8_t REG_ADDR_ERROR_COUNT = 0x10;
// 5V Main Error
const uint8_t REG_ADDR_5V_MAIN_ERR = 0x11;
// 5V Main Error
const uint8_t REG_ADDR_5V_AUX_ERR = 0x12;
// 12V Main Error
const uint8_t REG_ADDR_12V_MAIN_ERR = 0x13;
// 12V Main Error
const uint8_t REG_ADDR_12V_AUX_ERR = 0x14;
// 5V Main Overload
const uint8_t REG_ADDR_5V_MAIN_OVERLOAD = 0x15;
// 5V Main Overload
const uint8_t REG_ADDR_5V_AUX_OVERLOAD = 0x16;
// 12V Main Overload
const uint8_t REG_ADDR_12V_MAIN_OVERLOAD = 0x17;
// 12V Main Overload
const uint8_t REG_ADDR_12V_AUX_OVERLOAD = 0x18;
// Left Motor Error
const uint8_t REG_ADDR_LEFT_MOTOR_ERR = 0x19;
// Right Motor Error
const uint8_t REG_ADDR_RIGHT_MOTOR_ERR = 0x1A;
// PID Controller Proportional Value
const uint8_t REG_ADDR_PID_PROPORTIONAL = 0x1B;
// PID Controller Integral Value
const uint8_t REG_ADDR_PID_INTEGRAL = 0x1C;
// PID Controller Derivative Value
const uint8_t REG_ADDR_PID_DERIVATIVE = 0x1D;
// PID Controller Common Denominator Value
const uint8_t REG_ADDR_PID_DENOMINATOR = 0x1E;
// Turn on/off LED number 1
const uint8_t REG_ADDR_LED1 = 0x1F;
// Turn on/off LED number 2
const uint8_t REG_ADDR_LED2 = 0x20;
// Read Hardware Version Number
const uint8_t REG_ADDR_HDWR_VER = 0x21;
// Read Firmware Version Number
const uint8_t REG_ADDR_FMWR_VER = 0x22;
// Read Battery Voltage
const uint8_t REG_ADDR_BATTERY_VOLTAGE = 0x23;
// Read 5V Main Current
const uint8_t REG_ADDR_5V_MAIN_CURRENT = 0x24;
// Read 12V Main Current
const uint8_t REG_ADDR_12V_MAIN_CURRENT = 0x25;
// Read 5V Auxiliary Current
const uint8_t REG_ADDR_5V_AUX_CURRENT = 0x26;
// Read 12V Auxiliary Current
const uint8_t REG_ADDR_12V_AUX_CURRENT = 0x27;
// Left Motor Error
const uint8_t REG_ADDR_LEFT_MOTOR_SPEED = 0x28;
// Right Motor Error
const uint8_t REG_ADDR_RIGHT_MOTOR_SPEED = 0x29;
// Reserved Register #1


const unsigned int MOTOR_VAL_START = 0x00;
const unsigned int MOTOR_VAL_STOP = 0xFF;
const unsigned int MOTOR_VAL_BRAKE_ON = 0xFF;
const unsigned int MOTOR_VAL_BRAKE_OFF = 0x00;
const unsigned int MOTOR_VAL_HALT_ON = 0x00;
const unsigned int MOTOR_VAL_HALT_OFF = 0xFF;
const unsigned int MOTOR_VAL_DEBUG_LED_ON = 0x01;
const unsigned int MOTOR_VAL_DEBUG_LED_OFF = 0x00;

enum motor_error_id
{
  LEFT_MOTOR_ERROR = 0x01,
  RIGHT_MOTOR_ERROR = 0x02,
  FIVE_VOLT_MAIN_ERROR = 0x03,
  FIVE_VOLT_MAIN_OVERHEAT = 0x04,
  FIVE_VOLT_MAIN_OVERLOAD = 0x05,
  TWELVE_VOLT_MAIN_ERROR = 0x06,
  TWELVE_VOLT_MAIN_OVERHEAT = 0x07,
  TWELVE_VOLT_MAIN_OVERLOAD = 0x08,
  FIVE_VOLT_AUX_ERROR = 0x09,
  FIVE_VOLT_AUX_OVERHEAT = 0x0A,
  FIVE_VOLT_AUX_OVERLOAD = 0x0B,
  TWELVE_VOLT_AUX_ERROR = 0x0C,
  TWELVE_VOLT_AUX_OVERHEAT = 0x0D,
  TWELVE_VOLT_AUX_OVERLOAD = 0x0E,
  REBOOT_5V_MAIN = 0x0F,
  REBOOT_5V_AUX = 0x10,
  REBOOT_12V_MAIN = 0x11,
  REBOOT_12V_AUX = 0x12
};

union message_value
{
  int32_t i;
  uint32_t ui;
  uint8_t varray[4];
};

struct raw_message_typed {
  uint8_t delimiter;
  uint8_t version;
  uint8_t type;
  uint8_t addr;
  message_value val;
  uint8_t crc8;
};

union raw_message {
  raw_message_typed msg_typed;
  uint8_t msg_array[9];
};

class MotorMessage {
private:
    uint8_t m_Delimiter;       // Delimiter (0x7E), Protocol Version (0x01)
    uint8_t m_Version;
    uint8_t m_Type;            // Message Type - READ/WRITE/RESPONSE
    uint8_t m_Addr;            // Register Address for the message
    ur::message_value m_Value;     // Value
    uint8_t m_crc8;            // CRC-8 of the messsage

public:
  MotorMessage(const uint8_t type, const uint8_t addr, ur::message_value val);
  virtual ~MotorMessage();

  uint8_t getDelimiter(void);
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
};

} /* namespace ur */

#endif /* MOTORMESSAGE_H_ */
