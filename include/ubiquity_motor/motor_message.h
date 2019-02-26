/**
Copyright (c) 2016, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef MOTORMESSAGE_H
#define MOTORMESSAGE_H

#include <stdint.h>
#include <boost/array.hpp>
#include <vector>

typedef boost::array<uint8_t, 8> RawMotorMessage;


// It is CRITICAL that the values in the Registers enum remain in sync with Firmware register numbers.
// In fact once a register is defined and released, it should NOT be re-used at a later time for another purpose
//
class MotorMessage {
public:
    // Using default constructor and destructor
    MotorMessage(){};
    ~MotorMessage(){};

    // MessageTypes enum in class to avoid global namespace pollution
    enum MessageTypes {
        TYPE_READ = 0xA,
        TYPE_WRITE = 0xB,
        TYPE_RESPONSE = 0xC,
        TYPE_ERROR = 0xD
    };

    // Registers enum in class to avoid global namespace pollution
    enum Registers {
        REG_STOP_START = 0x00,
        REG_BRAKE_STOP = 0x01,
        REG_CRUISE_STOP = 0x02,

        REG_LEFT_PWM = 0x03,
        REG_RIGHT_PWM = 0x04,

        // skip 0x05 and 0x06

        REG_LEFT_SPEED_SET = 0x07,
        REG_RIGHT_SPEED_SET = 0x08,

        REG_LEFT_RAMP = 0x09,
        REG_RIGHT_RAMP = 0x0A,

        REG_LEFT_ODOM = 0x0B,
        REG_RIGHT_ODOM = 0x0C,

        REG_DEADMAN = 0x0D,

        REG_LEFT_CURRENT = 0x0E,
        REG_RIGHT_CURRENT = 0x0F,

        REG_ERROR_COUNT = 0x10,
        REG_5V_MAIN_ERROR = 0x11,
        REG_5V_AUX_ERROR = 0x12,
        REG_12V_MAIN_ERROR = 0x13,
        REG_12V_AUX_ERROR = 0x14,
        REG_5V_MAIN_OL = 0x15,
        REG_5V_AUX_OL = 0x16,
        REG_12V_MAIN_OL = 0x17,
        REG_12V_AUX_OL = 0x18,
        REG_LEFT_MOTOR_ERROR = 0x19,
        REG_RIGHT_MOTOR_ERROR = 0x1A,

        REG_PARAM_P = 0x1B,
        REG_PARAM_I = 0x1C,
        REG_PARAM_D = 0x1D,
        REG_PARAM_C = 0x1E,

        REG_LED_1 = 0x1F,
        REG_LED_2 = 0x20,

        REG_HARDWARE_VERSION = 0x21,    // The hardware revision time 10. This must be read from motor controler to initialize
        REG_FIRMWARE_VERSION = 0x22,

        REG_BATTERY_VOLTAGE = 0x23,
        REG_5V_MAIN_CURRENT = 0x24,
        REG_12V_MAIN_CURRENT = 0x25,
        REG_5V_AUX_CURRENT = 0x26,
        REG_12V_AUX_CURRENT = 0x27,

        REG_LEFT_SPEED_MEASURED = 0x28,
        REG_RIGHT_SPEED_MEASURED = 0x29,

        REG_BOTH_SPEED_SET = 0x2A,
        REG_MOVING_BUF_SIZE = 0x2B,

        REG_LIMIT_REACHED = 0x2C,
        REG_BOTH_ERROR = 0x2D,
        REG_BOTH_ODOM = 0x30,
        REG_ROBOT_ID = 0x31,	    // Indicates 0 for Magni controller or 1 for Loki robot controller as of late 2018

        REG_MOT_PWR_ACTIVE = 0x32,  // Readback register for host to know if motor controller thinks motor power is active
        REG_ESTOP_ENABLE   = 0x33,  // An override that may be set to 0 to force motor controller firmware to NOT use some ESTOP logic
        REG_PID_MAX_ERROR  = 0x34,  // A value that when non-zero enables motor firmware to limit harsh restarts in position after ESTOP release

        REG_MAX_SPEED_FWD  = 0x35,  // Max forward speed cap in a speed message 
        REG_MAX_SPEED_REV  = 0x36,  // Max reverse speed cap in a speed message  (This is negative)
        REG_MAX_PWM        = 0x37,  // The maximum wheel driver PWM value that will be used on the motor driver

        DEBUG_50 = 0x50,
        DEBUG_51 = 0x51,
        DEBUG_52 = 0x52,
        DEBUG_53 = 0x53,
        DEBUG_54 = 0x54,
        DEBUG_55 = 0x55,
        DEBUG_56 = 0x56,
        DEBUG_57 = 0x57,
        DEBUG_58 = 0x58
    };

    // Bitfield indicating which limits have been reached
    enum Limits {
        LIM_M1_PWM = 0x10,
        LIM_M2_PWM = 0x01,
        LIM_M1_INTEGRAL = 0x20,
        LIM_M2_INTEGRAL = 0x02
    };

    void setType(MotorMessage::MessageTypes type);
    MotorMessage::MessageTypes getType() const;

    void setRegister(MotorMessage::Registers reg);
    MotorMessage::Registers getRegister() const;

    void setData(int32_t data);
    int32_t getData() const;

    RawMotorMessage serialize() const;

    int deserialize(const RawMotorMessage &serialized);

    const static uint8_t delimeter = 0x7E;  // TODO: parameterize

private:
    // Type of message should be in MotorMessage::MessageTypes
    uint8_t type;
    // Register address should be in MotorMessage::Registers
    uint8_t register_addr;

    // 4 bytes of data, numbers should be in big endian format
    boost::array<uint8_t, 4> data;

    const static uint8_t protocol_version =
        0x03;  // Hard coded for now, should be parameterized

    const static uint8_t valid_types[];
    const static uint8_t valid_registers[];

    static int verifyType(uint8_t t);
    static int verifyRegister(uint8_t r);

    static uint8_t generateChecksum(const std::vector<uint8_t> &data);
    static uint8_t generateChecksum(const RawMotorMessage &data);
};

#endif
