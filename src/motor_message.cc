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

#include <ubiquity_motor/motor_message.h>
// #include <ubiquity_motor/motor_message_registers.h>
#include <ros/console.h>
#include <numeric>

uint8_t const MotorMessage::valid_types[] = {TYPE_READ, TYPE_WRITE,
                                             TYPE_RESPONSE, TYPE_ERROR};

uint8_t const MotorMessage::valid_registers[] = {REG_STOP_START,
                                                 REG_BRAKE_STOP,
                                                 REG_SYSTEM_EVENTS,
                                                 REG_LEFT_PWM,
                                                 REG_RIGHT_PWM,
                                                 REG_PWM_BOTH_WHLS,
                                                 REG_TINT_BOTH_WHLS,
                                                 REG_09,
                                                 REG_DRIVE_TYPE,
                                                 REG_WHEEL_NULL_ERR,
                                                 REG_WHEEL_DIR,
                                                 REG_DEADMAN,
                                                 REG_LEFT_CURRENT,
                                                 REG_RIGHT_CURRENT,
                                                 REG_WHEEL_TYPE,
                                                 REG_PID_ERROR_CAP,
                                                 REG_OPTION_SWITCH,
                                                 REG_PWM_OVERRIDE,
                                                 REG_PID_CONTROL,
                                                 REG_PARAM_V_RDY,
                                                 REG_PARAM_P_RDY,
                                                 REG_PARAM_I_RDY,
                                                 REG_PARAM_D_RDY,
                                                 REG_PARAM_C_RDY,
                                                 REG_PARAM_V,
                                                 REG_PARAM_P,
                                                 REG_PARAM_I,
                                                 REG_PARAM_D,
                                                 REG_PARAM_C,
                                                 REG_LED_1,
                                                 REG_LED_2,
                                                 REG_HARDWARE_VERSION,
                                                 REG_FIRMWARE_VERSION,
                                                 REG_BATTERY_VOLTAGE,
                                                 REG_5V_MAIN_CURRENT,
                                                 REG_MAINV_TPOINT,
                                                 REG_12V_MAIN_CURRENT,
                                                 REG_AUXV_TPOINT,
                                                 REG_BATT_VOL_LOW,
                                                 REG_VBUF_SIZ,
                                                 REG_BOTH_SPEED_SET,
                                                 REG_MOVING_BUF_SIZE,
                                                 REG_LIMIT_REACHED,
                                                 REG_BOTH_ERROR,
                                                 REG_BOTH_ODOM,
                                                 REG_ROBOT_ID,
                                                 REG_MOT_PWR_ACTIVE,
                                                 REG_ESTOP_ENABLE,
                                                 REG_PID_MAX_ERROR,
                                                 REG_MAX_SPEED_FWD,
                                                 REG_MAX_SPEED_REV,
                                                 REG_MAX_PWM,
                                                 REG_HW_OPTIONS,
                                                 REG_DEADZONE,
                                                 REG_FIRMWARE_DATE,
                                                 REG_STEST_REQUEST,
                                                 REG_STEST_RESULTS,
                                                 DEBUG_50,
                                                 DEBUG_51,
                                                 DEBUG_52,
                                                 DEBUG_53,
                                                 DEBUG_54,
                                                 DEBUG_55,
                                                 DEBUG_56,
                                                 DEBUG_57,
                                                 DEBUG_58};

void MotorMessage::setType(MotorMessage::MessageTypes type) {
    if (verifyType(type)) {
        this->type = type;
    }
}

MotorMessage::MessageTypes MotorMessage::getType() const {
    return static_cast<MotorMessage::MessageTypes>(this->type);
}

void MotorMessage::setRegister(MotorMessage::Registers reg) {
    if (verifyRegister(reg)) {
        this->register_addr = reg;
    }
}

MotorMessage::Registers MotorMessage::getRegister() const {
    return static_cast<MotorMessage::Registers>(this->register_addr);
}

void MotorMessage::setData(int32_t data) {
    // Spilt 32 bit data (system byte order) into 4 8bit elements in big endian
    // (network byte order)
    this->data[3] = (data >> 0) & 0xFF;
    this->data[2] = (data >> 8) & 0xFF;
    this->data[1] = (data >> 16) & 0xFF;
    this->data[0] = (data >> 24) & 0xFF;
}

int32_t MotorMessage::getData() const {
    // Take big endian (network byte order) elements and return 32 bit int
    return (this->data[0] << 24) | (this->data[1] << 16) |
           (this->data[2] << 8) | (this->data[3] << 0);
}

RawMotorMessage MotorMessage::serialize() const {
    RawMotorMessage out;
    out[0] = delimeter;
    out[1] = (protocol_version << 4) | type;
    out[2] = register_addr;
    std::copy(data.begin(), data.end(), out.begin() + 3);
    out[7] = generateChecksum(out);
    return out;
}

MotorMessage::ErrorCodes MotorMessage::deserialize(const RawMotorMessage &serialized) {
    if (serialized[0] == delimeter) {
        if ((serialized[1] & 0xF0) == (protocol_version << 4)) {
            if (generateChecksum(serialized) == serialized[7]) {
                if (verifyType(serialized[1] & 0x0F)) {
                    if (verifyRegister(serialized[2])) {
                        this->type = serialized[1] & 0x0F;
                        this->register_addr = serialized[2];
                        std::copy(serialized.begin() + 3,
                                  serialized.begin() + 7, data.begin());
                        return MotorMessage::ERR_NONE;
                    } else
                        return MotorMessage::ERR_UNKNOWN_REGISTER;
                } else
                    return MotorMessage::ERR_BAD_TYPE;;
            } else
                return MotorMessage::ERR_BAD_CHECKSUM;
        } else
            return MotorMessage::ERR_WRONG_PROTOCOL;
    } else
        return MotorMessage::ERR_DELIMITER;

    // ERROR codes returned are defined in MotorMessage class
    // First char not delimiter
    // wrong protocol version
    // bad checksum
    // bad type
    // bad register
}

int MotorMessage::verifyType(uint8_t t) {
    // Return 1 good
    // Return 0 for bad
    for (size_t i = 0; i < sizeof(valid_types) / sizeof(valid_types[0]); ++i) {
        if (t == valid_types[i]) return 1;
    }
    return 0;
}

int MotorMessage::verifyRegister(uint8_t r) {
    // Return 1 good
    // Return 0 for bad
    for (size_t i = 0; i < sizeof(valid_registers) / sizeof(valid_registers[0]);
         ++i) {
        if (r == valid_registers[i]) return 1;
    }
    return 0;
}

uint8_t MotorMessage::generateChecksum(const RawMotorMessage &data) {
    int sum = std::accumulate(data.begin() + 1, data.end() - 1, 0);

    if (sum > 0xFF) {
        int tmp;
        tmp = sum >> 8;
        tmp = tmp << 8;
        return 0xFF - (sum - tmp);
    } else {
        return 0xFF - sum;
    }
}
