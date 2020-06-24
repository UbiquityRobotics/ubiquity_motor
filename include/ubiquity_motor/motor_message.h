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

// To support enhanced firmware we identify the fw version for new registers 
// The idea is we do not want to make firmware message requests till a feature is supported
#define MIN_FW_RECOMMENDED        32
#define MIN_FW_MOT_POW_ACTIVE     32  
#define MIN_FW_ESTOP_SUPPORT      32  
#define MIN_FW_HW_VERSION_SET     32  
#define MIN_FW_MAX_SPEED_AND_PWM  34  
#define MIN_FW_ENC_6_STATE        35
#define MIN_FW_FIRMWARE_DATE      35
#define MIN_FW_DEADZONE           35
#define MIN_FW_PID_V_TERM         35
#define MIN_FW_SELFTESTS_REV1     36
#define MIN_FW_BATTERY_WARN       36
#define MIN_FW_PID_CONTROL_REV1   37
#define MIN_FW_WHEEL_TYPE_THIN    37
#define MIN_FW_SYSTEM_EVENTS      37
#define MIN_FW_OPTION_SWITCH      37
#define MIN_FW_PID_RDY_REGS       37

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
        REG_STOP_START = 0x00,          // Deprecated
        REG_BRAKE_STOP = 0x01,
        REG_SYSTEM_EVENTS = 0x02,	// This register can set event bits such as a power on has happened

        REG_LEFT_PWM = 0x03,            // ReadOnly: M1 motor PWM setting
        REG_RIGHT_PWM = 0x04,           // ReadOnly: M2 motor PWM setting

        // skip 0x05 and 0x06

        REG_LEFT_SPEED_SET = 0x07,      // Deprecated
        REG_RIGHT_SPEED_SET = 0x08,     // Deprecated

        REG_LEFT_RAMP = 0x09,           // Deprecated
        REG_RIGHT_RAMP = 0x0A,          // Deprecated

        REG_LEFT_ODOM = 0x0B,           // Deprecated
        REG_RIGHT_ODOM = 0x0C,          // Deprecated

        REG_DEADMAN = 0x0D,             // Deadman timer (VERY TRICKY VALUE, USE WITH CARE!)

        REG_LEFT_CURRENT = 0x0E,        // Electrical Current readback for M1 motor
        REG_RIGHT_CURRENT = 0x0F,       // Electrical Current readback for M2 motor

        REG_WHEEL_TYPE = 0x10,          // The type of wheel-motor in use
        REG_5V_MAIN_ERROR = 0x11,       // Deprecated
        REG_OPTION_SWITCH = 0x12,       // Setting of MCB option jumpers and rev
        REG_PWM_OVERRIDE = 0x13,
        REG_PID_CONTROL = 0x14,         // A write to this register controls Pid param setup and calculations

        // The PID parameters below are setup without impacting the PID loop then all made active at once
        REG_PARAM_V_RDY = 0x15,         // PID loop V factor being made ready but not in use yet
        REG_PARAM_P_RDY = 0x16,         // PID loop P factor being made ready but not in use yet
        REG_PARAM_I_RDY = 0x17,         // PID loop I factor being made ready but not in use yet
        REG_PARAM_D_RDY = 0x18,         // PID loop D factor being made ready but not in use yet
        REG_PARAM_C_RDY = 0x19,         // PID loop C factor being made ready but not in use yet

        // The PID parameters below are active values and they get set one register at a time
        REG_PARAM_V = 0x1A,             // PID loop V factor
        REG_PARAM_P = 0x1B,             // PID loop P factor
        REG_PARAM_I = 0x1C,             // PID loop I factor
        REG_PARAM_D = 0x1D,             // PID loop D factor
        REG_PARAM_C = 0x1E,             // PID loop C factor

        REG_LED_1 = 0x1F,
        REG_LED_2 = 0x20,

        REG_HARDWARE_VERSION = 0x21,    // Hardware version BUT is settable from HOST!
        REG_FIRMWARE_VERSION = 0x22,    // ReadOnly: Firmware version reported to HOST

        REG_BATTERY_VOLTAGE = 0x23,     // Electrical voltage of main battery
        REG_5V_MAIN_CURRENT = 0x24,     // Electrical current in use on  5 Volt Main supply
        REG_MAINV_TPOINT = 0x25,        // TP. Indicates 5V, 12V main power and used for selftests (MCB 5.2)
        REG_12V_MAIN_CURRENT = 0x26,    // Electrical current in use on 12 Volt Main supply
        REG_AUXV_TPOINT = 0x27,         // TP. Indicates 5V, 12V aux power and used for selftests

        REG_BATT_VOL_LOW = 0x28,        // Battery divider ADC counts when battery is low
        REG_VBUF_SIZ = 0x29,            // Velocity averaging buffer length

        REG_BOTH_SPEED_SET = 0x2A,      // Speed setting: Two signed 16-bit speeds set by user. Upper=left
        REG_MOVING_BUF_SIZE = 0x2B,     // PID loop moving average count. This caps at 100

        REG_LIMIT_REACHED = 0x2C,       // Holds bits showing if some sort of limit was hit
        REG_BOTH_ERROR = 0x2D,          // Two 16-bit signed error values for PID loop current Error value
        REG_BOTH_ODOM = 0x30,           // Two 16-bit signed ODOM values
        REG_ROBOT_ID = 0x31,	        // Type of robot. 0 for Magni or 1 for Loki 

        REG_MOT_PWR_ACTIVE = 0x32,      // Indicates if motor power is active (ESTOP not active)
        REG_ESTOP_ENABLE   = 0x33,      // Normally non 0. If 0 forces motor controller firmware to NOT all ESTOP logic
        REG_PID_MAX_ERROR  = 0x34,      // Used in pre rev 5.0 board crude ESTOP logic only. NOT rev 5

        REG_MAX_SPEED_FWD  = 0x35,      // Caps the max forward speed settable by user
        REG_MAX_SPEED_REV  = 0x36,      // Caps the max reverse speed settable by user
        REG_MAX_PWM        = 0x37,      // Caps max PWM value that will be used due to PID loop

        REG_HW_OPTIONS     = 0x38,      // ReadOnly: Indicates options such as 3 or 6 state enc or motor type
        REG_DEADZONE       = 0x39,      // When non-zero enables speed deadzone at zero speed
        REG_FIRMWARE_DATE  = 0x3a,      // ReadOnly: Hex encoded daycode such as 20190703 for July 3 2019
        REG_STEST_REQUEST  = 0x3b,      // Set to non-zero to request tests.  Cleared after tests done
        REG_STEST_RESULTS  = 0x3c,      // Last Selftest result bits are left in this register. 0=ok

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

    // PID Control values used in special register PID_CONTROL
    enum PidControlActions {
        PID_CTRL_RESET               = 1,
        PID_CTRL_PWM_OVERRIDE        = 2
    };

    // Bitfield values for hardware options enabled in the firmware
    enum HwOptions {
        OPT_ENC_6_STATE = 0x01,
        OPT_WHEEL_TYPE_THIN = 0x02,    // As of rev v37 we support a 'thin' wheel type, gearless
        OPT_WHEEL_TYPE_STANDARD = 0    // Default original, standard wheels
    };

    // Bitfield values indicating selftest involved. Most are used in test request and results.  
    // A set bit in STEST_RESULTS indicates failure once tests complete
    // STEST_IN_PROGRESS appears in RESULTS till tests are done then it goes to zero
    #define STEST_STATE_SHIFT  24
    enum StestRequestAndResults {
        STEST_IDLE           = 0x00000000,  // Indicates test is done or not requested yet
        STEST_MAINV          = 0x00000001,  // Main voltages not in limits for 5V_MAIN or perhaps 12V_MAIN
        STEST_AUXV           = 0x00000002,  // Aux  voltages not in limits for 5V_AUX  or perhaps 12V_AUX
        STEST_BATTERY        = 0x00000004,  // Indicates battery state not right or to do selftest
        STEST_BATTERY_LOW    = 0x00000008,  // Indicates battery is low (required for low check)
        STEST_MOT_PWR        = 0x00000080,  // Indicates motor power 
        STEST_MOTOR_AND_ENCS = 0x00000100,  // Move motors back and forth (Will set ENC bits if needed)
        STEST_MOTOR_POWER_ON = 0x00000400,  // If motor power is off this gets set so test with estop off
        STEST_M1_CURNT       = 0x00001000,  // Motor 1 current test
        STEST_M2_CURNT       = 0x00002000,  // Motor 2 current test
        STEST_MOT_M1_ENC     = 0x00010000,  // Left enc not responding to movement  (in MOT_MOVE test)
        STEST_MOT_M2_ENC     = 0x00020000,  // Right enc not responding to movement (in MOT_MOVE test)
        STEST_IN_PROGRESS    = 0x00800000,  // Indicates selftest is in progress
        STEST_STATE          = (0xF << STEST_STATE_SHIFT)   // Some tests may require a state 
    };

    // System Event Bits are set for things such as a power on has happened so host can see that
    // The host can read this then clear it.
    enum SystemEvents {
        SYS_EVENT_POWERON    = 0x00000001   // Indicates MCB has had a reset
    };

    // Bitfield indicating which limits have been reached
    enum Limits {
        LIM_M1_PWM = 0x10,
        LIM_M2_PWM = 0x01,
        LIM_M1_INTEGRAL = 0x20,
        LIM_M2_INTEGRAL = 0x02,
        LIM_M1_MAX_SPD  = 0x40,
        LIM_M2_MAX_SPD  = 0x4,
        LIM_PARAM_LIMIT = 0x80
    };

    // State bits for motor power
    const static int32_t MOT_POW_ACTIVE = 0x0001;

    void setType(MotorMessage::MessageTypes type);
    MotorMessage::MessageTypes getType() const;

    void setRegister(MotorMessage::Registers reg);
    MotorMessage::Registers getRegister() const;

    void setData(int32_t data);
    int32_t getData() const;

    RawMotorMessage serialize() const;


    // Error Codes that can be returned by deserializaion
    enum ErrorCodes {
        ERR_NONE             = 0, // Success code 
        ERR_DELIMITER        = 1, 
        ERR_WRONG_PROTOCOL   = 2, 
        ERR_BAD_CHECKSUM     = 3, 
        ERR_BAD_TYPE         = 4, 
        ERR_UNKNOWN_REGISTER = 5 
    };

    MotorMessage::ErrorCodes deserialize(const RawMotorMessage &serialized);

    const static uint8_t delimeter = 0x7E;

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
