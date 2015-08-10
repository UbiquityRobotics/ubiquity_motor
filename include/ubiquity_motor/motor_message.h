/**
Copyright (c) 2015, Ubiquity Robotics
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
#include <vector>

class MotorMessage{

	public:
		//Using default constructor and destructor
		MotorMessage() {};
		~MotorMessage() {};

		// MessageTypes enum in class to avoid global namespace pollution
		enum MessageTypes {
			TYPE_READ = 0xAA,
			TYPE_WRITE = 0xBB,
			TYPE_RESPONSE = 0xCC
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

			REG_HARDWARE_VERSION = 0x21,
			REG_FIRMWARE_VERSION = 0x22,

			REG_BATTERY_VOLTAGE = 0x23,
			REG_5V_MAIN_CURRENT = 0x24,
			REG_12V_MAIN_CURRENT = 0x25,
			REG_5V_AUX_CURRENT = 0x26,
			REG_12V_AUX_CURRENT = 0x27,

			REG_LEFT_SPEED_MEASURED = 0x28,
			REG_RIGHT_SPEED_MEASURED = 0x29
		};

		void setType(MotorMessage::MessageTypes type);
		MotorMessage::MessageTypes getType();

		void setRegister(MotorMessage::Registers reg);
		MotorMessage::Registers getRegister();

		void setData(int32_t data);
		int32_t getData();

		std::vector<uint8_t> serialize();
		int deserialize(std::vector<uint8_t> &serialized);


	private:
		uint8_t type; // Type of message should be in MotorMessage::MessageTypes
		uint8_t register_addr; // Register address should be in MotorMessage::Registers
		uint8_t data[4]; // 4 bytes of data, numbers should be in big endian format

		const static uint8_t delimeter = 0x7E; // Hard coded for now, should be parameterized
		const static uint8_t protocol_version = 0x02; // Hard coded for now, should be parameterized

		const static uint8_t valid_types[]; 
		const static uint8_t valid_registers[];

		int verifyType(uint8_t t);
		int verifyRegister(uint8_t r);

		uint8_t generateChecksum(std::vector<uint8_t> data);
};

#endif