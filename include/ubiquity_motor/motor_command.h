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

#ifndef MOTORCOMMAND_H
#define MOTORCOMMAND_H

#include <stdint.h>
#include <vector>

class MotorCommand{

	public:
		MotorCommand() {};
		~MotorCommand() {};

		enum CommandTypes {
			READ = 0xAA,
			WRITE = 0xBB,
			RESPONSE = 0xCC
		};

		enum Registers {
			STOP_START = 0x00,
			BRAKE_STOP = 0x01,
			CRUISE_STOP = 0x02,

			LEFT_PWM = 0x03,
			RIGHT_PWM = 0x04,

			// skip 0x05 and 0x06

			LEFT_SPEED_SET = 0x07,
			RIGHT_SPEED_SET = 0x08,

			LEFT_RAMP = 0x09,
			RIGHT_RAMP = 0x0A,

			LEFT_ODOM = 0x0B,
			RIGHT_ODOM = 0x0C,

			DEADMAN = 0x0D,

			LEFT_CURRENT = 0x0E,
			RIGHT_CURRENT = 0x0F,

			ERROR_COUNT = 0x10,
			REG_5V_MAIN_ERROR = 0x11,
			REG_5V_AUX_ERROR = 0x12,
			REG_12V_MAIN_ERROR = 0x13,
			REG_12V_AUX_ERROR = 0x14,
			REG_5V_MAIN_OL = 0x15,
			REG_5V_AUX_OL = 0x16,
			REG_12V_MAIN_OL = 0x17,
			REG_12V_AUX_OL = 0x18,
			LEFT_MOTOR_ERROR = 0x19,
			RIGHT_MOTOR_ERROR = 0x1A,

			PARAM_P = 0x1B,
			PARAM_I = 0x1C,
			PARAM_D = 0x1D,
			PARAM_C = 0x1E,

			LED_1 = 0x1F,
			LED_2 = 0x20,

			HARDWARE_VERSION = 0x21,
			FIRMWARE_VERSION = 0x22,

			BATTERY_VOLTAGE = 0x23,
			REG_5V_MAIN_CURRENT = 0x24,
			REG_12V_MAIN_CURRENT = 0x25,
			REG_5V_AUX_CURRENT = 0x26,
			REG_12V_AUX_CURRENT = 0x27,

			LEFT_SPEED_MEASURED = 0x28,
			RIGHT_SPEED_MEASURED = 0x29
		};

		void setType(MotorCommand::CommandTypes type);
		MotorCommand::CommandTypes getType();

		void setRegister(MotorCommand::Registers reg);
		MotorCommand::Registers getRegister();

		void setData(int32_t data);
		int32_t getData();

		std::vector<uint8_t> serialize();
		int deserialize(std::vector<uint8_t> &serialized);

	private:
		uint8_t type;
		uint8_t register_addr;
		uint8_t data[4];
		uint8_t checksum;

		uint8_t generateChecksum(std::vector<uint8_t> data);
};

#endif