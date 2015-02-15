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
			MOTOR_MSG_REQUEST_SPEED = 0x01,
			MOTOR_MSG_REQUEST_ACCELERATION = 0x02,
			MOTOR_MSG_ODOMETER = 0x03,
		};

		void setVel(int16_t motor0, int16_t motor1);
		void setAccel(int16_t motor0, int16_t motor1);

		int getOdom(int16_t *motor0, int16_t *motor1);

		std::vector<uint8_t> serialize();
		int deserialize(std::vector<uint8_t> &serialized);

	private:
		void setType(MotorCommand::CommandTypes t);

		uint8_t signBinary(std::vector<uint8_t> data);

		uint8_t crc8_;
		uint8_t type_;

		int16_t motor0_;
		int16_t motor1_;

};

#endif