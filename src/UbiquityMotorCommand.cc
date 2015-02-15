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

#include <ubiquity_motor/UbiquityMotorCommand.h>
#include <cstring>
#include "crc8.h"

void UbiquityMotorCommand::setVel(int16_t motor0, int16_t motor1){
	motor0_ = motor0;
	motor1_ = motor1;
	setType(MOTOR_MSG_REQUEST_SPEED);
}

void UbiquityMotorCommand::setAccel(int16_t motor0, int16_t motor1){
	UbiquityMotorCommand::motor0_ = motor0;
	UbiquityMotorCommand::motor1_ = motor1;
	setType(MOTOR_MSG_REQUEST_ACCELERATION);
}

int UbiquityMotorCommand::getOdom(int16_t *motor0, int16_t *motor1){
	if(type_ == MOTOR_MSG_ODOMETER){
		motor0 = &motor0_;
		motor1 = &motor1_;
		return 0;
	}
	else
		return 1;
}


std::vector<uint8_t> UbiquityMotorCommand::serialize(){
	std::vector<uint8_t> data, mot0, mot1;

	//Resize all vectors to required sizes
	data.resize(8);
	mot0.resize(2);
	mot1.resize(2);

	data[0] = 0xff; // Sync Byte 0
	data[1] = 0xfe; // Sync Byte 1

	data[3]	= type_; // message type

	std::memcpy(mot0.data(), &motor0_, sizeof(motor0_));
	data.insert(data.end(), mot0.begin(), mot0.end());

	std::memcpy(mot1.data(), &motor1_, sizeof(motor1_));
	data.insert(data.end(), mot0.begin(), mot0.end());

	signBinary(data); // sign the message with crc8 before serializing
	data[2] = crc8_; // crc8

	return data;
}

int UbiquityMotorCommand::deserialize(std::vector<uint8_t> &serialized){
	if (serialized[0] == 0xff && serialized[1] == 0xfe){
		crc8_ = serialized[2];
		if(crc8_ == signBinary(serialized)){
			type_ = serialized[3];
			motor0_ = (serialized[5] << 8) | (serialized[6]);
			motor1_ = (serialized[7] << 8) | (serialized[8]);
		} 
		else
			return 1;
	}
	else
		return 1;
}

void UbiquityMotorCommand::setType(UbiquityMotorCommand::CommandTypes t) {
	UbiquityMotorCommand::type_ = t;
}

uint8_t UbiquityMotorCommand::signBinary(std::vector<uint8_t> data) {
	uint8_t arr[5];
	std::copy(data.begin()+3, data.end(), arr);
	return crc8(arr,5);
}
