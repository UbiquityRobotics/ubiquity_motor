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

#include <ubiquity_motor/motor_serial.h>

MotorSerial::MotorSerial(const std::string& port, uint32_t baud_rate){
	serial_thread = new boost::thread(&MotorSerial::SerialThread, this);
}

int MotorSerial::transmitCommand(MotorCommand command) {
	input_mtx_.lock();
	this->input.push(command);
	input_mtx_.unlock();
	return 0;
}

MotorCommand MotorSerial::receiveCommand() {
	MotorCommand mc;
	output_mtx_.lock();
	if(!this->output.empty()){
		mc = this->output.front();
		this->output.pop();
	}
	output_mtx_.unlock();
	return mc;
}

int MotorSerial::commandAvailable() {
	output_mtx_.lock();
	int out = !(this->output.empty());
	output_mtx_.unlock();
	return out;
}

int MotorSerial::inputAvailable() {
	input_mtx_.lock();
	int out = !(this->input.empty());
	input_mtx_.unlock();
	return out;
}

void MotorSerial::appendOutput(MotorCommand command){
	output_mtx_.lock();
	this->output.push(command);
	output_mtx_.unlock();
}

void MotorSerial::SerialThread(){
	//while(1){
	MotorCommand mc;

	//Test good message
	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0E};

	std::vector<uint8_t> in(arr, arr + sizeof(arr)/ sizeof(uint8_t));
	mc.deserialize(in);
	this->appendOutput(mc);
	//}
}