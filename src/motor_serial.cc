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
#include <ros/ros.h>
#include <serial/serial.h>

MotorSerial::MotorSerial(const std::string& port, uint32_t baud_rate, double loopRate){
	// Make sure baud rate is valid
	switch (baud_rate) {
		case 110 :
		case 300 :
		case 600 :
		case 1200 :
		case 2400 :
		case 4800 :
		case 9600 :
		case 14400 :
		case 19200 :
		case 28800 :
		case 38400 :
		case 56000 :
		case 57600 :
		case 115200 :
		case 128000 :
		case 153600 :
		case 230400 :
		case 256000 :
		case 460800 :
		case 921600 :
			this->_baud_rate = baud_rate;
			break;
		default :
			this->_baud_rate = 9600;
			break;
	}

	// TODO verify that port is valid 
	this->_port = port;

	motors = new serial::Serial(_port, _baud_rate, serial::Timeout::simpleTimeout(10000));

	serial_loop_rate = new ros::Rate(loopRate);

	serial_thread = new boost::thread(&MotorSerial::SerialThread, this);
}

MotorSerial::~MotorSerial(){
	serial_thread->interrupt();
	serial_thread->join();
	motors->close();
	delete motors;
	delete serial_thread;
	delete serial_loop_rate;
}

int MotorSerial::transmitCommand(MotorCommand command) {
	// Make sure to lock mutex before accessing the input fifo
	input_mtx_.lock();
	this->input.push(command); // add latest command to end of fifo
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

MotorCommand MotorSerial::getInputCommand() {
	MotorCommand mc;
	input_mtx_.lock();
	if(!this->input.empty()){
		mc = this->input.front();
		this->input.pop();
	}
	input_mtx_.unlock();
	return mc;
}

void MotorSerial::appendOutput(MotorCommand command){
	output_mtx_.lock();
	this->output.push(command);
	output_mtx_.unlock();
}

void MotorSerial::SerialThread(){
	ros::Rate serial_loop_rate(100);
	try {
		while(motors->isOpen() && ros::ok()){
			while(motors->available() >= 9){
				std::vector<uint8_t> in(0);
				motors->read(in, 9);
				// ROS_ERROR("Len:%d", (int) in.size());
				MotorCommand mc;
				if (mc.deserialize(in) == 0) {
					// ROS_ERROR("appendOutput");
					appendOutput(mc);
				}
			}

			if(inputAvailable()){
				std::vector<uint8_t> out(9);
				out = getInputCommand().serialize();
				// ROS_ERROR("out %x %x %x %x %x %x %x %x %x", 
				// 	out[0],
				// 	out[1],
				// 	out[2],
				// 	out[3],
				// 	out[4],
				// 	out[5],
				// 	out[6],
				// 	out[7],
				// 	out[8]);
				motors->write(out);
			}

			// boost::posix_time::milliseconds loopDelay(10);
			// boost::this_thread::sleep(loopDelay);
			serial_loop_rate.sleep();
		}

	}
	catch (const boost::thread_interrupted& e) {
		ROS_ERROR("boost::thread_interrupted");
		motors->close();
	}
	catch (const serial::IOException& e) {
		ROS_ERROR("%s", e.what());
	}
	catch (const serial::PortNotOpenedException& e) {
		ROS_ERROR("%s", e.what());
	}
	catch (...) {
		ROS_ERROR("Unknown Error");
		throw;
	}
}