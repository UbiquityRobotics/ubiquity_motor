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

#ifndef MOTORSERIAL_H
#define MOTORSERIAL_H

#include <ubiquity_motor/motor_message.h>
#include <serial/serial.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <queue>

#include <gtest/gtest_prod.h>

class MotorSerial
{
	public:
		MotorSerial(const std::string& port = "/dev/ttyUSB0" , uint32_t baud_rate = 9600, double loopRate = 100);
		~MotorSerial();

		int transmitCommand(MotorMessage command);
		int transmitCommands(const std::vector<MotorMessage> &commands);
		
		MotorMessage receiveCommand();
		int commandAvailable();
		
		MotorSerial( const MotorSerial& other ); // non construction-copyable
		MotorSerial& operator=( const MotorSerial& ); // non copyable

	private:

		serial::Serial* motors;
		
		std::string _port;
		uint32_t _baud_rate;

		// bool to check for input to avoid unnecessary locking                
		bool have_input;
		//locking mutex for the input queue
		boost::mutex input_mtx_;
		// queue for messages that are to be transmitted
		std::queue<MotorMessage> input; 
		
		// bool to check for output to avoid unnecessary locking                
		bool have_output;
		//locking mutex for the output queue
		boost::mutex output_mtx_;
		//queue for messages that have been received
		std::queue<MotorMessage> output; 

		boost::thread* serial_thread;
		ros::Rate* serial_loop_rate;

		int inputAvailable();
		MotorMessage getInputCommand();
		void appendOutput(MotorMessage command);

		// Thread that has manages the serial port 
		void SerialThread();

		FRIEND_TEST(MotorSerialTests, invalidBaudDefaults);
		FRIEND_TEST(MotorSerialTests, serialClosedOnInterupt);
		FRIEND_TEST(MotorSerialTests, readQueuesDequeues);
		FRIEND_TEST(MotorSerialTests, writeQueues);
		FRIEND_TEST(MotorSerialTests, writeQueuesDequeues);
		FRIEND_TEST(MotorSerialTests, writeMultipleQueues);
		FRIEND_TEST(MotorSerialTests, writeMultipleQueuesDequeues);
};

#endif
