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

#include <ros/ros.h>
#include <serial/serial.h>
#include <ubiquity_motor/motor_message.h>
#include <ubiquity_motor/shared_queue.h>
#include <boost/thread.hpp>
#include <queue>

#include <gtest/gtest_prod.h>

class MotorSerial {
public:
    MotorSerial(const std::string& port = "/dev/ttyUSB0",
                uint32_t baud_rate = 9600);
    ~MotorSerial();

    int transmitCommand(MotorMessage command);
    int transmitCommands(const std::vector<MotorMessage>& commands);

    MotorMessage receiveCommand();
    int commandAvailable();
    void closePort();
    bool openPort();

    MotorSerial(MotorSerial const&) = delete;
    MotorSerial& operator=(MotorSerial const&) = delete;

private:
    serial::Serial motors;

    shared_queue<MotorMessage> output;

    boost::thread serial_thread;

    void appendOutput(MotorMessage command);

    int serial_errors;
    int error_threshold;

    // Thread that has manages serial reads
    void SerialThread();

    FRIEND_TEST(MotorSerialTests, serialClosedOnInterupt);
};

#endif
