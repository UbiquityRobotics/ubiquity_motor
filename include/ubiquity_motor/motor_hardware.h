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

#ifndef MOTORHARDWARE_H
#define MOTORHARDWARE_H

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "sensor_msgs/BatteryState.h"

#include <ubiquity_motor/motor_parmeters.h>
#include <ubiquity_motor/motor_serial.h>

#include <gtest/gtest_prod.h>

class MotorHardware : public hardware_interface::RobotHW {
public:
    MotorHardware(ros::NodeHandle nh, CommsParams serial_params,
                  FirmwareParams firmware_params);
    virtual ~MotorHardware();
    void readInputs();
    void writeSpeeds();
    void requestVersion();
    void setParams(FirmwareParams firmware_params);
    void sendParams();
    void setDeadmanTimer(int32_t deadman);
    void setDebugLeds(bool led1, bool led2);

    int firmware_version;

private:
    void _addOdometryRequest(std::vector<MotorMessage>& commands) const;
    void _addVelocityRequest(std::vector<MotorMessage>& commands) const;

    int16_t calculateTicsFromRadians(double radians) const;
    double calculateRadiansFromTics(int16_t tics) const;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    FirmwareParams pid_params;
    FirmwareParams prev_pid_params;

    int32_t deadman_timer;

    int32_t sendPid_count;

    struct Joint {
        double position;
        double velocity;
        double effort;
        double velocity_command;

        Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
    } joints_[2];

    ros::Publisher leftError;
    ros::Publisher rightError;

    ros::Publisher battery_state;

    MotorSerial* motor_serial_;

    FRIEND_TEST(MotorHardwareTests, nonZeroWriteSpeedsOutputs);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPosition);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPositionMax);
};

#endif
