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

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <time.h>
#include <ubiquity_motor/PIDConfig.h>
#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_message.h>
#include <ubiquity_motor/motor_parmeters.h>
#include <boost/thread.hpp>
#include <string>
#include "controller_manager/controller_manager.h"

static const double BILLION = 1000000000.0;

static FirmwareParams firmware_params;
static CommsParams serial_params;
static NodeParams node_params;

void PID_update_callback(const ubiquity_motor::PIDConfig& config,
                         uint32_t level) {
    if (level == 0xFFFFFFFF) {
        return;
    }

    if (level == 1) {
        firmware_params.pid_proportional = config.PID_P;
    } else if (level == 2) {
        firmware_params.pid_integral = config.PID_I;
    } else if (level == 4) {
        firmware_params.pid_derivative = config.PID_D;
    } else if (level == 8) {
        firmware_params.pid_denominator = config.PID_C;
    } else if (level == 16) {
        firmware_params.pid_moving_buffer_size = config.PID_W;
    } else {
        ROS_ERROR("Unsupported dynamic_reconfigure level %u", level);
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;

    firmware_params = FirmwareParams(nh);
    serial_params = CommsParams(nh);
    node_params = NodeParams(nh);

    ros::Rate r(node_params.controller_loop_rate);

    std::unique_ptr<MotorHardware> robot = nullptr;
    // Keep trying to open serial
    {
        int times = 0;
        while (ros::ok() && robot.get() == nullptr) {
            try {
                robot.reset(new MotorHardware(nh, serial_params, firmware_params));
            }
            catch (const serial::IOException& e) {
                if (times % 30 == 0)
                    ROS_FATAL("Error opening serial port %s, trying again", serial_params.serial_port.c_str());
            }
            r.sleep();
            times++;
        }
    }

    controller_manager::ControllerManager cm(robot.get(), nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    dynamic_reconfigure::Server<ubiquity_motor::PIDConfig> server;
    dynamic_reconfigure::Server<ubiquity_motor::PIDConfig>::CallbackType f;

    f = boost::bind(&PID_update_callback, _1, _2);
    server.setCallback(f);

    robot->setParams(firmware_params);
    robot->requestVersion();

    // Make sure firmware is listening
    {
        // Start times counter at 1 to prevent false error print (0 % n = 0)
        int times = 1;
        while (ros::ok() && robot->firmware_version == 0) {
            if (times % 30 == 0)
                ROS_ERROR("Firmware not reporting its version");
                robot->requestVersion();
            robot->readInputs();
            r.sleep();
            times++;
        }
    }

    ros::Time last_time;
    ros::Time current_time;
    ros::Duration elapsed;
    last_time = ros::Time::now();

    for (int i = 0; i < 5; i++) {
        r.sleep();
        robot->sendParams();
    }
    float expectedCycleTime = r.expectedCycleTime().toSec();
    float minCycleTime = 0.75 * expectedCycleTime;
    float maxCycleTime = 1.25 * expectedCycleTime;

    while (ros::ok()) {
        current_time = ros::Time::now();
        elapsed = current_time - last_time;
        last_time = current_time;
        robot->readInputs();
        float elapsedSecs = elapsed.toSec();
        if (minCycleTime < elapsedSecs && elapsedSecs < maxCycleTime) {
            cm.update(current_time, elapsed);
        }
        else {
            ROS_WARN("Resetting controller due to time jump %f seconds",
                     elapsedSecs);
            cm.update(current_time, r.expectedCycleTime(), true);
            robot->clearCommands();
        }
        robot->setParams(firmware_params);
        robot->sendParams();
        robot->writeSpeeds();

        r.sleep();
    }

    return 0;
}
