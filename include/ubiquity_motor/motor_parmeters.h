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

#ifndef UBIQUITY_MOTOR_MOTOR_PARMETERS_H
#define UBIQUITY_MOTOR_MOTOR_PARMETERS_H

#include <ros/ros.h>

template <typename T>
T getParamOrDefault(ros::NodeHandle nh, std::string parameter_name,
                    T default_val) {
    T value = default_val;
    if (!nh.getParam(parameter_name, value)) {
        nh.setParam(parameter_name, value);
    }
    return value;
}

struct FirmwareParams {
    int32_t pid_proportional;
    int32_t pid_integral;
    int32_t pid_derivative;
    int32_t pid_denominator;
    int32_t pid_moving_buffer_size;
    int32_t deadman_timer;
    float battery_voltage_multiplier;
    float battery_voltage_offset;

    FirmwareParams()
        : pid_proportional(5000),
          pid_integral(10),
          pid_derivative(1),
          pid_denominator(1000),
          pid_moving_buffer_size(10),
          deadman_timer(2400000),
          battery_voltage_multiplier(0.05185),
          battery_voltage_offset(0.40948){};

    FirmwareParams(ros::NodeHandle nh)
        : pid_proportional(5000),
          pid_integral(10),
          pid_derivative(1),
          pid_denominator(1000),
          pid_moving_buffer_size(10),
          deadman_timer(2400000),
          battery_voltage_multiplier(0.05185),
          battery_voltage_offset(0.40948){
        // clang-format off
        pid_proportional = getParamOrDefault(
            nh, "ubiquity_motor/pid_proportional", pid_proportional);
        pid_integral = getParamOrDefault(
            nh, "ubiquity_motor/pid_integral", pid_integral);
        pid_derivative = getParamOrDefault(
            nh, "ubiquity_motor/pid_derivative", pid_derivative);
        pid_denominator = getParamOrDefault(
            nh, "ubiquity_motor/pid_denominator", pid_denominator);
        pid_moving_buffer_size = getParamOrDefault(
            nh, "ubiquity_motor/window_size", pid_moving_buffer_size);
        deadman_timer = getParamOrDefault(
            nh, "ubiquity_motor/deadman_timer", deadman_timer);
        battery_voltage_offset = getParamOrDefault(
            nh, "ubiquity_motor/battery_voltage_offset", battery_voltage_offset);
        battery_voltage_multiplier = getParamOrDefault(
            nh, "ubiquity_motor/battery_voltage_multiplier", battery_voltage_multiplier);
        // clang-format on
    };
};

struct CommsParams {
    std::string serial_port;
    int32_t baud_rate;
    double serial_loop_rate;

    CommsParams()
        : serial_port("/dev/ttyS0"), baud_rate(9600), serial_loop_rate(100.0){};

    CommsParams(ros::NodeHandle nh)
        : serial_port("/dev/ttyS0"), baud_rate(9600), serial_loop_rate(100.0) {
        // clang-format off
        serial_port = getParamOrDefault(
            nh, "ubiquity_motor/serial_port", serial_port);
        baud_rate = getParamOrDefault(
            nh, "ubiquity_motor/serial_baud", baud_rate);
        // clang-format on
    };
};

struct NodeParams {
    double controller_loop_rate;

    NodeParams() : controller_loop_rate(10.0){};
    NodeParams(ros::NodeHandle nh) : controller_loop_rate(10.0) {
        // clang-format off
        controller_loop_rate = getParamOrDefault(
            nh, "ubiquity_motor/controller_loop_rate", controller_loop_rate);
        // clang-format on
    };
};

#endif  // UBIQUITY_MOTOR_MOTOR_PARMETERS_H
