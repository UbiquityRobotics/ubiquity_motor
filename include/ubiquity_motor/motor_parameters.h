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
    int32_t pid_velocity;
    int32_t pid_denominator;
    int32_t pid_moving_buffer_size;
    int32_t pid_control;
    int32_t controller_board_version;
    int32_t estop_detection;
    int32_t estop_pid_threshold;
    int32_t max_speed_fwd;
    int32_t max_speed_rev;
    int32_t max_pwm;
    int32_t deadman_timer;
    int32_t deadzone_enable;
    int32_t hw_options;
    int32_t option_switch;
    int32_t system_events;
    float battery_voltage_multiplier;
    float battery_voltage_offset;
    float battery_voltage_low_level;
    float battery_voltage_critical;

    FirmwareParams()
        : pid_proportional(4000),
          pid_integral(5),
          pid_derivative(-200),
          pid_velocity(0),
          pid_denominator(1000),
          pid_moving_buffer_size(10),
          pid_control(0),
          controller_board_version(49),
          estop_detection(1),
          estop_pid_threshold(1500),
          max_speed_fwd(80),
          max_speed_rev(-80),
          max_pwm(250),
          deadman_timer(2400000),
          deadzone_enable(0),
          hw_options(0),
          option_switch(0),
          system_events(0),
          battery_voltage_multiplier(0.05057),
          battery_voltage_offset(0.40948),
          battery_voltage_low_level(22.5),
          battery_voltage_critical(21.0){};

    FirmwareParams(ros::NodeHandle nh)
        : pid_proportional(4000),
          pid_integral(5),
          pid_derivative(-200),
          pid_velocity(0),
          pid_denominator(1000),
          pid_moving_buffer_size(10),
          pid_control(0),
          controller_board_version(49),
          estop_detection(1),
          estop_pid_threshold(1500),
          max_speed_fwd(80),
          max_speed_rev(-80),
          max_pwm(250),
          deadman_timer(2400000),
          deadzone_enable(0),
          hw_options(0),
          option_switch(0),
          system_events(0),
          battery_voltage_multiplier(0.05057),
          battery_voltage_offset(0.40948), 
          battery_voltage_low_level(22.5),
          battery_voltage_critical(21.0)
        {
        // clang-format off
        pid_proportional = getParamOrDefault(
            nh, "pid_proportional", pid_proportional);
        pid_integral = getParamOrDefault(
            nh, "pid_integral", pid_integral);
        pid_derivative = getParamOrDefault(
            nh, "pid_derivative", pid_derivative);
        pid_velocity = getParamOrDefault(
            nh, "pid_velocity", pid_velocity);
        pid_denominator = getParamOrDefault(
            nh, "pid_denominator", pid_denominator);
        pid_control = getParamOrDefault(
            nh, "pid_control", pid_control);
        pid_moving_buffer_size = getParamOrDefault(
            nh, "window_size", pid_moving_buffer_size);
        controller_board_version = getParamOrDefault(
            nh, "controller_board_version", controller_board_version);
        estop_detection = getParamOrDefault(
            nh, "fw_estop_detection", estop_detection);
        estop_pid_threshold = getParamOrDefault(
            nh, "fw_estop_pid_threshold", estop_pid_threshold);
        max_speed_fwd = getParamOrDefault(
            nh, "fw_max_speed_fwd", max_speed_fwd);
        max_speed_rev = getParamOrDefault(
            nh, "fw_max_speed_rev", max_speed_rev);
        max_pwm = getParamOrDefault(
            nh, "fw_max_pwm", max_pwm);
        deadman_timer = getParamOrDefault(
            nh, "deadman_timer", deadman_timer);
        deadzone_enable = getParamOrDefault(
            nh, "deadzone_enable", deadzone_enable);
        battery_voltage_offset = getParamOrDefault(
            nh, "battery_voltage_offset", battery_voltage_offset);
        battery_voltage_multiplier = getParamOrDefault(
            nh, "battery_voltage_multiplier", battery_voltage_multiplier);
        battery_voltage_low_level = getParamOrDefault(
            nh, "battery_voltage_low_level", battery_voltage_low_level);
        battery_voltage_critical = getParamOrDefault(
            nh, "battery_voltage_critical", battery_voltage_critical);
        // clang-format on
    };
};

struct CommsParams {
    std::string serial_port;
    int32_t baud_rate;

    CommsParams()
        : serial_port("/dev/ttyS0"), baud_rate(9600) {};

    CommsParams(ros::NodeHandle nh)
        : serial_port("/dev/ttyS0"), baud_rate(9600) {
        // clang-format off
        serial_port = getParamOrDefault(
            nh, "serial_port", serial_port);
        baud_rate = getParamOrDefault(
            nh, "serial_baud", baud_rate);
        // clang-format on
    };
};

struct NodeParams {
    double controller_loop_rate;
    std::string wheel_type;
    std::string wheel_direction;
    std::string drive_type;
    std::string left_joint;
    std::string right_joint;

    NodeParams()
        : controller_loop_rate(10.0),
          wheel_type("firmware_default"),
          wheel_direction("firmware_default"),
          drive_type("2wd"),
	  left_joint("left_wheel_joint"),
	  right_joint("right_wheel_joint"){};

    NodeParams(ros::NodeHandle nh) : controller_loop_rate(10.0),
        wheel_type("firmware_default"),
        wheel_direction("firmware_default"),
        drive_type("2wd") {
        // clang-format off
        controller_loop_rate = getParamOrDefault(
            nh, "controller_loop_rate", controller_loop_rate);
        wheel_type = getParamOrDefault(
            nh, "wheel_type", wheel_type);
        wheel_direction = getParamOrDefault(
            nh, "wheel_direction", wheel_direction);
        drive_type = getParamOrDefault(
            nh, "drive_type", drive_type);
        left_joint = getParamOrDefault(
            nh, "left_joint", left_joint);
        right_joint = getParamOrDefault(
            nh, "right_joint", right_joint);
        // clang-format on
    };
};

#endif  // UBIQUITY_MOTOR_MOTOR_PARMETERS_H
