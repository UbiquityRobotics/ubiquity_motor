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
#include "std_msgs/Bool.h"
#include "sensor_msgs/BatteryState.h"

#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <ubiquity_motor/motor_parameters.h>
#include <ubiquity_motor/motor_serial.h>

#include <gtest/gtest_prod.h>

struct MotorDiagnostics {
    MotorDiagnostics()
        : odom_update_status(
              diagnostic_updater::FrequencyStatusParam(&odom_min_freq, &odom_max_freq)) {}
    // Communication Statuses
    int firmware_version = 0;
    int firmware_date    = 0;
    int firmware_options = 0;
   
    double odom_max_freq = 1000;
    double odom_min_freq = 50;
    diagnostic_updater::FrequencyStatus odom_update_status;

    // Limits
    bool left_pwm_limit = false;
    bool right_pwm_limit = false;
    bool left_integral_limit = false;
    bool right_integral_limit = false;
    bool left_max_speed_limit = false;
    bool right_max_speed_limit = false;
    bool param_limit_in_firmware = false;

    // Power supply statuses
    float battery_voltage = 0.0;
    float battery_voltage_low_level = 22.5;
    float battery_voltage_critical = 21.0;

    /* For later implementation (firmware support)
    bool  main_5V_error = false;
    bool  main_5V_ol = false;
    bool  main_12V_error = false;
    bool  main_12V_ol = false;
    bool  aux_5V_error = false;
    bool  aux_5V_ol = false;
    bool  aux_12V_error = false;
    bool  aux_12V_ol = false;
    */

    bool  estop_motor_power_off = false;  // for Diagnostic reporting of ESTOP switch

    void firmware_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void limit_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void battery_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_power_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void firmware_options_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void firmware_date_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

class MotorHardware : public hardware_interface::RobotHW {
public:
    MotorHardware(ros::NodeHandle nh, CommsParams serial_params,
                  FirmwareParams firmware_params);
    virtual ~MotorHardware();
    void clearCommands();
    void readInputs();
    void writeSpeeds();
    void writeSpeedsInRadians(double  left_radians, double  right_radians);
    void requestFirmwareVersion();
    void requestFirmwareDate();
    void setParams(FirmwareParams firmware_params);
    void sendParams();
    void setDeadmanTimer(int32_t deadman);
    void setDeadzoneEnable(int32_t deadzone_enable);
    void setDebugLeds(bool led1, bool led2);
    void setHardwareVersion(int32_t hardware_version);
    void setEstopPidThreshold(int32_t estop_pid_threshold);
    void setEstopDetection(int32_t estop_detection);
    bool getEstopState(void);
    void setMaxFwdSpeed(int32_t max_speed_fwd);
    void setMaxRevSpeed(int32_t max_speed_rev);
    void setMaxPwm(int32_t max_pwm);
    void setHardwareOptions(int32_t hardware_options);
    int  getOptionSwitch(void);
    void setOptionSwitchReg(int32_t option_switch);
    void requestSystemEvents();
    void setSystemEvents(int32_t system_events);
    int firmware_version;
    int firmware_date;
    int firmware_options;
    int hardware_version;
    int estop_pid_threshold;
    int max_speed_fwd;
    int max_speed_rev;
    int max_pwm;
    int deadman_enable;
    int system_events;

    diagnostic_updater::Updater diag_updater;
private:
    void _addOdometryRequest(std::vector<MotorMessage>& commands) const;
    void _addVelocityRequest(std::vector<MotorMessage>& commands) const;

    int16_t calculateSpeedFromRadians(double radians) const;
    double calculateRadiansFromTicks(int16_t ticks) const;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    FirmwareParams fw_params;
    FirmwareParams prev_fw_params;

    int32_t deadman_timer;

    double  ticks_per_radian;       // Odom ticks per radian for wheel encoders in use

    int32_t sendPid_count;

    bool estop_motor_power_off;    // Motor power inactive, most likely from ESTOP switch

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
    ros::Publisher motor_power_active;

    MotorSerial* motor_serial_;

    MotorDiagnostics motor_diag_;

    FRIEND_TEST(MotorHardwareTests, nonZeroWriteSpeedsOutputs);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPosition);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPositionMax);
};

#endif
