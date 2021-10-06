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
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/BatteryState.h"
#include "ubiquity_motor/MotorState.h"

#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <ubiquity_motor/motor_parameters.h>
#include <ubiquity_motor/motor_serial.h>

#include <gtest/gtest_prod.h>

// Mininum hardware versions required for various features
#define MIN_HW_OPTION_SWITCH 50

struct MotorDiagnostics {
    MotorDiagnostics()
        : odom_update_status(
              diagnostic_updater::FrequencyStatusParam(&odom_min_freq, &odom_max_freq)) {}
    // Communication Statuses
    int firmware_version = 0;
    int firmware_date    = 0;
    int firmware_options = 0;

    // These are for diagnostic topic output 
    int fw_pid_proportional = 0;
    int fw_pid_integral = 0;
    int fw_pid_derivative = 0;
    int fw_pid_control = 0;
    int fw_pid_velocity = 0;
    int fw_pid_denominator = 0;
    int fw_pid_moving_buffer_size = 0;
    int fw_max_pwm = 0;
   
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

    // Wheel current states
    double motorCurrentLeft  = 0.0;
    double motorCurrentRight = 0.0;

    // ADC count for zero current. We could calibrate this if required. 
    // Nominally 1024 and goes up from there this lower value is used. 
    double motorAmpsZeroAdcCount = 1015;    

    int    motorPwmDriveLeft  = 0;
    int    motorPwmDriveRight = 0;

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
    void motor_pid_p_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_i_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_d_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_v_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_max_pwm_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void firmware_options_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void firmware_date_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

class MotorHardware : public hardware_interface::RobotHW {
public:
    MotorHardware(ros::NodeHandle nh, NodeParams node_params, CommsParams serial_params,
                  FirmwareParams firmware_params);
    virtual ~MotorHardware();
    void closePort();
    bool openPort();
    void clearCommands();
    void readInputs(uint32_t index);
    void writeSpeeds();
    void writeSpeedsInRadians(double left_radians, double right_radians);
    float calculateBatteryPercentage(float voltage, int cells, const float* type);
    int  areWheelSpeedsLower(double wheelSpeedRadPerSec);
    void requestFirmwareVersion();
    void requestFirmwareDate();
    void setParams(FirmwareParams firmware_params);
    void sendParams();
    void forcePidParamUpdates();
    float getBatteryVoltage(void);
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
    void setWheelType(int32_t wheel_type);
    void setWheelGearRatio(double wheel_gear_ratio);
    double getWheelGearRatio(void);
    double getWheelTicksPerRadian(void);
    void setDriveType(int32_t drive_type);
    void setPidControl(int32_t pid_control);
    void nullWheelErrors(void);
    void setWheelDirection(int32_t wheel_direction);
    void getMotorCurrents(double &currentLeft, double &currentRight);
    int  getOptionSwitch(void);
    int  getPidControlWord(void);
    void setOptionSwitchReg(int32_t option_switch);
    void requestSystemEvents();
    void setSystemEvents(int32_t system_events);
    void getWheelJointPositions(double &leftWheelPosition, double &rightWheelPosition);
    void setWheelJointVelocities(double leftWheelVelocity, double rightWheelVelocity);
    void publishMotorState(void);
    int firmware_version;
    int firmware_date;
    int firmware_options;
    int num_fw_params;  // This is used for sendParams as modulo count
    int hardware_version;
    int estop_pid_threshold;
    int max_speed_fwd;
    int max_speed_rev;
    int max_pwm;
    int pid_control;
    int deadman_enable;
    int system_events;
    int wheel_type;
    double wheel_gear_ratio;
    int drive_type;


    diagnostic_updater::Updater diag_updater;
private:
    void _addOdometryRequest(std::vector<MotorMessage>& commands) const;
    void _addVelocityRequest(std::vector<MotorMessage>& commands) const;

    int16_t calculateSpeedFromRadians(double radians);
    double calculateRadiansFromTicks(int16_t ticks);

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

    // MessageTypes enum for refering to motor or wheel number
    enum MotorOrWheelNumber {
        Motor_M1 = 1,
        Motor_M2 = 2
    };

    // MessageTypes enum in class to avoid global namespace pollution
    enum WheelJointLocation {
        Left  = 0,
        Right = 1
    };

    ros::Publisher leftError;
    ros::Publisher rightError;

    ros::Publisher leftCurrent;
    ros::Publisher rightCurrent;

    ros::Publisher leftTickInterval;
    ros::Publisher rightTickInterval;

    ros::Publisher battery_state;
    ros::Publisher motor_power_active;
    ros::Publisher motor_state;

    MotorSerial* motor_serial_;

    MotorDiagnostics motor_diag_;

    FRIEND_TEST(MotorHardwareTests, nonZeroWriteSpeedsOutputs);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPosition);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPositionMax);
};

#endif
