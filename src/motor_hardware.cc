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
#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_message.h>
#include <boost/assign.hpp>

#include <boost/math/special_functions/round.hpp>

//#define SENSOR_DISTANCE 0.002478

// For experimental purposes users will see that the wheel encoders are three phases 
// of very neaar 43 pulses per revolution or about 43*3 edges so we see very about 129 ticks per rev
// This leads to 129/(2*Pi)  or about 20.53 ticks per radian experimentally. 
// Below we will go with the exact ratio from gearbox specs 
// 60 ticks per revolution of the motor (pre gearbox)
// 17.2328767123 and  gear ratio of 4.29411764706:1
#define TICKS_PER_RADIAN_ENC_3_STATE (20.50251516)   // used to read more misleading value of (41.0058030317/2)
#define QTICKS_PER_RADIAN   (ticks_per_radian*4)      // Quadrature ticks makes code more readable later

#define VELOCITY_READ_PER_SECOND \
    10.0  // read = ticks / (100 ms), so we have scale of 10 for ticks/second
#define LOWEST_FIRMWARE_VERSION 28

// Debug verification use only
int32_t  g_odomLeft  = 0;
int32_t  g_odomRight = 0;
int32_t  g_odomEvent = 0;

MotorHardware::MotorHardware(ros::NodeHandle nh, CommsParams serial_params,
                             FirmwareParams firmware_params) {
    ros::V_string joint_names =
        boost::assign::list_of("left_wheel_joint")("right_wheel_joint");

    for (size_t i = 0; i < joint_names.size(); i++) {
        hardware_interface::JointStateHandle joint_state_handle(
            joint_names[i], &joints_[i].position, &joints_[i].velocity,
            &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(
            joint_state_handle, &joints_[i].velocity_command);
        velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    motor_serial_ =
        new MotorSerial(serial_params.serial_port, serial_params.baud_rate);

    leftError = nh.advertise<std_msgs::Int32>("left_error", 1);
    rightError = nh.advertise<std_msgs::Int32>("right_error", 1);

    battery_state = nh.advertise<sensor_msgs::BatteryState>("battery_state", 1);
    motor_power_active = nh.advertise<std_msgs::Bool>("motor_power_active", 1);

    sendPid_count = 0;

    estop_motor_power_off = false;  // Keeps state of ESTOP switch where true is in ESTOP state

    // Save hardware encoder specifics for ticks in one radian of rotation of main wheel
    ticks_per_radian  = TICKS_PER_RADIAN_ENC_3_STATE;

    fw_params = firmware_params;

    prev_fw_params.pid_proportional = -1;
    prev_fw_params.pid_integral = -1;
    prev_fw_params.pid_derivative = -1;
    prev_fw_params.pid_velocity = -1;
    prev_fw_params.pid_denominator = -1;
    prev_fw_params.pid_moving_buffer_size = -1;
    prev_fw_params.max_speed_fwd = -1;
    prev_fw_params.max_speed_rev = -1;
    prev_fw_params.max_pwm = -1;
    prev_fw_params.deadman_timer = -1;
    prev_fw_params.deadzone_enable = -1;
    prev_fw_params.hw_options = -1;
    prev_fw_params.controller_board_version = -1;
    prev_fw_params.estop_detection = -1;
    prev_fw_params.estop_pid_threshold = -1;
    prev_fw_params.max_speed_fwd = -1;
    prev_fw_params.max_speed_rev = -1;
    prev_fw_params.max_pwm = -1;

    hardware_version = 0;
    firmware_version = 0;
    firmware_date    = 0;

    diag_updater.setHardwareID("Motor Controller");
    diag_updater.add("Firmware", &motor_diag_, &MotorDiagnostics::firmware_status);
    diag_updater.add("Limits", &motor_diag_, &MotorDiagnostics::limit_status);
    diag_updater.add("Battery", &motor_diag_, &MotorDiagnostics::battery_status);
    diag_updater.add("MotorPower", &motor_diag_, &MotorDiagnostics::motor_power_status);
    diag_updater.add("FirmwareOptions", &motor_diag_, &MotorDiagnostics::firmware_options_status);
    diag_updater.add("FirmwareDate", &motor_diag_, &MotorDiagnostics::firmware_date_status);
}

MotorHardware::~MotorHardware() { delete motor_serial_; }

void MotorHardware::clearCommands() {
    for (size_t i = 0; i < sizeof(joints_) / sizeof(joints_[0]); i++) {
        joints_[i].velocity_command = 0;
    }
}

// readInputs() will receive serial and act on the response from motor controller
//
// The motor controller sends unsolicited messages periodically so we must read the
// messages to update status in near realtime
//
void MotorHardware::readInputs() {
    while (motor_serial_->commandAvailable()) {
        MotorMessage mm;
        mm = motor_serial_->receiveCommand();
        if (mm.getType() == MotorMessage::TYPE_RESPONSE) {
            switch (mm.getRegister()) {
                case MotorMessage::REG_FIRMWARE_VERSION:
                    if (mm.getData() < LOWEST_FIRMWARE_VERSION) {
                        ROS_FATAL("Firmware version %d, expect %d or above",
                                  mm.getData(), LOWEST_FIRMWARE_VERSION);
                        throw std::runtime_error("Firmware version too low");
                    } else {
                        ROS_INFO("Firmware version %d", mm.getData());
                        firmware_version = mm.getData();
			motor_diag_.firmware_version = firmware_version;
                    }
                    break;

                case MotorMessage::REG_FIRMWARE_DATE:
                    // Firmware date is only supported as of fw version MIN_FW_FIRMWARE_DATE
                    ROS_INFO("Firmware date 0x%x (format 0xYYYYMMDD)", mm.getData());
                    firmware_date = mm.getData();
		    motor_diag_.firmware_date = firmware_date;
                    break;

                case MotorMessage::REG_BOTH_ODOM: {
                    // These counts are the incremental number of ticks since last report
                    // WARNING: IF WE LOOSE A MESSAGE WE DRIFT FROM REAL POSITION
                    int32_t odom = mm.getData();
                    // ROS_ERROR("odom signed %d", odom);
                    int16_t odomLeft = (odom >> 16) & 0xffff;
                    int16_t odomRight = odom & 0xffff;

                    // Debug code to be used for verification
                    g_odomLeft  += odomLeft;
                    g_odomRight += odomRight;
                    g_odomEvent += 1;
                    //if ((g_odomEvent % 50) == 1) { ROS_ERROR("leftOdom %d rightOdom %d", g_odomLeft, g_odomRight); }

                    // Add or subtract from position using the incremental odom value
                    joints_[0].position += (odomLeft / ticks_per_radian);
                    joints_[1].position += (odomRight / ticks_per_radian);

		    motor_diag_.odom_update_status.tick(); // Let diag know we got odom
                    break;
                }
                case MotorMessage::REG_BOTH_ERROR: {
                    std_msgs::Int32 left;
                    std_msgs::Int32 right;
                    int32_t speed = mm.getData();
                    int16_t leftSpeed = (speed >> 16) & 0xffff;
                    int16_t rightSpeed = speed & 0xffff;

                    left.data = leftSpeed;
                    right.data = rightSpeed;
                    leftError.publish(left);
                    rightError.publish(right);
                    break;
                }
                case MotorMessage::REG_HW_OPTIONS: {
                    int32_t data = mm.getData();

                    // Enable or disable hardware options reported from firmware
                    motor_diag_.firmware_options = data;

                    // Set radians per encoder tic
                    if (data & MotorMessage::OPT_ENC_6_STATE) {
		     	fw_params.hw_options |= MotorMessage::OPT_ENC_6_STATE; 
                        ticks_per_radian  = TICKS_PER_RADIAN_ENC_3_STATE * 2;
                    } else {
		    	fw_params.hw_options &= ~MotorMessage::OPT_ENC_6_STATE; 
                        ticks_per_radian  = TICKS_PER_RADIAN_ENC_3_STATE;
                    }
                    break;
                }
                case MotorMessage::REG_LIMIT_REACHED: {
                    int32_t data = mm.getData();

                    if (data & MotorMessage::LIM_M1_PWM) {
                        ROS_WARN("left PWM limit reached");
		    	motor_diag_.left_pwm_limit = true; 
                    }
                    if (data & MotorMessage::LIM_M2_PWM) {
                        ROS_WARN("right PWM limit reached");
		    	motor_diag_.right_pwm_limit = true; 
                    }
                    if (data & MotorMessage::LIM_M1_INTEGRAL) {
                        ROS_DEBUG("left Integral limit reached");
		    	motor_diag_.left_integral_limit = true; 
                    }
                    if (data & MotorMessage::LIM_M2_INTEGRAL) {
                        ROS_DEBUG("right Integral limit reached");
		    	motor_diag_.right_integral_limit = true; 
                    }
                    if (data & MotorMessage::LIM_M1_MAX_SPD) {
                        ROS_WARN("left Maximum speed reached");
		    	motor_diag_.left_max_speed_limit = true; 
                    }
                    if (data & MotorMessage::LIM_M2_MAX_SPD) {
                        ROS_WARN("right Maximum speed reached");
		    	motor_diag_.right_max_speed_limit = true; 
                    }
                    if (data & MotorMessage::LIM_PARAM_LIMIT) {
                        ROS_WARN_ONCE("parameter limit in firmware");
		    	motor_diag_.param_limit_in_firmware = true; 
                    }
                    break;
                }
                case MotorMessage::REG_BATTERY_VOLTAGE: {
                    int32_t data = mm.getData();
                    sensor_msgs::BatteryState bstate;
                    bstate.voltage = (float)data * fw_params.battery_voltage_multiplier +
                                   fw_params.battery_voltage_offset;
                    bstate.current = std::numeric_limits<float>::quiet_NaN();
                    bstate.charge = std::numeric_limits<float>::quiet_NaN();
                    bstate.capacity = std::numeric_limits<float>::quiet_NaN();
                    bstate.design_capacity = std::numeric_limits<float>::quiet_NaN();
                    bstate.percentage = std::max(0.0, std::min(1.0, (bstate.voltage - 20.0) * 0.125));
                    bstate.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
                    bstate.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
                    bstate.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
                    battery_state.publish(bstate);

		    motor_diag_.battery_voltage = bstate.voltage; 
                    break;
                }
                case MotorMessage::REG_MOT_PWR_ACTIVE: {   // Starting with rev 5.0 board we can see power state
                    int32_t data = mm.getData();

                    if (data & MotorMessage::MOT_POW_ACTIVE) {
		    	if (estop_motor_power_off == true) { 
                            ROS_WARN("Motor power has gone from inactive to active. Most likely from ESTOP switch");
                        }
		    	estop_motor_power_off = false; 
                    } else {
		    	if (estop_motor_power_off == false) { 
                            ROS_WARN("Motor power has gone inactive. Most likely from ESTOP switch active");
                        }
		    	estop_motor_power_off = true; 
                    }
                    motor_diag_.estop_motor_power_off = estop_motor_power_off;  // A copy for diagnostics topic

		    std_msgs::Bool estop_message;
		    estop_message.data = !estop_motor_power_off;
		    motor_power_active.publish(estop_message);
                }
                default:
                    break;
            }
        }
    }
}

// writeSpeedsInRadians()  Take in radians per sec for wheels and send in message to controller
//
// A direct write speeds that allows caller setting speeds in radians
// This interface allows maintaining of system speed in state but override to zero
// which is of value for such a case as ESTOP implementation
//
void MotorHardware::writeSpeedsInRadians(double  left_radians, double  right_radians) {
    MotorMessage both;
    both.setRegister(MotorMessage::REG_BOTH_SPEED_SET);
    both.setType(MotorMessage::TYPE_WRITE);

    int16_t left_speed  = calculateSpeedFromRadians(left_radians);
    int16_t right_speed = calculateSpeedFromRadians(right_radians);

    // The masking with 0x0000ffff is necessary for handling -ve numbers
    int32_t data = (left_speed << 16) | (right_speed & 0x0000ffff);
    both.setData(data);

    std_msgs::Int32 smsg;
    smsg.data = left_speed;

    motor_serial_->transmitCommand(both);

    // ROS_ERROR("velocity_command %f rad/s %f rad/s",
    // joints_[0].velocity_command, joints_[1].velocity_command);
    // ROS_ERROR("SPEEDS %d %d", left.getData(), right.getData());
}

// writeSpeeds()  Take in radians per sec for wheels and send in message to controller
//
// Legacy interface where no speed overrides are supported
//
void MotorHardware::writeSpeeds() {
    // This call pulls in speeds from the joints array maintained by other layers

    double  left_radians = joints_[0].velocity_command;
    double  right_radians = joints_[1].velocity_command;

    writeSpeedsInRadians(left_radians, right_radians);
}

void MotorHardware::requestFirmwareVersion() {
    MotorMessage fw_version_msg;
    fw_version_msg.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
    fw_version_msg.setType(MotorMessage::TYPE_READ);
    fw_version_msg.setData(0);
    motor_serial_->transmitCommand(fw_version_msg);
}

// Firmware date register implemented as of MIN_FW_FIRMWARE_DATE
void MotorHardware::requestFirmwareDate() {
    MotorMessage fw_date_msg;
    fw_date_msg.setRegister(MotorMessage::REG_FIRMWARE_DATE);
    fw_date_msg.setType(MotorMessage::TYPE_READ);
    fw_date_msg.setData(0);
    motor_serial_->transmitCommand(fw_date_msg);
}


// Due to greatly limited pins on the firmware processor the host figures out the hardware rev and sends it to fw
// The hardware version is 0x0000MMmm  where MM is major rev like 4 and mm is minor rev like 9 for first units.
// The 1st firmware version this is set for is 32, before it was always 1
void MotorHardware::setHardwareVersion(int32_t hardware_version) {
    ROS_INFO("setting hardware_version to %x", (int)hardware_version);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_HARDWARE_VERSION);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(hardware_version);
    motor_serial_->transmitCommand(mm);
}

// Setup the controller board threshold to put into force estop protection on boards prior to rev 5.0 with hardware support
void MotorHardware::setEstopPidThreshold(int32_t estop_pid_threshold) {
    ROS_INFO("setting Estop PID threshold to %d", (int)estop_pid_threshold);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_PID_MAX_ERROR);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(estop_pid_threshold);
    motor_serial_->transmitCommand(mm);
}

// Setup the controller board to have estop button state detection feature enabled or not
void MotorHardware::setEstopDetection(int32_t estop_detection) {
    ROS_INFO("setting estop button detection to %x", (int)estop_detection);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_ESTOP_ENABLE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(estop_detection);
    motor_serial_->transmitCommand(mm);
}

// Returns true if estop switch is active OR if motor power is off somehow off
bool MotorHardware::getEstopState(void) {
    return estop_motor_power_off;
}

// Setup the controller board maximum settable motor forward speed 
void MotorHardware::setMaxFwdSpeed(int32_t max_speed_fwd) {
    ROS_INFO("setting max motor forward speed to %d", (int)max_speed_fwd);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_SPEED_FWD);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_speed_fwd);
    motor_serial_->transmitCommand(mm);
}

// Setup the controller board maximum settable motor reverse speed
void MotorHardware::setMaxRevSpeed(int32_t max_speed_rev) {
    ROS_INFO("setting max motor reverse speed to %d", (int)max_speed_rev);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_SPEED_REV);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_speed_rev);
    motor_serial_->transmitCommand(mm);
}

// Setup the controller board maximum PWM level allowed for a motor
void MotorHardware::setMaxPwm(int32_t max_pwm) {
    ROS_INFO("setting max motor PWM to %x", (int)max_pwm);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_PWM);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_pwm);
    motor_serial_->transmitCommand(mm);
}

void MotorHardware::setDeadmanTimer(int32_t deadman_timer) {
    ROS_ERROR("setting deadman to %d", (int)deadman_timer);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DEADMAN);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(deadman_timer);
    motor_serial_->transmitCommand(mm);
}

void MotorHardware::setDeadzoneEnable(int32_t deadzone_enable) {
    ROS_ERROR("setting deadzone enable to %d", (int)deadzone_enable);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DEADZONE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(deadman_timer);
    motor_serial_->transmitCommand(mm);
}

void MotorHardware::setParams(FirmwareParams fp) {
    fw_params.pid_proportional = fp.pid_proportional;
    fw_params.pid_integral = fp.pid_integral;
    fw_params.pid_derivative = fp.pid_derivative;
    fw_params.pid_velocity = fp.pid_velocity;
    fw_params.pid_denominator = fp.pid_denominator;
    fw_params.pid_moving_buffer_size = fp.pid_moving_buffer_size;
    fw_params.pid_denominator = fp.pid_denominator;
    fw_params.estop_pid_threshold = fp.estop_pid_threshold;
}

void MotorHardware::sendParams() {
    std::vector<MotorMessage> commands;

    // ROS_ERROR("sending PID %d %d %d %d",
    //(int)p_value, (int)i_value, (int)d_value, (int)denominator_value);

    // Only send one register at a time to avoid overwhelming serial comms
    // SUPPORT NOTE!  Adjust modulo for total parameters in the cycle
    //                and be sure no duplicate modulos are used!
    int cycle = (sendPid_count++) % 6;     // MUST BE THE TOTAL NUMBER IN THIS HANDLING

    if (cycle == 0 &&
        fw_params.pid_proportional != prev_fw_params.pid_proportional) {
        ROS_WARN("Setting P to %d", fw_params.pid_proportional);
        prev_fw_params.pid_proportional = fw_params.pid_proportional;
        MotorMessage p;
        p.setRegister(MotorMessage::REG_PARAM_P);
        p.setType(MotorMessage::TYPE_WRITE);
        p.setData(fw_params.pid_proportional);
        commands.push_back(p);
    }

    if (cycle == 1 && fw_params.pid_integral != prev_fw_params.pid_integral) {
        ROS_WARN("Setting I to %d", fw_params.pid_integral);
        prev_fw_params.pid_integral = fw_params.pid_integral;
        MotorMessage i;
        i.setRegister(MotorMessage::REG_PARAM_I);
        i.setType(MotorMessage::TYPE_WRITE);
        i.setData(fw_params.pid_integral);
        commands.push_back(i);
    }

    if (cycle == 2 &&
        fw_params.pid_derivative != prev_fw_params.pid_derivative) {
        ROS_WARN("Setting D to %d", fw_params.pid_derivative);
        prev_fw_params.pid_derivative = fw_params.pid_derivative;
        MotorMessage d;
        d.setRegister(MotorMessage::REG_PARAM_D);
        d.setType(MotorMessage::TYPE_WRITE);
        d.setData(fw_params.pid_derivative);
        commands.push_back(d);
    }

    if (cycle == 3 && (motor_diag_.firmware_version >= MIN_FW_PID_V_TERM) &&
        fw_params.pid_velocity != prev_fw_params.pid_velocity) {
        ROS_WARN("Setting V to %d", fw_params.pid_velocity);
        prev_fw_params.pid_velocity = fw_params.pid_velocity;
        MotorMessage v;
        v.setRegister(MotorMessage::REG_PARAM_V);
        v.setType(MotorMessage::TYPE_WRITE);
        v.setData(fw_params.pid_velocity);
        commands.push_back(v);
    }

    if (cycle == 4 &&
        fw_params.pid_denominator != prev_fw_params.pid_denominator) {
        ROS_WARN("Setting Denominator to %d", fw_params.pid_denominator);
        prev_fw_params.pid_denominator = fw_params.pid_denominator;
        MotorMessage denominator;
        denominator.setRegister(MotorMessage::REG_PARAM_C);
        denominator.setType(MotorMessage::TYPE_WRITE);
        denominator.setData(fw_params.pid_denominator);
        commands.push_back(denominator);
    }

    if (cycle == 5 &&
        fw_params.pid_moving_buffer_size !=
            prev_fw_params.pid_moving_buffer_size) {
        ROS_WARN("Setting D window to %d", fw_params.pid_moving_buffer_size);
        prev_fw_params.pid_moving_buffer_size =
            fw_params.pid_moving_buffer_size;
        MotorMessage winsize;
        winsize.setRegister(MotorMessage::REG_MOVING_BUF_SIZE);
        winsize.setType(MotorMessage::TYPE_WRITE);
        winsize.setData(fw_params.pid_moving_buffer_size);
        commands.push_back(winsize);
    }

    // SUPPORT NOTE!  Adjust max modulo for total parameters in the cycle, be sure no duplicates used!

    if (commands.size() != 0) {
        motor_serial_->transmitCommands(commands);
    }
}

void MotorHardware::setDebugLeds(bool led_1, bool led_2) {
    std::vector<MotorMessage> commands;

    MotorMessage led1;
    led1.setRegister(MotorMessage::REG_LED_1);
    led1.setType(MotorMessage::TYPE_WRITE);
    if (led_1) {
        led1.setData(0x00000001);
    } else {
        led1.setData(0x00000000);
    }
    commands.push_back(led1);

    MotorMessage led2;
    led2.setRegister(MotorMessage::REG_LED_2);
    led2.setType(MotorMessage::TYPE_WRITE);
    if (led_2) {
        led2.setData(0x00000001);
    } else {
        led2.setData(0x00000000);
    }
    commands.push_back(led2);

    motor_serial_->transmitCommands(commands);
}

// calculate the binary speed value sent to motor controller board
// using an input expressed in radians.
// The firmware uses the same speed value no matter what type of encoder is used
int16_t MotorHardware::calculateSpeedFromRadians(double radians) const {
    int16_t speed;
    double  encoderFactor = 1.0;

    // The firmware accepts same units for speed value
    // and will deal with it properly depending on encoder handling in use
    if (fw_params.hw_options & MotorMessage::OPT_ENC_6_STATE) {
        encoderFactor = 0.5;
    }

    speed =  boost::math::iround(encoderFactor * (radians * QTICKS_PER_RADIAN /
                               VELOCITY_READ_PER_SECOND));
    return speed;
}

double MotorHardware::calculateRadiansFromTicks(int16_t ticks) const {
    return (ticks * VELOCITY_READ_PER_SECOND / QTICKS_PER_RADIAN);
}

// Diagnostics Status Updater Functions
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_msgs::DiagnosticStatus;

void MotorDiagnostics::firmware_status(DiagnosticStatusWrapper &stat) {
    stat.add("Firmware Version", firmware_version);
    if (firmware_version == 0) {
        stat.summary(DiagnosticStatus::ERROR, "No firmware version reported. Power may be off.");
    }
    else if (firmware_version < MIN_FW_RECOMMENDED) {
        stat.summary(DiagnosticStatus::WARN, "Firmware is older than recommended! You must update firmware!");
    }
    else {
        stat.summary(DiagnosticStatus::OK, "Firmware version is OK");
    }
}

void MotorDiagnostics::firmware_date_status(DiagnosticStatusWrapper &stat) {

    // Only output status if the firmware daycode is supported
    if (firmware_version >= MIN_FW_FIRMWARE_DATE) {
        std::stringstream stream;
        stream << std::hex << firmware_date;
        std::string daycode(stream.str());

        stat.add("Firmware Date", daycode);
        stat.summary(DiagnosticStatus::OK, "Firmware daycode format is YYYYMMDD");
    }
}

// When a firmware limit condition is reported the diagnostic topic reports it.
// Once the report is made the condition is cleared till next time firmware reports that limit
void MotorDiagnostics::limit_status(DiagnosticStatusWrapper &stat) {
    stat.summary(DiagnosticStatus::OK, "Limits reached:");
    if (left_pwm_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::ERROR, " left pwm,");
	left_pwm_limit = false;
    }
    if (right_pwm_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::ERROR, " right pwm,");
	right_pwm_limit = false;
    }
    if (left_integral_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " left integral,");
	left_integral_limit = false;
    }
    if (right_integral_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " right integral,");
	right_integral_limit = false;
    }
    if (left_max_speed_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " left speed,");
	left_max_speed_limit = false;
    }
    if (right_max_speed_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " right speed,");
	right_max_speed_limit = false;
    }
    if (param_limit_in_firmware) {
        // A parameter was sent to firmware that was out of limits for the firmware register
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " firmware limit,");
	param_limit_in_firmware = false;
    }
}

void MotorDiagnostics::battery_status(DiagnosticStatusWrapper &stat) {
    stat.add("Battery Voltage", battery_voltage);
    if (battery_voltage < 22.5) {
        stat.summary(DiagnosticStatusWrapper::WARN, "Battery low");
    } 
    else if (battery_voltage < 21.0) {
        stat.summary(DiagnosticStatusWrapper::ERROR, "Battery critical");
    }
    else {
        stat.summary(DiagnosticStatusWrapper::OK, "Battery OK");
    }
}
void MotorDiagnostics::motor_power_status(DiagnosticStatusWrapper &stat) {
    stat.add("Motor Power", !estop_motor_power_off);
    if (estop_motor_power_off == false) {
        stat.summary(DiagnosticStatusWrapper::ERROR, "Motor power on");
    } 
    else {
        stat.summary(DiagnosticStatusWrapper::WARN, "Motor power off");
    }
}
// motor_encoder_mode returns 0 for legacy encoders and 1 for 6-state firmware
void MotorDiagnostics::firmware_options_status(DiagnosticStatusWrapper &stat) {
    stat.add("Firmware Options", firmware_options);
    if (firmware_options & MotorMessage::OPT_ENC_6_STATE) {
        stat.summary(DiagnosticStatusWrapper::OK, "High resolution encoders");
    } 
    else {
        stat.summary(DiagnosticStatusWrapper::OK, "Standard resolution encoders");
    }
}


