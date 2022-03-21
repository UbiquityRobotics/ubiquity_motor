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

// To access I2C we need some system includes
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#define  I2C_DEVICE  "/dev/i2c-1"     // This is specific to default Magni I2C port on host
const static uint8_t  I2C_PCF8574_8BIT_ADDR = 0x40; // I2C addresses are 7 bits but often shown as 8-bit

//#define SENSOR_DISTANCE 0.002478


// For experimental purposes users will see that the wheel encoders are three phases
// of very neaar 43 pulses per revolution or about 43*3 edges so we see very about 129 ticks per rev
// This leads to 129/(2*Pi)  or about 20.53 ticks per radian experimentally.
// 60 ticks per revolution of the motor (pre gearbox) and a gear ratio of 4.2941176:1 for 1st rev Magni wheels
// An unexplained constant from the past I leave here till explained is: 17.2328767

#define TICKS_PER_RAD_FROM_GEAR_RATIO   ((double)(4.774556)*(double)(2.0))  // Used to generate ticks per radian 
#define TICKS_PER_RADIAN_DEFAULT        41.004  // For runtime use  getWheelGearRatio() * TICKS_PER_RAD_FROM_GEAR_RATIO    
#define WHEEL_GEAR_RATIO_DEFAULT        WHEEL_GEAR_RATIO_1

// TODO: Make HIGH_SPEED_RADIANS, WHEEL_VELOCITY_NEAR_ZERO and ODOM_4WD_ROTATION_SCALE  all ROS params
#define HIGH_SPEED_RADIANS        (1.8)               // threshold to consider wheel turning 'very fast'
#define WHEEL_VELOCITY_NEAR_ZERO  ((double)(0.08))
#define ODOM_4WD_ROTATION_SCALE   ((double)(1.65))    // Used to correct for 4WD skid rotation error

#define MOTOR_AMPS_PER_ADC_COUNT   ((double)(0.0238)) // 0.1V/Amp  2.44V=1024 count so 41.97 cnt/amp

#define VELOCITY_READ_PER_SECOND   ((double)(10.0))   // read = ticks / (100 ms), so we scale of 10 for ticks/second
#define LOWEST_FIRMWARE_VERSION 28

// Debug verification use only
int32_t  g_odomLeft  = 0;
int32_t  g_odomRight = 0;
int32_t  g_odomEvent = 0;

//lead acid battery percentage levels for a single cell
const static float SLA_AGM[11] = {
    1.800, // 0
    1.837, // 10
    1.875, // 20
    1.912, // 30
    1.950, // 40
    2.010, // 50
    2.025, // 60
    2.062, // 70
    2.100, // 80
    2.137, // 90
    2.175, // 100
};


//li-ion battery percentage levels for a single cell
const static float LI_ION[11] = {
    3.3, // 0
    3.49, // 10
    3.53, // 20
    3.55, // 30
    3.60, // 40
    3.64, // 50
    3.70, // 60
    3.80, // 70
    3.85, // 80
    4.05, // 90
    4.20, // 100
};

// We sometimes need to know if we are rotating in place due to special ways of dealing with
// A 4wd robot must skid to turn. This factor approximates the actual rotation done vs what
// the wheel encoders have indicated.  This only applies if in 4WD mode
double   g_odom4wdRotationScale = ODOM_4WD_ROTATION_SCALE;

// 4WD robot chassis that has to use extensive torque to rotate in place and due to wheel slip has odom scale factor
double   g_radiansLeft  = 0.0;
double   g_radiansRight = 0.0;

// This utility opens and reads 1 or more bytes from a device on an I2C bus
// This method was taken on it's own from a big I2C class we may choose to use later
static int i2c_BufferRead(const char *i2cDevFile, uint8_t i2c8bitAddr,
                          uint8_t *pBuffer, int16_t chipRegAddr, uint16_t NumBytesToRead);

MotorHardware::MotorHardware(ros::NodeHandle nh, NodeParams node_params, CommsParams serial_params,
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

    // Insert a delay prior to serial port setup to avoid a race defect.
    // We see soemtimes the OS sets the port to 115200 baud just after we set it
    ROS_INFO("Delay before MCB serial port initialization");
    ros::Duration(5.0).sleep();
    ROS_INFO("Initialize MCB serial port '%s' for %d baud",
        serial_params.serial_port.c_str(), serial_params.baud_rate);

    motor_serial_ =
        new MotorSerial(serial_params.serial_port, serial_params.baud_rate);

     ROS_INFO("MCB serial port initialized");

    // For motor tunning and other uses we publish details for each wheel
    leftError = nh.advertise<std_msgs::Int32>("left_error", 1);
    rightError = nh.advertise<std_msgs::Int32>("right_error", 1);
    leftTickInterval  = nh.advertise<std_msgs::Int32>("left_tick_interval", 1);
    rightTickInterval = nh.advertise<std_msgs::Int32>("right_tick_interval", 1);

    firmware_state = nh.advertise<std_msgs::String>("firmware_version", 1, true);
    battery_state = nh.advertise<sensor_msgs::BatteryState>("battery_state", 1);
    motor_power_active = nh.advertise<std_msgs::Bool>("motor_power_active", 1);

    motor_state = nh.advertise<ubiquity_motor::MotorState>("motor_state", 1);
    leftCurrent = nh.advertise<std_msgs::Float32>("left_current", 1);
    rightCurrent = nh.advertise<std_msgs::Float32>("right_current", 1);

    sendPid_count = 0;
    num_fw_params = 8;     // number of params sent if any change

    estop_motor_power_off = false;  // Keeps state of ESTOP switch where true is in ESTOP state

    // Save default hardware encoder specifics for ticks in one radian of rotation of main wheel
    this->ticks_per_radian = TICKS_PER_RADIAN_DEFAULT; 
    this->wheel_gear_ratio = WHEEL_GEAR_RATIO_DEFAULT;

    fw_params = firmware_params;

    prev_fw_params.pid_proportional = -1;
    prev_fw_params.pid_integral = -1;
    prev_fw_params.pid_derivative = -1;
    prev_fw_params.pid_velocity = -1;
    prev_fw_params.pid_denominator = -1;
    prev_fw_params.pid_control = -1;
    prev_fw_params.pid_moving_buffer_size = -1;
    prev_fw_params.max_speed_fwd = -1;
    prev_fw_params.max_speed_rev = -1;
    prev_fw_params.deadman_timer = -1;
    prev_fw_params.deadzone_enable = -1;
    prev_fw_params.hw_options = -1;
    prev_fw_params.option_switch = -1;
    prev_fw_params.system_events = -1;
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
    diag_updater.add("PidParamP", &motor_diag_, &MotorDiagnostics::motor_pid_p_status);
    diag_updater.add("PidParamI", &motor_diag_, &MotorDiagnostics::motor_pid_i_status);
    diag_updater.add("PidParamD", &motor_diag_, &MotorDiagnostics::motor_pid_d_status);
    diag_updater.add("PidParamV", &motor_diag_, &MotorDiagnostics::motor_pid_v_status);
    diag_updater.add("PidMaxPWM", &motor_diag_, &MotorDiagnostics::motor_max_pwm_status);
    diag_updater.add("FirmwareOptions", &motor_diag_, &MotorDiagnostics::firmware_options_status);
    diag_updater.add("FirmwareDate", &motor_diag_, &MotorDiagnostics::firmware_date_status);
}

MotorHardware::~MotorHardware() { delete motor_serial_; }

// Close of the serial port is used in a special case of suspending the motor controller
// so that another service can load firmware or do direct MCB diagnostics
void MotorHardware::closePort() {
    motor_serial_->closePort();
}

// After we have given up the MCB we open serial port again using current instance of Serial
bool MotorHardware::openPort() {
    return motor_serial_->openPort();
}

void MotorHardware::clearCommands() {
    for (size_t i = 0; i < sizeof(joints_) / sizeof(joints_[0]); i++) {
        joints_[i].velocity_command = 0;
    }
}

// Read the current wheel positions in radians both at once for a snapshot of position
void MotorHardware::getWheelJointPositions(double &leftWheelPosition, double &rightWheelPosition) {
    leftWheelPosition  = joints_[WheelJointLocation::Left].position;
    rightWheelPosition = joints_[WheelJointLocation::Right].position;
    return;
}

// Set the current wheel joing velocities in radians/sec both at once for a snapshot of velocity
// This interface is supplied because MotorHardware does not do a loop on it's own
void MotorHardware::setWheelJointVelocities(double leftWheelVelocity, double rightWheelVelocity) {
    joints_[WheelJointLocation::Left].velocity  = leftWheelVelocity;
    joints_[WheelJointLocation::Right].velocity = rightWheelVelocity;
    return;
}

// Publish motor state conditions
void MotorHardware::publishMotorState(void) {
    ubiquity_motor::MotorState mstateMsg;

    mstateMsg.header.frame_id = "";   // Could be base_link.  We will use empty till required
    mstateMsg.header.stamp    = ros::Time::now();

    mstateMsg.leftPosition    = joints_[WheelJointLocation::Left].position;
    mstateMsg.rightPosition   = joints_[WheelJointLocation::Right].position;
    mstateMsg.leftRotateRate  = joints_[WheelJointLocation::Left].velocity;
    mstateMsg.rightRotateRate = joints_[WheelJointLocation::Right].velocity;
    mstateMsg.leftCurrent     = motor_diag_.motorCurrentLeft;
    mstateMsg.rightCurrent    = motor_diag_.motorCurrentRight;
    mstateMsg.leftPwmDrive    = motor_diag_.motorPwmDriveLeft;
    mstateMsg.rightPwmDrive   = motor_diag_.motorPwmDriveRight;
    motor_state.publish(mstateMsg);
    return;
}

// readInputs() will receive serial and act on the response from motor controller
//
// The motor controller sends unsolicited messages periodically so we must read the
// messages to update status in near realtime
//
void MotorHardware::readInputs(uint32_t index) {
    while (motor_serial_->commandAvailable()) {
        MotorMessage mm;
        mm = motor_serial_->receiveCommand();
        if (mm.getType() == MotorMessage::TYPE_RESPONSE) {
            switch (mm.getRegister()) {

                case MotorMessage::REG_SYSTEM_EVENTS:
                    if ((mm.getData() & MotorMessage::SYS_EVENT_POWERON) != 0) {
                        ROS_WARN("Firmware System Event for PowerOn transition");
                        system_events = mm.getData();
                    }
                    break;
                case MotorMessage::REG_FIRMWARE_VERSION:
                    if (mm.getData() < LOWEST_FIRMWARE_VERSION) {
                        ROS_FATAL("Firmware version %d, expect %d or above",
                                  mm.getData(), LOWEST_FIRMWARE_VERSION);
                        throw std::runtime_error("Firmware version too low");
                    } else {
                        ROS_INFO_ONCE("Firmware version %d", mm.getData());
                        firmware_version = mm.getData();
                        motor_diag_.firmware_version = firmware_version;
                    }
                    publishFirmwareInfo();
                    break;

                case MotorMessage::REG_FIRMWARE_DATE:
                    // Firmware date is only supported as of fw version MIN_FW_FIRMWARE_DATE
                    ROS_INFO_ONCE("Firmware date 0x%x (format 0xYYYYMMDD)", mm.getData());
                    firmware_date = mm.getData();
                    motor_diag_.firmware_date = firmware_date;

                    publishFirmwareInfo();
                    break;

                case MotorMessage::REG_BOTH_ODOM: {
                    /* 
                     * ODOM messages from the MCB tell us how far wheels have rotated
                     *
                     * It is here we keep track of wheel joint position 
                     * The odom counts from the MCB are the incremental number of ticks since last report
                     *  WARNING: IF WE LOOSE A MESSAGE WE DRIFT FROM REAL POSITION
                     */
                    int32_t odom = mm.getData();
                    // ROS_ERROR("odom signed %d", odom);
                    int16_t odomLeft = (odom >> 16) & 0xffff;
                    int16_t odomRight = odom & 0xffff;

                    // Debug code to be used for verification
                    g_odomLeft  += odomLeft;
                    g_odomRight += odomRight;
                    g_odomEvent += 1;
                    //if ((g_odomEvent % 50) == 1) { ROS_ERROR("leftOdom %d rightOdom %d", g_odomLeft, g_odomRight); }

                    // Due to extreme wheel slip that is required to turn a 4WD robot we are doing a scale factor.
                    // When doing a rotation on the 4WD robot that is in place where linear velocity is zero
                    // we will adjust the odom values for wheel joint rotation using the scale factor.
                    double odom4wdRotationScale = 1.0;

                    // Determine if we are rotating then set a scale to account for rotational wheel slip
                    double near0WheelVel = (double)(WHEEL_VELOCITY_NEAR_ZERO);
                    double leftWheelVel  =  g_radiansLeft;  // rotational speed of motor
                    double rightWheelVel =  g_radiansRight; // rotational speed of motor

                    int leftDir  = (leftWheelVel  >= (double)(0.0)) ? 1 : -1;
                    int rightDir = (rightWheelVel >= (double)(0.0)) ? 1 : -1;
                    int is4wdMode = (fw_params.hw_options & MotorMessage::OPT_DRIVE_TYPE_4WD);
                    if (
                        // Is this in 4wd robot mode
                        (is4wdMode != 0)

                        // Do the joints have Different rotational directions
                        && ((leftDir + rightDir) == 0)

                        // Are Both joints not near joint velocity of 0
                        && ((fabs(leftWheelVel)  > near0WheelVel) && (fabs(rightWheelVel) > near0WheelVel))

                        // Is the difference of the two absolute values of the joint velocities near zero
                        && ((fabs(leftWheelVel) - fabs(rightWheelVel)) < near0WheelVel) )  {

                        odom4wdRotationScale = g_odom4wdRotationScale;
                        if ((index % 16) == 1) {   // This throttles the messages for rotational torque enhancement
                            ROS_INFO("ROTATIONAL_SCALING_ACTIVE: odom4wdRotationScale = %4.2f [%4.2f, %4.2f] [%d,%d] opt 0x%x 4wd=%d",
                                odom4wdRotationScale, leftWheelVel, rightWheelVel, leftDir, rightDir, fw_params.hw_options, is4wdMode);
                        }
                    } else {
                        if (fabs(leftWheelVel) > near0WheelVel) {
                            ROS_DEBUG("odom4wdRotationScale = %4.2f [%4.2f, %4.2f] [%d,%d] opt 0x%x 4wd=%d",
                                odom4wdRotationScale, leftWheelVel, rightWheelVel, leftDir, rightDir, fw_params.hw_options, is4wdMode);
                        }
                    }

                    // Add or subtract from position in radians using the incremental odom value
                    joints_[WheelJointLocation::Left].position  +=
                        ((double)odomLeft  / (this->ticks_per_radian * odom4wdRotationScale));
                    joints_[WheelJointLocation::Right].position +=
                        ((double)odomRight / (this->ticks_per_radian * odom4wdRotationScale));

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

                case MotorMessage::REG_PWM_BOTH_WHLS: {
                    int32_t bothPwm = mm.getData();
                    motor_diag_.motorPwmDriveLeft  = (bothPwm >> 16) & 0xffff;
                    motor_diag_.motorPwmDriveRight = bothPwm & 0xffff;
                    break;
                }

                case MotorMessage::REG_LEFT_CURRENT: {
                    // Motor current is an absolute value and goes up from a nominal count of near 1024
                    // So we subtract a nominal offset then multiply count * scale factor to get amps
                    int32_t data = mm.getData() & 0xffff;
                    motor_diag_.motorCurrentLeft =
                        (double)(data - motor_diag_.motorAmpsZeroAdcCount) * MOTOR_AMPS_PER_ADC_COUNT;
                    break;
                }
                case MotorMessage::REG_RIGHT_CURRENT: {
                    // Motor current is an absolute value and goes up from a nominal count of near 1024
                    // So we subtract a nominal offset then multiply count * scale factor to get amps
                    int32_t data = mm.getData() & 0xffff;
                    motor_diag_.motorCurrentRight =
                        (double)(data - motor_diag_.motorAmpsZeroAdcCount) * MOTOR_AMPS_PER_ADC_COUNT;
                    break;
                }

                case MotorMessage::REG_HW_OPTIONS: {
                    int32_t data = mm.getData();

                    // Enable or disable hardware options reported from firmware
                    motor_diag_.firmware_options = data;

                    // Set radians per encoder tic based on encoder specifics
                    if (data & MotorMessage::OPT_ENC_6_STATE) {
                        ROS_WARN_ONCE("Encoder Resolution: 'Enhanced'");
                        fw_params.hw_options |= MotorMessage::OPT_ENC_6_STATE;
                        this->ticks_per_radian = this->getWheelTicksPerRadian();
                    } else {
                        ROS_WARN_ONCE("Encoder Resolution: 'Standard'");
                        fw_params.hw_options &= ~MotorMessage::OPT_ENC_6_STATE;
                        this->ticks_per_radian  = this->getWheelTicksPerRadian() / (double)(2.0);
                    }

                    if (data & MotorMessage::OPT_WHEEL_TYPE_THIN) {
                        ROS_WARN_ONCE("Wheel type is: 'thin'");
                        fw_params.hw_options |= MotorMessage::OPT_WHEEL_TYPE_THIN;
                    } else {
                        ROS_WARN_ONCE("Wheel type is: 'standard'");
                        fw_params.hw_options &= ~MotorMessage::OPT_WHEEL_TYPE_THIN;
                    }

                    if (data & MotorMessage::OPT_DRIVE_TYPE_4WD) {
                        ROS_WARN_ONCE("Drive type is: '4wd'");
                        fw_params.hw_options |= MotorMessage::OPT_DRIVE_TYPE_4WD;
                    } else {
                        ROS_WARN_ONCE("Drive type is: '2wd'");
                        fw_params.hw_options &= ~MotorMessage::OPT_DRIVE_TYPE_4WD;
                    }

                    if (data & MotorMessage::OPT_WHEEL_DIR_REVERSE) {
                        ROS_WARN_ONCE("Wheel direction is: 'reverse'");
                        fw_params.hw_options |= MotorMessage::OPT_WHEEL_DIR_REVERSE;
                    } else {
                        ROS_WARN_ONCE("Wheel direction is: 'standard'");
                        fw_params.hw_options &= ~MotorMessage::OPT_WHEEL_DIR_REVERSE;
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

                    // Hardcoded to a sealed lead acid 12S battery, but adjustable for future use
                    bstate.percentage = calculateBatteryPercentage(bstate.voltage, 12, SLA_AGM);
                    bstate.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
                    bstate.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
                    bstate.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
                    battery_state.publish(bstate);

                    motor_diag_.battery_voltage = bstate.voltage;
                    motor_diag_.battery_voltage_low_level = MotorHardware::fw_params.battery_voltage_low_level;
                    motor_diag_.battery_voltage_critical = MotorHardware::fw_params.battery_voltage_critical;
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

                case MotorMessage::REG_TINT_BOTH_WHLS: {   // As of v41 show time between wheel enc edges
                    int32_t data = mm.getData();
                    uint16_t leftTickSpacing = (data >> 16) & 0xffff;
                    uint16_t rightTickSpacing = data & 0xffff;
                    uint16_t tickCap = 0;    // We can cap the max value if desired

                    if ((tickCap > 0) && (leftTickSpacing  > tickCap)) { leftTickSpacing  = tickCap; }
                    if ((tickCap > 0) && (rightTickSpacing > tickCap)) { rightTickSpacing = tickCap; }

                    // Publish the two wheel tic intervals
                    std_msgs::Int32 leftInterval;
                    std_msgs::Int32 rightInterval;

                    leftInterval.data  = leftTickSpacing;
                    rightInterval.data = rightTickSpacing;

                    // Only publish the tic intervals when wheels are moving
                    if (data > 1) {     // Optionally show the intervals for debug
                        leftTickInterval.publish(leftInterval);
                        rightTickInterval.publish(rightInterval);

                        ROS_DEBUG("Tic Ints M1 %d [0x%x]  M2 %d [0x%x]",  
                            leftTickSpacing, leftTickSpacing, rightTickSpacing, rightTickSpacing);
                    }
                }
                default:
                    break;
            }
        }
    }
}


void MotorHardware::publishFirmwareInfo(){
    //publish the firmware version and optional date to ROS
    if(firmware_version > 0){

        std_msgs::String fstate;
        fstate.data = "v"+std::to_string(firmware_version);

        if(firmware_date > 0){
            std::stringstream stream;
            stream << std::hex << firmware_date;
            std::string daycode(stream.str());

            fstate.data +=" "+daycode;
        }

        firmware_state.publish(fstate);
    }
}

// calculateBatteryPercentage() takes in battery voltage, number of cells, and type; returns approximate percentage
//
// A battery type is defined by an array of 11 values, each corresponding to one 10% sized step from 0% to 100%.
// If the values fall between the steps, they are linearly interpolated to give a more accurate reading.
//
float MotorHardware::calculateBatteryPercentage(float voltage, int cells, const float* type) {
    float onecell = voltage / (float)cells;

    if(onecell >= type[10])
        return 1.0;
    else if(onecell <= type[0])
        return 0.0;

    int upper = 0;
    int lower = 0;

    for(int i = 0; i < 11; i++){
        if(onecell > type[i]){
            lower = i;
        }else{
            upper = i;
            break;
        }
    }

    float deltavoltage = type[upper] - type[lower];
    float between_percent = (onecell - type[lower]) / deltavoltage;

    return (float)lower * 0.1 + between_percent * 0.1;
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

    g_radiansLeft  = left_radians;
    g_radiansRight = right_radians;

    // We are going to implement a message when robot is moving very fast or rotating very fast
    if (((left_radians / VELOCITY_READ_PER_SECOND)  > HIGH_SPEED_RADIANS) || 
        ((right_radians / VELOCITY_READ_PER_SECOND) > HIGH_SPEED_RADIANS)) {
        ROS_WARN("Wheel rotation at high radians per sec.  Left %f rad/s Right %f rad/s",
            left_radians, right_radians);
    }

    int16_t left_speed  = calculateSpeedFromRadians(left_radians);
    int16_t right_speed = calculateSpeedFromRadians(right_radians);

    // The masking with 0x0000ffff is necessary for handling -ve numbers
    int32_t data = (left_speed << 16) | (right_speed & 0x0000ffff);
    both.setData(data);

    std_msgs::Int32 smsg;
    smsg.data = left_speed;

    motor_serial_->transmitCommand(both);

    // ROS_ERROR("velocity_command %f rad/s %f rad/s",
    // joints_[WheelJointLocation::Left].velocity_command, joints_[WheelJointLocation::Right].velocity_command);
    // joints_[LEFT_WHEEL_JOINT].velocity_command, joints_[RIGHT_WHEEL_JOINT].velocity_command);
    // ROS_ERROR("SPEEDS %d %d", left.getData(), right.getData());
}

// writeSpeeds()  Take in radians per sec for wheels and send in message to controller
//
// Legacy interface where no speed overrides are supported
//
void MotorHardware::writeSpeeds() {
    // This call pulls in speeds from the joints array maintained by other layers

    double  left_radians  = joints_[WheelJointLocation::Left].velocity_command;
    double  right_radians = joints_[WheelJointLocation::Right].velocity_command;

    writeSpeedsInRadians(left_radians, right_radians);
}

// areWheelSpeedsLower()  Determine if all wheel joint speeds are below given threshold
//
int MotorHardware::areWheelSpeedsLower(double wheelSpeedRadPerSec) {
    int retCode = 0;

    // This call pulls in speeds from the joints array maintained by other layers

    double  left_radians  = joints_[WheelJointLocation::Left].velocity_command;
    double  right_radians = joints_[WheelJointLocation::Right].velocity_command;

    if ((std::abs(left_radians)  < wheelSpeedRadPerSec) &&
        (std::abs(right_radians) < wheelSpeedRadPerSec)) {
        retCode = 1;
    }

    return retCode;
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

// Request the MCB system event register
void MotorHardware::requestSystemEvents() {
    MotorMessage sys_event_msg;
    sys_event_msg.setRegister(MotorMessage::REG_SYSTEM_EVENTS);
    sys_event_msg.setType(MotorMessage::TYPE_READ);
    sys_event_msg.setData(0);
    motor_serial_->transmitCommand(sys_event_msg);
}

// Read the wheel currents in amps
void MotorHardware::getMotorCurrents(double &currentLeft, double &currentRight) {
    currentLeft  = motor_diag_.motorCurrentLeft;
    currentRight = motor_diag_.motorCurrentRight;
    return;
}


// Due to greatly limited pins on the firmware processor the host figures out the hardware rev and sends it to fw
// The hardware version is 0x0000MMmm  where MM is major rev like 4 and mm is minor rev like 9 for first units.
// The 1st firmware version this is set for is 32, before it was always 1
void MotorHardware::setHardwareVersion(int32_t hardware_version) {
    ROS_INFO("setting hardware_version to %x", (int)hardware_version);
    this->hardware_version = hardware_version;
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

// Setup the Wheel Type. Overrides mode in use on hardware
// This used to only be standard but THIN_WHEELS were added in Jun 2020
void MotorHardware::setWheelType(int32_t new_wheel_type) {

    MotorMessage ho;
    switch(new_wheel_type) {
        case MotorMessage::OPT_WHEEL_TYPE_STANDARD:
        case MotorMessage::OPT_WHEEL_TYPE_THIN:
            ROS_INFO_ONCE("setting MCB wheel type %d", (int)new_wheel_type);
            wheel_type = new_wheel_type;
            ho.setRegister(MotorMessage::REG_WHEEL_TYPE);
            ho.setType(MotorMessage::TYPE_WRITE);
            ho.setData(wheel_type);
            motor_serial_->transmitCommand(ho);
            break;
        default:
            ROS_ERROR("Illegal MCB wheel type 0x%x will not be set!", (int)new_wheel_type);
    }
}

// A simple fetch of the wheel_gear_ratio 
double MotorHardware::getWheelGearRatio(void) {
    return this->wheel_gear_ratio;
}

// A simple fetch of the encoder ticks per radian of wheel rotation
double MotorHardware::getWheelTicksPerRadian(void) {
    double result = this->getWheelGearRatio() * TICKS_PER_RAD_FROM_GEAR_RATIO;
    return result;
}

// Setup the local Wheel gear ratio so the motor hardware layer can adjust wheel odom reporting
// This gear ratio was introduced for a new version of the standard wheels in late 2021 production
// This is slightly more complex in that it is compounded with the now default 6 state encoder hw option
void MotorHardware::setWheelGearRatio(double new_wheel_gear_ratio) {
    // This gear ratio is not used by the firmware so it is a simple state element in this module
    this->wheel_gear_ratio = new_wheel_gear_ratio;
    this->ticks_per_radian  = this->getWheelTicksPerRadian();   // Need to also reset ticks_per_radian
    if ((fw_params.hw_options & MotorMessage::OPT_ENC_6_STATE) == 0) {
        this->ticks_per_radian = this->ticks_per_radian / (double)(2.0);   // 3 state was half
    }
    ROS_INFO("Setting Wheel gear ratio to %6.4f and tics_per_radian to %6.4f",
        this->wheel_gear_ratio, this->ticks_per_radian);
}

// Setup the Drive Type. Overrides mode in use on hardware
// This used to only be 2WD and use of THIN_WHEELS set 4WD
// We are not trying to decouple wheel type from drive type
// This register always existed but was a do nothing till firmware v42
void MotorHardware::setDriveType(int32_t drive_type) {
    ROS_INFO_ONCE("setting MCB drive type %d", (int)drive_type);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DRIVE_TYPE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(drive_type);
    motor_serial_->transmitCommand(mm);
}

// Setup the PID control options. Overrides modes in use on hardware
void MotorHardware::setPidControl(int32_t pid_control_word) {
    ROS_INFO_ONCE("setting MCB pid control word to 0x%x", (int)pid_control_word);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_PID_CONTROL);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(pid_control_word);
    motor_serial_->transmitCommand(mm);
}

// Do a one time NULL of the wheel setpoint based on current position error
// This allows to relieve stress in static situation where wheels cannot slip to match setpoint
void MotorHardware::nullWheelErrors(void) {
    ROS_DEBUG("Nulling MCB wheel errors using current wheel positions");
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_WHEEL_NULL_ERR);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(MotorOrWheelNumber::Motor_M1|MotorOrWheelNumber::Motor_M2);
    motor_serial_->transmitCommand(mm);
}

// Setup the Wheel direction. Overrides mode in use on hardware
// This allows for customer to install wheels on cutom robots as they like
void MotorHardware::setWheelDirection(int32_t wheel_direction) {
    ROS_INFO("setting MCB wheel direction to %d", (int)wheel_direction);
    MotorMessage ho;
    ho.setRegister(MotorMessage::REG_WHEEL_DIR);
    ho.setType(MotorMessage::TYPE_WRITE);
    ho.setData(wheel_direction);
    motor_serial_->transmitCommand(ho);
}

// A simple fetch of the pid_control word from firmware params
int MotorHardware::getPidControlWord(void) {
    int pidControlWord;
    pidControlWord = motor_diag_.fw_pid_control;
    return pidControlWord;
}

// Read the controller board option switch itself that resides on the I2C bus but is on the MCB
// This call inverts the bits because a shorted option switch is a 0 where we want it as 1
// If return is negative something went wrong
int MotorHardware::getOptionSwitch(void) {
    uint8_t buf[16];
    int retBits = 0;
    ROS_INFO("reading MCB option switch on the I2C bus");
    int retCount = i2c_BufferRead(I2C_DEVICE, I2C_PCF8574_8BIT_ADDR, &buf[0], -1, 1);
    if (retCount < 0) {
        ROS_ERROR("Error %d in reading MCB option switch at 8bit Addr 0x%x",
            retCount, I2C_PCF8574_8BIT_ADDR);
        retBits = retCount;
    } else if (retCount != 1) {
        ROS_ERROR("Cannot read byte from MCB option switch at 8bit Addr 0x%x", I2C_PCF8574_8BIT_ADDR);
        retBits = -1;
    } else {
        retBits = (0xff) & ~buf[0];
    }

    return retBits;
}

// Setup the controller board option switch register which comes from the I2C 8-bit IO chip on MCB
void MotorHardware::setOptionSwitchReg(int32_t option_switch_bits) {
    ROS_INFO("setting MCB option switch register to 0x%x", (int)option_switch_bits);
    MotorMessage os;
    os.setRegister(MotorMessage::REG_OPTION_SWITCH);
    os.setType(MotorMessage::TYPE_WRITE);
    os.setData(option_switch_bits);
    motor_serial_->transmitCommand(os);
}

// Setup the controller board system event register or clear bits in the register
void MotorHardware::setSystemEvents(int32_t system_events) {
    ROS_INFO("setting MCB system event register to %d", (int)system_events);
    MotorMessage se;
    se.setRegister(MotorMessage::REG_SYSTEM_EVENTS);
    se.setType(MotorMessage::TYPE_WRITE);
    se.setData(system_events);
    motor_serial_->transmitCommand(se);
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
    fw_params.pid_control = fp.pid_control;
    fw_params.max_pwm = fp.max_pwm;
    fw_params.estop_pid_threshold = fp.estop_pid_threshold;
}

// Forces next calls to sendParams() to always update each parameter.
// KEEP THIS IN SYNC WITH CHANGES TO sendParams()
void MotorHardware::forcePidParamUpdates() {

    // Reset each of the flags that causes parameters to be  sent to MCB by sendParams()
    prev_fw_params.pid_proportional = -1;
    prev_fw_params.pid_integral = -1;
    prev_fw_params.pid_derivative = -1;
    prev_fw_params.pid_velocity = -1;
    prev_fw_params.pid_denominator = -1;
    prev_fw_params.pid_moving_buffer_size = -1;
    prev_fw_params.max_pwm = -1;
    prev_fw_params.pid_control = 1;
}

void MotorHardware::sendParams() {
    std::vector<MotorMessage> commands;

    // ROS_ERROR("sending PID %d %d %d %d",
    //(int)p_value, (int)i_value, (int)d_value, (int)denominator_value);

    // Only send one register at a time to avoid overwhelming serial comms
    // SUPPORT NOTE!  Adjust modulo for total parameters in the cycle
    //                and be sure no duplicate modulos are used!
    int cycle = (sendPid_count++) % num_fw_params;     // MUST BE THE TOTAL NUMBER IN THIS HANDLING

    if (cycle == 0 &&
        fw_params.pid_proportional != prev_fw_params.pid_proportional) {
        ROS_WARN("Setting PidParam P to %d", fw_params.pid_proportional);
        prev_fw_params.pid_proportional = fw_params.pid_proportional;
        motor_diag_.fw_pid_proportional = fw_params.pid_proportional;
        MotorMessage p;
        p.setRegister(MotorMessage::REG_PARAM_P);
        p.setType(MotorMessage::TYPE_WRITE);
        p.setData(fw_params.pid_proportional);
        commands.push_back(p);
    }

    if (cycle == 1 && fw_params.pid_integral != prev_fw_params.pid_integral) {
        ROS_WARN("Setting PidParam I to %d", fw_params.pid_integral);
        prev_fw_params.pid_integral = fw_params.pid_integral;
        motor_diag_.fw_pid_integral = fw_params.pid_integral;
        MotorMessage i;
        i.setRegister(MotorMessage::REG_PARAM_I);
        i.setType(MotorMessage::TYPE_WRITE);
        i.setData(fw_params.pid_integral);
        commands.push_back(i);
    }

    if (cycle == 2 &&
        fw_params.pid_derivative != prev_fw_params.pid_derivative) {
        ROS_WARN("Setting PidParam D to %d", fw_params.pid_derivative);
        prev_fw_params.pid_derivative = fw_params.pid_derivative;
        motor_diag_.fw_pid_derivative = fw_params.pid_derivative;
        MotorMessage d;
        d.setRegister(MotorMessage::REG_PARAM_D);
        d.setType(MotorMessage::TYPE_WRITE);
        d.setData(fw_params.pid_derivative);
        commands.push_back(d);
    }

    if (cycle == 3 && (motor_diag_.firmware_version >= MIN_FW_PID_V_TERM) &&
        fw_params.pid_velocity != prev_fw_params.pid_velocity) {
        ROS_WARN("Setting PidParam V to %d", fw_params.pid_velocity);
        prev_fw_params.pid_velocity = fw_params.pid_velocity;
        motor_diag_.fw_pid_velocity = fw_params.pid_velocity;
        MotorMessage v;
        v.setRegister(MotorMessage::REG_PARAM_V);
        v.setType(MotorMessage::TYPE_WRITE);
        v.setData(fw_params.pid_velocity);
        commands.push_back(v);
    }

    if (cycle == 4 &&
        fw_params.pid_denominator != prev_fw_params.pid_denominator) {
        ROS_WARN("Setting PidParam Denominator to %d", fw_params.pid_denominator);
        prev_fw_params.pid_denominator = fw_params.pid_denominator;
        motor_diag_.fw_pid_denominator = fw_params.pid_denominator;
        MotorMessage denominator;
        denominator.setRegister(MotorMessage::REG_PARAM_C);
        denominator.setType(MotorMessage::TYPE_WRITE);
        denominator.setData(fw_params.pid_denominator);
        commands.push_back(denominator);
    }

    if (cycle == 5 &&
        fw_params.pid_moving_buffer_size !=
            prev_fw_params.pid_moving_buffer_size) {
        ROS_WARN("Setting PidParam D window to %d", fw_params.pid_moving_buffer_size);
        prev_fw_params.pid_moving_buffer_size =
            fw_params.pid_moving_buffer_size;
        motor_diag_.fw_pid_moving_buffer_size = fw_params.pid_moving_buffer_size;
        MotorMessage winsize;
        winsize.setRegister(MotorMessage::REG_MOVING_BUF_SIZE);
        winsize.setType(MotorMessage::TYPE_WRITE);
        winsize.setData(fw_params.pid_moving_buffer_size);
        commands.push_back(winsize);
    }

    if (cycle == 6 &&
        fw_params.max_pwm != prev_fw_params.max_pwm) {
        ROS_WARN("Setting PidParam max_pwm to %d", fw_params.max_pwm);
        prev_fw_params.max_pwm = fw_params.max_pwm;
        motor_diag_.fw_max_pwm = fw_params.max_pwm;
        MotorMessage maxpwm;
        maxpwm.setRegister(MotorMessage::REG_MAX_PWM);
        maxpwm.setType(MotorMessage::TYPE_WRITE);
        maxpwm.setData(fw_params.max_pwm);
        commands.push_back(maxpwm);
    }

    if (cycle == 7 &&
        fw_params.pid_control != prev_fw_params.pid_control) {
        ROS_WARN("Setting PidParam pid_control to %d", fw_params.pid_control);
        prev_fw_params.pid_control = fw_params.pid_control;
        motor_diag_.fw_pid_control = fw_params.pid_control;
        MotorMessage mmsg;
        mmsg.setRegister(MotorMessage::REG_PID_CONTROL);
        mmsg.setType(MotorMessage::TYPE_WRITE);
        mmsg.setData(fw_params.pid_control);
        commands.push_back(mmsg);
    }

    // SUPPORT NOTE!  Adjust max modulo for total parameters in the cycle, be sure no duplicates used!

    if (commands.size() != 0) {
        motor_serial_->transmitCommands(commands);
    }
}

// Get current battery voltage
float MotorHardware::getBatteryVoltage(void) {
    return motor_diag_.battery_voltage;   // We keep battery_voltage in diagnostic context
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
int16_t MotorHardware::calculateSpeedFromRadians(double radians) {
    int16_t speed;
    double  encoderFactor = 1.0;
    double  speedFloat;

    // The firmware accepts same units for speed value
    // and will deal with it properly depending on encoder handling in use
    if (fw_params.hw_options & MotorMessage::OPT_ENC_6_STATE) {
        encoderFactor = (double)(0.5);
    }

    speedFloat = encoderFactor * radians * ((getWheelTicksPerRadian() * (double)(4.0)) / VELOCITY_READ_PER_SECOND);
    speed =  boost::math::iround(speedFloat);

    return speed;
}

double MotorHardware::calculateRadiansFromTicks(int16_t ticks) {
    double result;

    result =  ((double)ticks * VELOCITY_READ_PER_SECOND) / (getWheelTicksPerRadian() * (double)(4.0));
    return result;
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
    if (battery_voltage < battery_voltage_low_level) {
        stat.summary(DiagnosticStatusWrapper::WARN, "Battery low");
    }
    else if (battery_voltage < battery_voltage_critical) {
        stat.summary(DiagnosticStatusWrapper::ERROR, "Battery critical");
    }
    else {
        stat.summary(DiagnosticStatusWrapper::OK, "Battery OK");
    }
}

// PID parameters for motor control
void MotorDiagnostics::motor_pid_p_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam P", fw_pid_proportional);
    stat.summary(DiagnosticStatus::OK, "PID Parameter P");
}
void MotorDiagnostics::motor_pid_i_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam I", fw_pid_integral);
    stat.summary(DiagnosticStatus::OK, "PID Parameter I");
}
void MotorDiagnostics::motor_pid_d_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam D", fw_pid_derivative);
    stat.summary(DiagnosticStatus::OK, "PID Parameter D");
}
void MotorDiagnostics::motor_pid_v_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam V", fw_pid_velocity);
    stat.summary(DiagnosticStatus::OK, "PID Parameter V");
}
void MotorDiagnostics::motor_max_pwm_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam MaxPWM", fw_max_pwm);
    stat.summary(DiagnosticStatus::OK, "PID Max PWM");
}


void MotorDiagnostics::motor_power_status(DiagnosticStatusWrapper &stat) {
    stat.add("Motor Power", !estop_motor_power_off);
    if (estop_motor_power_off == false) {
        stat.summary(DiagnosticStatusWrapper::OK, "Motor power on");
    }
    else {
        stat.summary(DiagnosticStatusWrapper::WARN, "Motor power off");
    }
}


// Show firmware options and give readable decoding of the meaning of the bits
void MotorDiagnostics::firmware_options_status(DiagnosticStatusWrapper &stat) {
    stat.add("Firmware Options", firmware_options);
    std::string option_descriptions("");
    if (firmware_options & MotorMessage::OPT_ENC_6_STATE) {
        option_descriptions += "High resolution encoders";
    } else {
        option_descriptions += "Standard resolution encoders";
    }
    if (firmware_options & MotorMessage::OPT_WHEEL_TYPE_THIN) {
        option_descriptions +=  ", Thin gearless wheels";
    } else {
        option_descriptions +=  ", Standard wheels";
    }
    if (firmware_options & MotorMessage::OPT_DRIVE_TYPE_4WD) {
        option_descriptions +=  ", 4 wheel drive";
    } else {
        option_descriptions +=  ", 2 wheel drive";
    }
    if (firmware_options & MotorMessage::OPT_WHEEL_DIR_REVERSE) {
        // Only indicate wheel reversal if that has been set as it is non-standard
        option_descriptions +=  ", Reverse polarity wheels";
    }
    stat.summary(DiagnosticStatusWrapper::OK, option_descriptions);
}

// i2c_BufferRead()   A host OS system specific  utility to open, read, close from an I2C device
//
// The I2C address is the 8-bit address which is the 7-bit addr shifted left in some code
// If chipRegAddr is greater than 1 we write this out for the internal chip address for the following read(s)
//
// Returns number of bytes read where 0 or less implies some form of failure
//
// NOTE: The i2c8bitAddr will be shifted right one bit to use as 7-bit I2C addr
//
static int i2c_BufferRead(const char *i2cDevFile, uint8_t i2c8bitAddr,
                          uint8_t *pBuffer, int16_t chipRegAddr, uint16_t NumBytesToRead)
{
   int bytesRead = 0;
   int retCode   = 0;

    int fd;                                         // File descrition
    int  address   = i2c8bitAddr >> 1;              // Address of the I2C device
    uint8_t buf[8];                                 // Buffer for data being written to the i2c device

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {      // Open port for reading and writing
      retCode = -2;
      ROS_ERROR("Cannot open I2C def of %s with error %s", i2cDevFile, strerror(errno));
      goto exitWithNoClose;
    }

    // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    if (ioctl(fd, I2C_SLAVE, address) != 0) {        // Set the port options and addr of the dev
      retCode = -3;
      ROS_ERROR("Failed to get bus access to I2C device %s!  ERROR: %s", i2cDevFile, strerror(errno));
      goto exitWithFileClose;
    }

    if (chipRegAddr < 0) {     // Suppress reg address if negative value was used
      buf[0] = (uint8_t)(chipRegAddr);          // Internal chip register address
      if ((write(fd, buf, 1)) != 1) {           // Write both bytes to the i2c port
        retCode = -4;
        goto exitWithFileClose;
      }
    }

    bytesRead = read(fd, pBuffer, NumBytesToRead);
    if (bytesRead != NumBytesToRead) {      // verify the number of bytes we requested were read
      retCode = -9;
      goto exitWithFileClose;
    }
    retCode = bytesRead;

  exitWithFileClose:
    close(fd);

  exitWithNoClose:

  return retCode;
}
