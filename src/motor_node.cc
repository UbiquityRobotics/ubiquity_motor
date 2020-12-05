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
#include <ubiquity_motor/motor_parameters.h>
#include <boost/thread.hpp>
#include <string>
#include "controller_manager/controller_manager.h"

static const double BILLION = 1000000000.0;

static FirmwareParams firmware_params;
static CommsParams serial_params;
static NodeParams node_params;

// Dynamic reconfiguration callback for setting ROS parameters dynamically
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
    } else if (level == 32) {
        firmware_params.pid_velocity = config.PID_V;
    } else if (level == 64) {
        firmware_params.max_pwm = config.MAX_PWM;
    } else {
        ROS_ERROR("Unsupported dynamic_reconfigure level %u", level);
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    firmware_params = FirmwareParams(pnh);
    serial_params = CommsParams(pnh);
    node_params = NodeParams(pnh);

    ros::Rate ctrlLoopDelay(node_params.controller_loop_rate);

    // Until we have a holdoff for MCB message overruns we do this delay to be cautious
    // Twice the period for status reports from MCB
    ros::Duration mcbStatusPeriodSec(0.02);

    std::unique_ptr<MotorHardware> robot = nullptr;
    // Keep trying to open serial
    {
        int times = 0;
        while (ros::ok() && robot.get() == nullptr) {
            try {
                robot.reset(new MotorHardware(nh, node_params, serial_params, firmware_params));
            }
            catch (const serial::IOException& e) {
                if (times % 30 == 0)
                    ROS_FATAL("Error opening serial port %s, trying again", serial_params.serial_port.c_str());
            }
            ctrlLoopDelay.sleep();
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
    robot->requestFirmwareVersion();

    // wait for reply then we know firmware version
    mcbStatusPeriodSec.sleep();

    if (robot->firmware_version >= MIN_FW_FIRMWARE_DATE) {
        ROS_INFO("Request the firmware daycode");
        robot->requestFirmwareDate();
    }

    // Make sure firmware is listening
    {
        robot->diag_updater.broadcast(0, "Establishing communication with motors");
        // Start times counter at 1 to prevent false error print (0 % n = 0)
        int times = 1;
        while (ros::ok() && robot->firmware_version == 0) {
            if (times % 30 == 0)
                ROS_ERROR("The Firmware not reporting its version");
                robot->requestFirmwareVersion();
            robot->readInputs();
            mcbStatusPeriodSec.sleep();
            times++;
        }
    }

    if (robot->firmware_version >= MIN_FW_FIRMWARE_DATE) {
        // If supported by firmware also request date code for this version
        robot->requestFirmwareDate();
    }

    // Determine hardware options that can be set by the host to override firmware defaults
    int32_t wheel_type = 0;
    if (node_params.wheel_type == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        ROS_INFO("Firmware default wheel_type will be used.");
    } else {
        // Any other setting leads to host setting the wheel type
        if (node_params.wheel_type == "standard") {
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
            ROS_INFO("Host is specifying wheel_type of '%s'", "standard");
        } else if (node_params.wheel_type == "thin"){
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_THIN;
            ROS_INFO("Host is specifying wheel_type of '%s'", "thin");
        } else {
            ROS_WARN("Invalid wheel_type of '%s' specified! Using wheel type of standard", 
                node_params.wheel_type.c_str());
            node_params.wheel_type = "standard";
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
        }
        // Write out the wheel type setting
        robot->setWheelType(wheel_type);
        mcbStatusPeriodSec.sleep();
    }

    int32_t wheel_direction = 0;
    if (node_params.wheel_direction == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        ROS_INFO("Firmware default wheel_direction will be used.");
    } else {
        // Any other setting leads to host setting the wheel type
        if (node_params.wheel_direction == "standard") {
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
            ROS_INFO("Host is specifying wheel_direction of '%s'", "standard");
        } else if (node_params.wheel_direction == "reverse"){
            wheel_type = MotorMessage::OPT_WHEEL_DIR_REVERSE;
            ROS_INFO("Host is specifying wheel_direction of '%s'", "reverse");
        } else {
            ROS_WARN("Invalid wheel_direction of '%s' specified! Using wheel direction of standard", 
                node_params.wheel_direction.c_str());
            node_params.wheel_direction = "standard";
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
        }
        // Write out the wheel direction setting
        robot->setWheelDirection(wheel_direction);
        mcbStatusPeriodSec.sleep();
    }


    // Tell the controller board firmware what version the hardware is at this time.
    // TODO: Read from I2C.   At this time we only allow setting the version from ros parameters
    if (robot->firmware_version >= MIN_FW_HW_VERSION_SET) {
        ROS_INFO_ONCE("Firmware is version %d. Setting Controller board version to %d", 
            robot->firmware_version, firmware_params.controller_board_version);
        robot->setHardwareVersion(firmware_params.controller_board_version);
        ROS_DEBUG("Controller board version has been set to %d", 
            firmware_params.controller_board_version);
        mcbStatusPeriodSec.sleep();
    }

    // Certain 4WD robots rely on wheels to skid to reach final positions.
    // For such robots when loaded down the wheels can get in a state where they cannot skid.
    // This leads to motor overheating.  This code below sacrifices accurate odometry which
    // in not achievable in such robots anyway to relieve high wattage static drive currents
    // at zero velocity that are due to inability of wheels to slip tiny amounts.
    int32_t wheel_slip_nulling = 0;
    if ((robot->firmware_version >= MIN_FW_WHEEL_NULL_ERROR) && (node_params.drive_type == "4wd")) {
        wheel_slip_nulling = 1;
        ROS_INFO_ONCE("Wheel slip nulling will be enabled for this 4wd system when velocity remains at zero.");
    }

    wheel_slip_nulling = 1;   // !!! DEBUG HACK !!! Force to 1 for avalon code till param flows to here
    ROS_INFO("DEBUG: FORCING Chassis drive_type to 4wd till ROS param works");
    // Use when ROS Param works ROS_INFO("Chassis drive_type is set to '%s'", node_params.drive_type.c_str());

    // Tell the MCB board what the I2C port on it is set to (mcb cannot read it's own switchs!)
    // We could re-read periodically but perhaps only every 5-10 sec but should do it from main loop
    if (robot->firmware_version >= MIN_FW_OPTION_SWITCH) {
        firmware_params.option_switch = robot->getOptionSwitch();
        ROS_INFO_ONCE("Setting firmware option register to 0x%x.", firmware_params.option_switch);
        robot->setOptionSwitchReg(firmware_params.option_switch);
        mcbStatusPeriodSec.sleep();
    }
    
    if (robot->firmware_version >= MIN_FW_SYSTEM_EVENTS) {
        // Start out with zero for system events
        robot->setSystemEvents(0);  // Clear entire system events register
        robot->system_events = 0;
        mcbStatusPeriodSec.sleep();
    }

    // Setup other firmware parameters that could come from ROS parameters
    if (robot->firmware_version >= MIN_FW_ESTOP_SUPPORT) {
        robot->setEstopPidThreshold(firmware_params.estop_pid_threshold);
        mcbStatusPeriodSec.sleep();
        robot->setEstopDetection(firmware_params.estop_detection);
        mcbStatusPeriodSec.sleep();
    }

    if (robot->firmware_version >= MIN_FW_MAX_SPEED_AND_PWM) {
        robot->setMaxFwdSpeed(firmware_params.max_speed_fwd);
        mcbStatusPeriodSec.sleep();
        robot->setMaxRevSpeed(firmware_params.max_speed_rev);
        mcbStatusPeriodSec.sleep();
    }

    // Send out the refreshable firmware parameters, most are the PID terms
    // We must be sure num_fw_params is set to the modulo used in sendParams()
    for (int i = 0; i < robot->num_fw_params; i++) {
        mcbStatusPeriodSec.sleep();
        robot->sendParams();
    }

    float expectedCycleTime = ctrlLoopDelay.expectedCycleTime().toSec();
    ros::Duration minCycleTime = ros::Duration(0.75 * expectedCycleTime);
    ros::Duration maxCycleTime = ros::Duration(1.25 * expectedCycleTime);

    // Clear any commands the robot has at this time
    robot->clearCommands();

    double leftLastWheelPos   = 0.0;
    double rightLastWheelPos  = 0.0;
    double leftWheelPos  = 0.0;
    double rightWheelPos = 0.0;
    robot-> getWheelJointPositions(leftLastWheelPos, rightWheelPos);
    ros::Duration zeroVelocityTime(0.0);
    ros::Duration wheelSlipNullingPeriod(5.0);

    ROS_INFO("Starting motor control node now");

    // Implement a speed reset while ESTOP is active and a delay after release
    double estopReleaseDeadtime = 0.8;
    double estopReleaseDelay    = 0.0;

    // Setup to be able to do periodic operations based on elapsed times
    ros::Time current_time;
    ros::Duration sysMaintPeriod(60.0);     // A periodic MCB maintenance operation
    ros::Duration jointUpdatePeriod(0.10);  // A periodic time to update joint velocity

    ros::Time last_loop_time = ros::Time::now();
    ros::Duration elapsed_loop_time;
    ros::Time last_sys_maint_time = last_loop_time;
    ros::Time last_joint_time = last_loop_time;
    ctrlLoopDelay.sleep();                  // Do delay to setup periodic loop delays

    while (ros::ok()) {
        current_time = ros::Time::now();
        elapsed_loop_time = current_time - last_loop_time;
        last_loop_time = current_time;

        // Determine and set wheel velocities in rad/sec from hardware positions in rads
        ros::Duration elapsed_time = current_time - last_joint_time;
        if (elapsed_time > jointUpdatePeriod) {
            last_joint_time = ros::Time::now();
            double leftWheelVel  = 0.0;
            double rightWheelVel = 0.0;
            robot-> getWheelJointPositions(leftWheelPos, rightWheelPos);
            leftWheelVel  = (leftWheelPos  - leftLastWheelPos)  / elapsed_time.toSec();
            rightWheelVel = (rightWheelPos - rightLastWheelPos) / elapsed_time.toSec();
            robot-> setWheelJointVelocities(leftWheelVel, rightWheelVel); // rad/sec
            leftLastWheelPos  = leftWheelPos;
            rightLastWheelPos = rightWheelPos;

            // Publish motor state at this time
            robot->publishMotorState();

            // Implement static wheel slippage relief
            // Deal with auto-null of MCB wheel setpoints if wheel slip nulling is enabled
            if (wheel_slip_nulling != 0) {
                if (robot->areWheelSpeedsLower(0.01) != 0) {
                    zeroVelocityTime += jointUpdatePeriod;   // add to time at zero velocity
                    if (zeroVelocityTime > wheelSlipNullingPeriod) {
                        // null wheel error if at zero velocity for the nulling check period
                        // OPTION: We could also just null wheels at high wheel power   
                        ROS_DEBUG("Applying wheel slip relief  now");
                        robot->nullWheelErrors();
                        zeroVelocityTime = ros::Duration(0.0);   // reset time we have been at zero velocity
                    }
                } else {
                    zeroVelocityTime = ros::Duration(0.0);   // reset time we have been at zero velocity
                }
            }

        }

        robot->readInputs();
        if ((minCycleTime < elapsed_loop_time) && (elapsed_loop_time < maxCycleTime)) {
            cm.update(current_time, elapsed_loop_time);
        }
        else {
            ROS_WARN("Resetting controller due to time jump %f seconds",
                     elapsed_loop_time.toSec());
            cm.update(current_time, ctrlLoopDelay.expectedCycleTime(), true);
            robot->clearCommands();
        }
        robot->setParams(firmware_params);
        robot->sendParams(); 

        // Periodically watch for MCB board having been reset which is an MCB system event
        // This is also a good place to refresh or show status that may have changed
        elapsed_time = current_time - last_sys_maint_time;
        if ((robot->firmware_version >= MIN_FW_SYSTEM_EVENTS) && (elapsed_time > sysMaintPeriod)) {
            robot->requestSystemEvents();
            mcbStatusPeriodSec.sleep();
            last_sys_maint_time = ros::Time::now();

            // Post a status message for MCB state periodically. This may be nice to do more on as required
            ROS_INFO("Battery = %5.2f V, MCB sys events 0x%x, PidCtrl 0x%x, WheelType '%s'",
                robot->getBatteryVoltage(), robot->system_events, robot->getPidControlWord(),
                (wheel_type == MotorMessage::OPT_WHEEL_TYPE_THIN) ? "thin" : "standard");

            // If we detect a power-on of MCB we should re-initialize MCB
            if ((robot->system_events & MotorMessage::SYS_EVENT_POWERON) != 0) {
                ROS_WARN("Detected Motor controller PowerOn event!");
                robot->setSystemEvents(0);  // Clear entire system events register
                robot->system_events = 0;
                mcbStatusPeriodSec.sleep();
              
                // TODO: Need to re-initialize MCB here. Refer to ubiquity_motor issue #98
            }

            // a periodic refresh of wheel type which is a safety net due to it's importance.
            // This can be removed when a solid message protocol is developed
            if (robot->firmware_version >= MIN_FW_WHEEL_TYPE_THIN) {
                // Refresh the wheel type setting
                robot->setWheelType(wheel_type);
                mcbStatusPeriodSec.sleep();
            }
        }


        // Update motor controller speeds.
        if (robot->getEstopState()) {
            robot->writeSpeedsInRadians(0.0, 0.0);    // We send zero velocity when estop is active
            estopReleaseDelay = estopReleaseDeadtime;
        } else {
            if (estopReleaseDelay > 0.0) {
                // Implement a delay after estop release where velocity remains zero
                estopReleaseDelay -= (1.0/node_params.controller_loop_rate);
                robot->writeSpeedsInRadians(0.0, 0.0);
            } else {
                robot->writeSpeeds();   // Normal operation using current system speeds
            }
        }

        robot->diag_updater.update();
        ctrlLoopDelay.sleep();        // Allow controller to process command
    }

    return 0;
}
