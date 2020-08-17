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

// Process global parameters
static FirmwareParams g_firmware_params;
static CommsParams g_serial_params;
static NodeParams g_node_params;

// Until we have a holdoff for MCB message overruns we do this delay to be cautious
// Twice the period for status reports from MCB
ros::Duration mcbStatusPeriodSec(0.02);

// Dynamic reconfiguration callback for setting ROS parameters dynamically
void PID_update_callback(const ubiquity_motor::PIDConfig& config,
                         uint32_t level) {
    if (level == 0xFFFFFFFF) {
        return;
    }

    if (level == 1) {
        g_firmware_params.pid_proportional = config.PID_P;
    } else if (level == 2) {
        g_firmware_params.pid_integral = config.PID_I;
    } else if (level == 4) {
        g_firmware_params.pid_derivative = config.PID_D;
    } else if (level == 8) {
        g_firmware_params.pid_denominator = config.PID_C;
    } else if (level == 16) {
        g_firmware_params.pid_moving_buffer_size = config.PID_W;
    } else if (level == 32) {
        g_firmware_params.pid_velocity = config.PID_V;
    } else if (level == 64) {
        g_firmware_params.max_pwm = config.MAX_PWM;
    } else {
        ROS_ERROR("Unsupported dynamic_reconfigure level %u", level);
    }
}

//  initMcbParameters()
//
//  Setup MCB parameters that are from host level options or settings
//
void initMcbParameters(std::unique_ptr<MotorHardware> &robot )
{
    // Force future calls to sendParams() to update current pid parametes on the MCB
    robot->forcePidParamUpdates();

    // Determine hardware options that can be set by the host to override firmware defaults
    int32_t wheel_type = 0;
    if (g_node_params.wheel_type == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        ROS_INFO("Firmware default wheel_type will be used.");
    } else {
        // Any other setting leads to host setting the wheel type
        if (g_node_params.wheel_type == "standard") {
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
            ROS_INFO("Host is specifying wheel_type of '%s'", "standard");
        } else if (g_node_params.wheel_type == "thin"){
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_THIN;
            ROS_INFO("Host is specifying wheel_type of '%s'", "thin");
        } else {
            ROS_WARN("Invalid wheel_type of '%s' specified! Using wheel type of standard", 
                g_node_params.wheel_type.c_str());
            g_node_params.wheel_type = "standard";
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
        }
        // Write out the wheel type setting
        robot->setWheelType(wheel_type);
        mcbStatusPeriodSec.sleep();
    }

    int32_t wheel_direction = 0;
    if (g_node_params.wheel_direction == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        ROS_INFO("Firmware default wheel_direction will be used.");
    } else {
        // Any other setting leads to host setting the wheel type
        if (g_node_params.wheel_direction == "standard") {
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
            ROS_INFO("Host is specifying wheel_direction of '%s'", "standard");
        } else if (g_node_params.wheel_direction == "reverse"){
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_REVERSE;
            ROS_INFO("Host is specifying wheel_direction of '%s'", "reverse");
        } else {
            ROS_WARN("Invalid wheel_direction of '%s' specified! Using wheel direction of standard", 
                g_node_params.wheel_direction.c_str());
            g_node_params.wheel_direction = "standard";
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
            robot->firmware_version, g_firmware_params.controller_board_version);
        robot->setHardwareVersion(g_firmware_params.controller_board_version);
        ROS_DEBUG("Controller board version has been set to %d", 
            g_firmware_params.controller_board_version);
        mcbStatusPeriodSec.sleep();
    }

    // Tell the MCB board what the I2C port on it is set to (mcb cannot read it's own switchs!)
    // We could re-read periodically but perhaps only every 5-10 sec but should do it from main loop
    if (robot->firmware_version >= MIN_FW_OPTION_SWITCH) {
        g_firmware_params.option_switch = robot->getOptionSwitch();
        ROS_INFO("Setting firmware option register to 0x%x.", g_firmware_params.option_switch);
        robot->setOptionSwitchReg(g_firmware_params.option_switch);
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
        robot->setEstopPidThreshold(g_firmware_params.estop_pid_threshold);
        mcbStatusPeriodSec.sleep();
        robot->setEstopDetection(g_firmware_params.estop_detection);
        mcbStatusPeriodSec.sleep();
    }

    if (robot->firmware_version >= MIN_FW_MAX_SPEED_AND_PWM) {
        robot->setMaxFwdSpeed(g_firmware_params.max_speed_fwd);
        mcbStatusPeriodSec.sleep();
        robot->setMaxRevSpeed(g_firmware_params.max_speed_rev);
        mcbStatusPeriodSec.sleep();
    }
        
    return;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;

    // Initialize global parameters
    g_firmware_params = FirmwareParams(nh);
    g_serial_params   = CommsParams(nh);
    g_node_params     = NodeParams(nh);

    ros::Rate ctrlLoopDelay(g_node_params.controller_loop_rate);

    std::unique_ptr<MotorHardware> robot = nullptr;
    // Keep trying to open serial
    {
        int times = 0;
        while (ros::ok() && robot.get() == nullptr) {
            try {
                robot.reset(new MotorHardware(nh, g_serial_params, g_firmware_params));
            }
            catch (const serial::IOException& e) {
                if (times % 30 == 0)
                    ROS_FATAL("Error opening serial port %s, trying again", g_serial_params.serial_port.c_str());
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

    robot->setParams(g_firmware_params);
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

    // Setup MCB parameters that are defined by host parameters in most cases
    ROS_INFO("Initializing MCB");
    initMcbParameters(robot);
    ROS_INFO("Initialization of MCB completed.");

    // At node startup we clear any MCB events for a clean start
    robot->setSystemEvents(0);  // Clear entire system events register
    robot->system_events = 0;
    mcbStatusPeriodSec.sleep();

    ros::Time last_time;
    ros::Time current_time;
    ros::Duration elapsed;
    ros::Duration sysMaintPeriod(60.0);     // A periodic MCB maintenance operation

    // Send out the refreshable firmware parameters, most are the PID terms
    // We must be sure num_fw_params is set to the modulo used in sendParams()
    for (int i = 0; i < robot->num_fw_params; i++) {
        mcbStatusPeriodSec.sleep();
        robot->sendParams();
    }

    float expectedCycleTime = ctrlLoopDelay.expectedCycleTime().toSec();
    float minCycleTime = 0.75 * expectedCycleTime;
    float maxCycleTime = 1.25 * expectedCycleTime;

    // Clear any commands the robot has at this time
    robot->clearCommands();

    last_time = ros::Time::now();
    ros::Time last_sys_maint_time = last_time;
    ros::Duration elapsed_since_last_action;

    ROS_WARN("Starting motor control node now");

    // Implement a speed reset while ESTOP is active and a delay after release
    double estopReleaseDeadtime = 0.8;
    double estopReleaseDelay    = 0.0;
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
            cm.update(current_time, ctrlLoopDelay.expectedCycleTime(), true);
            robot->clearCommands();
        }
        robot->setParams(g_firmware_params);
        robot->sendParams(); 

        // Periodically watch for MCB board having been reset which is an MCB system event
        // This is also a good place to refresh or show status that may have changed
        elapsed_since_last_action = current_time - last_sys_maint_time;
        if ((robot->firmware_version >= MIN_FW_SYSTEM_EVENTS) &&
           (elapsed_since_last_action > sysMaintPeriod)) {
            robot->requestSystemEvents();
            mcbStatusPeriodSec.sleep();
            last_sys_maint_time = ros::Time::now();

            // Post a status message for MCB state periodically. This may be nice to do more on as required
            ROS_INFO("Battery = %5.2f volts, MCB system events 0x%x, Wheel type '%s'",
                robot->getBatteryVoltage(), robot->system_events, 
                (robot->wheel_type == MotorMessage::OPT_WHEEL_TYPE_THIN) ? "thin" : "standard");

            // If we detect a power-on of MCB we should re-initialize MCB
            if ((robot->system_events & MotorMessage::SYS_EVENT_POWERON) != 0) {
                ROS_WARN("Detected Motor controller PowerOn event! Re-initialize MCB");
                robot->setSystemEvents(0);  // Clear entire system events register
                robot->system_events = 0;
                mcbStatusPeriodSec.sleep();
              
                // Setup MCB parameters that are defined by host parameters in most cases
                initMcbParameters(robot);
                ROS_WARN("Motor controller has been re-initialized");
            }

            // a periodic refresh of wheel type which is a safety net due to it's importance.
            // This can be removed when a solid message protocol is developed
            if (robot->firmware_version >= MIN_FW_WHEEL_TYPE_THIN) {
                // Refresh the wheel type setting
                robot->setWheelType(robot->wheel_type);
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
                estopReleaseDelay -= (1.0/g_node_params.controller_loop_rate);
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
