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
#include <string>
#include <boost/thread.hpp>
#include <time.h>
#include "controller_manager/controller_manager.h"
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ubiquity_motor/PIDConfig.h>

static const double BILLION = 1000000000.0;

static int32_t pid_proportional = 5000;
static int32_t pid_integral = 1;
static int32_t pid_derivative = 1 ;
static int32_t pid_denominator = 1000;
static int32_t moving_buffer_size = 10;

void PID_update_callback(const ubiquity_motor::PIDConfig &config, uint32_t level) {
	if(level == 1){
		pid_proportional = config.PID_P;
	}
	else if(level == 2){
		pid_integral = config.PID_I;
	}
	else if(level == 4){
		pid_derivative = config.PID_D;
	}
	else if(level == 8){
		pid_denominator = config.PID_C;
	}
	else if(level == 16){
		moving_buffer_size = config.PID_W;
	}
	else {
		ROS_ERROR("Unsupported dynamic_reconfigure level %u", level);
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "motor_node");
	ros::NodeHandle nh;
	MotorHardware robot(nh);
	controller_manager::ControllerManager cm(&robot,nh);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	dynamic_reconfigure::Server<ubiquity_motor::PIDConfig> server;
	dynamic_reconfigure::Server<ubiquity_motor::PIDConfig>::CallbackType f;

	f = boost::bind(&PID_update_callback, _1, _2);
	server.setCallback(f);	


	int32_t deadman_timer;

	if (!nh.getParam("ubiquity_motor/pid_proportional", pid_proportional)) {
		pid_proportional = 450;
		nh.setParam("ubiquity_motor/pid_proportional", pid_proportional);
	}

	if (!nh.getParam("ubiquity_motor/pid_integral", pid_integral)) {
		pid_integral = 120;
		nh.setParam("ubiquity_motor/pid_integral", pid_integral);
	}

	if (!nh.getParam("ubiquity_motor/pid_derivative", pid_derivative)) {
		pid_derivative = 70;
		nh.setParam("ubiquity_motor/pid_derivative", pid_derivative);
	}

	if (!nh.getParam("ubiquity_motor/pid_denominator", pid_denominator)) {
		pid_derivative = 1000;
		nh.setParam("ubiquity_motor/pid_denominator", pid_denominator);
	}
	if (!nh.getParam("ubiquity_motor/window_size", moving_buffer_size)) {
		moving_buffer_size = 10;
		nh.setParam("ubiquity_motor/window_size", moving_buffer_size);
	}

	//robot.setDeadmanTimer(deadman_timer);

	robot.setWindowSize(moving_buffer_size);
	robot.setPid(pid_proportional,pid_integral,pid_derivative,pid_denominator);
	
	double controller_loop_rate;
	if (!nh.getParam("ubiquity_motor/controller_loop_rate", controller_loop_rate)) {
		controller_loop_rate = 10;
		nh.setParam("ubiquity_motor/controller_loop_rate", controller_loop_rate);
	}

	if (!nh.getParam("ubiquity_motor/deadman_timer", deadman_timer)) {
		deadman_timer = 2400000;
		nh.setParam("ubiquity_motor/deadman_timer", deadman_timer);
	}
	ros::Rate r(controller_loop_rate);
	robot.requestVersion();


	struct timespec last_time;
	struct timespec current_time;
	clock_gettime(CLOCK_MONOTONIC, &last_time);


        for (int i=0; i<5; i++) {
                r.sleep();
		robot.sendPid();
	}

	while (ros::ok()) {
		clock_gettime(CLOCK_MONOTONIC, &current_time);
		ros::Duration elapsed = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
		last_time = current_time;
		robot.readInputs();
		cm.update(ros::Time::now(), elapsed);
		robot.setPid(pid_proportional,pid_integral,pid_derivative,pid_denominator);
		robot.setWindowSize(moving_buffer_size);
		robot.sendPid();
		robot.writeSpeeds();
		
		r.sleep();
	}

	return 0;
}
