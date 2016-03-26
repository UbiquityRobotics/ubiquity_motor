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
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN AiiiiiiiiiiiNY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
#include <boost/assign.hpp>
#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_message.h>

#include <boost/math/special_functions/round.hpp>

//#define SENSOR_DISTANCE 0.002478

// 60 tics per revolution of the motor (pre gearbox)
//17.2328767123
// gear ratio of 4.29411764706:1
#define TICS_PER_RADIAN (41.0058030317/2)
#define SECONDS_PER_VELOCITY_READ 10.0 //read = ticks / (100 ms), so we have scale of 10 for ticks/second
#define CURRENT_FIRMWARE_VERSION 18

MotorHardware::MotorHardware(ros::NodeHandle nh){
	ros::V_string joint_names = boost::assign::list_of("left_wheel_joint")("right_wheel_joint");

	for (unsigned int i = 0; i < joint_names.size(); i++) {
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
		    &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
		joint_state_interface_.registerHandle(joint_state_handle);

		hardware_interface::JointHandle joint_handle(
		joint_state_handle, &joints_[i].velocity_command);
		velocity_joint_interface_.registerHandle(joint_handle);
	}
	registerInterface(&joint_state_interface_);
	registerInterface(&velocity_joint_interface_);



	std::string sPort;
	int sBaud;

	double sLoopRate;

	if (!nh.getParam("ubiquity_motor/serial_port", sPort))
	{
		sPort.assign("/dev/ttyS0");
		nh.setParam("ubiquity_motor/serial_port", sPort);
	}

	if (!nh.getParam("ubiquity_motor/serial_baud", sBaud))
	{
		sBaud = 9600;
		nh.setParam("ubiquity_motor/serial_baud", sBaud);
	}

	if (!nh.getParam("ubiquity_motor/serial_loop_rate", sLoopRate))
	{
		sLoopRate = 100;
		nh.setParam("ubiquity_motor/serial_loop_rate", sLoopRate);
	}

	motor_serial_ = new MotorSerial(sPort,sBaud,sLoopRate);
}

MotorHardware::~MotorHardware(){
	delete motor_serial_;
}

void MotorHardware::readInputs(){
	while(motor_serial_->commandAvailable()){
		MotorMessage mm;
		mm = motor_serial_-> receiveCommand();
		if(mm.getType() == MotorMessage::TYPE_RESPONSE){
			switch(mm.getRegister()){
				case MotorMessage::REG_FIRMWARE_VERSION:
					if (!mm.getData() > CURRENT_FIRMWARE_VERSION) { 
						ROS_FATAL("Firmware version %d, expect %d or above",
							mm.getData(), CURRENT_FIRMWARE_VERSION);
					}
					else {
						ROS_INFO("Firmware version %d", mm.getData());
					}
					break;
				case MotorMessage::REG_LEFT_ODOM:
					joints_[0].position += mm.getData()/TICS_PER_RADIAN;
					break;
				case MotorMessage::REG_RIGHT_ODOM:
					joints_[1].position += mm.getData()/TICS_PER_RADIAN;
					break;
				case MotorMessage::REG_LEFT_SPEED_MEASURED:
					joints_[0].velocity = mm.getData()*SECONDS_PER_VELOCITY_READ/TICS_PER_RADIAN;
					break;
				case MotorMessage::REG_RIGHT_SPEED_MEASURED:
					joints_[1].velocity = mm.getData()*SECONDS_PER_VELOCITY_READ/TICS_PER_RADIAN;
					break;
			}
		}
	}
}

void MotorHardware::writeSpeeds(){
	std::vector<MotorMessage> commands;
	//requestOdometry();
	//requestVelocity();
	//requestVersion();

	MotorMessage left_odom;
	left_odom.setRegister(MotorMessage::REG_LEFT_ODOM);
	left_odom.setType(MotorMessage::TYPE_READ);
	left_odom.setData(0);
	commands.push_back(left_odom);

	MotorMessage right_odom;
	right_odom.setRegister(MotorMessage::REG_RIGHT_ODOM);
	right_odom.setType(MotorMessage::TYPE_READ);
	right_odom.setData(0);
	commands.push_back(right_odom);




	MotorMessage left_vel;
	left_vel.setRegister(MotorMessage::REG_LEFT_SPEED_MEASURED);
	left_vel.setType(MotorMessage::TYPE_READ);
	left_vel.setData(0);
	commands.push_back(left_vel);

	MotorMessage right_vel;
	right_vel.setRegister(MotorMessage::REG_RIGHT_SPEED_MEASURED);
	right_vel.setType(MotorMessage::TYPE_READ);
	right_vel.setData(0);
	commands.push_back(right_vel);




	MotorMessage left;
	left.setRegister(MotorMessage::REG_LEFT_SPEED_SET);
	left.setType(MotorMessage::TYPE_WRITE);
	left.setData(boost::math::lround(joints_[0].velocity_command*TICS_PER_RADIAN/SECONDS_PER_VELOCITY_READ));
	commands.push_back(left);

	MotorMessage right;
	right.setRegister(MotorMessage::REG_RIGHT_SPEED_SET);
	right.setType(MotorMessage::TYPE_WRITE);
	right.setData(boost::math::lround(joints_[1].velocity_command*TICS_PER_RADIAN/SECONDS_PER_VELOCITY_READ));	
	commands.push_back(right);


	//Send all commands to serial thread in one go to reduce locking
	motor_serial_->transmitCommands(commands);

	//ROS_ERROR("velocity_command %f rad/s %f rad/s", joints_[0].velocity_command, joints_[1].velocity_command);
	// ROS_ERROR("SPEEDS %d %d", left.getData(), right.getData());
}

void MotorHardware::requestVersion(){                                                                                          
    MotorMessage version;
	version.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
	version.setType(MotorMessage::TYPE_READ);
	version.setData(0);
	motor_serial_->transmitCommand(version);

}

void MotorHardware::requestOdometry(){
	//ROS_ERROR("TICKR");
	std::vector<MotorMessage> commands;

	MotorMessage left_odom;
	left_odom.setRegister(MotorMessage::REG_LEFT_ODOM);
	left_odom.setType(MotorMessage::TYPE_READ);
	left_odom.setData(0);
	commands.push_back(left_odom);

	MotorMessage right_odom;
	right_odom.setRegister(MotorMessage::REG_RIGHT_ODOM);
	right_odom.setType(MotorMessage::TYPE_READ);
	right_odom.setData(0);
	commands.push_back(right_odom);

	motor_serial_->transmitCommands(commands);
}

void MotorHardware::requestVelocity(){
	std::vector<MotorMessage> commands;

	MotorMessage left_vel;
	left_vel.setRegister(MotorMessage::REG_LEFT_SPEED_MEASURED);
	left_vel.setType(MotorMessage::TYPE_READ);
	left_vel.setData(0);
	commands.push_back(left_vel);

	MotorMessage right_vel;
	right_vel.setRegister(MotorMessage::REG_RIGHT_SPEED_MEASURED);
	right_vel.setType(MotorMessage::TYPE_READ);
	right_vel.setData(0);
	commands.push_back(right_vel);

	motor_serial_->transmitCommands(commands);
}


void MotorHardware::setPid(int32_t p_set, int32_t i_set, int32_t d_set, int32_t denominator_set){
	p_value = p_set;
	i_value = i_set;
	d_value = d_set;
	denominator_value = denominator_set;
}

void MotorHardware::sendPid() {
	std::vector<MotorMessage> commands;

	MotorMessage p;
	p.setRegister(MotorMessage::REG_PARAM_P);
	p.setType(MotorMessage::TYPE_WRITE);
	p.setData(p_value);
	commands.push_back(p);

	MotorMessage i;
	i.setRegister(MotorMessage::REG_PARAM_I);
	i.setType(MotorMessage::TYPE_WRITE);
	i.setData(i_value);
	commands.push_back(i);

	MotorMessage d;
	d.setRegister(MotorMessage::REG_PARAM_D);
	d.setType(MotorMessage::TYPE_WRITE);
	d.setData(d_value);
	commands.push_back(d);

	MotorMessage denominator;
	denominator.setRegister(MotorMessage::REG_PARAM_C);
	denominator.setType(MotorMessage::TYPE_WRITE);
	denominator.setData(denominator_value);
	commands.push_back(denominator);

	motor_serial_->transmitCommands(commands);
}

void MotorHardware::setDebugLeds(bool led_1, bool led_2) {
	std::vector<MotorMessage> commands;
	
	MotorMessage led1;
	led1.setRegister(MotorMessage::REG_LED_1);
	led1.setType(MotorMessage::TYPE_WRITE);
	if(led_1) {
		led1.setData(0x00000001);
	}
	else {
		led1.setData(0x00000000);
	}
	commands.push_back(led1);

	MotorMessage led2;
	led2.setRegister(MotorMessage::REG_LED_2);
	led2.setType(MotorMessage::TYPE_WRITE);
	if(led_2) {
		led2.setData(0x00000001);
	}
	else {
		led2.setData(0x00000000);
	}
	commands.push_back(led2);

	motor_serial_->transmitCommands(commands);
}
