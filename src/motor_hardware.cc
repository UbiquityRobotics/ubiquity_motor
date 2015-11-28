/**
Copyright (c) 2015, Ubiquity Robotics
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

	if (!n.getParam("ubiquity_motor/serial_port", sPort))
	{
		sPort.assign("/dev/ttyS0");
		n.setParam("ubiquity_motor/serial_port", sPort);
	}

	if (!n.getParam("ubiquity_motor/serial_baud", sBaud))
	{
		sBaud = 9600;
		n.setParam("ubiquity_motor/serial_baud", sBaud);
	}

	if (!n.getParam("ubiquity_motor/serial_loop_rate", sLoopRate))
	{
		sLoopRate = 100;
		n.setParam("ubiquity_motor/serial_loop_rate", sLoopRate);
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
				        if (mm.getData() != CURRENT_FIRMWARE_VERSION) { 
                 		                ROS_ERROR("Firmware version %d, expect %d",
							  mm.getData(), CURRENT_FIRMWARE_VERSION);
					}
					break;
				case MotorMessage::REG_LEFT_ODOM:
				  //ROS_ERROR("TICK: %d", mm.getData());
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
	requestOdometry();
	requestVelocity();
	//requestVersion();
	MotorMessage left;
	left.setRegister(MotorMessage::REG_LEFT_SPEED_SET);
	left.setType(MotorMessage::TYPE_WRITE);
	left.setData(boost::math::lround(joints_[0].velocity_command*TICS_PER_RADIAN/SECONDS_PER_VELOCITY_READ));
	motor_serial_->transmitCommand(left);
	MotorMessage right;
	right.setRegister(MotorMessage::REG_RIGHT_SPEED_SET);
	right.setType(MotorMessage::TYPE_WRITE);
	right.setData(boost::math::lround(joints_[1].velocity_command*TICS_PER_RADIAN/SECONDS_PER_VELOCITY_READ));
	motor_serial_->transmitCommand(right);
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
	MotorMessage left;
	left.setRegister(MotorMessage::REG_LEFT_ODOM);
	left.setType(MotorMessage::TYPE_READ);
	left.setData(0);
	motor_serial_->transmitCommand(left);
	MotorMessage right;
	right.setRegister(MotorMessage::REG_RIGHT_ODOM);
	right.setType(MotorMessage::TYPE_READ);
	right.setData(0);
	motor_serial_->transmitCommand(right);
}

void MotorHardware::requestVelocity(){
  MotorMessage left;
  left.setRegister(MotorMessage::REG_LEFT_SPEED_MEASURED);
  left.setType(MotorMessage::TYPE_READ);
  left.setData(0);
  motor_serial_->transmitCommand(left);
  MotorMessage right;
  right.setRegister(MotorMessage::REG_RIGHT_SPEED_MEASURED);
  right.setType(MotorMessage::TYPE_READ);
  right.setData(0);
  motor_serial_->transmitCommand(right);
}


void MotorHardware::setPid(int32_t p_set, int32_t i_set, int32_t d_set, int32_t denominator_set){
        p_value = p_set;
        i_value = i_set;
        d_value = d_set;
        denominator_value = denominator_set;
}

void MotorHardware::sendPid() {
	MotorMessage p;
	p.setRegister(MotorMessage::REG_PARAM_P);
	p.setType(MotorMessage::TYPE_WRITE);
	p.setData(p_value);
	motor_serial_->transmitCommand(p);
	MotorMessage i;
	i.setRegister(MotorMessage::REG_PARAM_I);
	i.setType(MotorMessage::TYPE_WRITE);
	i.setData(i_value);
	motor_serial_->transmitCommand(i);
	MotorMessage d;
	d.setRegister(MotorMessage::REG_PARAM_D);
	d.setType(MotorMessage::TYPE_WRITE);
	d.setData(d_value);
	motor_serial_->transmitCommand(d);
	MotorMessage denominator;
	denominator.setRegister(MotorMessage::REG_PARAM_C);
	denominator.setType(MotorMessage::TYPE_WRITE);
	denominator.setData(denominator_value);
	motor_serial_->transmitCommand(denominator);
}
