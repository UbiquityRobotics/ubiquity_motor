#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_command.h>
#include <string>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <time.h>
#include "controller_manager/controller_manager.h"
#include <boost/asio/io_service.hpp>
#include <ros/ros.h>

static const double BILLION = 1000000000.0;

void controlLoop(ros::Rate r,
	MotorHardware &robot,
	controller_manager::ControllerManager &cm){

	struct timespec last_time;
	struct timespec current_time;
	clock_gettime(CLOCK_MONOTONIC, &last_time);

	while (ros::ok()) {
		clock_gettime(CLOCK_MONOTONIC, &current_time);
		ros::Duration elapsed = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
		last_time = current_time;
		robot.readInputs();
		cm.update(ros::Time::now(), elapsed);
		robot.writeSpeeds();
		r.sleep();
	}

}

main(int argc, char* argv[]) {
	ros::init(argc, argv, "motor_node");
	ros::NodeHandle nh;
	MotorHardware robot(nh);
	controller_manager::ControllerManager cm(&robot,nh);

	double controller_loop_rate;

	if (!nh.getParam("ubiquity_motor/controller_loop_rate", controller_loop_rate))
	{
		controller_loop_rate = 10;
		nh.setParam("ubiquity_motor/controller_loop_rate", controller_loop_rate);
	}

	ros::Rate r(controller_loop_rate);

	boost::thread controlLoopThread(controlLoop, r , boost::ref(robot), boost::ref(cm));

	ros::spin();
}