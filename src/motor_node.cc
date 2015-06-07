
#include "ros/ros.h"
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

main(int argc, char* argv[]) {
  ros::init(argc, argv, "motor_node");
  MotorHardware robot;
  controller_manager::ControllerManager cm(&robot);
  
  struct timespec last_time;
  struct timespec current_time;

  clock_gettime(CLOCK_MONOTONIC, &last_time);

  ros::Rate r(50);
  while (true) {
     clock_gettime(CLOCK_MONOTONIC, &current_time);
  	 ros::Duration elapsed = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
     last_time = current_time;
     //robot.read();
     cm.update(ros::Time::now(), elapsed);
     robot.writeSpeeds();
     r.sleep();
  }
}