
#include "ros/ros.h"
#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_command.h>
#include <string>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "controller_manager/controller_manager.h"
#include <boost/asio/io_service.hpp>

typedef boost::chrono::steady_clock time_source;

main(int argc, char* argv[]) {
  ros::init(argc, argv, "motor_node");
  MotorHardware robot;
  controller_manager::ControllerManager cm(&robot);
  time_source::time_point last_time = time_source::now();
  ros::Rate r(50);
  while (true) {
  	 time_source::time_point this_time = time_source::now();
  	 boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  	 ros::Duration elapsed(elapsed_duration.count());
     last_time = this_time;
     //robot.read();
     cm.update(ros::Time::now(), elapsed);
     robot.writeSpeeds();
     r.sleep();
  }
}