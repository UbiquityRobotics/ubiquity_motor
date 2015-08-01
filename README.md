# ubiquity_motor
[![Build Status](https://travis-ci.org/UbiquityRobotics/ubiquity_motor.svg?branch=hydro)](https://travis-ci.org/UbiquityRobotics/ubiquity_motor)

Package that provides a ROS interface for the motors in UbiquityRobotics robots

## To run this node:

	rosrun ubiquity_motor ubiquity_motor

## Parameters and defaults

        /ubiquity/motor/device_name /dev/ttyUSB0 
	/ubiquity/motor/baud_rate 9600
	/ubiquity/motor/odom_rate 1.0/20.0
	/ubiquity/motor/diag_rate 2
	/ubiquity/motor/spin_rate 1.0/100.0

## Subscribes to

	/cmd_vel 	geometry_msgs::Twist

## Publishes
 
	/odom		nav_msgs::Odometry	


