/*
 * ubiquity_motor.cpp
 *
 *  Created on: Apr 4, 2015
 *      Author: kurt
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>

#include "MotorController.h"


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ubiquity_motor");
  ur::MotorController mc;
  ros::NodeHandle n;

  // parameters to pass to the motor controller
  double odom_x_, odom_y_, odom_theta_;
  int baud_rate_ = 0;
  std::string device_name_;
  std::string left_motor_joint_name_;
  std::string right_motor_joint_name_;
  std::string velocity_topic_;
  int32_t pid_p_;
  int32_t pid_i_;
  int32_t pid_d_;
  int32_t pid_c_;
  int32_t deadman_;
  double odom_rate_;
  double diag_rate_;
  double spin_rate_;

  // pick up parameters
  if (!n.getParam("/ubiquity/motor/device_name", device_name_))
    device_name_ = "/dev/ttyUSB0";
  mc.setDeviceName(device_name_);

  n.param("/ubiquity/motor/baud_rate", baud_rate_, 9600);
  mc.setBaudRate(baud_rate_);
  n.param<double>("/ubiquity/motor/odom_rate", odom_rate_, 1/20);  // loop ~ 20 times a second
  n.param<double>("/ubiquity/motor/diag_rate", diag_rate_, 2);  // duration of one second before publishing
  n.param<double>("/ubiquity/motor/spin_rate", spin_rate_, 1/100);

  if (!n.getParam("/ubiquity/motor/left/name", left_motor_joint_name_))
    left_motor_joint_name_ = "base_l_wheel_joint";
  mc.setLeftMotorJointName(left_motor_joint_name_);

  if (!n.getParam("/ubiquity/motor/right/name", right_motor_joint_name_))
    right_motor_joint_name_ = "base_r_wheel_joint";
  mc.setRightMotorJointName(right_motor_joint_name_);

  if (!n.getParam("/ubiquity/motor/velocity_topic", velocity_topic_))
    velocity_topic_ = "/cmd_vel";
  mc.setVelocityTopic(velocity_topic_);

  // save odometry information to the parameter server between runs
  if(!n.getParam("/ubiquity/motor/x", odom_x_))
    odom_x_ = 0.0;

  if(!n.getParam("/ubiquity/motor/y", odom_y_))
    odom_y_ = 0.0;

  if(!n.getParam("/ubiquity/motor/theta", odom_theta_))
    odom_theta_ = 0.0;

  // Interface to amrit's board
  mc.connect();

  if (n.getParam("/ubiquity/motor/pid_proportional", pid_p_))
    mc.setPIDProportional(pid_p_);
  if (n.getParam("/ubiquity/motor/pid_integral", pid_i_))
    mc.setPIDIntegral(pid_i_);
  if (n.getParam("/ubiquity/motor/pid_derivative", pid_d_))
    mc.setPIDDerivative(pid_d_);
  if (n.getParam("/ubiquity/motor/pid_denominator", pid_c_))
    mc.setPIDDenominator(pid_c_);
  if (n.getParam("/ubiquity/motor/deadman_timer", deadman_))
    mc.setDeadmanTimer(deadman_);

  // ROS topics
  ros::Subscriber vel_sub_ =
      n.subscribe(velocity_topic_, 1, &ur::MotorController::twistCallback, &mc);

  ros::Publisher joint_states_pub_ =
      n.advertise<sensor_msgs::JointState>("/joint_states", 1);
  mc.setJointStatesPublisher(&joint_states_pub_);

  ros::Publisher odometry_pub_ =
      n.advertise<nav_msgs::Odometry>("/odom", 1);
  mc.setOdometryPublisher(&odometry_pub_);

  tf::TransformBroadcaster tf_broadcaster_;
  mc.setTransformBroadcaster(&tf_broadcaster_);

  ROS_INFO("ubiquity_motor initialized");

  ros::Duration rate(spin_rate_);

  ros::Time current_time, last_odom, last_diag;
  current_time = last_odom = last_diag = ros::Time::now();
  ros::Duration odom(odom_rate_), diag(diag_rate_);

  ROS_INFO("Odometry rate is %f", odom_rate_);


  while ( n.ok() )
  {
    current_time = ros::Time::now();

    if ( current_time >= (last_odom + odom) )
    {
      mc.readOdometry();
      last_odom = current_time;
    }
    if ( current_time >= (last_diag + diag) )
    {
//      mc.readDiagnostics();
      last_diag = current_time;
    }
    mc.listen();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
} // end main()


