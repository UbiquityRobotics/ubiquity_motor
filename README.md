# ubiquity_motor
[![Build Status](https://travis-ci.org/UbiquityRobotics/ubiquity_motor.svg?branch=indigo-devel)](https://travis-ci.org/UbiquityRobotics/ubiquity_motor)

## Introduction

This is a Package that provides a ROS interface for the motors in UbiquityRobotics robots. It communicates with the motor controller via a serial port.

## Installation

This package may be installed from binaries for both x86 and ARM architectures.

The package may be installed with:

`sudo apt-get install ros-kinetic-ubiquity-motor`

Configuration launch files for the _Magni_ robot are in the package [magni_robot](https://github.com/UbiquityRobotics/magni_robot).

## ROS API

### Subscribed topics

`cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
The command input.

`system_control` [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/Sring.html)
A channel to command the motor node to change mode of operation. 
A text string with a commnad followed by parameter such as `enable` or `disable` are used.

    motor_control disable      Stops the MCB control over serial (used for firmware load)
    motor_control enable       Re-attaches the MCB control over serial for normal operation


    speed_control disable      Stops the robot so no speed control is used for collision avoiance
    speed_control enable       Re-enables the motor node to respond to cmd_vel speed messages
      

### Published topics

`odom` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
Odometry computed from motor controller messages.

`/tf` [tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html)
The transform from `odom` to `base_link`

`battery_state` [sensor_msgs/BatteryState](http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html)
Charge state of the robot's batteries.

`publish_cmd` [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html)
The value of `cmd_vel` after limits were applied. Available if the `publish_cmd` paramater is set.

`motor_power_active` [std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html)
The state of motor power being active is published.  This follows the ESTOP switch within about a half second.

`left_error` [std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html)
The error in expected left wheel position relative to current left wheel position is published for diagnostics purposes.

`right_error` [std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html)
The error in expected right wheel position relative to current right wheel position is published for diagnostics purposes.

`left_tick_interval` [std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html)
A value that is proportional to the time between left wheel encoder ticks for usage in diagnostics. This unsigned value is only published when velocity is non zero.

`right_tick_interval` [std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html)
A value that is proportional to the time between right wheel encoder ticks for usage in diagnostics. This unsigned value is only published when velocity is non zero.

### Parameters

#### Comms paramaters

`serial_port` (string, default: "/dev/ttyS0")
Name of device with which to communicate with motor controller hardware.

`baud_rate` (int, default: 9600)
Baud rate for serial communication with motor controller hardware.

#### Firmware parameters

`pid_proportional` (int, default: 5000)
The `P` paramater for the motor controller's `PID` controller.

`pid_integral` (int, default: 10)
The `I` paramater for the motor controller's `PID` controller.

`pid_derivative` (int, default: 1)
The `D` paramater for the motor controller's `PID` controller.

`pid_denominator` (int, default: 1000)
Divisor for the above `PID` paramaters.

`pid_moving_buffer_size` (int, default: 10)
Size of a moving buffer used in the control loop.

`deadman_timer` (int, default: 2400000)
If a message is not received after this interval (in MCU clock ticks), the motor controller should assume an error condition and stop.

`battery_voltage_multiplier` (float, default: 0.5185)
Used to calculate battery voltage from motor controller messages.

`battery_voltage_offset` (float, default: 0.40948)
Used to calculate battery voltage from motor controller messages.

#### Node parameters

`controller_loop_rate` (double, default: 20.0)
Rate (in Hz) at which to send commands to the motor controller hardware.

#### diff_drive_controller paramaters

`left_wheel` (string | string[...])
Left wheel joint name or list of joint names.

`right_wheel` (string | string[...])
Right wheel joint name or list of joint names.

`pose_covariance_diagonal` (double[6])
Diagonal of the covariance matrix for odometry pose publishing.

`twist_covariance_diagonal` (double[6])
Diagonal of the covariance matrix for odometry twist publishing.

`publish_rate` (double, default: 50.0)
Frequency (in Hz) at which the odometry is published. Used for both tf and odom.

`wheel_separation_multiplier` (double, default: 1.0)
Multiplier applied to the wheel separation parameter. This is used to account for a difference between the robot model and a real robot.

`wheel_radius_multiplier` (double, default: 1.0)
Multiplier applied to the wheel radius parameter. This is used to account for a difference between the robot model and a real robot.

`cmd_vel_timeout` (double, default: 0.5)
Allowed period (in seconds) allowed between two successive velocity commands. After this delay, a zero speed command will be sent to the wheels.

`base_frame_id` (string, default: base_link)
Base frame_id, which is used to fill in the child_frame_id of the Odometry messages and TF.

`linear/x/has_velocity_limits` (bool, default: false)
Whether the controller should limit linear speed.

`linear/x/max_velocity` (double)
Maximum linear velocity (in m/s).

`linear/x/min_velocity` (double)
Minimum linear velocity (in m/s). Setting this to 0.0 will disable backwards motion. When unspecified, -max_velocity is used.

`linear/x/has_acceleration_limits` (bool, default: false)
Whether the controller should limit linear acceleration.

`linear/x/max_acceleration` (double)
Maximum linear acceleration (in m/s^2).

`linear/x/min_acceleration` (double)
Minimum linear acceleration (in m/s^2). When unspecified, -max_acceleration is used.

`linear/x/has_jerk_limits` (bool, default: false)
Whether the controller should limit linear jerk.

`linear/x/max_jerk` (double)
Maximum linear jerk (in m/s^3).

`angular/z/has_velocity_limits` (bool, default: false)
Whether the controller should limit angular velocity.

`angular/z/max_velocity` (double)
Maximum angular velocity (in rad/s).

`angular/z/min_velocity` (double)
Minimum angular velocity (in rad/s). Setting this to 0.0 will disable counter-clockwise rotation. When unspecified, -max_velocity is used.

`angular/z/has_acceleration_limits` (bool, default: false)
Whether the controller should limit angular acceleration.

`angular/z/max_acceleration` (double)
Maximum angular acceleration (in rad/s^2).

`angular/z/min_acceleration` (double)
Minimum angular acceleration (in rad/s^2). When unspecified, -max_acceleration is used.

`angular/z/has_jerk_limits` (bool, default: false)
Whether the controller should limit angular jerk.

`angular/z/max_jerk` (double)
Maximum angular jerk (in m/s^3).

`enable_odom_tf` (bool, default: true)
Whether to publish to TF directly.

`wheel_separation` (double)
The distance of the left and right wheel(s). The diff_drive_controller will attempt to read the value from the URDF if this parameter is not specified.

`wheel_radius` (double)
Radius of the wheels. It is expected they all have the same size. The diff_drive_controller will attempt to read the value from the URDF if this parameter is not specified.

`odom_frame_id` (string, default: "/odom")
Name of frame in which to publish odometry.

`publish_cmd` (bool, default: false)
Publish the velocity command to be executed. It is to monitor the effect of limiters on the controller input.

`allow_multiple_cmd_vel_publishers` (bool, default: false)
When enabled the controller will brake if there is more than one publishers on its input topic, `~/cmd_vel`.
