#include <string>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class UbiquityMotor {
 public:
  UbiquityMotor();
  ~UbiquityMotor();

  int fd() const { return fd_; }
  bool done() const { return done_; }

 private:
  friend void *UbiquityMotorReaderThread(void *arg);

  // Conversion factor for the time units used on the microcontroller board.
  static const float time_units_per_second;

  // Distance between the center of the contact patches of the wheels.
  static const float wheel_base;

  // Diameter of the wheel.
  static const float wheel_diameter;

  // Absolute number of magnets in the motor.
  static const unsigned int magnet_count;

  // Absolute number of teeth on the inner gear in the motor assembly.
  static const unsigned int sun_gear_teeth;

  // Absolute number of teeth on the planet gears in the motor assembly.
  static const unsigned int planet_gear_teeth;

  // Absolute number of teeth on the ring gears in the motor assembly.
  static const unsigned int ring_gear_teeth;

  // Absolute number of teeth on the planet gears in the motor assembly.
  static const float wheel_circumference;

  // Number of motor revolutions per revolution of the wheel.
  static const float gear_ratio;

  // Number of Hall-effect sensor transitions observed per magnet per
  // motor revolution.
  static const float transitions_per_magnet;

  // Distance traveled for each Hall-effect sensor transition.
  static const float sensor_distance;

  // Callback function for "/cmd_vel"
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &movement);
  void SetWheelVelocities(float left_vel, float right_vel);

  std::string controller_tty_name_;
  std::string left_motor_joint_name_;
  std::string right_motor_joint_name_;

  pthread_t reader_thread_;

  // Indicates that the destructor has been called.
  bool done_;

  ros::NodeHandle nh_;
  ros::Subscriber vel_sub_;
  ros::Publisher odometry_pub_;

  ros::Publisher nav_odom_pub_;
  tf::TransformBroadcaster tf_pub_;

  // The TTY file descriptor
  int fd_;

  // True iff we have started receiving sane odometer data.
  bool has_odometer_;

  // Last raw odometer data from the motor controller.
  uint16_t odometer_[2];

  // Time of last update of odometer_.
  ros::Time last_odometry_;

  // Odometer corrected for integer overflow.
  int64_t position_[2];

  // X,Y, theta odometry
  double odom_x_, odom_y_, odom_theta_;
};
