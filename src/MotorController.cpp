/*
 * MotorController.cpp
 *
 *  Created on: Apr 4, 2015
 *      Author: kurt
 */

#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind.hpp>

#include "CRC.h"
#include "MotorMessage.h"
#include "MotorController.h"

namespace ur {

MotorController::MotorController() {
  // TODO Auto-generated constructor stub
  m_iBaud = 0x00;
  m_iOdomLeft = 0x00;
  m_iOdomRight = 0x00;
  m_bHaveOdomLeft = false;
  m_bHaveOdomRight = false;
  m_iPositionLeft = 0x00;
  m_iPositionRight = 0x00;
  m_rosLastOdomTime = (ros::Time) 0x00;
  m_rosOdomRate = (ros::Duration) 0x00;
  m_rosLastDiagnosticTime = (ros::Time) 0x00;
  m_rosDiagnosticRate = (ros::Duration) 0x00;
  m_pubJointStates = (ros::Publisher *) NULL;
  m_pubOdometry = (ros::Publisher *) NULL;
  m_pubTransform = (tf::TransformBroadcaster *) NULL;
  m_dOdometryX = 0.0;
  m_dOdometryY = 0.0;
  m_dOdometryTheta = 0.0;
  // Temporary until we get values properly
  m_iMagnetCount = 20;
  m_iSunGearTeeth = 17;
  m_iPlanetGearTeeth = 28;
  m_iRingGearTeeth = 73;
  m_fUnitsPerSecond = 1.0e-6 * 0xFFFFFF;
  m_fWheelBase = 0.33f;
  m_fWheelDiameter = 0.203f;
  m_fWheelCircumference = m_fWheelDiameter * M_PI;
  m_fGearRatio = m_iRingGearTeeth / m_iSunGearTeeth;
  m_fTransitionsPerMagnet = 3.0;
  m_fSensorDistance = m_fWheelCircumference / m_fGearRatio / m_iMagnetCount / m_fTransitionsPerMagnet;

  for ( int i = 0; i < 36; i++ )
    m_faOdometryCovariance[i] = 0.0;

  m_faOdometryCovariance[0] =
          m_faOdometryCovariance[7] =
          m_faOdometryCovariance[14] =
          m_faOdometryCovariance[21] =
          m_faOdometryCovariance[28] =
          m_faOdometryCovariance[35] = 0.2;

  initializeMessageRead();
  m_timeout = 100;  // hard code 10th of a second for now....


//  m_tListener = (boost::thread *)NULL;
}

MotorController::~MotorController() {
  // TODO Auto-generated destructor stub
  stop();
}

// Public Functions

void MotorController::connect(void) {
  ROS_INFO("Opening TTY %s", m_sDevice.c_str());


  boost::system::error_code ec;

  m_pBoard = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(m_ioService));
  m_pBoard->open(m_sDevice, ec);
  m_pBoard->set_option(boost::asio::serial_port_base::baud_rate(m_iBaud));
  // For now hard code everything else
  m_pBoard->set_option(boost::asio::serial_port_base::character_size(8));
  m_pBoard->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  m_pBoard->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  m_pBoard->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

//  boost::thread t( boost::bind( &ur::MotorController::listen, this ) );
//  t.detach();
//  t.start_thread();
}

void MotorController::setDeviceName(std::string dev) {
  m_sDevice = dev;
}

void MotorController::setBaudRate(int baud) {
  m_iBaud = baud;
}

void MotorController::setLeftMotorJointName(std::string n) {
  m_sLeftMotorJointName = n;
}

void MotorController::setRightMotorJointName(std::string n) {
  m_sRightMotorJointName = n;
}

void MotorController::setVelocityTopic(std::string n) {
  m_sVelocityTopic = n;
}

void MotorController::setOdomX(double val) {
  m_dOdometryX= val;
}

void MotorController::setOdomY(double val) {
  m_dOdometryY = val;
}

void MotorController::setOdomTheta(double val) {
  m_dOdometryTheta = val;
}

void MotorController::setOdomDuration(double duration)
{
  m_rosOdomRate = ros::Duration(duration);
}

void MotorController::setDiagDuration(double duration)
{
  m_rosDiagnosticRate = ros::Duration(duration);

}

void MotorController::setJointStatesPublisher(ros::Publisher *pub)
{
  m_pubJointStates = pub;
}

void MotorController::setOdometryPublisher(ros::Publisher *pub)
{
  m_pubOdometry = pub;
}

void MotorController::setTransformBroadcaster(tf::TransformBroadcaster *broadcaster)
{
  m_pubTransform = broadcaster;
}

boost::shared_ptr<boost::asio::serial_port> MotorController::getBoard(void)
{
  return (m_pBoard);
}

void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr& cmd) {
  geometry_msgs::Twist vel = *cmd;

  float ang_vel = -0.5 * m_fWheelBase * vel.angular.z;

  float left = vel.linear.x + ang_vel;
  float right = vel.linear.x - ang_vel;

  setWheelVelocities(left, right);
}

void MotorController::setPIDProportional(int32_t i)
{
  ur::message_value v;
  v.i = i;

  ur::MotorMessage msg(ur::MOTOR_MSG_TYPE_WRITE, ur::REG_ADDR_PID_PROPORTIONAL, v);
  sendMessage(msg);
}

void MotorController::setPIDIntegral(int32_t i)
{
  ur::message_value v;
  v.i = i;

  ur::MotorMessage msg(MOTOR_MSG_TYPE_WRITE, REG_ADDR_PID_INTEGRAL, v);
  sendMessage(msg);
}

void MotorController::setPIDDerivative(int32_t i)
{
  ur::message_value v;
  v.i = i;

  ur::MotorMessage msg(MOTOR_MSG_TYPE_WRITE, REG_ADDR_PID_DERIVATIVE, v);
  sendMessage(msg);
}

void MotorController::setPIDDenominator(int32_t i)
{
  ur::message_value v;
  v.i = i;

  ur::MotorMessage msg(MOTOR_MSG_TYPE_WRITE, REG_ADDR_PID_DENOMINATOR, v);
  msg.setChecksum();
  sendMessage(msg);
}

void MotorController::setDeadmanTimer(int32_t i)
{
  ur::message_value v;
  v.i = i;

  ur::MotorMessage msg(MOTOR_MSG_TYPE_WRITE, REG_ADDR_DEADMAN_TIMER, v);
  msg.setChecksum();
  sendMessage(msg);
}

void MotorController::readOdometry(void)
{
  getRegister(REG_ADDR_LEFT_TICS);
  getRegister(REG_ADDR_RIGHT_TICS);
}

//calls get register for each of the diagnostic values we collect
void MotorController::readDiagnostics(void)
{

}

// Private Functions

void MotorController::setWheelVelocities(float left, float right) {
  ur::message_value v[2];
  
  // Convert from m/s to sensors/robot_time_unit
  left *= m_fUnitsPerSecond  / m_fSensorDistance;
  right *= m_fUnitsPerSecond  / m_fSensorDistance;

  v[0].i = roundf(left);
  v[1].i = roundf(right);

  // hack to slow things down
  v[0].i /= 200;
  v[1].i /= 200;
  // hack to reverse the direction
  v[0].i = -v[0].i;
  v[1].i = -v[1].i;

  MotorMessage left_msg(MOTOR_MSG_TYPE_WRITE, REG_ADDR_SET_LEFT_SPEED, v[0]);
  MotorMessage right_msg(MOTOR_MSG_TYPE_WRITE, REG_ADDR_SET_RIGHT_SPEED, v[1]);
  sendMessage(left_msg);
  sendMessage(right_msg);
}

void MotorController::sendMessage(MotorMessage msg) {
  CRC crc;
  uint8_t data[9];
  boost::system::error_code ec;

  msg.htob();
  ur::message_value v = msg.getValue();
  size_t size = sizeof(data);

  data[0] = msg.getDelimiter();
  data[1] = msg.getVersion();
  data[2] = msg.getType();
  data[3] = msg.getAddr();
  data[4] = v.varray[0];
  data[5] = v.varray[1];
  data[6] = v.varray[2];
  data[7] = v.varray[3];
  data[8] = crc.checksum(data, 1,7);

  size_t offset = 0;

  while (offset < size) {
    size_t ret;

    mtx.lock();
    ret = m_pBoard->write_some(boost::asio::buffer(data, size), ec);
    mtx.unlock();

    if (ret < 0) {
      ROS_FATAL("Failed to write %zu bytes", size - offset);
      ros::shutdown();
    } else if (!ret) {
      ROS_FATAL("EOF while attempting to write %zu bytes", size - offset);
      ros::shutdown();
    }

    offset += ret;
  }
  ROS_INFO("sendMessage() - %2x %2x %2x %2x %2x %2x %2x %2x %2x",
      data[0],
      data[1],
      data[2],
      data[3],
      data[4],
      data[5],
      data[6],
      data[7],
      data[8]);
}

void MotorController::processMessage(MotorMessage *msg)
{

  switch(msg->getAddr())
  {
  case REG_ADDR_ON_OFF :
    delete msg;
    break;
  case REG_ADDR_BRAKE :
    delete msg;
    break;
  case REG_ADDR_HALT :
    delete msg;
    break;
  case REG_ADDR_LEFT_PWM :
    delete msg;
    break;
  case REG_ADDR_RIGHT_PWM :
    delete msg;
    break;
  case REG_ADDR_LEFT_DIRECTION :
    delete msg;
    break;
  case REG_ADDR_RIGHT_DIRECTION :
    delete msg;
    break;
  case REG_ADDR_SET_LEFT_SPEED :
    delete msg;
    break;
  case REG_ADDR_SET_RIGHT_SPEED :
    delete msg;
    break;
  case REG_ADDR_LEFT_RAMP :
    delete msg;
    break;
  case REG_ADDR_RIGHT_RAMP :
    delete msg;
    break;
  case REG_ADDR_LEFT_TICS :
    collectOdometry(msg->getValue(), MOTOR_LEFT);
    delete msg;
    break;
  case REG_ADDR_RIGHT_TICS :
    collectOdometry(msg->getValue(), MOTOR_RIGHT);
    delete msg;
    break;
  case REG_ADDR_DEADMAN_TIMER :
    delete msg;
    break;
  case REG_ADDR_LEFT_CURRENT_SENSE :
    delete msg;
    break;
  case REG_ADDR_RIGHT_RIGHT_CURRENT_SENSE :
    delete msg;
    break;
  case REG_ADDR_ERROR_COUNT :
    delete msg;
    break;
  case REG_ADDR_5V_MAIN_ERR :
    delete msg;
    break;
  case REG_ADDR_5V_AUX_ERR :
    delete msg;
    break;
  case REG_ADDR_12V_MAIN_ERR :
    delete msg;
    break;
  case REG_ADDR_12V_AUX_ERR :
    delete msg;
    break;
  case REG_ADDR_5V_MAIN_OVERLOAD :
    delete msg;
    break;
  case REG_ADDR_5V_AUX_OVERLOAD :
    delete msg;
    break;
  case REG_ADDR_12V_MAIN_OVERLOAD :
    delete msg;
    break;
  case REG_ADDR_12V_AUX_OVERLOAD :
    delete msg;
    break;
  case REG_ADDR_LEFT_MOTOR_ERR :
    delete msg;
    break;
  case REG_ADDR_RIGHT_MOTOR_ERR :
    delete msg;
    break;
  case REG_ADDR_PID_PROPORTIONAL :
    delete msg;
    break;
  case REG_ADDR_PID_INTEGRAL :
    delete msg;
    break;
  case REG_ADDR_PID_DERIVATIVE :
    delete msg;
    break;
  case REG_ADDR_PID_DENOMINATOR :
    delete msg;
    break;
  case REG_ADDR_LED1 :
    delete msg;
    break;
  case REG_ADDR_LED2 :
    delete msg;
    break;
  case REG_ADDR_HDWR_VER :
    delete msg;
    break;
  case REG_ADDR_FMWR_VER :
    delete msg;
    break;
  case REG_ADDR_BATTERY_VOLTAGE :
    delete msg;
    break;
  case REG_ADDR_5V_MAIN_CURRENT :
    delete msg;
    break;
  case REG_ADDR_12V_MAIN_CURRENT :
    delete msg;
    break;
  case REG_ADDR_5V_AUX_CURRENT :
    delete msg;
    break;
  case REG_ADDR_12V_AUX_CURRENT :
    delete msg;
    break;
  case REG_ADDR_LEFT_MOTOR_SPEED :
    delete msg;
    break;
  case REG_ADDR_RIGHT_MOTOR_SPEED :
    delete msg;
    break;
  default :
    throw "Invalid register address";
    delete msg;
    break;
  }
}

void MotorController::getRegister(uint8_t reg) {
  ur::message_value v;
  v.i = 0x00;
  MotorMessage msg(MOTOR_MSG_TYPE_READ, reg, v);

  msg.setChecksum();
  sendMessage(msg);
}

void MotorController::setRegister(uint8_t reg, message_value v) {

#define CLAMP_TO_RANGE(v, max, min) \
  (((v) < min) ? (min) : ((v) > max) ? (max) : (v))

  switch(reg)
  {
  case REG_ADDR_ON_OFF :
  case REG_ADDR_BRAKE :
  case REG_ADDR_HALT :
  case REG_ADDR_LED1 :
  case REG_ADDR_LED2 :
    if ( v.ui != 0x00 )  // only 0x00 and 0xFF allowed
      v.ui = 0xFF;
    break;
  case REG_ADDR_LEFT_PWM :
  case REG_ADDR_RIGHT_PWM :
  case REG_ADDR_LEFT_TICS :
  case REG_ADDR_RIGHT_TICS :
  case REG_ADDR_LEFT_CURRENT_SENSE :
  case REG_ADDR_RIGHT_RIGHT_CURRENT_SENSE :
  case REG_ADDR_HDWR_VER :
  case REG_ADDR_FMWR_VER :
  case REG_ADDR_BATTERY_VOLTAGE :
  case REG_ADDR_5V_MAIN_CURRENT :
  case REG_ADDR_12V_MAIN_CURRENT :
  case REG_ADDR_5V_AUX_CURRENT :
  case REG_ADDR_12V_AUX_CURRENT :
  case REG_ADDR_LEFT_MOTOR_SPEED :
  case REG_ADDR_RIGHT_MOTOR_SPEED :
    ROS_ERROR("Attempted to set the read only register: %d", reg);
    break;
  case REG_ADDR_SET_LEFT_SPEED :
  case REG_ADDR_SET_RIGHT_SPEED :
    v.i = CLAMP_TO_RANGE(v.i, 730, -730);  // signed 0x02 DA
    break;
  case REG_ADDR_LEFT_RAMP :
  case REG_ADDR_RIGHT_RAMP :
    v.ui = CLAMP_TO_RANGE(v.ui, 16777215, 0);  // unsigned 0xFF FF FF
    break;
  case REG_ADDR_DEADMAN_TIMER :
  case REG_ADDR_PID_PROPORTIONAL :
  case REG_ADDR_PID_INTEGRAL :
  case REG_ADDR_PID_DERIVATIVE :
  case REG_ADDR_PID_DENOMINATOR :
    v.i = CLAMP_TO_RANGE(v.i, 16777215, -16777215);  // signed 0xFF FF FF
    break;
  default :
    throw "Invalid register address";
    break;
  }
#undef CLAMP_TO_RANGE
//  ============= TBD =============
//  case REG_ADDR_ERROR_COUNT :
//  case REG_ADDR_5V_MAIN_ERR :
//  case REG_ADDR_5V_AUX_ERR :
//  case REG_ADDR_12V_MAIN_ERR :
//  case REG_ADDR_12V_AUX_ERR :
//  case REG_ADDR_5V_MAIN_OVERLOAD :
//  case REG_ADDR_5V_AUX_OVERLOAD :
//  case REG_ADDR_12V_MAIN_OVERLOAD :
//  case REG_ADDR_12V_AUX_OVERLOAD :
//  case REG_ADDR_LEFT_MOTOR_ERR :
//  case REG_ADDR_RIGHT_MOTOR_ERR :

  MotorMessage msg(MOTOR_MSG_TYPE_WRITE, reg, v);

  msg.setChecksum();
  sendMessage(msg);
}

void MotorController::listen(void)
{
//  if ( m_pBoard->get_io_service() == NULL || !m_pBoard->is_open() )
//    return;
  ROS_INFO("running in listen()");

  m_pBoard->get_io_service().reset();
  boost::asio::deadline_timer timer(m_pBoard->get_io_service());

  m_pBoard->async_read_some(boost::asio::buffer(m_Buffer, 256),
                            boost::bind(&MotorController::on_receive,
                                        this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred,
                                        &timer));


  timer.expires_from_now(boost::posix_time::milliseconds(m_timeout));
  timer.async_wait(boost::bind(&MotorController::time_out,
                               this,
                               boost::asio::placeholders::error));

  mtx.lock();
  m_pBoard->get_io_service().run();
  mtx.unlock();

}

void MotorController::on_receive(const boost::system::error_code& ec, size_t bytes_transferred, boost::asio::deadline_timer *timer)
{
  size_t len = sizeof(m_msgReading);
  ur::MotorMessage *msg;

  if ( !m_pBoard->is_open() )
    return;
  if ( ec )
  {
    timer->cancel();
    return;
  }

  ROS_INFO("running in on_receive() - bytes_transferred is %lu", bytes_transferred);
  for ( unsigned int i = 0; i < bytes_transferred; i++ )
  {
    uint8_t c = m_Buffer[i];
    ROS_ERROR("read char \"%c\"", m_Buffer[i]);
    if ( ( m_msgOffset == 0 ) && ( c != MOTOR_MSG_DELIM ) )
      continue;  // throw away and continue to look for the beginning of a message

    if ( ( m_msgOffset == 1 ) && ( c != MOTOR_MSG_PROTOCOL_VER ) )
    {
      initializeMessageRead();  // Wrong protocol version.  Throw message away and start looking for the beginning of the next message.
      continue;
    }

    m_msgReading.msg_array[m_msgOffset] = c;
    m_msgOffset++;

    if (  m_msgOffset == len ) // we have read in a complete message
    {
      // Instantiate a new message
      msg = new MotorMessage(m_msgReading.msg_typed.type,
                             m_msgReading.msg_typed.addr,
                             m_msgReading.msg_typed.val);

      // clear out the buffer array for reading in the next message
      initializeMessageRead();

      // check message integrity
      // final byte is the checksum
      if ( !msg->checkChecksum( (unsigned char) c) )  // last byte is still in scope
      { // throw away the bad message
        delete msg;
        continue;
      }

      // We have a valid message
      // convert endianness
      msg->btoh();

      // Debug...
      ROS_INFO("listen-processed() - %2x %2x %2x %2x %2x %2x %2x %2x %2x",
          msg->getDelimiter(),
          msg->getVersion(),
          msg->getType(),
          msg->getAddr(),
          msg->getValue().varray[0],
          msg->getValue().varray[1],
          msg->getValue().varray[2],
          msg->getValue().varray[3],
          msg->getChecksum());

      // doit
      processMessage(msg);
    }
  }
  timer->cancel();
}

void MotorController::collectOdometry(ur::message_value v, int32_t motor_id)
{
  if ( motor_id == MOTOR_LEFT )
  {
    m_iOdomLeft += v.i;
    m_bHaveOdomLeft = true;
  }
  else if ( motor_id == MOTOR_RIGHT )
  {
    m_iOdomRight += v.i;
    m_bHaveOdomRight = true;
  }

  if ( m_bHaveOdomLeft && m_bHaveOdomRight )
  {
    publishOdometry();
    m_iOdomLeft = m_iOdomRight = 0x00;
    m_bHaveOdomLeft = m_bHaveOdomRight = false;
  }
}

void MotorController::publishOdometry(void)
{
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - m_rosLastOdomTime).toSec();

  m_iPositionLeft += m_iOdomLeft;
  m_iPositionRight += m_iOdomRight;

  // compute odometry in a typical way give the velocities of the robot
  double d_left = m_iOdomLeft * m_fSensorDistance;
  double d_right = m_iOdomRight * m_fSensorDistance;
  double d_theta = (d_right - d_left)/m_fWheelBase;

  double dist = (d_left + d_right)/2.0f;
  double dx = dist * cos(m_dOdometryTheta);
  double dy = dist * sin(m_dOdometryTheta);

  m_dOdometryX += dx;
  m_dOdometryY += dy;
  m_dOdometryTheta += d_theta;

  // publish the messages
  double left_angle = m_iPositionLeft * m_fSensorDistance /
                                   (m_fWheelDiameter * 0.5f);
  double right_angle = m_iPositionRight * m_fSensorDistance /
                                   (m_fWheelDiameter * 0.5f);
  double left_vel = m_iOdomLeft * m_fSensorDistance / dt;
  double right_vel = m_iOdomRight * m_fSensorDistance / dt;
   //Publish joints
  sensor_msgs::JointState joints;

  joints.header.stamp = current_time;

  joints.name.push_back(m_sLeftMotorJointName);
  joints.position.push_back(left_angle);
  joints.position.push_back(left_vel);
  joints.effort.push_back(0.0);
  joints.name.push_back(m_sRightMotorJointName);
  joints.position.push_back(right_angle);
  joints.position.push_back(right_vel);
  joints.effort.push_back(0.0);

  m_pubJointStates->publish(joints);

  //Publish Odometry
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = m_dOdometryX;
  odom.pose.pose.position.y = m_dOdometryY;
  odom.pose.pose.position.z = 0.0;
  // Compute the quaternion for our yaw
  geometry_msgs::Quaternion odom_quat;

  odom_quat.x = m_dOdometryX;
  odom_quat.y = m_dOdometryY;
  odom_quat.z = 0.0;
  odom_quat.w = m_dOdometryTheta;
  odom.pose.pose.orientation = odom_quat;

  for( int32_t i = 0; i < 36; i++ )
  {
    odom.pose.covariance[i] = m_faOdometryCovariance[i];
    odom.twist.covariance[i] = m_faOdometryCovariance[i];
  }

  m_pubOdometry->publish(odom);
  // Generate the transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = m_dOdometryX;
  odom_trans.transform.translation.y = m_dOdometryY;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  m_pubTransform->sendTransform(odom_trans);

  m_rosLastOdomTime = current_time;
}

// Create a diagnostics message for the state of the motor controller
// As messages come in from the board, they are sent to this function
// which stores the information in an array.
void MotorController::collectDiagnostics(ur::message_value v)
{

}

void MotorController::publishDiagnostics(void)
{

}

// function both zero's out the message character array
// as well as sets the offset to the first byte of the array
void MotorController::initializeMessageRead(void)
{
  //Initialize the raw message buffer;
  m_msgOffset = 0;
  for ( int i = 0; i < 9; i++ )
  {
    m_msgReading.msg_array[i] = (uint8_t) 0;
  }
}

void MotorController::stop(void)
{
  boost::mutex::scoped_lock look(mtx);

  if ( m_pBoard )
  {
    m_pBoard->cancel();
    m_pBoard->close();
    m_pBoard.reset();
  }
  m_ioService.stop();
  m_ioService.reset();
}

void MotorController::time_out(const boost::system::error_code& ec)
{
  if ( ec )
  {
    // we had an error
    return;
  }

  // we timed out
  m_pBoard->cancel();
}


} /* namespace ur */
