/*
 * MotorController.h
 *
 *  Created on: Apr 4, 2015
 *      Author: kurt
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>

#include "MotorMessage.h"

namespace ur {

class MotorController {
private:
  int32_t m_iBaud;
  uint32_t m_iMagnetCount;
  uint32_t m_iSunGearTeeth;
  uint32_t m_iPlanetGearTeeth;
  uint32_t m_iRingGearTeeth;
  uint32_t m_iOdomLeft;
  uint32_t m_iOdomRight;
  bool m_bHaveOdomLeft;
  bool m_bHaveOdomRight;
  int64_t m_iPositionLeft;
  int64_t m_iPositionRight;
  float m_fUnitsPerSecond;
  float m_fWheelBase;
  float m_fWheelDiameter;
  float m_fWheelCircumference;
  float m_fGearRatio;
  float m_fTransitionsPerMagnet;
  float m_fSensorDistance;
  float m_faOdometryCovariance[36];
  double m_dOdometryX;
  double m_dOdometryY;
  double m_dOdometryTheta;
  std::string m_sDevice;
  std::string m_sLeftMotorJointName;
  std::string m_sRightMotorJointName;
  std::string m_sVelocityTopic;
//  boost::thread *m_tListener;
  ros::Time m_rosLastOdomTime;
  ros::Duration m_rosOdomRate;
  ros::Time m_rosLastDiagnosticTime;
  ros::Duration m_rosDiagnosticRate;
  ros::Publisher *m_pubJointStates;
  ros::Publisher *m_pubOdometry;
  tf::TransformBroadcaster *m_pubTransform;

  boost::shared_ptr<boost::asio::serial_port> m_pBoard;
  boost::asio::io_service m_ioService;
  size_t m_timeout;
  boost::mutex mtx;
  char m_Buffer[256];
  raw_message m_msgReading;
  size_t m_msgOffset;

  void setWheelVelocities(float left, float right);

  // only function that 'write()'s to the motor controller board.
  // Accepts a Motor Message.  This function deals with the endianness
  // of the user data within the message.
  void sendMessage(ur::MotorMessage msg);

  // processMessage() is for processing messages originating from the
  // motor controller board.  All messages need to be deleted once processed.
  void processMessage(ur::MotorMessage *msg);

  // getRegister() generates a MOTOR_MSG_TYPE_READ which is sent to the board.
  // The parameter "reg" is the register address (REG_ADDR_{}) for which you
  // want values.  Function causes the motor controller board to respond with
  // a MOTOR_MSG_TYPE_RESPONSE which get handled by processMessage()
  void getRegister(uint8_t reg);

  // setRegister() generates a MOTOR_MSG_TYPE_WRITE which is sent to the board.
  // The Parameters:  "reg" - The register address of the write.
  //                  "v" - The user data to write to the register.
  void setRegister(uint8_t reg, ur::message_value v);

  void on_receive(const boost::system::error_code& ec, size_t bytes_transferred, boost::asio::deadline_timer *timer);

  // The interface requires two calls to obtain the odometry for both wheels.
  // The motor controller node should make two calls at the same time
  // requesting odometry tics for the left motor and for the right motor.
  // When responses come back from the motor controller board, they are
  // passed to this function which collects each until it has both, then
  // calls the different reports
  void collectOdometry(ur::message_value v, int32_t motor_id);

  void publishOdometry(void);

  // Create a diagnostics message for the state of the motor controller
  // As messages come in from the board, they are sent to this function
  // which stores the information in an array.
  // The second mara
  void collectDiagnostics(ur::message_value v);

  void publishDiagnostics(void);

  void initializeMessageRead(void);

  void stop(void);

  void time_out(const boost::system::error_code& ec);

public:
  MotorController();
  virtual ~MotorController();

  void setDeviceName(std::string dev);
  void setBaudRate(int baud);
  void setLeftMotorJointName(std::string n);
  void setRightMotorJointName(std::string n);
  void setVelocityTopic(std::string n);
  void setOdomX(double val);
  void setOdomY(double val);
  void setOdomTheta(double val);
  void setOdomDuration(double duration);
  void setDiagDuration(double duration);
  void setJointStatesPublisher(ros::Publisher *pub);
  void setOdometryPublisher(ros::Publisher *pub);
  void setTransformBroadcaster(tf::TransformBroadcaster *broadcaster);
  boost::shared_ptr<boost::asio::serial_port> getBoard(void);

  void connect(void);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& cmd);
  void setPIDProportional(int32_t i);
  void setPIDIntegral(int32_t i);
  void setPIDDerivative(int32_t i);
  void setPIDDenominator(int32_t i);

  // Sets the delay for the deadman timer.  Parameter i is a value
  // between 0x00 and 0xFF FF FF.  0x00 noops the deadman timer.
  // 0x01 and above set the timing in 200ms increments.  i.e. setting
  // the deadman to 0x05 would set it to one second.
  void setDeadmanTimer(int32_t i);

  // calls get register for each of the wheels tic count
  void readOdometry(void);

  //calls get register for each of the diagnostic values we collect
  void readDiagnostics(void);

  // only function that 'read()'s from the motor controller board.
  // Function generates new MotorMessages and deals with the endianness
  // of the user data within the message.
  // Function passes valid read() messages to processMessage() which is
  // responsible for deleting the messages.
  void listen(void);
};

} /* namespace ur */

#endif /* MOTORCONTROLLER_H_ */
