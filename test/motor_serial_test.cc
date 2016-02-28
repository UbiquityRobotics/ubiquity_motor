#include <gtest/gtest.h>

#include <ubiquity_motor/motor_serial.h>
#include <ubiquity_motor/motor_message.h>
#include <ros/ros.h>


#include <string>

#if defined(__linux__)
#include <pty.h>
#else
#include <util.h>
#endif

class MotorSerialTests : public ::testing::Test {
protected:
  virtual void SetUp() {
    if (openpty(&master_fd, &slave_fd, name, NULL, NULL) == -1) {
      perror("openpty");
      exit(127);
    }

    ASSERT_TRUE(master_fd > 0);
    ASSERT_TRUE(slave_fd > 0);
    ASSERT_TRUE(std::string(name).length() > 0);

    ros::Time::init();
    motors = new MotorSerial(std::string(name), 9600, 100);
  }

  virtual void TearDown() {
    delete motors;
  }

  MotorSerial * motors;
  int master_fd;
  int slave_fd;
  char name[100];
};

TEST_F(MotorSerialTests, writeWorks) {
	MotorMessage version;
	version.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
	version.setType(MotorMessage::TYPE_READ);
	version.setData(0);
	motors->transmitCommand(version);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}