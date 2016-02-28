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
    motors = new MotorSerial(std::string(name), 9600, 10);
  }

  virtual void TearDown() {
    delete motors;
  }

  MotorSerial * motors;
  int master_fd;
  int slave_fd;
  char name[100];
};

TEST_F(MotorSerialTests, readWorks){
  uint8_t test[]= {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0E};
  //char test[]= {0x0E, 0x2C, 0x01, 0x00, 0x00, 0x07, 0xBB, 0x02, 0x7E};
  write(master_fd, test, 9);

  while(!motors->commandAvailable()) {
  }

  MotorMessage mm;
  mm = motors-> receiveCommand();
  ASSERT_EQ(300, mm.getData());
  ASSERT_EQ(MotorMessage::TYPE_WRITE, mm.getType());
  ASSERT_EQ(MotorMessage::REG_LEFT_SPEED_SET, mm.getRegister());
}

TEST_F(MotorSerialTests, writeWorks) {
	MotorMessage version;
	version.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
	version.setType(MotorMessage::TYPE_READ);
	version.setData(0);
	motors->transmitCommand(version);

  uint8_t arr[9];
  read(master_fd, arr, 9);

  std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

  ASSERT_EQ(input, version.serialize());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}