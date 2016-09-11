#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_message.h>

#if defined(__linux__)
#include <pty.h>
#else
#include <util.h>
#endif

class MotorHardwareTests : public ::testing::Test {
protected:
    virtual void SetUp() {
        if (openpty(&master_fd, &slave_fd, name, NULL, NULL) == -1) {
            perror("openpty");
            exit(127);
        }

        ASSERT_TRUE(master_fd > 0);
        ASSERT_TRUE(slave_fd > 0);
        ASSERT_TRUE(std::string(name).length() > 0);

        nh.setParam("ubiquity_motor/serial_port", std::string(name));
        CommsParams cp(nh);
        cp.serial_loop_rate = 5000.0;
        FirmwareParams fp(nh);

        robot = new MotorHardware(nh, cp, fp);
    }

    virtual void TearDown() { delete robot; }

    MotorHardware *robot;
    ros::NodeHandle nh;
    int master_fd;
    int slave_fd;
    char name[100];
};

TEST_F(MotorHardwareTests, writeSpeedsOutputs) {
    robot->writeSpeeds();
    usleep(1000);

    int aval;
    RawMotorMessage out;
    // Make sure that we get exactly 1 message out on port
    ASSERT_NE(-1, ioctl(master_fd, FIONREAD, &aval));
    ASSERT_EQ(out.size(), aval);
    ASSERT_EQ(out.size(), read(master_fd, out.c_array(), out.size()));

    MotorMessage mm;
    ASSERT_EQ(0, mm.deserialize(out));
    ASSERT_EQ(MotorMessage::REG_BOTH_SPEED_SET, mm.getRegister());
    ASSERT_EQ(0, mm.getData());
}

TEST_F(MotorHardwareTests, oldFirmware) {
    MotorMessage mm;
    mm.setType(MotorMessage::TYPE_RESPONSE);
    mm.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
    mm.setData(10);

    RawMotorMessage out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));

    usleep(5000);
    ASSERT_THROW(robot->readInputs(), std::runtime_error);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "param_test");
    return RUN_ALL_TESTS();
}