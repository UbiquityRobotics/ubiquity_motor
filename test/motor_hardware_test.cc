#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_message.h>
#include <limits>

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

        CommsParams cp(nh);
        cp.serial_port = std::string(name);
        FirmwareParams fp(nh);
	NodeParams np(nh);

        robot = new MotorHardware(nh, np, cp, fp);
    }

    virtual void TearDown() { delete robot; }

    void wait_for_read(){
        unsigned int count = 0;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(master_fd, &fds);
        struct timeval timeout = { 10, 0 }; /* 10 seconds */
        int ret = select(master_fd+1, &fds, NULL, NULL, &timeout);
        EXPECT_EQ(1, ret);

        ASSERT_NE(-1, ioctl(master_fd, TIOCINQ, &count));
        EXPECT_EQ(8, count);
        usleep(1000);
    }

    void wait_for_write(){
        unsigned int count_out = 8;
        while (count_out > 0) {
            ASSERT_NE(-1, ioctl(master_fd, TIOCOUTQ, &count_out));
        }
        EXPECT_EQ(0, count_out);
        usleep(10000);
    }

    MotorHardware *robot;
    ros::NodeHandle nh;
    int master_fd;
    int slave_fd;
    char name[100];
};


TEST_F(MotorHardwareTests, writeSpeedsOutputs) {
    robot->writeSpeeds();
    
    int aval;
    RawMotorMessage out;

    wait_for_read();

    // Make sure that we get exactly 1 message out on port
    ASSERT_NE(-1, ioctl(master_fd, FIONREAD, &aval));
    ASSERT_EQ(out.size(), aval);
    ASSERT_EQ(out.size(), read(master_fd, out.c_array(), out.size()));

    MotorMessage mm;
    ASSERT_EQ(0, mm.deserialize(out));
    ASSERT_EQ(MotorMessage::REG_BOTH_SPEED_SET, mm.getRegister());
    ASSERT_EQ(0, mm.getData());
}

TEST_F(MotorHardwareTests, nonZeroWriteSpeedsOutputs) {
    robot->joints_[0].velocity_command = 5;
    robot->joints_[1].velocity_command = -5;
    robot->writeSpeeds();

    int aval;
    RawMotorMessage out;

    wait_for_read();

    // Make sure that we get exactly 1 message out on port
    ASSERT_NE(-1, ioctl(master_fd, FIONREAD, &aval));
    ASSERT_EQ(out.size(), aval);
    ASSERT_EQ(out.size(), read(master_fd, out.c_array(), out.size()));

    MotorMessage mm;
    ASSERT_EQ(0, mm.deserialize(out));
    ASSERT_EQ(MotorMessage::REG_BOTH_SPEED_SET, mm.getRegister());
    int16_t left = (mm.getData() >> 16) & 0xffff;
    int16_t right = mm.getData() & 0xffff;

    // Left is 5 rad/s so it should be positive
    ASSERT_LT(0, left);
    // Right is -5 rad/s so it should be positive
    ASSERT_GT(0, right);
}

TEST_F(MotorHardwareTests, odomUpdatesPosition) {
    MotorMessage mm;
    mm.setType(MotorMessage::TYPE_RESPONSE);
    mm.setRegister(MotorMessage::REG_BOTH_ODOM);
    mm.setData((50 << 16) | (-50 & 0x0000ffff));

    RawMotorMessage out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));
    wait_for_write();
    robot->readInputs(0);

    double left = robot->joints_[0].position;
    double right = robot->joints_[1].position;

    // Left is 5 rad/s so it should be positive
    ASSERT_LT(0, left);
    // Right is -5 rad/s so it should be negative
    ASSERT_GT(0, right);

    // Send zero and re-read
    mm.setData((0 << 16) | (0 & 0x0000ffff));
    out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));
    wait_for_write();
    robot->readInputs(0);

    // Make sure that the value stays same
    ASSERT_EQ(left, robot->joints_[0].position);
    ASSERT_EQ(right, robot->joints_[1].position);

    // Send original message again and re-read
    mm.setData((50 << 16) | (-50 & 0x0000ffff));
    out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));
    wait_for_write();
    robot->readInputs(0);

    // Make sure that the value accumulates
    ASSERT_DOUBLE_EQ(left * 2, robot->joints_[0].position);
    ASSERT_DOUBLE_EQ(right * 2, robot->joints_[1].position);

    // Invert the odom message and re-send/read
    mm.setData((-50 << 16) | (50 & 0x0000ffff));
    out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));
    wait_for_write();
    robot->readInputs(0);

    // Values should be back the the first reading
    ASSERT_DOUBLE_EQ(left, robot->joints_[0].position);
    ASSERT_DOUBLE_EQ(right, robot->joints_[1].position);
}

TEST_F(MotorHardwareTests, odomUpdatesPositionMax) {
    MotorMessage mm;
    mm.setType(MotorMessage::TYPE_RESPONSE);
    mm.setRegister(MotorMessage::REG_BOTH_ODOM);
    mm.setData((std::numeric_limits<int16_t>::max() << 16) |
               (std::numeric_limits<int16_t>::min() & 0x0000ffff));

    RawMotorMessage out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));
    wait_for_write();
    robot->readInputs(0);

    double left = robot->joints_[0].position;
    double right = robot->joints_[1].position;

    // Left is + rad/s so it should be positive
    ASSERT_LT(0, left);
    // Right is - rad/s so it should be negative
    ASSERT_GT(0, right);

    // Send zero and re-read
    mm.setData((0 << 16) | (0 & 0x0000ffff));
    out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));
    wait_for_write();
    robot->readInputs(0);

    // Make sure that the value stays same
    ASSERT_EQ(left, robot->joints_[0].position);
    ASSERT_EQ(right, robot->joints_[1].position);

    // Send original message again and re-read
    mm.setData((std::numeric_limits<int16_t>::max() << 16) |
               (std::numeric_limits<int16_t>::min() & 0x0000ffff));
    out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));
    wait_for_write();
    robot->readInputs(0);

    // Make sure that the value accumulates
    ASSERT_DOUBLE_EQ(left * 2, robot->joints_[0].position);
    ASSERT_DOUBLE_EQ(right * 2, robot->joints_[1].position);

    // Invert the odom message and re-send/read
    mm.setData((std::numeric_limits<int16_t>::min() << 16) |
               (std::numeric_limits<int16_t>::max() & 0x0000ffff));
    out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));
    wait_for_write();
    robot->readInputs(0);

    // Values should be back the the first reading
    // Need to use NEAR due to high precision loss
    ASSERT_NEAR(left, robot->joints_[0].position, 0.1);
    ASSERT_NEAR(right, robot->joints_[1].position, 0.1);
}

TEST_F(MotorHardwareTests, requestVersionOutputs) {
    robot->requestFirmwareVersion();

    wait_for_read();

    int aval;
    RawMotorMessage out;
    // Make sure that we get exactly 1 message out on port
    ASSERT_NE(-1, ioctl(master_fd, FIONREAD, &aval));
    ASSERT_EQ(out.size(), aval);
    ASSERT_EQ(out.size(), read(master_fd, out.c_array(), out.size()));

    MotorMessage mm;
    ASSERT_EQ(0, mm.deserialize(out));
    ASSERT_EQ(MotorMessage::REG_FIRMWARE_VERSION, mm.getRegister());
    ASSERT_EQ(0, mm.getData());
}

TEST_F(MotorHardwareTests, oldFirmwareThrows) {
    MotorMessage mm;
    mm.setType(MotorMessage::TYPE_RESPONSE);
    mm.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
    mm.setData(10);

    RawMotorMessage out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));

    wait_for_write();
    ASSERT_THROW(robot->readInputs(0), std::runtime_error);
}

TEST_F(MotorHardwareTests, setDeadmanTimerOutputs) {
    robot->setDeadmanTimer(1000);
    wait_for_read();

    int aval;
    RawMotorMessage out;
    // Make sure that we get exactly 1 message out on port
    ASSERT_NE(-1, ioctl(master_fd, FIONREAD, &aval));
    ASSERT_EQ(out.size(), aval);
    ASSERT_EQ(out.size(), read(master_fd, out.c_array(), out.size()));

    MotorMessage mm;
    ASSERT_EQ(0, mm.deserialize(out));
    ASSERT_EQ(MotorMessage::REG_DEADMAN, mm.getRegister());
    ASSERT_EQ(1000, mm.getData());
}

/*
TEST_F(MotorHardwareTests, setParamsSendParams) {
    FirmwareParams fp;
    fp.pid_proportional = 12;
    fp.pid_integral = 12;
    fp.pid_derivative = 12;
    fp.pid_denominator = 12;
    fp.pid_moving_buffer_size = 12;

    robot->setParams(fp);

    int aval;
    RawMotorMessage out;

    for (int i; i < 5; ++i) {
        robot->sendParams();
        wait_for_read();
        // Make sure that we get exactly 1 message out on port each time
        ASSERT_NE(-1, ioctl(master_fd, FIONREAD, &aval));
        ASSERT_EQ(out.size(), aval);
        ASSERT_EQ(out.size(), read(master_fd, out.c_array(), out.size()));
    }
}

static bool called;

void callbackU(const std_msgs::UInt32 &data) {
    ASSERT_EQ(10, data.data);
    called = true;
}

void callbackS(const std_msgs::Int32 &data) {
    ASSERT_EQ(-10, data.data);
    called = true;
}

TEST_F(MotorHardwareTests, debugRegisterUnsignedPublishes) {
    called = false;
    ros::Subscriber sub = nh.subscribe("u50", 1, callbackU);

    MotorMessage mm;
    mm.setType(MotorMessage::TYPE_RESPONSE);
    mm.setRegister(static_cast<MotorMessage::Registers>(0x50));
    mm.setData(10);

    RawMotorMessage out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));

    wait_for_write();
    robot->readInputs(0);
    usleep(5000);
    ros::spinOnce();
    ASSERT_TRUE(called);

    called = false;
}

TEST_F(MotorHardwareTests, debugRegisterSignedPublishes) {
    called = false;
    ros::Subscriber sub = nh.subscribe("s50", 1, callbackS);

    MotorMessage mm;
    mm.setType(MotorMessage::TYPE_RESPONSE);
    mm.setRegister(static_cast<MotorMessage::Registers>(0x50));
    mm.setData(-10);

    RawMotorMessage out = mm.serialize();
    ASSERT_EQ(out.size(), write(master_fd, out.c_array(), out.size()));

    wait_for_write();
    robot->readInputs(0);
    usleep(5000);
    ros::spinOnce();
    ASSERT_TRUE(called);

    called = false;
}
*/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "param_test");
    return RUN_ALL_TESTS();
}
