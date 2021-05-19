#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ubiquity_motor/motor_parameters.h>

TEST(MotorParameterTests, getParamOrDefaultTest) {
    ros::NodeHandle nh;
    ASSERT_EQ(5, getParamOrDefault(nh, "getParamOrDefaultTest", 5));
    int foo;
    ASSERT_TRUE(nh.getParam("getParamOrDefaultTest", foo));
    ASSERT_EQ(5, foo);

    ASSERT_EQ(5, getParamOrDefault(nh, "getParamOrDefaultTest", 10));
}

TEST(MotorParameterTests, NodeParamTest) {
    ros::NodeHandle nh;
    NodeParams np(nh);
    ASSERT_DOUBLE_EQ(10.0, np.controller_loop_rate);
    nh.setParam("ubiquity_motor/controller_loop_rate", 50.0);
    np = NodeParams(nh);
    ASSERT_DOUBLE_EQ(50.0, np.controller_loop_rate);
}

TEST(MotorParameterTests, CommsParamsTest) {
    ros::NodeHandle nh;
    CommsParams cp(nh);
    ASSERT_EQ("/dev/ttyS0", cp.serial_port);
    nh.setParam("ubiquity_motor/serial_port", "/dev/foo");
    cp = CommsParams(nh);
    ASSERT_EQ("/dev/foo", cp.serial_port);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "param_test");
    return RUN_ALL_TESTS();
}
