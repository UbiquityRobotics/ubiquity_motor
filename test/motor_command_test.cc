#include <ubiquity_motor/motor_command.h>
#include <gtest/gtest.h>

TEST(ubiquity_motor, test1) {
	MotorCommand mc;
	mc.setRegister(MotorCommand::Registers::STOP_START);
	ASSERT_EQ(0x00, mc.getRegister());
	mc.setType(MotorCommand::READ);
	ASSERT_EQ(0xAA, mc.getType());
}