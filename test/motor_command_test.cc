#include <ubiquity_motor/motor_command.h>
#include <gtest/gtest.h>

TEST(ubiquity_motor, motor_command_commandtype) {
	MotorCommand mc;

	mc.setType(MotorCommand::TYPE_READ);
	ASSERT_EQ(MotorCommand::TYPE_READ, mc.getType());

	mc.setType(MotorCommand::TYPE_WRITE);
	ASSERT_EQ(MotorCommand::TYPE_WRITE, mc.getType());
}

TEST(ubiquity_motor, motor_command_register) {
	MotorCommand mc;

	mc.setRegister(MotorCommand::REG_BRAKE_STOP);
	ASSERT_EQ(MotorCommand::REG_BRAKE_STOP, mc.getRegister());
	ASSERT_NE(MotorCommand::REG_STOP_START, mc.getRegister());
}

TEST(ubiquity_motor, motor_command_data) {
	MotorCommand mc;
	int32_t i = -0xABC;
	mc.setData(i);
	ASSERT_EQ(-0xABC, mc.getData());

	int16_t abcd = -0xABC;
	mc.setData(abcd);
	ASSERT_EQ(-0xABC, mc.getData());

	abcd = 522;
	mc.setData(abcd);
	ASSERT_EQ(522, mc.getData());
}

TEST(ubiquity_motor, motor_command_serialize) {
	MotorCommand mc;
	mc.setData(300);
	mc.setType(MotorCommand::TYPE_WRITE);
	mc.setRegister(MotorCommand::REG_LEFT_SPEED_SET);

	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0E};

	std::vector<uint8_t> expect(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(expect, mc.serialize());
}