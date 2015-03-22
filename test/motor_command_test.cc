#include <ubiquity_motor/motor_command.h>
#include <gtest/gtest.h>

TEST(ubiquity_motor_command, motor_command_commandtype) {
	MotorCommand mc;

	mc.setType(MotorCommand::TYPE_READ);
	ASSERT_EQ(MotorCommand::TYPE_READ, mc.getType());

	mc.setType(MotorCommand::TYPE_WRITE);
	ASSERT_EQ(MotorCommand::TYPE_WRITE, mc.getType());
}

TEST(ubiquity_motor_command, motor_command_commandtype_bad) {
	MotorCommand mc;

	mc.setType(static_cast<MotorCommand::CommandTypes>(0xF3));
	ASSERT_NE(0xF3, mc.getType());

	mc.setType(static_cast<MotorCommand::CommandTypes>(0xAB));
	ASSERT_NE(0xAB, mc.getType());
}

TEST(ubiquity_motor_command, motor_command_register) {
	MotorCommand mc;

	mc.setRegister(MotorCommand::REG_BRAKE_STOP);
	ASSERT_EQ(MotorCommand::REG_BRAKE_STOP, mc.getRegister());
	ASSERT_NE(MotorCommand::REG_STOP_START, mc.getRegister());
}

TEST(ubiquity_motor_command, motor_command_register_bad) {
	MotorCommand mc;

	mc.setRegister(static_cast<MotorCommand::Registers>(0xF3));
	ASSERT_NE(0xF3, mc.getRegister());

	mc.setRegister(static_cast<MotorCommand::Registers>(0x05));
	ASSERT_NE(0x05, mc.getRegister());
}

TEST(ubiquity_motor_command, motor_command_data) {
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

TEST(ubiquity_motor_command, motor_command_serialize) {
	MotorCommand mc;
	mc.setData(300);
	mc.setType(MotorCommand::TYPE_WRITE);
	mc.setRegister(MotorCommand::REG_LEFT_SPEED_SET);

	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0E};

	std::vector<uint8_t> expect(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(expect, mc.serialize());
}

TEST(ubiquity_motor_command, motor_command_deserialize_good) {
	MotorCommand mc;

	//Test good message
	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0E};

	std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(0, mc.deserialize(input));
	ASSERT_EQ(300, mc.getData());
	ASSERT_EQ(MotorCommand::TYPE_WRITE, mc.getType());
	ASSERT_EQ(MotorCommand::REG_LEFT_SPEED_SET, mc.getRegister());
}

TEST(ubiquity_motor_command, motor_command_deserialize_bad_checksum) {
	MotorCommand mc;

	//Test bad checksum
	uint8_t arr1[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0F};

	std::vector<uint8_t> input1(arr1, arr1 + sizeof(arr1)/ sizeof(uint8_t));

	ASSERT_EQ(3, mc.deserialize(input1));
}

TEST(ubiquity_motor_command, motor_command_deserialize_bad_type) {
	MotorCommand mc;

	//Test type with good checksum
	uint8_t arr1[] = {0x7E, 0x02, 0xBA, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0F};

	std::vector<uint8_t> input1(arr1, arr1 + sizeof(arr1)/ sizeof(uint8_t));

	ASSERT_EQ(4, mc.deserialize(input1));
}

TEST(ubiquity_motor_command, motor_command_deserialize_bad_register) {
	MotorCommand mc;

	//Test bad register with good checksum
	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x30, 0x00, 0x00, 0x01, 0x2C, 0xE5};

	std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(5, mc.deserialize(input));
}