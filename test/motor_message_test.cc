#include <ubiquity_motor/motor_message.h>
#include <gtest/gtest.h>

TEST(ubiquity_motor_message, motor_message_commandtype) {
	MotorMessage mc;

	mc.setType(MotorMessage::TYPE_READ);
	ASSERT_EQ(MotorMessage::TYPE_READ, mc.getType());

	mc.setType(MotorMessage::TYPE_WRITE);
	ASSERT_EQ(MotorMessage::TYPE_WRITE, mc.getType());
}

TEST(ubiquity_motor_message, motor_message_commandtype_bad) {
	MotorMessage mc;

	mc.setType(static_cast<MotorMessage::MessageTypes>(0xF3));
	ASSERT_NE(0xF3, mc.getType());

	mc.setType(static_cast<MotorMessage::MessageTypes>(0xAB));
	ASSERT_NE(0xAB, mc.getType());
}

TEST(ubiquity_motor_message, motor_message_register) {
	MotorMessage mc;

	mc.setRegister(MotorMessage::REG_BRAKE_STOP);
	ASSERT_EQ(MotorMessage::REG_BRAKE_STOP, mc.getRegister());
	ASSERT_NE(MotorMessage::REG_STOP_START, mc.getRegister());
}

TEST(ubiquity_motor_message, motor_message_register_bad) {
	MotorMessage mc;

	mc.setRegister(static_cast<MotorMessage::Registers>(0xF3));
	ASSERT_NE(0xF3, mc.getRegister());

	mc.setRegister(static_cast<MotorMessage::Registers>(0x05));
	ASSERT_NE(0x05, mc.getRegister());
}

TEST(ubiquity_motor_message, motor_message_data) {
	MotorMessage mc;
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

TEST(ubiquity_motor_message, motor_message_serialize) {
	MotorMessage mc;
	mc.setData(300);
	mc.setType(MotorMessage::TYPE_WRITE);
	mc.setRegister(MotorMessage::REG_LEFT_SPEED_SET);

	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0E};

	std::vector<uint8_t> expect(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(expect, mc.serialize());
}

TEST(ubiquity_motor_message, motor_message_deserialize_good) {
	MotorMessage mc;

	//Test good message
	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0E};

	std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(0, mc.deserialize(input));
	ASSERT_EQ(300, mc.getData());
	ASSERT_EQ(MotorMessage::TYPE_WRITE, mc.getType());
	ASSERT_EQ(MotorMessage::REG_LEFT_SPEED_SET, mc.getRegister());
}

TEST(ubiquity_motor_message, motor_message_deserialize_bad_checksum) {
	MotorMessage mc;

	//Test bad checksum
	uint8_t arr1[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0F};

	std::vector<uint8_t> input1(arr1, arr1 + sizeof(arr1)/ sizeof(uint8_t));

	ASSERT_EQ(3, mc.deserialize(input1));
}

TEST(ubiquity_motor_message, motor_message_deserialize_bad_type) {
	MotorMessage mc;

	//Test type with good checksum
	uint8_t arr1[] = {0x7E, 0x02, 0xBA, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0F};

	std::vector<uint8_t> input1(arr1, arr1 + sizeof(arr1)/ sizeof(uint8_t));

	ASSERT_EQ(4, mc.deserialize(input1));
}

TEST(ubiquity_motor_message, motor_message_deserialize_bad_register) {
	MotorMessage mc;

	//Test bad register with good checksum
	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x30, 0x00, 0x00, 0x01, 0x2C, 0xE5};

	std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(5, mc.deserialize(input));
}