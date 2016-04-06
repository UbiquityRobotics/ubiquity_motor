/**
Copyright (c) 2015, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN AiiiiiiiiiiiNY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <ubiquity_motor/motor_message.h>
#include <gtest/gtest.h>

TEST(ubiquity_motor_message, motor_message_commandtype) {
	MotorMessage mc;

	mc.setType(MotorMessage::TYPE_READ);
	ASSERT_EQ(MotorMessage::TYPE_READ, mc.getType());

	mc.setType(MotorMessage::TYPE_WRITE);
	ASSERT_EQ(MotorMessage::TYPE_WRITE, mc.getType());

	mc.setType(MotorMessage::TYPE_RESPONSE);
	ASSERT_EQ(MotorMessage::TYPE_RESPONSE, mc.getType());

	mc.setType(MotorMessage::TYPE_ERROR);
	ASSERT_EQ(MotorMessage::TYPE_ERROR, mc.getType());
}

TEST(ubiquity_motor_message, motor_message_commandtype_values) {
	MotorMessage mc;

	mc.setType(static_cast<MotorMessage::MessageTypes>(0xAA));
	ASSERT_EQ(MotorMessage::TYPE_READ, mc.getType());

	mc.setType(static_cast<MotorMessage::MessageTypes>(0xBB));
	ASSERT_EQ(MotorMessage::TYPE_WRITE, mc.getType());

	mc.setType(static_cast<MotorMessage::MessageTypes>(0xCC));
	ASSERT_EQ(MotorMessage::TYPE_RESPONSE, mc.getType());

	mc.setType(static_cast<MotorMessage::MessageTypes>(0xDD));
	ASSERT_EQ(MotorMessage::TYPE_ERROR, mc.getType());
}

TEST(ubiquity_motor_message, motor_message_commandtype_invalid) {
	MotorMessage mc;

	mc.setType(static_cast<MotorMessage::MessageTypes>(0xAB));
	ASSERT_NE(0xAB, mc.getType());
}

TEST(ubiquity_motor_message, motor_message_commandtype_overflow) {
	MotorMessage mc;

	mc.setType(static_cast<MotorMessage::MessageTypes>(0xFFFFFF));
	ASSERT_NE(0xFFFFFF, mc.getType());
}

TEST(ubiquity_motor_message, motor_message_commandtype_neg) {
	MotorMessage mc;

	mc.setType(static_cast<MotorMessage::MessageTypes>(-0xAA));
	ASSERT_NE(-0xAA, mc.getType());
	ASSERT_NE(0xAA, mc.getType());
}

TEST(ubiquity_motor_message, motor_message_register) {
	MotorMessage mc;

	mc.setRegister(MotorMessage::REG_BRAKE_STOP);
	ASSERT_EQ(MotorMessage::REG_BRAKE_STOP, mc.getRegister());
	ASSERT_NE(MotorMessage::REG_STOP_START, mc.getRegister());
}

TEST(ubiquity_motor_message, motor_message_register_values) {
	MotorMessage mc;

	mc.setRegister(static_cast<MotorMessage::Registers>(0x01));
	ASSERT_EQ(MotorMessage::REG_BRAKE_STOP, mc.getRegister());

	mc.setRegister(static_cast<MotorMessage::Registers>(0x10));
	ASSERT_EQ(MotorMessage::REG_ERROR_COUNT, mc.getRegister());

	mc.setRegister(static_cast<MotorMessage::Registers>(0x0B));
	ASSERT_EQ(MotorMessage::REG_LEFT_ODOM, mc.getRegister());
}

TEST(ubiquity_motor_message, motor_message_register_invalid) {
	MotorMessage mc;

	mc.setRegister(static_cast<MotorMessage::Registers>(0x05));
	ASSERT_NE(0x05, mc.getRegister());
}

TEST(ubiquity_motor_message, motor_message_register_overflow) {
	MotorMessage mc;

	mc.setRegister(static_cast<MotorMessage::Registers>(0xFFFFFF));
	ASSERT_NE(0xFFFFFF, mc.getRegister());
}

TEST(ubiquity_motor_message, motor_message_register_neg) {
	MotorMessage mc;

	mc.setRegister(static_cast<MotorMessage::Registers>(-0x07));
	ASSERT_NE(-0x07, mc.getRegister());
	ASSERT_NE(0x07, mc.getRegister());
}

TEST(ubiquity_motor_message, motor_message_data_64bit_pos) {
	MotorMessage mc;
	int64_t i = 0xABC;
	mc.setData(i);
	ASSERT_EQ(0xABC, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_64bit_zero) {
	MotorMessage mc;
	int64_t i = 0;
	mc.setData(i);
	ASSERT_EQ(0, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_64bit_neg) {
	MotorMessage mc;
	int64_t i = -0xABC;
	mc.setData(i);
	ASSERT_EQ(-0xABC, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_32bit_pos) {
	MotorMessage mc;
	int32_t i = 0xABC;
	mc.setData(i);
	ASSERT_EQ(0xABC, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_32bit_zero) {
	MotorMessage mc;
	int32_t i = 0;
	mc.setData(i);
	ASSERT_EQ(0, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_32bit_neg) {
	MotorMessage mc;
	int32_t i = -0xABC;
	mc.setData(i);
	ASSERT_EQ(-0xABC, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_16bit_pos) {
	MotorMessage mc;
	int16_t i = 0xABC;
	mc.setData(i);
	ASSERT_EQ(0xABC, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_16bit_zero) {
	MotorMessage mc;
	int16_t i = 0;
	mc.setData(i);
	ASSERT_EQ(0, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_16bit_neg) {
	MotorMessage mc;
	int16_t i = -0xABC;
	mc.setData(i);
	ASSERT_EQ(-0xABC, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_8bit_pos) {
	MotorMessage mc;
	int8_t i = 0x50;
	mc.setData(i);
	ASSERT_EQ(0x50, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_8bit_zero) {
	MotorMessage mc;
	int8_t i = 0;
	mc.setData(i);
	ASSERT_EQ(0, mc.getData());
}

TEST(ubiquity_motor_message, motor_message_data_8bit_neg) {
	MotorMessage mc;
	int8_t i = -0x50;
	mc.setData(i);
	ASSERT_EQ(-0x50, mc.getData());
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

TEST(ubiquity_motor_message, motor_message_deserialize_delimeter_in_data) {
	MotorMessage mc;

	//Test good message
	uint8_t arr[] = {0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x7E, 0xBC};

	std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(0, mc.deserialize(input));
	ASSERT_EQ(382, mc.getData());
	ASSERT_EQ(MotorMessage::TYPE_WRITE, mc.getType());
	ASSERT_EQ(MotorMessage::REG_LEFT_SPEED_SET, mc.getRegister());
}

// TEST(ubiquity_motor_message, motor_message_deserialize_double_delimeter) {
// 	MotorMessage mc;

// 	//Test bad delimeter with good checksum
// 	uint8_t arr[] = {0x7E, 0x7E, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x01, 0x2C, 0x0E};

// 	std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

// 	ASSERT_EQ(0, mc.deserialize(input));
// 	ASSERT_EQ(300, mc.getData());
// 	ASSERT_EQ(MotorMessage::TYPE_WRITE, mc.getType());
// 	ASSERT_EQ(MotorMessage::REG_LEFT_SPEED_SET, mc.getRegister());
// }


TEST(ubiquity_motor_message, motor_message_deserialize_bad_delimeter) {
	MotorMessage mc;

	//Test bad delimeter with good checksum
	uint8_t arr[] = {0x67, 0x02, 0xBB, 0x07, 0x00, 0x00, 0x00, 0x00, 0x3B};

	std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(1, mc.deserialize(input));
}

TEST(ubiquity_motor_message, motor_message_deserialize_bad_protocol) {
	MotorMessage mc;

	//Test bad protocol_verstion with good checksum
	uint8_t arr[] = {0x7E, 0x01, 0xBB, 0x07, 0x00, 0x00, 0x00, 0x00, 0x3C};

	std::vector<uint8_t> input(arr, arr + sizeof(arr)/ sizeof(uint8_t));

	ASSERT_EQ(2, mc.deserialize(input));
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

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}