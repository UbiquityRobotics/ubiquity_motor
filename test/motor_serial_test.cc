#include <ubiquity_motor/motor_message.h>
#include <ubiquity_motor/motor_serial.h>
#include <gtest/gtest.h>
#include <unistd.h>

// TEST(ubiquity_motor_serial, motor_serial) {
// 	MotorSerial motor_serial("/dev/null", 9600);
// 	//while (1){
// 		sleep(1);
// 		while(!motor_serial.commandAvailable()){
// 			usleep(100);
// 		}
// 		MotorMessage mc = motor_serial.receiveCommand();
// 		ASSERT_EQ(0x07, mc.getRegister());
// 		//break;
// 	//}
// }