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
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <ubiquity_motor/motor_message.h>
// #include <ubiquity_motor/motor_message_registers.h>
#include <ros/console.h>

uint8_t const MotorMessage::valid_types[] = {
  TYPE_READ,
  TYPE_WRITE,
  TYPE_RESPONSE
};

uint8_t const MotorMessage::valid_registers[] = {
  REG_STOP_START,
  REG_BRAKE_STOP,
  REG_CRUISE_STOP,
  REG_LEFT_PWM,
  REG_RIGHT_PWM,
  REG_LEFT_SPEED_SET,
  REG_RIGHT_SPEED_SET,
  REG_LEFT_RAMP,
  REG_RIGHT_RAMP,
  REG_LEFT_ODOM,
  REG_RIGHT_ODOM,
  REG_DEADMAN,
  REG_LEFT_CURRENT,
  REG_RIGHT_CURRENT,
  REG_ERROR_COUNT,
  REG_5V_MAIN_ERROR,
  REG_5V_AUX_ERROR,
  REG_12V_MAIN_ERROR,
  REG_12V_AUX_ERROR,
  REG_5V_MAIN_OL,
  REG_5V_AUX_OL,
  REG_12V_MAIN_OL,
  REG_12V_AUX_OL,
  REG_LEFT_MOTOR_ERROR,
  REG_RIGHT_MOTOR_ERROR,
  REG_PARAM_P,
  REG_PARAM_I,
  REG_PARAM_D,
  REG_PARAM_C,
  REG_LED_1,
  REG_LED_2,
  REG_HARDWARE_VERSION,
  REG_FIRMWARE_VERSION,
  REG_BATTERY_VOLTAGE,
  REG_5V_MAIN_CURRENT,
  REG_12V_MAIN_CURRENT,
  REG_5V_AUX_CURRENT,
  REG_12V_AUX_CURRENT,
  REG_LEFT_SPEED_MEASURED,
  REG_RIGHT_SPEED_MEASURED
};

void MotorMessage::setType(MotorMessage::MessageTypes type){
  if (verifyType(type)){
    this->type = type;
  }
}

MotorMessage::MessageTypes MotorMessage::getType(){
  if (verifyType(this->type)){
    return static_cast<MotorMessage::MessageTypes>(this->type);
  }
}

void MotorMessage::setRegister(MotorMessage::Registers reg){
  if (verifyRegister(reg)){
    this->register_addr = reg;
  }
}

MotorMessage::Registers MotorMessage::getRegister(){
  return static_cast<MotorMessage::Registers>(this->register_addr);
}

/*bool MotorMessage::isDelimeter(uint8_t data) {
  return (delimeter == data);
  }*/

void MotorMessage::setData(int32_t data){
  // Spilt 32 bit data (system byte order) into 4 8bit elements in big endian (network byte order)
  this->data[3] = (data >> 0) & 0xFF;
  this->data[2] = (data >> 8) & 0xFF;
  this->data[1] = (data >> 16) & 0xFF;
  this->data[0] = (data >> 24) & 0xFF;
}

int32_t MotorMessage::getData(){
  // Take big endian (network byte order) elements and return 32 bit int
  return (this->data[0] << 24)
               | (this->data[1] << 16)
               | (this->data[2] << 8)
               | (this->data[3] << 0);
}

std::vector<uint8_t> MotorMessage::serialize(){
  std::vector<uint8_t> out(9);
  out[0] = delimeter;
  out[1] = protocol_version;
  out[2] = type;
  out[3] = register_addr;
  out[4] = data[0];
  out[5] = data[1];
  out[6] = data[2];
  out[7] = data[3];
  out[8] = generateChecksum(out);
  return out;
}

int MotorMessage::deserialize(std::vector<uint8_t> &serialized){
  if(serialized[0] == delimeter) {
    if (serialized[1] == protocol_version) {
      if (generateChecksum(serialized) == serialized[8]) {
        if (verifyType(serialized[2])) {
          if (verifyRegister(serialized[3])) {
            this->type = serialized[2];
            this->register_addr = serialized[3];
            this->data[0] = serialized[4];
            this->data[1] = serialized[5];
            this->data[2] = serialized[6];
            this->data[3] = serialized[7];
            return 0;
          }
          else 
            return 5;
        }
        else 
          return 4;
      }
      else
        return 3;
    }
    else return 2;
  }
  else
    return 1;

  // TODO use exceptions instead of cryptic error codes

  // ERROR codes returned:
  // 1 First char not delimiter
  // 2 wrong protocol version
  // 3 bad checksum
  // 4 bad type
  // 5 bad register
} 

int MotorMessage::verifyType(uint8_t t){
  //Return 1 good
  //Return 0 for bad
  for (int i = 0; i < sizeof(valid_types) / sizeof(valid_types[0]); ++i)
  {
    if (t == valid_types[i])
      return 1;
  }
  return 0;
}

int MotorMessage::verifyRegister(uint8_t r){
  //Return 1 good
  //Return 0 for bad
  for (int i = 0; i < sizeof(valid_registers) / sizeof(valid_registers[0]); ++i)
  {
    if (r == valid_registers[i])
      return 1;
  }
  return 0;
}

uint8_t MotorMessage::generateChecksum(std::vector<uint8_t> data) {
  int sum = data [1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7];

  if (sum > 0xFF) {
    int tmp;
    tmp = sum >> 8;
    tmp = tmp << 8;
    return 0xFF - (sum-tmp);
  }
  else {
    return 0xFF - sum;
  }

}
