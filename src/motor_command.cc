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

#include <ubiquity_motor/motor_command.h>
#include <cstring>
#include <math.h>

void MotorCommand::setType(MotorCommand::CommandTypes type){
  this->type = type;
}

MotorCommand::CommandTypes MotorCommand::getType(){
  return static_cast<MotorCommand::CommandTypes>(this->type);
}

void MotorCommand::setRegister(MotorCommand::Registers reg){
  this->register_addr = reg;
}

MotorCommand::Registers MotorCommand::getRegister(){
  return static_cast<MotorCommand::Registers>(this->register_addr);
}

void MotorCommand::setData(int32_t data){
  this->data[3] = (data >> 0) & 0xFF;
  this->data[2] = (data >> 8) & 0xFF;
  this->data[1] = (data >> 16) & 0xFF;
  this->data[0] = (data >> 24) & 0xFF;
}

int32_t MotorCommand::getData(){
  return (this->data[0] << 24)
               | (this->data[1] << 16)
               | (this->data[2] << 8)
               | (this->data[3] << 0);
}

std::vector<uint8_t> MotorCommand::serialize(){
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

int MotorCommand::deserialize(std::vector<uint8_t> &serialized){
  if(serialized[0] = delimeter) {
    if (serialized[1] = protocol_version)
    {
      if (generateChecksum(serialized) == serialized[8]){
        this->type = serialized[2];
        this->register_addr = serialized[3];
        this->data[0] = serialized[4];
        this->data[1] = serialized[5];
        this->data[2] = serialized[6];
        this->data[3] = serialized[7];
        return 0;
      }
      else
        return 1;
    }
    else return 1;
  }
  else
    return 1;
}

uint8_t MotorCommand::generateChecksum(std::vector<uint8_t> data) {
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