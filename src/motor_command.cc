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
#include "crc8.h"

void MotorCommand::setType(MotorCommand::CommandTypes type){
  this.type = type;
}

MotorCommand::CommandTypes MotorCommand::getType(){
  return this.type;
}

void MotorCommand::setRegister(MotorCommand::Registers reg){
  this.register_addr = reg;
}

MotorCommand::Registers MotorCommand::getRegister(){
  return this.register_addr;
}

void MotorCommand::setData(int32_t data){
  this.data[3] = (data >> 0) & 0xFF;
  this.data[2] = (data >> 8) & 0xFF;
  this.data[1] = (data >> 16) & 0xFF;
  this.data[0] = (data >> 24) & 0xFF;
}

int32_t MotorCommand::getData(){
  return (this.data[0] << 24)
               | (this.data[1] << 16)
               | (this.data[2] << 8)
               | (this.data[3] << 0);
}

std::vector<uint8_t> MotorCommand::serialize(){

}

int MotorCommand::deserialize(std::vector<uint8_t> &serialized){

}

uint8_t MotorCommand::generateChecksum(std::vector<uint8_t> data) {
  
}