/**
Copyright (c) 2016, Ubiquity Robotics
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

#include <ros/ros.h>
#include <serial/serial.h>
#include <ubiquity_motor/motor_serial.h>

MotorSerial::MotorSerial(const std::string& port, uint32_t baud_rate)
    : motors(port, baud_rate, serial::Timeout::simpleTimeout(100)), 
      serial_errors(0), error_threshold(20) {
    serial_thread = boost::thread(&MotorSerial::SerialThread, this);
}

MotorSerial::~MotorSerial() {
    serial_thread.interrupt();
    serial_thread.join();
    motors.close();
}

int MotorSerial::transmitCommand(MotorMessage command) {
    RawMotorMessage out = command.serialize();
    ROS_DEBUG("out %02x %02x %02x %02x %02x %02x %02x %02x", out[0], out[1],
              out[2], out[3], out[4], out[5], out[6], out[7]);
    motors.write(out.c_array(), out.size());
    return 0;
}

int MotorSerial::transmitCommands(const std::vector<MotorMessage>& commands) {
    for (auto& command : commands) {
        RawMotorMessage out = command.serialize();
        ROS_DEBUG("out %02x %02x %02x %02x %02x %02x %02x %02x", out[0], out[1],
                  out[2], out[3], out[4], out[5], out[6], out[7]);
        motors.write(out.c_array(), out.size());
        boost::this_thread::sleep(boost::posix_time::milliseconds(2));
    }
    return 0;
}

MotorMessage MotorSerial::receiveCommand() {
    MotorMessage mc;
    if (!this->output.empty()) {
        mc = output.front_pop();
    }
    return mc;
}

int MotorSerial::commandAvailable() { return !output.fast_empty(); }

void MotorSerial::appendOutput(MotorMessage command) { output.push(command); }

void MotorSerial::closePort() { return motors.close(); }

// After we have been offine this is called to re-open serial port
// This returns true if port was open or if port opened with success
bool MotorSerial::openPort()  { 
    bool retCode = true;

    if (motors.isOpen() == true) {
        return true; 
    }

    //  Port was closed so must open it using info from prior open()
    try {
        motors.open();
    } catch (const serial::IOException& e) {
        ROS_ERROR("%s", e.what());
        retCode = false;
    } catch (const std::invalid_argument) {
        ROS_ERROR("MotorSerial::openPort Invalid argument");
        retCode = false;
    } catch (const serial::SerialException& e) {
        ROS_ERROR("%s", e.what());
        retCode = false;
    } catch (...) {
        ROS_ERROR("Unknown Error");
        retCode = false;
    }

    return retCode;
}

void MotorSerial::SerialThread() {
    try {
        while (motors.isOpen()) {
            boost::this_thread::interruption_point();
            if (motors.waitReadable()) {
                RawMotorMessage innew = {0, 0, 0, 0, 0, 0, 0, 0};

                motors.read(innew.c_array(), 1);
                if (innew[0] != MotorMessage::delimeter) {
                    // The first byte was not the delimiter, so re-loop
                    if (++serial_errors > error_threshold) {
                        ROS_WARN("REJECT %02x", innew[0]);
                    }
                    continue;
                }

                // This will wait for the transmission time of 8 bytes
                motors.waitByteTimes(innew.size());

                // Read in next 7 bytes
                motors.read(&innew.c_array()[1], 7);
                ROS_DEBUG("Got message %x %x %x %x %x %x %x %x", innew[0],
                          innew[1], innew[2], innew[3], innew[4], innew[5],
                          innew[6], innew[7]);

                MotorMessage mc;
                int error_code = mc.deserialize(innew);
                if (error_code == 0) {
                    appendOutput(mc);
                    if (mc.getType() == MotorMessage::TYPE_ERROR) {
                        ROS_ERROR("GOT error from Firm 0x%02x",
                                  mc.getRegister());
                    }
                } else {
                    if (++serial_errors > error_threshold) {
                        if (error_code == MotorMessage::ERR_UNKNOWN_REGISTER) {
                            ROS_WARN_ONCE("Message deserialize found an unrecognized firmware register");
                        } else {
                            ROS_ERROR("DESERIALIZATION ERROR! - %d", error_code);
                        }
                    }
                }
            }
        }

    } catch (const boost::thread_interrupted& e) {
        motors.close();
    } catch (const serial::IOException& e) {
        ROS_ERROR("%s", e.what());
    } catch (const serial::PortNotOpenedException& e) {
        ROS_ERROR("%s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown Error");
        throw;
    }
}
