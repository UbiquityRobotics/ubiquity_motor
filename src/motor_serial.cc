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

MotorSerial::MotorSerial(const std::string& port, uint32_t baud_rate,
                         double loopRate)
    : motors(port, baud_rate, serial::Timeout::simpleTimeout(10000)),
      serial_loop_rate(loopRate) {
    serial_thread = boost::thread(&MotorSerial::SerialThread, this);
}

MotorSerial::~MotorSerial() {
    serial_thread.interrupt();
    serial_thread.join();
    motors.close();
}

int MotorSerial::transmitCommand(MotorMessage command) {
    input.push(command);  // add latest command to end of fifo
    return 0;
}

int MotorSerial::transmitCommands(const std::vector<MotorMessage>& commands) {
    input.push(commands);
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

int MotorSerial::inputAvailable() { return !input.fast_empty(); }

MotorMessage MotorSerial::getInputCommand() {
    MotorMessage mc;
    if (!this->input.empty()) {
        mc = input.front_pop();
    }

    return mc;
}

void MotorSerial::appendOutput(MotorMessage command) { output.push(command); }

void MotorSerial::SerialThread() {
    try {
        RawMotorMessage in;
        bool failed_update = false;

        while (motors.isOpen()) {
            while (motors.available() >= (failed_update ? 1 : 8)) {
                RawMotorMessage innew;
                motors.read(innew.c_array(), failed_update ? 1 : 8);

                // TODO use circular_buffer instead of manual shifting
                if (!failed_update) {
                    in.swap(innew);
                } else {
                    // Shift array contents up one
                    for (size_t i = 0; i < in.size() - 1; ++i) {
                        in[i] = in[i + 1];
                    }
                    in[in.size() - 2] = in[in.size() - 1];

                    in[in.size() - 1] = innew[0];
                }

                MotorMessage mc;
                int error_code = mc.deserialize(in);

                if (error_code == 0) {
                    if (mc.getType() == MotorMessage::TYPE_ERROR) {
                        ROS_ERROR(
                            "GOT ERROR RESPONSE FROM PSOC FOR REGISTER 0x%02x",
                            mc.getRegister());
                    }
                    appendOutput(mc);
                    failed_update = false;
                } else if (error_code == 1) {
                    failed_update = true;
                    char rejected[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
                    for (size_t i = 0; i < in.size() && i < 8; i++) {
                        rejected[i] = in.at(i);
                    }
                    ROS_ERROR("REJECT: %s", rejected);
                } else {
                    failed_update = true;
                    ROS_ERROR("DESERIALIZATION ERROR! - %d", error_code);
                }
            }

            bool did_update = false;
            while (inputAvailable()) {
                did_update = true;

                RawMotorMessage out = getInputCommand().serialize();
                ROS_DEBUG("out %02x %02x %02x %02x %02x %02x %02x %02x", out[0],
                          out[1], out[2], out[3], out[4], out[5], out[6],
                          out[7]);
                motors.write(out.c_array(), out.size());
                boost::this_thread::sleep(boost::posix_time::milliseconds(2));
            }

            if (did_update) {
                motors.flushOutput();
            }

            boost::this_thread::interruption_point();
            serial_loop_rate.sleep();
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
