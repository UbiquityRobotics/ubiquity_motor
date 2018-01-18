/**
Copyright (c) 2018, Ubiquity Robotics
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
#include <serial/serial.h>

struct Options
{
	bool robot = false;
	bool firmware = false;
	bool pcb_rev = false;
	bool verbose = false;
    bool help = false;
    std::string serial_port = "";
};

bool find_switch(const std::vector<std::string>& args, 
    const std::string& short_sw, const std::string& long_sw) {

    bool has_short = (std::find(args.cbegin(), args.cend(), short_sw)) != args.cend();
    bool has_long = (std::find(args.cbegin(), args.cend(), long_sw)) != args.cend();

    return (has_long || has_short);
}

std::string get_option(const std::vector<std::string>& args, 
    const std::string& option, const std::string& default_val) {

    auto opt = std::find(args.cbegin(), args.cend(), option);

    if (opt != args.cend() && opt++ != args.cend()) {
        return *(opt++);
    }
    else {
        return default_val;
    }
}

Options parse_args(const std::vector<std::string>& args) {
    Options op;

    op.robot = find_switch(args, "-r", "--robot");
    op.firmware = find_switch(args, "-f", "--firmware");
    op.pcb_rev = find_switch(args, "-p", "--pcb");
    op.verbose = find_switch(args, "-v", "--verbose");
    op.help = find_switch(args, "-h", "--help");

    if (find_switch(args, "-a", "--all")) {
        op.robot = true;
        op.firmware = true;
        op.pcb_rev = true;
    }

    op.serial_port = get_option(args, "-D", "/dev/ttyAMA0");

    return op;
}

int main(int argc, char const *argv[])
{
	std::vector<std::string> args(argv + 1, argv + argc);

    Options op = parse_args(args);

    if (op.help) {
        printf("%s\n", "Used to probe Ubiquity Robotics robots for version info");
        printf("%s\n", "Options:");
        printf("%s\n", "  -D {PATH TO PORT}: Set the Serial Port to use (e.g. /dev/ttyAMA0)");
        printf("%s\n", "Switches:");
        printf("%s\n", "  -a --all: Print out all options.");
        printf("%s\n", "  -r --robot: Print out robot platform (e.g. loki, magni, none)");
        printf("%s\n", "  -f --firmware: Print out firmware revision");
        printf("%s\n", "  -p --pcb: Print out the PCB revision");
        printf("%s\n", "  -v --verbose: Print out debugging information.");
        printf("%s\n", "  -h --help: Print out this help");

        return 0;
    }

    if (op.verbose) {
        fprintf(stderr, "%s\n", "Parsed Args:");
        fprintf(stderr, "  Robot ID: %d\n", op.robot);
        fprintf(stderr, "  Firmware: %d\n", op.firmware);
        fprintf(stderr, "  PCB: %d\n", op.pcb_rev);
        fprintf(stderr, "  Verbose: %d\n", op.verbose);
        fprintf(stderr, "  Help: %d\n", op.help);
        fprintf(stderr, "  Serial Port: %s\n", op.serial_port.c_str());
    }

    try {
        serial::Serial robot(op.serial_port, 115200, serial::Timeout::simpleTimeout(100));

        if (op.robot) {
            MotorMessage robot_id;
            robot_id.setRegister(MotorMessage::REG_ROBOT_ID);
            robot_id.setType(MotorMessage::TYPE_READ);
            robot_id.setData(0);

            RawMotorMessage out = robot_id.serialize();
            robot.write(out.c_array(), out.size());

            robot.flush();

            bool timedout = false;
            struct timespec start, current_time;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            start = current_time;

            while (robot.isOpen() && !timedout) {
                if (robot.waitReadable()) {
                    RawMotorMessage innew = {0, 0, 0, 0, 0, 0, 0, 0};

                    robot.read(innew.c_array(), 1);
                    if (innew[0] != MotorMessage::delimeter) continue;

                    robot.waitByteTimes(innew.size()-1);
                    robot.read(&innew.c_array()[1], 7);

                    MotorMessage rsp;
                    rsp.deserialize(innew);

                    if (rsp.getData() == 0x00) {
                        printf("%s\n", "magni");
                    }
                    if (rsp.getData() == 0x01) {
                        printf("%s\n", "loki");
                    }

                    break;
                }

                clock_gettime(CLOCK_MONOTONIC, &current_time);
                timedout = (current_time.tv_sec >= start.tv_sec + 5);
            }

            if (timedout) {
                if (op.verbose) {fprintf(stderr, "%s\n", "Timed out waiting for response");}
                printf("%s\n", "none");
                return 2;
            }
        }
    }
    catch (const serial::IOException& e) {
        if (op.verbose) {fprintf(stderr, "Error opening Serial Port %s\n", op.serial_port.c_str());}
        return 1;
    }
	
	return 0;
}