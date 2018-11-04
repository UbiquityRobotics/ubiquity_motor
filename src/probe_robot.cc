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

#include <serial/serial.h>
#include <ubiquity_motor/motor_message.h>

struct Options {
    bool robot = false;
    bool firmware = false;
    bool pcb_rev = false;
    bool verbose = false;
    bool help = false;
    std::string serial_port = "";
    int baud_rate = 0;
};

bool find_switch(const std::vector<std::string> &args, const std::string &short_sw,
                 const std::string &long_sw) {
    bool has_short = (std::find(args.cbegin(), args.cend(), short_sw)) != args.cend();
    bool has_long = (std::find(args.cbegin(), args.cend(), long_sw)) != args.cend();

    return (has_long || has_short);
}

std::string get_option(const std::vector<std::string> &args, const std::string &option,
                       const std::string &default_val) {
    auto opt = std::find(args.cbegin(), args.cend(), option);
    if (opt != args.cend() && std::next(opt) != args.cend()) {
        return *(++opt);
    } else {
        return default_val;
    }
}

int get_option(const std::vector<std::string> &args, const std::string &option,
                       const int default_val) {
    auto opt = std::find(args.cbegin(), args.cend(), option);
    if (opt != args.cend() && std::next(opt) != args.cend()) {
        return std::stoi(*(++opt));
    } else {
        return default_val;
    }
}

Options parse_args(const std::vector<std::string> &args) {
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

    try {
        op.baud_rate = get_option(args, "-B", 38400);
    } catch (std::invalid_argument e) {
        fprintf(stderr, "%s\n", "Expected integer for option -B");
        throw e;
    }

    return op;
}

class TimeoutException : public std::exception {
    // Disable copy constructors
    TimeoutException &operator=(const TimeoutException &);
    std::string e_what_;

public:
    TimeoutException(const char *description) { e_what_ = description; }
    TimeoutException(const TimeoutException &other) : e_what_(other.e_what_) {}
    virtual ~TimeoutException() throw() {}
    virtual const char *what() const throw() { return e_what_.c_str(); }
};

MotorMessage readRegister(MotorMessage::Registers reg, serial::Serial &robot) {
    MotorMessage req;
    req.setRegister(reg);
    req.setType(MotorMessage::TYPE_READ);
    req.setData(0);

    RawMotorMessage out = req.serialize();
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

            robot.waitByteTimes(innew.size() - 1);
            robot.read(&innew.c_array()[1], 7);

            MotorMessage rsp;
            if (!rsp.deserialize(innew)) {
                if (rsp.getType() == MotorMessage::TYPE_RESPONSE && rsp.getRegister() == reg) {
                    return rsp;
                }
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &current_time);
        timedout = (current_time.tv_sec >= start.tv_sec + 5);
    }

    if (timedout) {
        throw TimeoutException("Timed Out Waiting for Serial Response");
    }
    else {
        throw std::runtime_error("unknown error while wating for serial");
    }
}

int main(int argc, char const *argv[]) {
    std::vector<std::string> args(argv + 1, argv + argc);

    int ret_code = 0;
    Options op;

    try {
        op = parse_args(args);
    }
    catch (...) {
        fprintf(stderr, "%s\n", "Error parsing arguments");
        return 2;
    }

    if (op.help) {
        printf("%s\n", "Used to probe Ubiquity Robotics robots for version info");
        printf("%s\n", "Options:");
        printf("%s\n", "  -D {PATH TO PORT}: Set the Serial Port to use (e.g. /dev/ttyAMA0)");
        printf("%s\n", "  -B {BAUD RATE}: Set the Baud Rate to use (e.g. 38400)");
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
        fprintf(stderr, "  Baud Rate: %d\n", op.baud_rate);
    }

    try {
        serial::Serial robot(op.serial_port, op.baud_rate, serial::Timeout::simpleTimeout(100));

        if (op.robot) {
            try {
                MotorMessage robot_id = readRegister(MotorMessage::REG_ROBOT_ID, robot);
                int id_num = robot_id.getData();
                if (id_num == 0x00) {
                    printf("%s\n", "magni");
                } else if (id_num == 0x01) {
                    printf("%s\n", "loki");
                } else {
                    printf("%s\n", "unknown");
                }
            } catch (TimeoutException e) {
                printf("%s\n", "none");
                ret_code = 2;
            }
        }
        if (op.firmware) {
            try {
                MotorMessage firmware = readRegister(MotorMessage::REG_FIRMWARE_VERSION, robot);
                printf("%d\n", firmware.getData());
            } catch (TimeoutException e) {
                fprintf(stderr, "%s\n", "Timeout getting firmware version");
                ret_code = 2;
            }
        }
        if (op.pcb_rev) {
            try {
                MotorMessage hardware = readRegister(MotorMessage::REG_HARDWARE_VERSION, robot);
                printf("%d\n", hardware.getData());
            } catch (TimeoutException e) {
                fprintf(stderr, "%s\n", "Timeout getting hardware version");
                ret_code = 2;
            }
        }
    } catch (const serial::IOException &e) {
        if (op.verbose) {
            fprintf(stderr, "Error opening Serial Port %s\n", op.serial_port.c_str());
        }
        return 1;
    }

    return ret_code;
}