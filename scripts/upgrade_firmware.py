#! /usr/bin/python
import requests

import time
import serial
import os, sys, subprocess

import argparse

print """-------------------------------------------------------------
Welcome to the Ubiquity Robotics Firmware Updater!

Please make sure that you are not running any ROS nodes.
- sudo systemctl stop magni-base

Note: Updating the firmware requires access to the internet.
-------------------------------------------------------------
"""

TMP_FILE_PATH = '/tmp/firmware'


parser = argparse.ArgumentParser(description='Ubiquity Robotics Firmware Updater')
parser.add_argument('--device', help='Serial port to use (eg /dev/ttyAMA0)', default='/dev/ttyAMA0')
parser.add_argument('--file', help='Path to a firmware file', default='')
args = parser.parse_args()

serial_port = args.device

if (subprocess.call(['fuser','-v', serial_port], stdout=None) == 0):
    print ""
    print "Another process is using the serial port, cannot upgrade firmware"
    print "Make sure you stopped any running ROS nodes. To stop default nodes:"
    print "sudo systemctl stop magni-base"
    sys.exit(1)

if args.file:
    path_to_file = args.file

else:
    path_to_file = TMP_FILE_PATH

    email = raw_input("Please enter your email address: ").strip()

    if email != "":
        r = requests.post('https://api.ubiquityrobotics.com/token/', json = {'email': email}) 

        if r.status_code == 500:
            print "Error: 500 Internal Server Error. Something went wrong, try again in a few minutes. If the error persists, contact support."
        elif r.status_code != 200:
                print "Error with requesting a token %d" % r.status_code
                sys.exit(1)

    print "An access token was sent to your email address"

    token = raw_input("Please enter your access token: ").strip()

    version = raw_input("What version would you like (press enter for latest): ").strip()

    if version == "":
        version = "latest"

    auth_headers = {"Authorization": "Bearer %s" % token}
    r = requests.get('https://api.ubiquityrobotics.com/firmware/%s' % version, headers=auth_headers)

    if r.status_code == 401:
        print "Error: 401 Unauthorized. Please make sure that your token is valid and entered correctly."
        sys.exit(1)
    elif r.status_code == 404:
        print "Error: 404 Not Found. The firmware version that you requested was not found."
    elif r.status_code == 500:
        print "Error: 500 Internal Server Error. Something went wrong, try again in a few minutes. If the error persists, contact support."
    elif r.status_code != 200:
        print "Error downloading firmware %d" % r.status_code
        sys.exit(1)

    with open(path_to_file, 'w+b') as fd:
        for chunk in r.iter_content(chunk_size=128):
            fd.write(chunk)

print "\nUpdating firmware now. Do not power off the robot. This is expected to take less than a minute."


# Begin the code firmware uploading code
DEBUG = False
class InvalidFileException(Exception):
    def __init__(self):
        pass

class ReadFile:
    def __init__(self, content, encrypted):
        self.content = content
        self.encrypted = encrypted
        self.size = len(content)
    def is_encrypted(self):
        return self.encrypted
    def read(self, bc):
        if type(bc) != int or bc < 1:
            raise Exception("Invalid read: " + str(bc))
        r = self.content[0:bc]
        self.content = self.content[bc:]
        return r
    def is_open(self):
        return len(self.content) > 0
    def get_size(self):
        return self.size
    def get_position(self):
        return self.size - len(self.content)
        

def load_hex(filename):
    f = open(filename, "r")

    fcontent = ""
    fout = ""
    encrypted = False
    not_encrypted = False
    for l in f:
        for c in l:
            if c == ":":
                not_encrypted = True
            if c == "+":
                encrypted = True
            if c == ":" and encrypted:
                raise Exception("Partial encryption")
            if c == "+" and not_encrypted:
                raise Exception("Partial encryption")
            if c not in ['\n', '\r', '\t', ' ', ':', "+"]:
                fcontent = fcontent + c
            while len(fcontent) >= 2:
                fout += chr(int(fcontent[0:2], 16))
                fcontent = fcontent[2:]
    assert not_encrypted or encrypted
    return ReadFile(fout, encrypted)

def convert_num(arr):
    out = 0
    scale = 1
    for b in arr:
        out = out + ord(b) * scale
        scale = scale * 256
    return out
def file_convert_num(arr):
    arrc = [x for x in arr]
    arrc.reverse()
    return convert_num(arrc)

def read__header(f):
    asilicon_id = [x for x in f.read(4)]
    asilicon_id.reverse()
    silicon_id = ""
    for a in asilicon_id:
        silicon_id += a
    
    silicon_rev = f.read(1)
    checksum_type = f.read(1)
    return silicon_id, silicon_rev, checksum_type
def read__flash_line(f):
    flash_id = file_convert_num(f.read(1))
    row_number = file_convert_num(f.read(2))
    data_length = file_convert_num(f.read(2))
    data = f.read(data_length)
    checksum = file_convert_num(f.read(1))
    return flash_id, row_number, data_length, data, checksum

def read__encrypted_flash_line(f):
    flash_id = file_convert_num(f.read(1))
    row_number = file_convert_num(f.read(2))
    data_length = file_convert_num(f.read(2))
    data = f.read(data_length + 12)
    return flash_id, row_number, data_length, data

def cstr(x):
    if type(x) == str:
        return str(x) + " <- " + str([hex(ord(l)) for l in x])
    else:
        return str(type(x)) + " -> " + str(x)

class Packet:
    def __init__(self, ser, cmd):
        self.ser = ser
        self.cmd = chr(cmd)
        self.out = []
        self.is_sent = False
        self.id_ptr = 0
        self.inp = []
        self.ignore_response = False

    def do_ignore_response(self): #For exit packet
        self.ignore_response = True

    def require_len(self, mylen):
        if type(mylen) != int or mylen < 0:
            raise Exception("Invalid length goal for packet: " + cstr(mylen))
        if len(self.inp) != mylen:
            raise Exception("Invalid length for packet: " + len(self.inp) + " should be " + str(inp))

    def write(self, x, mylen = None):
        if mylen != None:
            num = long(x)
            numa = []
            for step in xrange(0, mylen):
                numa.append(chr(num % 256))
                num = num / 256
            self.out.extend(numa)
            return
        if self.is_sent:
            raise Exception("Write to packet after send")
        if type(x) == list:
            self.out.extend(x)
        elif type(x) == str:
            self.out.extend([i for i in x])
        else:
            self.out.append(chr(x))

    def read(self, bytes):
        if not self.is_sent:
            raise Exception("Read from packet before send")
        if type(bytes) != int or bytes < 1:
            raise Exception("Invalid byte read: " + cstr(bytes))
        r = self.inp[self.id_ptr:self.id_ptr+bytes]
        self.id_ptr += bytes
        return r

    def read_num(self, bytes):
        return convert_num(self.read(bytes))

    def send(self):
        self.is_sent = True
        init_bytes = [chr(0x1), self.cmd, 
                      chr((len(self.out) / 001) % 256),
                      chr((len(self.out) / 256) % 256)]
        init_bytes.extend(self.out)

        if DEBUG: print "-"*120
        if DEBUG: print "Send packet: " + cstr(self.out)

        ib_buf = ""
        for c in init_bytes: ib_buf += c
        if DEBUG: print "Packet without end and checksum is", cstr(ib_buf)

        checksum = compute_checksum(ib_buf)
        if DEBUG: print "Checksum is", checksum, "LSB", (checksum / 001) % 256, "MSB", (checksum / 256) % 256

        ib_buf += chr((checksum / 001) % 256)
        ib_buf += chr((checksum / 256) % 256)
        ib_buf += chr(0x17)

        self.ser.write(ib_buf)
        self.ser.flush()

        if self.ignore_response:
            return

        #Now we should read back.
        header = self.ser.read(4)
        start_byte = header[0:1]
        if start_byte != str(chr(0x01)):
            raise Exception("Packet did not start with valid start byte, instead: " + cstr(start_byte) + " -- header: " + cstr(header))
        status_code = header[1:2]
        if status_code != str(chr(0x00)):
            raise Exception("Packet did not start with valid status, instead: " + cstr(status_code) + " -- header: " + cstr(header))
        data_length_raw = header[2:4]
        data_length = (ord(data_length_raw[0])) + (ord(data_length_raw[1])) * 256
        if DEBUG: print "Read in", data_length, "from", cstr(data_length_raw)
        self.inp = self.ser.read(data_length)
        if len(self.inp) != data_length:
            raise Exception("We tried to read: " + str(data_length) + " but instead read: " + str(len(self.inp)) + " -- header: " + cstr(header))
        
        #Append many arrays:
        checksum_bytes = start_byte + status_code + data_length_raw + self.inp
        checksum_comp = compute_checksum(checksum_bytes)
        
        checksum_raw = self.ser.read(2)
        checksum = (ord(checksum_raw[0])) + (ord(checksum_raw[1]) * 256)
        if checksum != checksum_comp:
            raise Exception("Invalid checksum for packet: " + cstr(checksum) + " != " + cstr(checksum_comp) + " -- header: " + cstr(header))
        
        end_byte = self.ser.read(1)
        if end_byte != str(chr(0x17)):
            raise Exception("Invalid terminator for packet: " + cstr(end_byte) + " -- header: " + cstr(header))

def compute_checksum(bytes):
    return twos_complement(bytes)

def twos_complement(bytes):
    x = 0
    for b in bytes:
        x += int(ord(b))
        x = x & 0xFFFF
    x = x ^ 0xFFFF
    x = x + 1
    return x

#CRC16-CCITT    
def crc16_ccitt(bytes):
    crc = 0xFFFF
    x = 0
    def ushort(x):
        return x & 0xFFFF
    for byte in bytes:
        x = (crc >> 8) ^ (int(ord(byte)) & 0xFF)
        x ^= (x >> 4)
        crc = ushort(ushort(crc << 8) ^ ushort(x << 12) ^ ushort(x << 5) ^ ushort(x))
    return crc

def send__erase_row(ser, flash_id, row_number):
    p = Packet(ser, 0x34)
    p.write(flash_id, 1)
    p.write(row_number, 2)
    p.send()
    p.require_len(0)
    return None

def send__enter_bootloader(ser):
    p = Packet(ser, 0x38)
    p.send()
    p.require_len(8)
    silicon_id = p.read(4)
    silicon_rev = p.read(1)
    bootloader_version = p.read(3)
    return silicon_id, silicon_rev, bootloader_version

def send__get_flash_size(ser, flash_id):
    p = Packet(ser, 0x32)
    p.write(flash_id, 1)
    p.send()
    p.require_len(4)
    flash_first_row = p.read_num(2)
    flash_last_row = p.read_num(2)
    return flash_first_row, flash_last_row

def send__program_row(ser, flash_id, row_number, data_next):
    p = Packet(ser, 0x39)
    p.write(flash_id, 1)
    p.write(row_number, 2)
    p.write(data_next)
    p.send()
    p.require_len(0)
    return None

def send__encrypted_program_row(ser, flash_id, row_number, data_next):
    p = Packet(ser, 0x3D)
    p.write(flash_id, 1)
    p.write(row_number, 2)
    p.write(data_next)
    p.send()
    p.require_len(0)
    return None

def send__verify_row(ser, flash_id, row_number):
    p = Packet(ser, 0x3A)
    p.write(flash_id, 1)
    p.write(row_number, 2)
    p.send()
    p.require_len(1)
    checksum = p.read_num(1)
    return checksum

def send__data(ser, data_next):
    p = Packet(ser, 0x37)
    p.write(data_next)
    p.send()
    p.require_len(0)
    return None

def send__verify_checksum(ser):
    p = Packet(ser, 0x31)
    p.send()
    p.require_len(1)
    r = p.read_num(1)
    return r

def send__get_application_status(ser, app_number):
    p = Packet(ser, 0x33)
    p.write(app_number, 1)
    p.send()
    p.require_len(2)
    valid_app_number = p.read_num(1)
    active_app_number = p.read_num(1)
    return valid_app_number, active_app_number

def send__set_active_application(ser, app_number):
    p = Packet(ser, 0x36)
    p.write(app_number, 1)
    p.send()
    p.require_len(0)
    return

def send__sync_bootloader(ser):
    p = Packet(ser, 0x35)
    p.send()
    p.require_len(0)
    return

def send__exit_bootloader(ser):
    p = Packet(ser, 0x3B)
    p.do_ignore_response()
    p.send()
    p.require_len(0)
    return

def add8(a, b):
    return ((a & 0xFF) + (b & 0xFF)) & 0xFF
    
def convert_checksum(checksum, flash_id, row_number, row_size):
    r = add8(checksum, flash_id)
    r = add8(r, row_number)
    r = add8(r, row_number >> 8)
    r = add8(r, row_size)
    r = add8(r, row_size >> 8)
    if DEBUG: print "Checksum convert:", checksum, flash_id, row_number, row_size, "out:", r
    return r

hex_stream = None
try:
    hex_stream = load_hex(path_to_file)
except IOError:
    print "Unable to open file: ", path_to_file
except InvalidFileException:
    print "File is not of the correct format"
print("Encryption:", hex_stream.is_encrypted())

ser = serial.Serial(serial_port, 38400, timeout=1, bytesize=8,
                    parity=serial.PARITY_NONE, stopbits=1, xonxoff=0, rtscts=0)

# Write to request the bootloader to the correct state.
request_bootloader = [0x7E, 0x3E, 0x01, 0x01, 0x01, 0x01, 0x01]
request_bootloader.append(0xFF - sum(request_bootloader[1:]))
ser.write("".join([chr(x) for x in request_bootloader]))
ser.flush()
ser.read(65536) # Clear out the response

silicon_id, silicon_rev, bootloader_version = send__enter_bootloader(ser)
file_silicon_id, file_silicon_rev, file_checksum_type = read__header(hex_stream)
if file_silicon_id != silicon_id:
    raise Exception("The silicon ids did not match " + cstr(file_silicon_id) + " != " + cstr(silicon_id))
if file_silicon_rev != silicon_rev:
    raise Exception("The silicon revs did not match")

if file_checksum_type != str(chr(0)):
    raise Exception("Invalid checksum type: " + cstr(file_checksum_type))

last_percent = 0.0

#send__sync_bootloader(ser)

while hex_stream.is_open():
    if not hex_stream.is_encrypted():
        flash_id, row_number, data_length, data, checksum = read__flash_line(hex_stream)
        print "Writing row", row_number, "for", flash_id, "process at", ("%.2f" % last_percent), "percent completion"
        last_percent = (100.0 * hex_stream.get_position()) / hex_stream.get_size()

        real_checksum = convert_checksum(checksum, flash_id, row_number, len(data))
        if DEBUG: print "Write", flash_id, row_number, data_length, len(data), checksum, "->", real_checksum

        flash_first_row, flash_last_row = send__get_flash_size(ser, flash_id)
        if DEBUG: print "Flash: ", flash_first_row, flash_last_row
        if row_number < flash_first_row or row_number > flash_last_row:
            raise Exception("Invalid row: must be between " + str(flash_first_row)
                            + " and " + str(flash_last_row) + " but is " + str(row_number))

        send__erase_row(ser, flash_id, row_number)

        data_strip = data
        data_sent = 0
        while data_strip != [] and data_strip != "":
            bytes_at_a_time = 256
            data_next = data_strip[0:bytes_at_a_time]
            data_strip = data_strip[bytes_at_a_time:]
            data_sent += bytes_at_a_time
            if DEBUG: print "Sending", len(data_next), "Sent: ", data_sent, "Left: ", len(data_strip)
            if data_strip == [] or data_strip == "":
                #Program write
                if DEBUG: print "Last packet"
                send__program_row(ser, flash_id, row_number, data_next)
                comp_checksum = send__verify_row(ser, flash_id, row_number)
                if real_checksum != comp_checksum:
                    raise Exception("Checksum error at " + flash_id + " " + row_number 
                                    + ". We wanted " + cstr(real_checksum) + " but we got " + cstr(comp_checksum))
                else:
                    if DEBUG: print "Checksum valid", flash_id, row_number, "->", real_checksum, "==", comp_checksum
            else:
                send__data(ser, data_next)
    else:
        flash_id, row_number, data_length, data = read__encrypted_flash_line(hex_stream)
        print "Writing row", row_number, "for", flash_id, "process at", ("%.2f" % last_percent), "percent completion"
        last_percent = (100.0 * hex_stream.get_position()) / hex_stream.get_size()
        assert len(data) < 256
        send__encrypted_program_row(ser, flash_id, row_number, data)

if send__verify_checksum(ser) == 0:
    raise Exception("Application checksum invalid")

try:
    hex_stream.close()
except:
    pass

send__exit_bootloader(ser)

ser.close()

print "Finished upgrading firmware!"
