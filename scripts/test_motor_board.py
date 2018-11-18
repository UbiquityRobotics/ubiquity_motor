#!/usr/bin/python -u

#
# Command control direct to Ubiquity Robotics Magni Controller Board
#
# This is diagnostic code that can be usedto directly run the motors
# without ROS and all the overhead of joysticks and so on.
# The idea started as a simple serial controller to do basic tests.
#
# Usage:
# Verify that serialDev is set for default Magni com port of /dev/ttyAMA0
# or some other serial port if you are using a non-standard configuration.
# Stop the ROS magni control using:  sudo systemctl stop    magni-base
# sudo python magni_cmd.py     # User needs permission for serial port
# Enter single keys followed by return to change speeds.   
# Example:  enter '1' for slow speed then enter 3 and speed goes to 3
# Change speeds like this and when done enter 'e' for exit or 's' for stop.
#
# The Magni uses a binary packet and this program uses only the speed command
# Packet Structure    0x7E CC RR AU AL BU BL SS   
#   Where 0x7E starts a packet and SS is cksum (CC -> B0) truncate 8 bit
#   CC - Command   MS Nibble is 3 for protocol version and LS Nibble:  
#                  Read=0xA, Write=0xB, Response=0xC, Error=0xD
#   RR - Register  We only use the speed control which is 0x2A
#   AUAL  SpeedA   signed 16-bit motor speed value for one motor
#   BUBL  SpeedB   signed 16-bit motor speed value for one motor
#
# Test Rig Control
# You can not plug in the Raspberry Pi into the motor controller and can
# power the Raspberry Pi with an external 5V micro USB power supply.
# Next connect the Pi Pins 8,10,12 from your Pi to the controller CAREFULLY!
# By doing this you can run the PI from telnet and not have to worry about
# a bad controller board or power loss on controller board from hard reboot of Pi
#   
# Developed by Mark Johnston, Aug 2018
#

from __future__ import print_function
import sys
import time
import serial
import string
import thread

# simple version string
g_version = "20180812"


# This debug flag if set True enables prints and so on but cannot be used in production
g_debug = False


# Simple wrappers to prevent log overhead and use desired log functions for your system
def logAlways(message):
    print(message)

def logDebug(message):
    global g_debug
    if g_debug == True:
        print(message)

# Define an input thread that will pick up keyboard presses
def keyboard_thread(intKeys):
    # logAlways("\nkeyboard input thread wait for key")
    inChar = raw_input()
    # logAlways("  keyboard input thread got a key")
    intKeys.append(inChar)

# Calculate a packet checksum for the given byte array
def calcPacketCksum(msgBytes):
    cksum = 0
    idx = 0
    for c in msgBytes:
        if idx > 0:
            cksum = cksum + int(c)
        idx = idx + 1
    cksum = 0xff - (cksum & 0xff)
    return cksum

# Form a bytearray holding a speed message with the left and right speed values
# This routine only handles -254 to 255 speeds but that is generally quite alright
def formMagniSpeedMessage(rightSpeed, leftSpeed):
    # Start with  a speed message for zero speed but without checksum
    speedCmd = [ 0x7e, 0x3b, 0x2a, 0, 0, 0, 0 ]
    # fill in speeds (just positive values so get this working!
    if rightSpeed < 0:
        speedCmd[3] = 0xff
        speedCmd[4] = 0xff - ((-1 * rightSpeed) & 0xff)
    else:
        speedCmd[4] = rightSpeed & 0xff

    if leftSpeed < 0:
        speedCmd[5] = 0xff
        speedCmd[6] = 0xff - ((-1 * leftSpeed) & 0xff)
    else:
        speedCmd[6] = leftSpeed & 0xff
    pktCksum = calcPacketCksum(speedCmd)
    speedCmd.append(pktCksum)
    return speedCmd

# Run at the given speed till a key is entered
# This routine does not work yet so the program does crude inline like this
def runTillKeypress(speed):
    print("runTillKeypress starting using speed ", int(speed))
    keyBuf  = []
    thread.start_new_thread(keyboard_thread, (keyBuf,))
    logAlways("runTillKeypress drop into loop")
    while not keyBuf:
        logAlways("runTillKeypress do the speed command")
        packet = formMagniSpeedMessage(speed,speed)
        print(packet)
        ser.write(packet)    # BUG!!!  In this context the ser.write blows up yet it works in main
        logAlways("runTillKeypress do the delay")
        time.sleep(0.02)
    logAlways("Finished with run command")
    cmdPacket = formMagniSpeedMessage(0, 0)
    ser.write(cmdPacket)
    return keyBuf

    
# Since stop is so common we have a function to send all stop to the motors
def stopMotors():
    cmdPacket = formMagniSpeedMessage(0, 0)
    ser.write(cmdPacket)
    return 0

class serCommander():
    def __init__(self):
        intKeys = []
        keyBuf  = []

        # Setup serial port for default Magni of ttyAMA0 or other serial port
        # serialDev = '/dev/ttyUSB0'
        serialDev = '/dev/ttyAMA0'
        
        # The bits we see here are placed in globals by notification callback

        nextInput=''
        lastSpeed=0
        lastNegativeSpeed=0
        cycleOnPeriod=50        # cycles of constant running or time motor is ON for cycle test
        cycleOffPeriod=50       # cycles of stopped running or time motor is OFF for cycle test

        # start thread to pick up keyboard presses
        intKeys = []
        nextCmd = ''

        # Magni Rasperry Pi serial port values
        # ser = serial.Serial('/dev/ttyS0', 38400, 8, 'N', 1, timeout=1)
        # Native Pi3B+ Ubiquity Robotics serial port Gnd=6 Tx=8  Rx=10 (+3.3V = 1)
        # ser = serial.Serial('/dev/ttyAMA0', 38400, 8, 'N', 1, timeout=1)  

        # First USB serial port (safer for reasons of static and power blowouts
        print("Start Serial port using device ", serialDev)
        ser = serial.Serial(serialDev, 38400, 8, 'N', 1, timeout=1)
        logAlways("Serial port started")

        # always set speed to zero at the start to initialize motor controller zero point
        logAlways("Set motors to run at 0.0 MPs")
        lastSpeed = 0
        cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
        ser.write(cmdPacket)
        time.sleep(0.05)
        logAlways("Finished with run command")

        try:
          while True :
            logAlways("Run Commands:    0 thru 9, s   Single number then enter for new speed each time")
            logAlways("Single Commands: q=quit e=exit c=crawl  i=forward  f=fast   These do just one cmd then stop")

            # clear out interrupt keys if any are present
            intKeys = []

            # get keyboard input
            if nextInput == '':
                input = raw_input(">> ")
            else:
                input = nextInput
                nextInput = ''

            # Python 3 users
            # input = input(">> ")

            if input == '0':
                logAlways("Run at 0.0 MPs till a key is pressed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 0
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '1':
                logAlways("Run at 0.1 MPs till a key is pressed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 5
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '2':
                logAlways("Run at 0.2 MPs till a key is pressed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 16
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '3':
                logAlways("Run at 0.3 MPs till a key is pressed")
                lastSpeed = 24 
                # DEBUG: We want this to work but it cannot do the send so ... debug later
                # intKeys = runTillKeypress(lastSpeed)
                # nextInput = intKeys[0]
                thread.start_new_thread(keyboard_thread, (intKeys,))
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '4':
                logAlways("Run at 0.4 MPs till a key is pressed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 32
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '5':
                logAlways("Run at 0.5 MPs till a key is pressed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 40
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.03)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '6':
                logAlways("Run at 0.65 MPS or 1 rev per second")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 51
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.03)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '7':
                logAlways("Run at 0.5 MPs till a key is pressed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 56
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.03)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '8':
                logAlways("Run at 0.5 MPs till a key is pressed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 64
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.04)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == '9':
                logAlways("Run FAST till a key is pressed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                lastSpeed = 90
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.04)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == 'n':
                lastNegativeSpeed = -16
                logAlways("Run reverse using slow negative speed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastNegativeSpeed, lastNegativeSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == 'N':
                lastNegativeSpeed = -60
                logAlways("Run reverse using fast negative speed")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastNegativeSpeed, lastNegativeSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == 'c':          # Cycle from stop to last speed that was set over and over
                logAlways("Cycle between last speed that was set to zero and back over and over")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                while not intKeys:
                    loops = 1
                    print("Cycle to the  ON speed for ", cycleOnPeriod, " cycles")
                    while not intKeys and loops < cycleOnPeriod:
                        cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                        ser.write(cmdPacket)
                        time.sleep(0.02)
                        loops = loops + 1;
                    loops = 1
                    print("Cycle to the OFF speed for ", cycleOffPeriod, " cycles")
                    while not intKeys and loops < cycleOffPeriod:
                        cmdPacket = formMagniSpeedMessage(lastNegativeSpeed, lastNegativeSpeed)
                        ser.write(cmdPacket)
                        time.sleep(0.02)
                        loops = loops + 1;
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == 'l':
                logAlways("Rotate left ")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(6, 15)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == 'r':
                logAlways("Rotate right ")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(15, 6)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]


            if input == 's':
                logAlways("send stop command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)

            if input == 'e':
                logAlways("Exit after sending stop command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                exit()

            if input == 'q':
                logAlways("Exit after sending stop command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                exit()

            if input == 'R':          # Resume to last speed that was set
                logAlways("Resume to last speed that was set")
                thread.start_new_thread(keyboard_thread, (intKeys,))
                while not intKeys:
                    cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
                    ser.write(cmdPacket)
                    time.sleep(0.02)
                logAlways("Finished with run command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                nextInput = intKeys[0]

            if input == 'v':
                logAlways("Fetch version information")
                # send query for the version
                queryVersion = [ 0x7e, 0x3a, 0x22, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryVersion)
                queryVersion.append(pktCksum)

                ser.flushInput()
                ser.write(queryVersion)

                # get each byte of the reply and wait for the reply we need (totally nasty! why all the junk?)
                # The Magni controller 'spews' forth loads of status all the time.  VERY MESSY!
                # We just read it all and search for the packet we need that starts with hex  7e 3c 22
                time.sleep(0.02)
                charsRead = 0
                charState = 0
                fwRev = '00'
                read_byte = ser.read()
                while read_byte is not None:
                    charsRead += 1
                    if (charsRead > 40):
                        break
                    hexData = read_byte.encode('hex')
                    # print("char: ",hexData)
                    if (charState == 6):
                        fwRev = hexData
                        break
                    if (charState == 5):
                        charState += 1
                    if (charState == 4):
                        charState += 1
                    if (charState == 3):
                        charState += 1
                    if (charState == 2):
                        if (hexData == '22'):
                            charState += 1
                        else:
                            charState = 0     # if we are not on the packet we want reset the state
                    if (charState == 1):
                        if (hexData == '3c'):
                            charState += 1
                        else:
                            charState = 0     # if we are not on the packet we want reset the state
                    if (charState == 0) and (hexData == '7e'):
                        charState += 1
                    read_byte = ser.read()

                print("fw rev query returned hex ", fwRev)

                time.sleep(0.02)

        except RuntimeError,e: 
          logAlways("Exception in magni_cmd: " + e.message)
        except KeyboardInterrupt:
          logAlways("terminated by keyboard interrupt! Zero the motor speed and exit")
          lastSpeed = 0
          cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)
          ser.write(cmdPacket)
          time.sleep(0.05)
        except Exception:
          logAlways("magni_cmd terminated.")


if __name__ == '__main__':
    print("Running with version: " + g_version)
 
    try:
        serCommander()
    except RuntimeError,e: 
        logAlways("Exception in magni_cmd: " + e.message)
    except Exception:
        logAlways("magni_cmd terminated.")
