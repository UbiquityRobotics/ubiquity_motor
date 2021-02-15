#!/usr/bin/python -u

#
# Command control direct to Ubiquity Robotics Magni Controller Board
#
# This is diagnostic code that can be usedto directly run the motors
# without ROS and all the overhead of joysticks and so on.
# The idea started as a simple serial controller to do basic tests.
#
# Usage:
# Stop the ROS magni control using:  sudo systemctl stop magni-base
# sudo python magni_cmd.py    # OPTIONALLY add --dev /dev/ttyUSB0 for other port
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
import smbus

# simple version string
g_version = "20210215"

# default serial device.  An  adapter in USB is often  '/dev/ttyUSB0'
g_serialDev = '/dev/ttyAMA0' 

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

# Form a bytearray holding the LSB for a parameter set message to a given register
def formMagniParamSetMessage(registerNumber, paramValue):
    # Start with  a general message 
    paramSetCmd = [ 0x7e, 0x3b, 0x32, 0, 0, 0, 0 ]
   
    # fill in the register number
    #paramSetCmd[2] = int(registerNumber) & int(0xff)
    paramSetCmd[2] = registerNumber

    # fill in the least significant byte of the 32 bit value in the message
    msByte =  (int(paramValue)/int(256))
    paramSetCmd[5] = msByte & 0xff
    paramSetCmd[6] = int(paramValue) & 0xff

    # print("formMsg5 ", int(paramSetCmd[5]), " and lsb ", int(paramSetCmd[6]))
    pktCksum = calcPacketCksum(paramSetCmd)
    paramSetCmd.append(pktCksum)
    return paramSetCmd

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

# Fetch a single byte from the fixed length reply for a parameter fetch of a given type
def fetchReplyByte(ser, cmdHex, regHex):
    # get each byte of the reply and wait for the reply we need (totally nasty! why all the junk?)
    # The Magni controller 'spews' forth loads of status all the time.  VERY MESSY!
    # We just read it all and search for the packet we need that starts with hex  7e 3c 22
    time.sleep(0.02)
    charsRead = 0
    charState = 0
    replyByte = '00'
    read_byte = ser.read()
    while read_byte is not None:
        charsRead += 1
        if (charsRead > 80):
            print("fetchReplyByte: Too many reply chars ")
            break
        hexData = read_byte.encode('hex')
        # print("char: ",hexData)
        if (charState == 6):
            replyByte = hexData
            break
        if (charState == 5):
            charState += 1
        if (charState == 4):
            charState += 1
        if (charState == 3):
            charState += 1
        if (charState == 2):
            if (hexData ==  regHex):
                charState += 1
            else:
                charState = 0     # if we are not on the packet we want reset the state
        if (charState == 1):
            if (hexData ==  cmdHex):
                charState += 1
            else:
                charState = 0     # if we are not on the packet we want reset the state
        if (charState == 0) and (hexData == '7e'):
            charState += 1
        read_byte = ser.read()
    return replyByte


# Fetch a 16 bit value from the fixed length reply for a parameter fetch of a given type
def fetchReplyWord(ser, cmdHex, regHex):
    # get each byte of the reply and wait for the reply we need (totally nasty! why all the junk?)
    # The Magni controller 'spews' forth loads of status all the time.  VERY MESSY!
    # We just read it all and search for the packet we need that starts with hex  7e 3c 22
    time.sleep(0.02)
    charsRead = 0
    charState = 0
    replyMsb  = '00'
    replyWord = '0000'
    read_byte = ser.read()
    while read_byte is not None:
        charsRead += 1
        if (charsRead > 80):
            print("fetchReplyWord: Too many reply chars ")
            break
        hexData = read_byte.encode('hex')
        # print("char: ",hexData)
        if (charState == 6):
            replyWord = replyMsb + hexData
            break
        if (charState == 5):
            replyMsb = hexData
            charState += 1
        if (charState == 4):
            charState += 1
        if (charState == 3):
            charState += 1
        if (charState == 2):
            # if (hexData ==  int(regDec,16)):
            if (hexData ==  regHex):
                charState += 1
            else:
                charState = 0     # if we are not on the packet we want reset the state
        if (charState == 1):
            if (hexData ==  cmdHex):
                charState += 1
            else:
                charState = 0     # if we are not on the packet we want reset the state
        if (charState == 0) and (hexData == '7e'):
            charState += 1
        read_byte = ser.read()
    return replyWord


# Fetch a 32 bit value from the fixed length reply for a parameter fetch of a given type
def fetchReplyLongWord(ser, cmdHex, regHex):
    # get each byte of the reply and wait for the reply we need (totally nasty! why all the junk?)
    # The Magni controller 'spews' forth loads of status all the time.  VERY MESSY!
    # We just read it all and search for the packet we need that starts with hex  7e 3c 22
    time.sleep(0.02)
    charsRead = 0
    charState = 0
    reply24  = '00'
    reply16  = '00'
    reply08  = '00'
    replyLongWord = '0000'
    read_byte = ser.read()
    while read_byte is not None:
        charsRead += 1
        if (charsRead > 80):
            print("fetchReplyWord: Too many reply chars ")
            break
        hexData = read_byte.encode('hex')
        # print("char: ",hexData)
        if (charState == 6):
            replyLongWord = reply24 + reply16 + reply08 + hexData
            break
        if (charState == 5):
            reply08 = hexData
            charState += 1
        if (charState == 4):
            reply16 = hexData
            charState += 1
        if (charState == 3):
            reply24 = hexData
            charState += 1
        if (charState == 2):
            # if (hexData ==  int(regDec,16)):
            if (hexData ==  regHex):
                charState += 1
            else:
                charState = 0     # if we are not on the packet we want reset the state
        if (charState == 1):
            if (hexData ==  cmdHex):
                charState += 1
            else:
                charState = 0     # if we are not on the packet we want reset the state
        if (charState == 0) and (hexData == '7e'):
            charState += 1
        read_byte = ser.read()
    return replyLongWord


# utility to set right and left wheel speeds then exit when key is hit
def setSpeedTillKeypress(ser, speed1, speed2):
    intKeys = []
    thread.start_new_thread(keyboard_thread, (intKeys,))
    while not intKeys:
        cmdPacket = formMagniSpeedMessage(speed1, speed2)
        ser.write(cmdPacket)
        time.sleep(0.02)
    logAlways("Finished with run command")
    cmdPacket = formMagniSpeedMessage(0, 0)
    ser.write(cmdPacket)
    keyInput = intKeys[0]
    return keyInput

def showHelp():
    logAlways("ONLY USE THIS LOW LEVEL UTILITY WITH EXPLICIT HELP FROM Ubiquity Robotics SUPPORT!")
    logAlways("ONLY RUN WHEN MAGNI IS STOPPED!. Use  sudo systemctl stop magni-base.service")
    logAlways("Commands: h or ? for help.  v for versions of firmware and hardware. Use Control-C to quit or E")
    logAlways("Speeds:   Enter 0 - 9 fwd speed. n,N slow/fast reverse. s for 'any speed' or c cycle last fwd/reverse")
    logAlways("  v  - Query firmware and hw version setting      o - Query 8-bit hardware option port with real board rev")
    logAlways("  p  - Query PID control loop parameters          O - Query firmware hw options")
    logAlways("  D  - Query range of registers for 32bit vals.   S - Set word value for any register. Reg in hex, value as decimal")
    logAlways("  q  - Query a 16 bit word value from register.   Q - Query 32 bit register value")
    logAlways("  32 = Query if firmware thinks motors active    33 = Set to 1 to enable any exit of ESTOP feature")
    logAlways("  34 = Set to max PID threshold where pre rev 5.0 boards did a safer ESTOP release. 0 to disable")
    logAlways("  35 = Set to the max forward limit speed        36 - Set to a max negative reverse limit speed")
    logAlways("  37 = Set max PWM setting [250]         ")
    return 0


class serCommander():
    def __init__(self):
        global g_serialDev
        intKeys = []
        keyBuf  = []

        serialDev = g_serialDev
        
        # The bits we see here are placed in globals by notification callback

        nextInput=''
        lastSpeed=0
        lastNegativeSpeed=0
        cycleOnPeriod=40        # cycles of constant running or time motor is ON for cycle test
        cycleOffPeriod=40       # cycles of stopped running or time motor is OFF for cycle test

        # start thread to pick up keyboard presses
        intKeys = []
        nextCmd = ''

        # Magni Rasperry Pi serial port values
        # ser = serial.Serial('/dev/ttyS0', 38400, 8, 'N', 1, timeout=1)
        # Native Pi3B+ Ubiquity Robotics serial port Gnd=6 Tx=8  Rx=10 (+3.3V = 1)
        # ser = serial.Serial('/dev/ttyAMA0', 38400, 8, 'N', 1, timeout=1)  

        # First USB serial port (safer for reasons of static and power blowouts
        print("Start Serial port using device ", serialDev)
        try:
            ser = serial.Serial(serialDev, 38400, 8, 'N', 1, timeout=1)
        except Exception:
            logAlways("Unable to open serial port. Verify correct port was specified")
            exit()

        logAlways("Serial port opened")

        # always set speed to zero at the start to initialize motor controller zero point
        logAlways("Set motors to run at 0.0 MPs")
        lastSpeed = 0
        cmdPacket = formMagniSpeedMessage(lastSpeed, lastSpeed)

        try:
            ser.write(cmdPacket)
        except Exception:
            logAlways("Unable to properly send the command to Magni. This pregram canno run with Magni running.")
            logAlways("Did you stop main software using   sudo systemctl stop magni-base.service")
            exit()

        time.sleep(0.05)
        logAlways("Finished with run command")

        showHelp()

        try:
          while True :

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

            if input == 'h':
                showHelp()
            if input == '?':
                showHelp()

            if input == '0':
                logAlways("Run at 0.0 MPs till a key is pressed")
                lastSpeed = 0
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '1':
                logAlways("Run at 0.1 MPs till a key is pressed")
                lastSpeed = 5
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '2':
                logAlways("Run at 0.2 MPs till a key is pressed")
                lastSpeed = 10
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '3':
                logAlways("Run at 0.3 MPs till a key is pressed")
                lastSpeed = 24 
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '4':
                logAlways("Run at 0.4 MPs till a key is pressed")
                lastSpeed = 32
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '5':
                logAlways("Run at 0.5 MPs till a key is pressed")
                lastSpeed = 40
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '6':
                logAlways("Run at 0.65 MPS or 1 rev per second")
                lastSpeed = 48
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '7':
                logAlways("Run at 0.5 MPs till a key is pressed")
                lastSpeed = 56
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '8':
                logAlways("Run at 0.5 MPs till a key is pressed")
                lastSpeed = 64
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == '9':
                logAlways("Run FAST till a key is pressed")
                lastSpeed = 72
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == 's':
                logAlways("Set speed to any value")
                lastSpeed = int(raw_input("Enter peed value 0-255 max integer: "))
                nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == 'n':
                logAlways("Run reverse using slow negative speed")
                lastNegativeSpeed = -10
                nextInput = setSpeedTillKeypress(ser, lastNegativeSpeed, lastNegativeSpeed)

            if input == 'N':
                logAlways("Run reverse using fast negative speed")
                lastNegativeSpeed = -95
                nextInput = setSpeedTillKeypress(ser, lastNegativeSpeed, lastNegativeSpeed)

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

            if input == 'B':
                cmdPacket = formMagniParamSetMessage(0x21, 50)
                ser.write(cmdPacket)
                logAlways("Forced Board Revision to rev 5.0")
                time.sleep(0.02)

            if input == 'b':
                cmdPacket = formMagniParamSetMessage(0x21, 49)
                ser.write(cmdPacket)
                logAlways("Forced Board Revision to rev 4.9")
                time.sleep(0.02)

            if input == 'D':          # Do register dump of a range of registers
                logAlways("Query any control register to any value up to one long word size")
                cmdFirstRegAsHex  = raw_input("Enter first control register number in hex: ")
                cmdFirstRegNumber = int(cmdFirstRegAsHex,16)
                cmdLastRegAsHex   = raw_input("Enter last control register number in hex: ")
                cmdLastRegNumber  = int(cmdLastRegAsHex,16)
                print("Dump MCB from hex Reg ", cmdFirstRegAsHex,"[",cmdFirstRegNumber,"] to hex Reg ", cmdLastRegAsHex,"[",cmdLastRegNumber,"]")
                for reg in range(cmdFirstRegNumber, cmdLastRegNumber, 1):
                    queryBytes = [ 0x7e, 0x3a, 0x34, 0, 0, 0, 0 ]
                    queryBytes[2] = reg
                    pktCksum = calcPacketCksum(queryBytes)
                    queryBytes.append(pktCksum)
                    ser.flushInput()
                    ser.write(queryBytes)
                    hexRegValue = '{:02x}'.format(reg)
                    registerValue = fetchReplyLongWord(ser, '3c', hexRegValue)
                    integerValue = int(registerValue,16)
                    if integerValue > 2147483647:
                        integerValue = (4294967296 - integerValue) * -1
                    print("Reg ", hexRegValue, " value = ", registerValue, " hex : or dec ", integerValue)

                time.sleep(0.02)

            if input == 'E':  # Enable ESET stop speed feature in firmware
                logAlways("Enable firmware ESET stop safety feature")
                queryBytes = [ 0x7e, 0x3a, 0x33, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryBytes)
                queryBytes.append(pktCksum)
                ser.flushInput()
                ser.write(queryBytes)
                estopEnableState = fetchReplyByte(ser, '3c', '33')
                print("Prior ESTOP enable setting was ", estopEnableState)
                time.sleep(0.02)
                cmdPacket = formMagniParamSetMessage(0x33, 1)
                ser.write(cmdPacket)
                logAlways("Enabled firmware ESET stop safety feature")

            if input == 'i':
                logAlways("Fetch Robot type ID")
                queryBytes = [ 0x7e, 0x3a, 0x31, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryBytes)
                queryBytes.append(pktCksum)
                ser.flushInput()
                ser.write(queryBytes)
                robotTypeId = fetchReplyByte(ser, '3c', '31')
                print("Robot type ID is ", robotTypeId)
                time.sleep(0.02)

            if input == 'l':
                logAlways("Rotate left ")
                rightSpeed = 6
                leftSpeed  = 15
                nextInput = setSpeedTillKeypress(ser, rightSpeed, leftSpeed)

            if input == 'm':
                logAlways("Fetch motor controller motor power state")
                queryBytes = [ 0x7e, 0x3a, 0x32, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryBytes)
                queryBytes.append(pktCksum)
                ser.flushInput()
                ser.write(queryBytes)
                motPowState = fetchReplyByte(ser, '3c', '32')
                print("Motor controller things motor power state is ", motPowState)
                time.sleep(0.02)

            if input == 'r':
                logAlways("Rotate right ")
                rightSpeed = 15
                leftSpeed  = 6
                nextInput = setSpeedTillKeypress(ser, rightSpeed, leftSpeed)

            if input == 'q':          # query any register to any value
                logAlways("Query any control register to any value up to one word size")
                cmdRegAsHex = raw_input("Enter control register number in hex: ")
                cmdRegNumber = int(cmdRegAsHex,16)
                queryBytes = [ 0x7e, 0x3a, 0x34, 0, 0, 0, 0 ]
                queryBytes[2] = cmdRegNumber
                pktCksum = calcPacketCksum(queryBytes)
                queryBytes.append(pktCksum)
                ser.flushInput()
                ser.write(queryBytes)
                registerValue = fetchReplyWord(ser, '3c', cmdRegAsHex)
                print("Register was set to  ", int(registerValue,16), " decimal", registerValue, " hex")
                time.sleep(0.02)

            if input == 'O':          # query the firmware options register set of bits
                logAlways("Query the current hardware option bit settings")
                cmdRegAsHex = '38'
                cmdRegNumber = int(cmdRegAsHex,16)
                queryBytes = [ 0x7e, 0x3a, 0x34, 0, 0, 0, 0 ]
                queryBytes[2] = cmdRegNumber
                pktCksum = calcPacketCksum(queryBytes)
                queryBytes.append(pktCksum)
                ser.flushInput()
                ser.write(queryBytes)
                logAlways("24")
                registerValue = fetchReplyLongWord(ser, '3c', cmdRegAsHex)
                logAlways("26")
                print("Hardware option bits are set to  ", int(registerValue,16), " decimal", registerValue, " hex")
                if ((int(registerValue,16) & 0x01) != 0):
                    print ("  High resolution encoders")
                else:
                    print ("  Standard resolution encoders")
                if ((int(registerValue,16) & 0x02) != 0):
                    print ("  Thin gearless wheels")
                else:
                    print ("  Standard wheels")
                if ((int(registerValue,16) & 0x04) != 0):
                    print ("  Reverse the wheel direction")
                time.sleep(0.02)

            if input == 'Q':          # query any register for a Long (32 bit value)
                logAlways("Query any control register to any value up to one long word size")
                cmdRegAsHex = raw_input("Enter control register number in hex: ")
                cmdRegNumber = int(cmdRegAsHex,16)
                queryBytes = [ 0x7e, 0x3a, 0x34, 0, 0, 0, 0 ]
                queryBytes[2] = cmdRegNumber
                pktCksum = calcPacketCksum(queryBytes)
                queryBytes.append(pktCksum)
                ser.flushInput()
                ser.write(queryBytes)
                registerValue = fetchReplyLongWord(ser, '3c', cmdRegAsHex)
                integerValue = int(registerValue,16)
                if integerValue > 2147483647:
                    integerValue = (4294967296 - integerValue) * -1
                print("Reg ", cmdRegAsHex, " value = ", registerValue, " hex : or dec ", integerValue)
                time.sleep(0.02)

            if input == 'S':          # Set any register to any value
                logAlways("Set any control register to any value up to one word size")
                cmdRegAsHex  = raw_input("Enter control register number in as hex digits:  ")
                cmdRegValue  = raw_input("Enter control register value to be set in decimal: ")
                cmdRegNumber = int(cmdRegAsHex,16)
                cmdPacket = formMagniParamSetMessage(cmdRegNumber, cmdRegValue)
                ser.write(cmdPacket)
                time.sleep(0.02)
                # nextInput = setSpeedTillKeypress(ser, lastSpeed, lastSpeed)

            if input == 'x':
                logAlways("Exit after sending stop command")
                cmdPacket = formMagniSpeedMessage(0, 0)
                ser.write(cmdPacket)
                exit()

            if input == 'o':          # Read the option bits and board rev and if motor power is on or not
                # i2c address of PCF8574 on the motor controller board
                PCF8574 = 0x20

                logAlways("Query hardware version from I2C interface on motor controller board")
                i2cbus = smbus.SMBus(1)
                print ("Setup 8-bit I2C port to set as inputs. Also detect if port is present")
                portPresent = 0
                try:
                    i2cbus.write_byte(PCF8574,0xff) 
                    portPresent = 1
                    time.sleep(0.2)
                except Exception:
                    print ("Did not detect 8-bit port which is only on rev 5.0 and later boards OR I2C failure")

                if (portPresent == 1):
                    inputPortBits = i2cbus.read_byte(PCF8574)
                    # print ("Port returned: ", inputPortBits, " decimal")
                    if ((inputPortBits & 0x80) == 0):
                        print ("Motor power is OFF")
                    else:
                        print ("Motor power is ON")
                    # The 4 board revision bits are negative logic.
                    # They are 0x0E for binary 1 which is board revision 5.0
                    # We will only change the revision when hardware capabilities are different
                    boardRev = 49 + (15 - (inputPortBits & 0x0f))
                    print ("Motor Controller Board Revision is: ", boardRev)
                    optionBits = (inputPortBits & 0x70) >> 4
                    print ("Option jumper block is set to: (install a jumper sets a bit to 0)", optionBits)

            if input == 'p':
                logAlways("Fetch PID Factors")
                # send queries to fetch PID cooeficients
                queryPid = [ 0x7e, 0x3a, 0x1b, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryPid)
                queryPid.append(pktCksum)
                ser.flushInput()
                ser.write(queryPid)
                pidReg = fetchReplyWord(ser, '3c', '1b')
                print("  P (1b)     = ", int(pidReg,16), " [", pidReg, " hex]")
                queryPid = [ 0x7e, 0x3a, 0x1c, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryPid)
                queryPid.append(pktCksum)
                ser.flushInput()
                ser.write(queryPid)
                pidReg = fetchReplyWord(ser, '3c', '1c')
                print("  I (1c)     = ", int(pidReg,16), " [", pidReg, " hex]")
                queryPid = [ 0x7e, 0x3a, 0x1d, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryPid)
                queryPid.append(pktCksum)
                ser.flushInput()
                ser.write(queryPid)
                pidReg = fetchReplyWord(ser, '3c', '1d')
                print("  D (1d)     = ", int(pidReg,16), " [", pidReg, " hex]")
                queryPid = [ 0x7e, 0x3a, 0x37, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryPid)
                queryPid.append(pktCksum)
                ser.flushInput()
                ser.write(queryPid)
                pidReg = fetchReplyWord(ser, '3c', '37')
                print("  MaxPWM (37)= ", int(pidReg,16), " [", pidReg, " hex]")
                time.sleep(0.02)

            if input == 'v':
                logAlways("Fetch software and hardware version information")
                # send query for the firmware version
                queryVersion = [ 0x7e, 0x3a, 0x22, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryVersion)
                queryVersion.append(pktCksum)
                ser.flushInput()
                ser.write(queryVersion)
                fwRev = fetchReplyByte(ser, '3c', '22')
                fwRevInt = int(fwRev,16)
                print("fw revision ", fwRevInt)
                if fwRevInt >= 35:
                    # return daycode register if firmware version supports it
                    queryVersion = [ 0x7e, 0x3a, 0x3a, 0, 0, 0, 0 ]
                    pktCksum = calcPacketCksum(queryVersion)
                    queryVersion.append(pktCksum)
                    ser.flushInput()
                    ser.write(queryVersion)
                    registerValue = fetchReplyLongWord(ser, '3c', '3a')
                    print("fw daycode  ", registerValue)
                time.sleep(0.02)
                # send query for the controller board hardware version
                queryVersion = [ 0x7e, 0x3a, 0x21, 0, 0, 0, 0 ]
                pktCksum = calcPacketCksum(queryVersion)
                queryVersion.append(pktCksum)
                ser.flushInput()
                ser.write(queryVersion)
                fwRev = fetchReplyByte(ser, '3c', '21')
                boardRev = int(fwRev,16) / 10.0
                print("hw revision ", boardRev)
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
          logAlways("NOTE: Program requires prior use of:  sudo systemctl stop magni-base")


if __name__ == '__main__':
    print("Running with script version: " + g_version)
   
    if str(len(sys.argv)) == "3":
        if str(sys.argv[1]) == "--dev" or str(sys.argv[1]) == "--device":
            g_serialDev = str(sys.argv[2])
    print("Using serial device file: " + g_serialDev)

    try:
        serCommander()
    except RuntimeError,e: 
        logAlways("Exception in magni_cmd: " + e.message)
    except Exception:
        logAlways("magni_cmd terminated.")
