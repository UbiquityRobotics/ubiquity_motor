#!/usr/bin/env python3

#
# A test of GPIO Lines on Rapberry Pi that requires loopback connector
#
# This test requires a standalone Raspberry Pi with GPIO pins connected in pairs
# where output and input test array index is the same
#   Wire for test1outputs[] and test1inputs[]
#   Wire for test2outputs[] and test2inputs[]
#
#
import time
import os

import sys
import getopt
import RPi.GPIO as GPIO

# Configure Pi GPIO lines we will use for pure GPIO IO input and output tests
# test1outputs = []
# test1inputs  = []
# test2outputs = []
# test2inputs  = []

# GPIO Lines must be connected between same indexes for test1 and for test2
test1outputs = [ 2, 4,14,17,22,10,11,24,8, 5, 6,19,26]
test1inputs  = [ 3,27,15,18,23, 9,21,25,7,13,12,16,20]
test2outputs = [ 2, 4,14,17,22,10,11,24,8, 5, 6,19,26]
test2inputs  = [ 3,27,15,18,23, 9,21,25,7,13,12,16,20]

# Version for this background monitor script
scriptVersion = '20181029.1'

# Get a starting time in unix time in seconds for this script
scriptStartTime = time.time()

# Define Paths used by this tool all up front when possible
# IMPORTANT SUPPORT INFO: These paths must EXACTLY match usages in config_server.cpp!
logFilePath           = '/home/ubuntu'

# set to 1 for showing some info on progress.
# there are many more debugs that can be enabled with modifications
g_simpleDebug = 0

# Define the LEDS in terms of GPIO lines
statusLed=12

GPIO.setwarnings(False)

# configure GPIO lines we will use for control
GPIO.setmode(GPIO.BCM)

# ---------------------------  Helper Routines  ------------------------------
# Log output from this script to a log file
def logLine(line):
    try:
        f = open(logFilePath, 'a')
        try:
            lineForLog = time.asctime(time.gmtime()) + '  ' + line + '\n'
            f.write(lineForLog)
        finally:
            f.close()
    except IOError:
        pass

# Blink a single led the number of times requested with requested blink period
# The led line is first set low then high and after done are all set to final value
def blinkLed( ledGpioLine, numBlinks, timeOfBlinks, finalLedState ):
    for x in range (0, numBlinks):
        GPIO.output(ledGpioLine, 1)
        time.sleep(timeOfBlinks)
        GPIO.output(ledGpioLine, 0)
        time.sleep(timeOfBlinks)
    GPIO.output(ledGpioLine, finalLedState)

def printUsage():
    print('usage:  python testPiGpio.py')

# -----------------------------   Start Main ----------------------------------
#
# Setup for log lines to a file
if not os.path.isfile(logFilePath):
    os.system('touch ' + logFilePath + ' 2> /dev/null');

retValue = 0;

logLine(os.path.basename(__file__) + ' version ' + scriptVersion + ' test starting now.')

# Get options for this run of the script
try:
    opts, args = getopt.getopt(sys.argv[1:], "hrsf")
except getopt.GetoptError as err:
    print(str(err))  # will print something like "option -a not recognized"
    printUsage()
    logLine(os.path.basename(__file__) + ' Pi Connector test exiting due to bad option entererd.')
    sys.exit(2)

output = None
verbose = False
# ...


# --------------------------  mainloop: Background Loop  -------------------------------

# Wrap to catch exceptions such as KeyboardInterrupt so we can indicate in log when done
try:
    # Start of the main program ========================================================

    bitError = 0

    print("Test of Raspberry Pi GPIO lines.  Version: " + str(scriptVersion))
    print("(The test fixture to connect associated GPIO lines is required)")
    print("")

    # ------------------------ Check first direction of GPIO pins --------------------
    for i in range(0, len(test1outputs)):
        if g_simpleDebug == 1:
            print("Dir 1 Test: Output port " + str(test1outputs[i]) + " input port " + str(test1inputs[i]))
        GPIO.setup(test1inputs[i],  GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(test1outputs[i], GPIO.OUT)

    print("Testing forward direction for digital GPIO connectivity...")
    time.sleep(0.1)

    # first we set one level then next pass test the other level
    print("Testing first direction for one level")
    for i in range(0, len(test1outputs)):
        # Set an output bit and read it back
        if ((i & 1) == True):
            # print("Set output port " + str(test1outputs[i]) + " to 1 ")
            GPIO.output(test1outputs[i], 1)
        else:
            # print("Set output port " + str(test1outputs[i]) + " to 0 ")
            GPIO.output(test1outputs[i], 0)

    time.sleep(0.1)
    for i in range(0, len(test1inputs)):
        # Set an output bit and read it back
        # print("Test input port " + str(test1inputs[i]))
        if ((i & 1) == True):
            if GPIO.input(test1inputs[i]) != 1:
                print("FAILURE! Readback of port " + str(test1inputs[i]) + " set by port " + str(test1outputs[i]) + " was not 1!")
                bitError |= 1 << i
        else:
            if GPIO.input(test1inputs[i]) != 0:
                print("FAILURE! Readback of port " + str(test1inputs[i]) + " set by port " + str(test1outputs[i]) + " was not 0!")
                bitError |= 1 << i

    # now set the one level for this direction
    print("Testing first direction for other level")
    for i in range(0, len(test1outputs)):
        # Set an output bit and read it back
        if ((i & 1) == True):
            # print("Set output port " + str(test1outputs[i]) + " to 0 ")
            GPIO.output(test1outputs[i], 0)
        else:
            # print("Set output port " + str(test1outputs[i]) + " to 1 ")
            GPIO.output(test1outputs[i], 1)

    time.sleep(0.1)
    for i in range(0, len(test1inputs)):
        # Test an input bit and read it back
        # print("Test input port " + str(test1inputs[i]))
        if ((i & 1) == True):
            if GPIO.input(test1inputs[i]) != 0:
                print("FAILURE! Readback of port " + str(test1inputs[i]) + " set by port " + str(test1outputs[i]) + " was not 0!")
                bitError |= 1 << i
        else:
            if GPIO.input(test1inputs[i]) != 1:
                print("FAILURE! Readback of port " + str(test1inputs[i]) + " set by port " + str(test1outputs[i]) + " was not 1!")
                bitError |= 1 << i


    print("Testing reverse direction for digital GPIO connectivity...")
    for i in range(0, len(test2outputs)):
        if g_simpleDebug == 1:
            print("Dir 2 Test: Output port " + str(test2outputs[i]) + " input port " + str(test2inputs[i]))
        GPIO.setup(test2inputs[i],  GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(test2outputs[i], GPIO.OUT)


    # first we set one level then next pass test the other level
    print("Testing reverse direction for one level")
    for i in range(0, len(test2outputs)):
        # Set an output bit and read it back
        if ((i & 1) == True):
            # print("Set output port " + str(test2outputs[i]) + " to 1 ")
            GPIO.output(test2outputs[i], 1)
        else:
            # print("Set output port " + str(test2outputs[i]) + " to 0 ")
            GPIO.output(test2outputs[i], 0)

    time.sleep(0.1)
    for i in range(0, len(test2inputs)):
        # Set an output bit and read it back
        # print("Test input port " + str(test2inputs[i]))
        if ((i & 1) == True):
            if GPIO.input(test2inputs[i]) != 1:
                print("FAILURE! Readback of port " + str(test2inputs[i]) + " set by port " + str(test2outputs[i]) + " was not 1!")
                bitError |= 1 << i
        else:
            if GPIO.input(test1inputs[i]) != 0:
                print("FAILURE! Readback of port " + str(test2inputs[i]) + " set by port " + str(test2outputs[i]) + " was not 0!")
                bitError |= 1 << i

    # now set the one level for this direction
    print("Testing reverse direction for other level")
    for i in range(0, len(test2outputs)):
        # Set an output bit and read it back
        if ((i & 1) == True):
            # print("Set output port " + str(test2outputs[i]) + " to 0 ")
            GPIO.output(test2outputs[i], 0)
        else:
            # print("Set output port " + str(test2outputs[i]) + " to 1 ")
            GPIO.output(test2outputs[i], 1)

    time.sleep(0.1)
    for i in range(0, len(test2inputs)):
        # Test an input bit and read it back
        # print("Test input port " + str(test2inputs[i]))
        if ((i & 1) == True):
            if GPIO.input(test2inputs[i]) != 0:
                print("FAILURE! Readback of port " + str(test2inputs[i]) + " set by port " + str(test2outputs[i]) + " was not 0!")
                bitError |= 1 << i
        else:
            if GPIO.input(test2inputs[i]) != 1:
                print("FAILURE! Readback of port " + str(test2inputs[i]) + " set by port " + str(test2outputs[i]) + " was not 1!")
                bitError |= 1 << i

    # Show results
    if (bitError == 0):
        print("")
        print("All tests passed!")
    else:
        print("")
        print("One or more failures were detected on this pass!")
        print("  Note that the test fixture to connect associated GPIO lines is required")
        print("  Intermittent failures are often a sign that some other task is controlling GPIO")


    # End of the main program   ==============================================================

except KeyboardInterrupt:
    logLine(os.path.basename(__file__) + ' keyboard exception. background monitor done.')
    raise
except:
    logLine(os.path.basename(__file__) + ' exception or program exit. background monitor done.')

sys.exit(0)




