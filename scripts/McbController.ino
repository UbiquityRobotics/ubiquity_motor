/*
  MCB Controller That Runs On The Arduino Platform for debug or very very simple controllers

  Some basic MCB speed control for the Ubiquity Robotics Magni Robot is implemented
  This code is meant for the Arduino Nano model.

  This code could be used to build upon a far more complex robot based on the Arduino platform
  or other platform supported by the Arduino IDE libraries.

  As a minimum the Arduino 5V level TX signal goes to MCB board P508  RX pin and Arduino Ground goes to P508 ground.
  Optionally you can power the Arduino all from P508 because there is 5V VCC on P508 as well.
  (Note that to do that you must tie MCB Vcc on P508 to Arduino 5V power because the spec for VIN is for 6V or more)

  Usage Instructions
  - Ground the ENABLE_SPEED_PIN (can be used as an ESTOP)
  - Connect tow 1k to 20k pots with each of the ends on GND and Vcc.  
    Center tap of one pot to A1, center tap of other pot to A2.
    Default with no jumpers is for each pot to separately control a single wheel.
  - Optionally ground the ENABLE_JOYSTICK pin to interprit two pots as joystick if you hook up dual pot joystick
  - Optionally a normaly closed switch from ground to ENABLE_SPEED_PIN for switch that must close to run motors
    When the ENABLE_SPEED_PIN is floating motors are enabled. When it goes to ground motors stop
  - Optionally have a switch to ground for going into AutoRun mode.  (reverses direction on settable time interval)

  Hardware Connections To Ubiquity Robotics MCB board
  - An Arduino Nano is hooked to 3 lines on the MCB P508 FTDI Jack. No converter is required.
  - Arduino GND to pin 1 (MCB Ground)
  - Arduino  5V to pin 3 (MCB 5V+)
  - Arduino  TX to pin 5 (MCB RX to receive serial at 38.4 kBaud

  Author:  mj@ubiquityrobotics.com
  Created: 20210103  Basic constant running with simple pot speed controls
  Updated: 20210312  Added AutoRun mode for back and forth wheel tests. Added reed relay on when in 'forward'. Cleanup speed set routine
  
*/

// Defines for the Arduino Nano.  Use these to hook up the optional jumpers and the required potentiometers
#define ENABLE_JOYSTICK  2   // Pull to ground to enable interpritation of pots as a joystick. Normally float this pin.
#define ENABLE_SPEED_PIN 3   // Release from ground to enable speed messages (thus use NC switch to use this feature)
#define VAR_AUTO_RUN     4   // If pulled to ground we are in automatic test mode where one pot is speed, other is time till reverse dir
#define VIRT_GND_1       5   // A pin set to ground to use for a 2-pin jumper that is adjacent to AUTO_RUN so connect with 0.1" jumper for AutoRun
#define FIXED_AUTO_RUN   6   // Jumper this to VIRT_GND for fixed speed/duration auto-run thus NO pots. Fixed test, easy with 1 jumper 
#define RELAY_1_DRIVE    7   // We flip a relay when in auto-run mode which may be useful for other purposes. Active Low so relay also to +5
#define VIRT_VCC_1       8   // A pin driven high to source a low current LED anode with resistor.  This is 'on' for forward in AutoRun
#define SPEED_POT_RIGHT  A1  // Motor speed control for right wheel in normal mode but in AutoRun this is the interval between reversals
#define SPEED_POT_LEFT   A2  // Motor speed control for left  wheel or for AutoRun is motor speed for BOTH pins and gets reversed periodically

int  g_debugMode      = 0;  // set to 1 to use serial monitor but normally 0 for real speed commands out
long g_loopTimeMs     = 50; // Need to keep repeating speeds or deadman timer kicks in on MCB
int  g_xmitEnable     = 0;  // When non-zero we allow speed messages to be continuously sent. Set to -1 to now use this feature
int  g_joystickEnable = 0;  // When positive (input pin ground) this interprites pots as from a joystick. Set to -1 for never joystick
int  g_varAutoRun     = 0;  // When non-zero indicates auto-run mode that runs then reverses diretion with given period till reversal
int  g_fixedAutoRun   = 0;  // Simplest of all fixed auto-run so no pots are required.  This makes standalone test super easy wiring


// Calculate a simple single byte checksum from a byte array of given length
// We do this treating each byte as an unsigned 0-255 value 
// Then return a 0-255 byte unsigned byte value
byte calcPacketCksum( byte *msg, int msgBytes) {
    unsigned int sum = 0;
    byte cksum = 0;
    int idx = 0;
    //Serial.print("CalcChecksum: ");
    for (idx = 0; idx < msgBytes ; idx++) { 
        sum += msg[idx];
    }

    // Take the full sum and form the special 1 byte checksum value 
    if (sum > 0xFF) {
        int tmp;
        tmp = sum >> 8;
        tmp = tmp << 8;
        cksum = 0xFF - (sum - tmp);
    } else {
        cksum = 0xFF - sum;
    }
    return cksum;
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN,      OUTPUT);
  pinMode(VIRT_GND_1,       OUTPUT);
  pinMode(RELAY_1_DRIVE,    OUTPUT);
  pinMode(VIRT_VCC_1,       OUTPUT);
  
  pinMode(ENABLE_SPEED_PIN, INPUT_PULLUP);
  pinMode(ENABLE_JOYSTICK,  INPUT_PULLUP);
  pinMode(SPEED_POT_RIGHT,  INPUT);
  pinMode(SPEED_POT_LEFT,   INPUT);
  pinMode(VAR_AUTO_RUN,     INPUT_PULLUP);
  pinMode(FIXED_AUTO_RUN,   INPUT_PULLUP);

  Serial.begin(38400);     // Set to speed for MCB control

  digitalWrite(RELAY_1_DRIVE, HIGH);
  digitalWrite(VIRT_VCC_1,    HIGH);
  digitalWrite(VIRT_GND_1,    LOW);
}

// Send out a speed command for both motors
void setMotorSpeeds(int leftSpeed, int rightSpeed)  {
  // Format a basic speed command.  Bytes 3,4 are MSB,LSB of signed right speed and bytes 5,6 are signed left speed
  byte speedCmd[]={0x7e, 0x3b, 0x2a, 0, 0, 0, 0, 0};
  int  cmdPacketLen = 8;
  int  msgXmitLen   = 0;
  
  // Bytes 3,4 are MSB,LSB of signed right speed and bytes 5,6 are signed left speed
  // Pack in the 2 speed bytes
  if (rightSpeed < 0) {
        speedCmd[3] = 0xff;
        speedCmd[4] = 0xff - ((-1 * rightSpeed) & 0xff);
  } else {
        speedCmd[3] = 0;
        speedCmd[4] = rightSpeed & 0xff;
  }
  if (leftSpeed < 0) {
        speedCmd[5] = 0xff;
        speedCmd[6] = 0xff - ((-1 * leftSpeed) & 0xff);
  } else {
        speedCmd[5] = 0;
        speedCmd[6] = leftSpeed & 0xff;
  }

  // The start byte is 0x7E so the cmdPacket where we calculate checksum does not include that
  cmdPacketLen = 6;
  speedCmd[cmdPacketLen+1] = calcPacketCksum( &speedCmd[1], cmdPacketLen);
  msgXmitLen = cmdPacketLen + 2;  // add in start byte and checksum bytes for total xmit length
  
  // Write out the speed command over serial port
  Serial.write(speedCmd, msgXmitLen);
}


int  leftSpeed    = 0;
int  rightSpeed   = 0;
int  linearSpeed  = 0;
int  rotateSpeed  = 0;
int  reversePeriodMs = 1000;
int  autoDirection = 1;   
int  periodMs     = 30000;  // Make it big to start
  
// the loop function runs over and over again forever
void loop() {

  // Setup mode based on pins we see in a given state. If this was set to -1 always enable motors
  // The idea here is that a normally closed deadman switch can be used so motors stop if switch is not pressed
  if (g_xmitEnable >= 0) {
    g_xmitEnable = digitalRead(ENABLE_SPEED_PIN);
  }

  g_varAutoRun   = digitalRead(VAR_AUTO_RUN);     // See if we are to do variable auto-run mode
  g_fixedAutoRun = digitalRead(FIXED_AUTO_RUN);   // See if we are to to fixed speed and period auto-run mode

  // Allow two pots to be interprited as a simple joystick control.  Default is each pot controls one motor.
  if (g_joystickEnable >= 0) {
    g_joystickEnable = 0;
    if (digitalRead(ENABLE_JOYSTICK) == 0) {
      g_joystickEnable = 1;
    }
  }

  // Determine motor right and left speed based on mode we are in at the time
  if ((g_varAutoRun == 0) || (g_fixedAutoRun == 0)) {
    if (g_fixedAutoRun == 0)  {   // Used constants so no pot is required
      // Fixed default test where very simple wiring does auto reversing default speeds and reversal time
      leftSpeed = 20;
      reversePeriodMs = 4000;
    } else {
      // Left pot is speed and right pot is time interval between speed reversals
      reversePeriodMs = ((long)analogRead(SPEED_POT_RIGHT) * 10) + 500;
      leftSpeed  = ((long)analogRead(SPEED_POT_LEFT) - 512)  / 8;     // allow signed 0-64
    }
    
    // Reversal time has a minimum time and then pot setting adds to that up to 10 sec more
    
    if (periodMs > reversePeriodMs) {
      autoDirection = autoDirection * -1;    // Reverse direction
      periodMs = 0;                          // reset clock for next period 
    }
    periodMs += g_loopTimeMs;

    // Left pot is signed speed for when autoDirection is 1 and gets reversed when autoDirection = -1
    leftSpeed = leftSpeed * autoDirection;  
    rightSpeed = leftSpeed;    // In autoRun mode both speeds are the same

    // We drive an optional reed relay that alternates on to off as an extra feature
    if (autoDirection > 0) {
      digitalWrite(RELAY_1_DRIVE, LOW);
    } else {
      digitalWrite(RELAY_1_DRIVE, HIGH);
    }
    
  } else if (g_joystickEnable <= 0) {
    // Directly determine speeds from two pots in joystick mode
    leftSpeed  = ((long)analogRead(SPEED_POT_LEFT) - 512)  / 8;     // allow signed 0-64
    rightSpeed = ((long)analogRead(SPEED_POT_RIGHT) - 512) / 8;     // allow signed 0-64
    
  } else {
    // Joystick mode: Interprit pots as one is forward/reverse and the other is bias to right or left (rotation)
    linearSpeed  = ((long)analogRead(SPEED_POT_LEFT) - 512)  / 8;     // allow signed 0-64
    rotateSpeed  = ((long)analogRead(SPEED_POT_RIGHT) - 512) * 2;     // This is near +- 1024

    // We are going to simply pull off a fraction of left to give to right based on the rotate
    // This will not yield true joystick performance but will allow turning
    // We will also ignore small rotations to allow straight driving to be easier
    float diffSpeed = (float)linearSpeed * ((float)rotateSpeed/(float)1024);
    leftSpeed = linearSpeed;
    rightSpeed = linearSpeed;
    if (((float)rotateSpeed > (float)(0.05)) || ((float)rotateSpeed < (float)(-0.05))) {
      leftSpeed  = linearSpeed - diffSpeed;
      rightSpeed = linearSpeed + diffSpeed;
    } 
  }

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  // Set motor speeds if enabled
  // Write out the speed command if transmit is enabled
  if ((g_xmitEnable != 0) && (g_debugMode == 0)) {
     setMotorSpeeds(leftSpeed, rightSpeed);
  }
  
  if (g_debugMode == 1) {
    g_loopTimeMs = 1000;
    if (g_joystickEnable == 0) {
      Serial.print("MotorSpeeds:  Left= "); Serial.print(leftSpeed); Serial.print("  Right= "); Serial.println(rightSpeed);
    } else {
      Serial.print("JoystickSpeeds:  Left= "); Serial.print(leftSpeed); Serial.print("  Right= "); Serial.println(rightSpeed);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  
  delay(g_loopTimeMs);                 // remain off this long
}
