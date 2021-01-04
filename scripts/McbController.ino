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
  - Ground the ENABLE_SPEED_PIN
  - Connect tow 1k to 20k pots with each of the ends on GND and Vcc.  
    Center tap of one pot to A1, center tap of other pot to A2.
    Default with no jumpers is for each pot to separately control a single wheel.
  - Optionally ground the ENABLE_JOYSTICK pin to interprit two pots as joystick
  - Optionally a normaly closed switch from ground to ENABLE_SPEED_PIN for switch that must close to run motors
    When the ENABLE_SPEED_PIN is floating motors are enabled. When it goes to ground motors stop

  Author:  mj@ubiquityrobotics.com
  Date:    20210103
  
*/

#define ENABLE_SPEED_PIN 3   // Release from ground to enable speed messages (thus use NC switch to use this feature)
#define ENABLE_JOYSTICK  2   // Pull to ground to enable interpritation of pots as a joystick
#define SPEED_POT_RIGHT  A1  // Motor speed control for right wheel
#define SPEED_POT_LEFT   A2  // Motor speed control for left  wheel

int debugMode = 0;        // set to 1 to use serial monitor but normally 0 for real speed commands out

long loopTimeMs = 50;     // Need to keep repeating speeds or deadman timer kicks in on MCB
long pulseOnTimeMs;       // On time in millisec
long pulseOffTimeMs;      // Off time in millisec

int  leftSpeed    = 0;
int  rightSpeed   = 0;
int  linearSpeed  = 0;
int  rotateSpeed  = 0;

int  xmitEnable     = 0; // When non-zero we allow speed messages to be continuously sent. Set to -1 to now use this feature
int  joystickEnable = 0; // When positive (input pin ground) this interprites pots as from a joystick. Set to -1 for never joystick

// Format a basic speed command.  Bytes 3,4 are MSB,LSB of signed right speed and bytes 5,6 are signed left speed
byte speedCmd[]={0x7e, 0x3b, 0x2a, 0, 0, 0, 0, 0};
int  cmdPacketLen = 8;
int  msgXmitLen   = 0;


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
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENABLE_SPEED_PIN, INPUT_PULLUP);
  pinMode(ENABLE_JOYSTICK,  INPUT_PULLUP);
  pinMode(SPEED_POT_RIGHT,  INPUT);
  pinMode(SPEED_POT_LEFT,   INPUT);

  Serial.begin(38400);     // Set to speed for MCB control
}

// the loop function runs over and over again forever
void loop() {

  // Setup mode based on pins we see in a given state. If this was set to -1 always enable motors
  // The idea here is that a normally closed deadman switch can be used so motors stop if switch is not pressed
  if (xmitEnable >= 0) {
    xmitEnable = digitalRead(ENABLE_SPEED_PIN);
  }

  // Allow two pots to be interprited as a simple joystick control.  Default is each pot controls one motor.
  if (joystickEnable >= 0) {
    joystickEnable = 0;
    if (digitalRead(ENABLE_JOYSTICK) == 0) {
      joystickEnable = 1;
    }
  }

  if (joystickEnable <= 0) {
    leftSpeed  = ((long)analogRead(SPEED_POT_LEFT) - 512)  / 8;     // allow signed 0-64
    rightSpeed = ((long)analogRead(SPEED_POT_RIGHT) - 512) / 8;     // allow signed 0-64
  } else {
    // Interprit pots as one is forward/reverse and the other is bias to right or left (rotation)
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

  // Format a basic speed command.  Bytes 3,4 are MSB,LSB of signed right speed and bytes 5,6 are signed left speed

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

  if (debugMode == 1) {
    loopTimeMs = 1000;
    if (joystickEnable == 0) {
      Serial.print("MotorSpeeds:  Left= "); Serial.print(leftSpeed); Serial.print("  Right= "); Serial.println(rightSpeed);
    } else {
      Serial.print("JoystickSpeeds:  Left= "); Serial.print(leftSpeed); Serial.print("  Right= "); Serial.println(rightSpeed);
    }
    Serial.print("Msg: ");
    for (int i = 0; i<msgXmitLen ; i++) {
      Serial.print("  "); Serial.print( speedCmd[i]);
    }
    Serial.println("\n");
  }
  
  // Write out the speed command if transmit is enabled
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  if ((xmitEnable != 0) && (debugMode == 0)) {
     Serial.write(speedCmd, msgXmitLen);
  }
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  
  delay(loopTimeMs);                 // remain off this long
}
