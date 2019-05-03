# ubiquity_motor
[![Build Status](https://travis-ci.org/UbiquityRobotics/ubiquity_motor.svg?branch=indigo-devel)](https://travis-ci.org/UbiquityRobotics/ubiquity_motor)

## Master Controller Board Firmware and Hardware Revision History 

Here is where we list in reverse time order the important high level changes for Firmware and Hardware revisions.

We will use the term MCB for Master Controller Board (the main board containing the motor controller, power supplies, power management, safety systems, real time clock, etc.)

## Firmware Revisions

Here is a history of what was included or fixed for each firmware revision that was considered stable

Firmware Revision History

* `Rev 33`  BETA Firmware. Available as version v33 using upgrade_firmware tool and is not our default. Not recommended for general use yet.
    * If the wheels move when ESTOP switch is activated the Magni we do NOT jump upon ESTOP release
    * Improved PID loop reset on detection of no motor power (ESTOP activated) so we are at a fully PID loop reset state on ESTOP release.
    * DEFECT: On startup prior to first movement we can see a state sometimes where the motors are not in a 'breaking' state and we are looking into this.    This is reported so far if you don't fully reset the linux software but have power cycled the motor controller board which is possible when not using the Raspberry Pi as the CPU.

* `Rev 32`  2019-03-25  This is our default latest stable firmware
    * Many improvements to ESTOP behavior to prevent large movements upon ESTOP release on rev 5.0 boards
    * Support to enable the improved ESTOP behavior if high level ROS enables the feature
    * A worse-case default ESTOP detect logic that will work on boards prior to rev 5.0 but is very crude and still moves
    * Detection of when motors fall too far behind from speed settings indicating velocity settings are too high.
    * Improved recovery for when ROS motor node is stopped when it is issuing movement commands we will not move in drastic ways as the ROS motor node is restarted due to development or recovery reasons.
    * Parameter to set maximum wheel velocity settings to prevent unobtainable speed settings from causing problems.
    * Parameter to set maximum PWM setting the motors can be set to so bad PID cooeficients cannot lead to dangerous movements of the Magni unit.
    * Build environment cleanup of unused files (not a 'fix' but needs to be mentioned to be complete

* `Rev 30`  2018-07-21
    * Motor encoder inputs use double sync to prevent false readings that gave bad encoder readings
    * Improvements to the bootloader and firmware download mechanisms to be ready for production

* `Rev 29`  A near ready for production release used on pre-production units with rev 4.7 boards. 

* `Rev 28`  A production test firmware revision that is not intended to be sent to customers.

## Hardware Revisions

The MCB hardware revisions start with those board revisions that may have ended up at sites outside of Ubiquity Robotics.   Below is a list in reverse time order of the boards and approximate time of first availability.


* `5.1` A minor but important revision was first produced April 2019.  
    * A major reliability fix to prevent failures of the motor ECB power switch/protection circuit. A re-design of the Motor Power ECB circuit that . Many value changes are involved as is the change of the M903_ECB_MOT MosFet to a much more powerful MosFet.

* `5.0` This major revision was first produced December 2018.  Quite a few changes for reliability, production and safety issues.
    * Fixes to allow proper low voltage detection for main and motor power REQUIRE the use of the new switch board with 4 Resistors.  It is thus IMPORTANT to know that a rev 5 board MUST use a switch board that now says rev 2.0 on it and will not power up with older switch board that had 3 resistors.  Earlier boards will work with the new rev 2.0 switch board but rev 5.0 MUST use new switch board with 4 resistors.
    * A new chip, U3, and some circuit changes allows host CPU to read over I2C board revision, motor power switch state and a 3-bit option jumper block.
    * Layout and parts changes for the high current sense resistors to allow larger resistors that can be in parallel for BOTH ECB circuits
    * Many changes in the top and bottom layer copper relief around P602 adn P603 to make it no longer possible for the high current battery power to short.
    * Changes so the on-board processor can quickly detect if the ESTOP switch is pressed or not.
    * Added leds that show when Main power ECB is powered on and also when Motor ECB is powered on (Shows ESTOP effect to motor power)
    * The 5-pin motor encoder jacks became 6-pin so the white wire could be attached as well and not left hanging
    * Jack P2 and nearby mounting holes allow a separate I2C connector or the use of a future OLED display once we decide to ship this feature.
    * Some movements of large through hole capacitors to minimize production and interference issues.  
    * Many more changes to fix mechanical issues with the 14-pin jack to the switch board and interference with the jack and parts
    * A LOT of silkscreen changes to better show PIN 1 placements and better label jacks like the two 4-pin white power jacks on top.
    * Several of the 0.1" connector strips were set to NO-LOAD or moved to the back to prevent shorts on never used jacks and to allow USB plugs to be easier to plug in and not risk shorting out these 0.1" long pinned jacks.
    * Some resistors moved to not be under the large 3-pin green motor 3-phase terminal strips.
    * REWORK:  C312_5M mistake required a mod for this large capacitor. 

* `4.9` The first production run boards that had undergone qualification tests required for shipment to end users.  A great many things changed from rev 4.7 and they will not all be listed because this is technically the first official board version for customer shipments.  A few key things are listed.
    * Addition of many ground plane and EMC emission suppression fixes so we pass required tests to ship the product.
    * Started using chips U1 and U2 for active Serial level converters to allow faster communicaitons and better signals.
    * Changes around the 14 pin jack that connects to the switch board to address components in the way of the jack.
    * Some silkscreen changes

* `4.7` A pre-release board revision prior to the first production release.  Alpha and Beta sites and users who needed to try things in a lab only environment were sent this board for a small number of early evaluation units.

