# ubiquity_motor
[![Build Status](https://travis-ci.org/UbiquityRobotics/ubiquity_motor.svg?branch=indigo-devel)](https://travis-ci.org/UbiquityRobotics/ubiquity_motor)

## Master Controller Board Firmware and Hardware Revision History

Here is where we list in reverse time order the important high level changes for Firmware and Hardware revisions.

We will use the term MCB for Master Controller Board (the main board containing the motor controller, power supplies, power management, safety systems, real time clock, etc.)

## Firmware Revisions

Below is presented a history of what was included or fixed for each firmware revision that was considered stable.
To do a firmware upgrade for the motor controller board please see our instructions [HERE](https://learn.ubiquityrobotics.com/firmware-upgrade)

Firmware Revision History

* `v40`  20201209.  Our most current Magni release candidate firmware. This has been looking very good as of Jan 2021
    * Just about the same as v39 but has proper defaults for a shipping Magni Products. Has everything discussed for v29

* `v39`  2020-11-29. NOT A MAGNI Compatible Release! Custom build for a 4 wheel drive architecture in development
    * Adds new modes for advanced wheel control options to improve higher load turning response.
    * Adds new feedback and registers for indication of wheel currents and control state
    * Added support for MCB rev 5.3 testpoints.  

* `v37`  2020-06-20. Used as initial code on MCB board production. Not qualified for final product.
    * Contains built in selftest ability including a wheel test.
    * Contains runtime checks for power supply and main battery levels

* `v35`  2019-08-15. Available as version v35 using upgrade_firmware tool.
    * This is a major release with first support for double resolution wheels
    * THIS VERSION REQUIRES HOST SIDE CODE UPDATE TO Nov 15, 2019 apt upgrade or later to function properly!
    * Doubles the wheel encoder resolution.
    * Improves motor PID speed controller for added responsiveness as well as options for setting certain limits.

ALL REVISIONS FROM HERE ON REQUIRE HOST SIDE UPDATE TO Nov 15, 2019 or later

* `v33`  BETA Firmware. DO NOT USE THIS version.  Available as version v33 using upgrade_firmware tool and is not our default. Not to be used due to some defects.
    * If the wheels move when ESTOP switch is activated the Magni we do NOT jump upon ESTOP release
    * Improved PID loop reset on detection of no motor power (ESTOP activated) so we are at a fully PID loop reset state on ESTOP release.
    * DEFECT: On startup prior to first movement we can see a state sometimes where the motors are not in a 'breaking' state and we are looking into this.    This is reported so far if you don't fully reset the linux software but have power cycled the motor controller board which is possible when not using the Raspberry Pi as the CPU.

* `v32`  2019-03-25  This was our default firmware for much of 2019
    * Many improvements to ESTOP behavior to prevent large movements upon ESTOP release on rev 5.0 boards
    * Support to enable the improved ESTOP behavior if high level ROS enables the feature
    * A worse-case default ESTOP detect logic that will work on boards prior to rev 5.0 but is very crude and still moves
    * Detection of when motors fall too far behind from speed settings indicating velocity settings are too high.
    * Improved recovery for when ROS motor node is stopped when it is issuing movement commands we will not move in drastic ways as the ROS motor node is restarted due to development or recovery reasons.
    * Parameter to set maximum wheel velocity settings to prevent unobtainable speed settings from causing problems.
    * Parameter to set maximum PWM setting the motors can be set to so bad PID cooeficients cannot lead to dangerous movements of the Magni unit.
    * Build environment cleanup of unused files (not a 'fix' but needs to be mentioned to be complete

* `v30`  2018-07-21
    * Motor encoder inputs use double sync to prevent false readings that gave bad encoder readings
    * Improvements to the bootloader and firmware download mechanisms to be ready for production

* `v29`  A near ready for production release used on pre-production units with rev 4.7 boards.

* `v28`  A production test firmware revision that is not intended to be sent to customers.

## Hardware Revisions

The MCB hardware revisions start with those board revisions that may have ended up at sites outside of Ubiquity Robotics.   Below is a list in reverse time order of the boards and approximate time of first availability.

To identify the motor controller board you have you can try to read the version very low on the top left side of the board or you can visit our website [HERE](https://learn.ubiquityrobotics.com/PC_Board_RevId)

* `5.3` A modest revision with many improvements and better service features
    * F701 fuse is a 10 AMP (not 35 amp) fuse and F700 is a new non-loaded fuse to replace rev 5.2 R706 0 ohm jumper which we have learned could lead to failures on hard shorts
    * use 3 pin back loaded header, P706, with center pin to ground and side pins are TP3 and TP4. This allows jumper usage in testing board.
    * Just one 25mm standoff for Raspberry Pi support in middle of board. Two standoffs next to P701 removed for assembly reasons.
    * BOM corrections for C411 (12V Main) and C1411 (12V Aux) 3.5mm lead space 680uf 25V, C312 (5V Aux) and C1312 (5V Main) 3.5mm lead space and will be 1000uf @ 16V
    * Fix 'WiFi' diode silkscreen (reversed) and make 'WiFi' text larger
    * Q903 is loaded large topside Mosfet, TI CSD19532KTT, for ECB Main (reliability fix)
    * R1901 (100k) and Q1904 are now backside loaded parts on the bom and higher current and take the place of topside Q1903. (reliability fix)
    * R604 and R603 become 0.1 percent to improve accuracy of battery voltage reading for worse case resistor tolerance.
    * Assorted improvements to silkscreen for easier support and service of the board.

* `5.2` A major revision placed into production in early April 2020.
    * The CAD system was changed to be a full KiCAD based design.  This was a huge change that required renumbering of most all of the part reference designators and of course the goal was after it was done to have an identical board.
    * An onboard 3.3V regulator supplies the onboard 3.3V for some parts.  This means that we no longer put the Raspberry Pi 3.3 at risk that can destroy a Raspberry Pi. This also means we have more current locally on 3.3V.
    * Two leds now indicate serial traffic has made it through the level converters. This is a huge debug assist as we can tell if the host raspberry Pi is actively talking and can detect failures in level converters.
    * A WiFi led has been added so for users who do not use a sonar board they can see the state of the WiFi for Hotspot or user wifi connect debug.
    * Added optional 2-pin connectors for wheel breaks. Normally not loaded.
    * Two of the Analog pins now are fed with multiple resistors to allow verification that the 4 power supplies are ok AND to set power-on options.
    * Test points available to force full selftest on powerup if one is grounded
    * There are now 4 locations for power MosFets for ability to improve our current handling ability for both main and motor power.
    * The direct battery power that we used to supply to the large 50 pin jack now comes from after the main power ECB circuit so we can prevent direct shorts.
    * A great many silkscreen improvements have been made.
    * Added 3-pin 12V jack for user to have a place to hook in a fan.
    * Added a couple holes in lower right of board to be useful for mounting plate for different processors than the Raspberry Pi series.


* `5.1` Minor layout change but important. First produced April 2019.  
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

* `4.6` An extremely early pre-release board long before first official shipments.  This was only sent to Alpha and Beta test sites. Not very many units.
