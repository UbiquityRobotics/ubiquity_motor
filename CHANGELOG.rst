^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ubiquity_motor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.14.1 (2022-03-19)
-------------------
* bugfix: reverted input back to py2 to fix #169
* Contributors: Janez Cimerman

0.14.0 (2022-03-09)
-------------------
* made upgrade_firmware be run with py2 because `#149 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/149>`_
* Rename v43 to no longer be beta2wd in the name
* Update4wd and gear ratio for noetic (`#160 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/160>`_)
* Updated readme to include new parameters and topics.
* Adding clarity in comments only to our interface to controller_manager and thus DiffDriveController
* Fix issue of i2c routine and a left over git merge marker failed make
* Update motor_hardware_test for changes to readInputs and MotorHardware interfaces.
* Major merge of 4WD features to Magni 2wd. This is include and message files
* Major merge of 4WD features to Magni 2wd. This is initial motor_hardware source changes.
* Major merge of 4WD features to Magni 2wd. This is initial motor_node source changes.
* Adding v43_20210829_beta2wd_enc.cyacd for testing
* Contributors: Janez Cimerman, Mark Johnston, Rohan Agrawal

0.13.2 (2021-07-27)
-------------------
* Added ros noetic github action
* Install the shared lib for our custom serial
* Check for mininum hardware version before reading option switch
* Contributors: Rohan Agrawal

0.13.1 (2021-07-22)
-------------------
* Migration to Python3.8
* upgrade_firmware.py migrated
* test_velocity.py migrated
* test_pi_gpio.py migrated
* test_motor_board.py migrated
* Contributors: Rohan Agrawal, Teo Podobnik

0.13.0 (2021-07-20)
-------------------
* Depend on python3 version of requests
* Hack to vendor serial because it is still unreleased on noetic
* Merge commit '32b9fad54bb1e54b9fa61f6b5c51f5ec4bf6d547' as 'serial'
* Updated unit tests with new changes so that they pass
* Adding beta for v41 firmware
* Add tic interval topics and fix bat volt readings and enhance test_velocity tool (`#126 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/126>`_)
* Cleanup of the README for better formating and clarity for tick intervals
* Removed a diagnostic cap on interval of 2000 that was for testing
* Adding system_control command of speed_control  disable and enable.  Moving the state for both speed control AND earlier added motor control to disable mcb control by motor node both into NodeParams class.    Also move the defines for topic name and both commands into motor_parameters.h which is better place since now mcbSpeedControl and mcbMotorControl are now members of NodeParams which is also defined in motor_parameters.h (`#122 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/122>`_)
* Add query of pid parameters. Fix issue with the S command where we had to use a second carrage return to enter value before.   Some help menu cleanup
* Updating to include v40 for latest Magni release candidate
* Fix init of wheel_type for case of no yaml entry for wheel type so we will use default of standard in that case (`#117 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/117>`_)
* Merge pull request `#115 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/115>`_ from UbiquityRobotics/addSystemControlForMcbDisable
  Add system control for mcb disable
* First working disable of MCB serial to allow firmware upgrade then re-enable motor node.  This has been tested and we can disable - upgrade - re-enable.
* Now able to have motor_node receive on system_power topic a   motor_control disable and it allows firmware upgrade.  Next the more tricky part is to re-open and re-init MCB.  The final piece is to reopen serial port as the code to re-init mcb is in this branch already
* More incremental changes to better show battery too low
* Working version to allow disable and re-enable of MCB on the new /system_control topic.
* src/motor_node.cc
* Improved the comments for clearer usage info
* Adding arduino nano program for simple MCB diagnostics to be used without any Raspberry Pi connected.
* Adding firmware folder. No impact to any code and is for ability to have revisions available to users to try different versions as required.
* Adding firmware rev v39 and v37 for firmware descriptions
* Adding board revision hardware changes done for rev 5.3 MCB
* Merge pull request `#109 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/109>`_ from UbiquityRobotics/i2c-lock
  Open i2c device for read-only.
* Open device for read-only.
* Reset and reinitialize MCB if we see it has itself been reset (`#103 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/103>`_)
  Have re-tested today since it has been a while since review start.   Hit MCB reset switch and we re-init from host params.
  It has been intentially set to an infrequent check because it is likely a very rare event and I wanted to keep I2C traffic to a minimum.
* Merge pull request `#104 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/104>`_ from UbiquityRobotics/i2c-lock
  Removing the write() call to PCF8574 in a I2C communication
* Only read() call.
* Arbitrary slave register.
* Compiled version
* Re-added goto.
* Some changes
* WriteRead in one go.
* Adding wheel joint state velocity to fix ubiquity_motor issue `#59 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/59>`_ (`#101 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/101>`_)
* Fix wheel direction issues (`#100 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/100>`_)
* Fix15sec deadman hickup (`#99 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/99>`_)
* Contributors: Mark Johnston, MoffKalast, Rohan Agrawal, Teodor, 

0.12.0 (2020-07-13)
-------------------
* Fixes ODOM bug and modifies way we set wheel type to safer system. (`#87 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/87>`_)
* Sync up registers with v37 firmware. This adds many thing like system_events pid control selftest and thin wheel and low voltage level.
* A workaround for issue 83.  Insert a 2 second delay prior to serial port setup.  Tests so far show this seems to avoid the issue where some OS components sets the serial port to 115200. The odd thing about the defect is stty still shows it as 38.4k but it is in fact 115.2k baud
* Contributors: Mark Johnston

0.11.0 (2020-06-12)
-------------------
* Fixed unit test error
* Sync up registers with v37 firmware. This adds many thing like system_events pid control selftest and thin wheel and low voltage level. Our main need here is thin wheels. Also add system_info script to be used for simplier field state info gathering on a magni
* Continued mod to add more recent firmware changes into host side.  Have added max_pwm to dynamic config.   Made some changes to system_info.py info tool
* Fix a repetitive logs for firmware date and version. Main new code is for support of v37 firmware registers and features.  Not function complete yet for all changes
* Putting into action mcb reset detection from system_event register and reading of mcb option switch so it can be sent back down to the MCB since mcb cannot read the onboard option register itself.
* First commit of function complete code for support of wheel type in base.yaml, notification of MCB reset conditions, reading I2C option switch and pushing it to MCB register as the main new additions
* Adding updates to firmware and hardware rev description
* Better error messages for common errors
* Adding v35 information for firmware revisions
* Contributors: Mark Johnston, Rohan Agrawal

0.10.0 (2019-11-10)
-------------------
* Support for higher resolution odometry (firmware v35+ required)
* Tool to verify odometry consistency
* Support for target velocity term in the PID (firmware v35+ required)
* Firmware update script now supports local firmware files
* Firmware update script can use different serial port
* Support for reading motor controller version from I2C
* Safer E-STOP behavior (MCB 5.0+)
* Publishing the state of the estop switch
* Improved documentation 
* Improvements to the testing scripts

* Contributors: Alexander Sergeenko, David Crawley, Mark Johnston, Rohan Agrawal

0.9.0 (2019-04-03)
------------------
* Allow selecting what firmware version to download
* Adding max motor forward and reverse speeds and max pwm settings all the way from ROS parameters to being pushed to the controller board.
* Analyze information to create diagnostics statuses
* Major update to test_motor_board.py that accepts greatly improved parameter read and set as well as ability to specify com port device to be used
* Adds support for set of hw rev and for pre rev 5.0 estop threshold
* Contributors: Mark Johnston, Rohan Agrawal

0.8.0 (2019-01-01)
------------------
* Added firmware loading tool
* Added misc testing scripts
* Don't die when communication not working, only print error
* Use std mutex/atomic instead of boost
* Reduce print level on integral/pid limits
* Contributors: Rohan Agrawal

0.7.1 (2018-06-16)
------------------
* new pid params
* Contributors: Rohan Agrawal

0.7.0 (2018-04-15)
------------------
* Add script to probe the robot for information
* Add Serial Protocol Documentation
  Fixes `#33 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/33>`_
* Add ROS API documentation (`#32 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/32>`_)
  * Add API documentation
  * Remove unused serial_loop_rate variable
* Contributors: Jim Vaughan, Rohan Agrawal

0.6.1 (2017-11-12)
------------------
* Reset controller when time jumps (`#31 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/31>`_)
  Reset the controller and zero commanded velocity an unexpected time change occurs (such as by NTP).  This prevents unexpected robot motion.
* Contributors: Jim Vaughan, Rohan Agrawal

0.6.0 (2017-09-15)
------------------
* Publish battery voltage messages (`#29 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/29>`_)
  Added battery status message calibrated on 4.4 board serial no 450
* Contributors: Jim Vaughan, Rohan Agrawal, David Crawley

0.5.2 (2017-05-06)
------------------
* Remove debug topics (`#25 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/25>`_)
  * Remove debug topics
  * Remove tests of debug registers
* Merge pull request `#22 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/22>`_ from UbiquityRobotics/suppresserrorsatstartup
  Supress some potentially confusing warnings
* Increase error_threshold
* Merge pull request `#23 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/23>`_ from UbiquityRobotics/fix_acceleration_limits
  Fix computaion of elapsed time so that it is +ve
* Fix computaion of elapsed time so that it is +ve
* Supress some potentially confusing warnings
* Clean out serial loop (`#20 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/20>`_)
  * Transmit the the same thread caller, not in serial thread
  * go back to debug on tranmissions
  * Use smarter waits and reads in reading thread
  * Get rid of serial loop rate
  * Reformat
* Contributors: Jim Vaughan, Rohan Agrawal

0.5.1 (2017-03-04)
------------------
* Reduce flakey-ness of the tests
* Try to get firmware version, throw after 10 tries
* Code cleanup
* Use fixed sized arrays (not vectors) where they make sense
* Use a seperate shared_queue class
* Performance improvements
* Contributors: Rohan Agrawal

0.5.0 (2016-09-04)
------------------
* **NOTE:** This version drops support for firmware versions before 24
* Use new 8-byte serial protocol
* Add support for using dynamic_reconfigure to change PID parameters
* Add support for setting the deadman timer via a parameter
* Add support for debug registers, do enable better firmware diagnostics
* Add support for limit reached warnings from firmware
* Improved testing, more coverage and cleaner tests
* Have motor_node explicitly return an exit code
* Reduce memory allocations caused by resizing vectors
* Use size_t instead of int for iterating
* Contributors: Rohan Agrawal, Jim Vaughan

0.4.1 (2016-04-09)
------------------
* add support for firmware version 19
* add support for 0xDD (checksum) error response
* Make variable name for rejected bytes 'rejected'
* Reduce memcopy-ing
* Contributors: Rohan Agrawal

0.4.0 (2016-03-08)
------------------
* Cleanup deps, have motor_node be linked to shared lib
* Update Copyright Dates
* Removed old motor_unit_test
* Moved motor_message_test
* Make the serial thread loop at the passed in value instead of always 1000
* Add interruption point to Serial Thread
* Comment out serial tests
* Added motor_serial_tests
* Always print firmware version
* fix up code that checks a firmware version response
* Using Async Spinner instead of roscontrol thread
* more command grouping
* reduced unnecessary output locking
  using bool method like tony did with input
* reduce locking by grouping commands to send together
* Contributors: Rohan Agrawal

0.3.2 (2015-11-28)
------------------
* Many fixes for bad odometery, more robust serial protocol
* Add code to speed up serial. Major improvements in latency to the motor board
* Contributors: atp42

0.3.1 (2015-10-12)
------------------
* fixed install rules for Cmake
* added license to test code
* Contributors: Rohan Agrawal

0.3.0 (2015-09-20)
------------------
* Remove annoying debug print
* Fix numerous PID issues. First, add velocity reporting. Second, fix 10x unit error between specified velocity and actual. Third, make PID parameter changes actually world.
* actually calling the function now
* added pid params (hopefully)
* added more unit tests
* added some more unit test coverage, b/c I spent half an hour on an avoidable wild goose chase
* updated unit tests
* renamed motor command to motor message
* Contributors: Jim Vaughan, Rohan Agrawal

* Updated package.xml with new dependencies, bumped version number
* updated travis button
* Merge refactoring branch into indigo-devel
* cleaned up some stuff, and got odometery running
* added odom, and changed loop rates
* using the correct tics to radians, and now at 20hz
* cleaned up travis file
* forgot to remove bad include
* moved control loop to seperate thread to make it work
* converted branch to the indigo-devel code
* added missing ubiquity_motor.cpp
* changed logging from the output speeds to the input speeds
* whoops, fotgot to remove typedef for chrono
* removed boost_chrono dependency, less dependencies is better right
* explictly apt-get boost-chrono
* manually installing boost in travis
* changed travis notfications for slack
* Fix issue where serial data wouldn't print
  Using a pointer for the motors object in the Motor Serial class, this allows for the initalization to be in the constructer.
* added slack integration to travis
* add slack intergration to travis
* working on making diff_drive_controller work
* fixed boost expection error
* Print ros_error on catch for better debugging
* worked on serial thread loop
* fix test
* more exception handling in thread
* added baud rate switch/case to prevent invalid bauds
* basic serial thread working
* change testsuite naming to ubiquity_motor_CLASS
* added catkin_make to travis because build errors fixed
* added mutexed add and get command functions
* added test cases for invalid type/register
* put header ifndef in motor_serial header
* redid motor_serial header with boost threads and std::queues
* added enum checking to getters and setters
* added deserialize verification
* started added comments to the code
* removed unnessary imports
* added incorrect checksum test case
* added deserialize funtionality and test
* added serialize and checksum functions
* fix typo in travis file
* changed travis config to run the tests
* added tests for motor command class
* Started adding unit tests
* fixed enum scoping errors
* fix wierd git problem
* renamed source files to follow ROS standard practices
* removed old files that we are not going to use
* changed motor command class to use new serial protocol spec
* testing build
* Merge pull request `#2 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/2>`_ from jim-v/hydro
  Added child_frame_id to odom messages.
* Added child_frame_id to odom messages.
* Changed MotorDriver to MotorHardware
* added missing dependencies to package.xml to fix build errors
* Update README.md
* added header for motor driver class
* Merge pull request `#1 <https://github.com/UbiquityRobotics/ubiquity_motor/issues/1>`_ from jim-v/hydro
  Added covariance to the outgoing odom messages.
* Added covariance to the outgoing odom messages.
* removed old node files
* added serial dependency to package.xml
* Fixed build errors and removed Ubiquity prefix from classes
* changed travis branch to refactoring
* added serial reader thread and callback
* added BSD license to crc8 files
* added UbiquityMotorSerial class
* created UbiquityMotorCommand class
* add build status to README
* changed node graph name
* added travis configureation file
* Contributors: Jim Vaughan, Kurt Christofferson, Rohan Agrawal

0.1.0 (2015-02-14)
------------------
* updated verstion number
* Added install rule
* Initial Commit
* Contributors: Rohan Agrawal
