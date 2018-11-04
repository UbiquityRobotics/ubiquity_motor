^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ubiquity_motor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
