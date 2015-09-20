^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ubiquity_motor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
