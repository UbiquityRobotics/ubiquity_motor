cmake_minimum_required(VERSION 2.8.3)
project(ubiquity_motor)

set(CMAKE_CXX_STANDARD 14) # use C++14

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	message_generation
	roscpp 
	rospy 
	geometry_msgs  
	std_msgs  
	nav_msgs 
	hardware_interface 
	controller_manager
	diagnostic_updater
	dynamic_reconfigure)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
	thread
	atomic)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
generate_dynamic_reconfigure_options(
  cfg/PID.cfg
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Leds.msg
# )

add_message_files(DIRECTORY msg FILES MotorState.msg)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs message_runtime
#  LIBRARIES ubiquity_motor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########


###########
# SERIAL LIBRARY
# Remove once serial is released into noetic
###########

if(APPLE)
	find_library(IOKIT_LIBRARY IOKit)
	find_library(FOUNDATION_LIBRARY Foundation)
endif()

## Sources
set(serial_SRCS
    serial/src/serial.cc
    serial/include/serial/serial.h
    serial/include/serial/v8stdint.h
)
if(APPLE)
    # If OSX
    list(APPEND serial_SRCS serial/src/impl/unix.cc)
    list(APPEND serial_SRCS serial/src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS serial/src/impl/unix.cc)
    list(APPEND serial_SRCS serial/src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS serial/src/impl/win.cc)
    list(APPEND serial_SRCS serial/src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial library
add_library(ur_serial ${serial_SRCS})
if(APPLE)
	target_link_libraries(ur_serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
	target_link_libraries(ur_serial rt pthread)
else()
	target_link_libraries(ur_serial setupapi)
endif()

##H HACK to contain all serial stuff in here
list(APPEND catkin_LIBRARIES ur_serial)
list(APPEND catkin_INCLUDE_DIRS serial/include)

## Mark executables and/or libraries for installation
install(TARGETS ur_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#########
# END SERIAL
##########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
add_library(ubiquity_motor
  src/motor_message.cc src/motor_serial.cc src/motor_hardware.cc
)

# enable message generation for ubiquity_motor
add_dependencies(ubiquity_motor ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(ubiquity_motor
 ${catkin_LIBRARIES}
 ${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(motor_node src/motor_node.cc)

# enable message generation for motor_node
add_dependencies(motor_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(motor_node
 ubiquity_motor
 ${catkin_LIBRARIES}
 ${Boost_LIBRARIES}
)

add_executable(probe_robot src/probe_robot.cc)
target_link_libraries(probe_robot
 ubiquity_motor
 ${catkin_LIBRARIES}
 ${Boost_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(controller)

## Specify libraries to link a library or executable target against
#target_link_libraries(motor_node
 # ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/upgrade_firmware.py
  scripts/test_motor_board.py
  scripts/test_pi_gpio.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ubiquity_motor motor_node probe_robot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	catkin_add_gtest(motor_message_test 
		test/motor_message_test.cc)
	target_link_libraries(motor_message_test ubiquity_motor ${catkin_LIBRARIES})

	catkin_add_gtest(motor_serial_test 
		test/motor_serial_test.cc)
	target_link_libraries(motor_serial_test ubiquity_motor util ${catkin_LIBRARIES})

	catkin_add_gtest(shared_queue_test 
		test/shared_queue_test.cc)
	target_link_libraries(shared_queue_test ubiquity_motor ${catkin_LIBRARIES})

	add_rostest_gtest(motor_param_test 
		test/rostest/motor_parameters.test 
		test/motor_parameter_test.cc )
	target_link_libraries(motor_param_test ubiquity_motor ${catkin_LIBRARIES})

	add_rostest_gtest(motor_hardware_test 
		test/rostest/motor_hardware.test 
		test/motor_hardware_test.cc )
	target_link_libraries(motor_hardware_test ubiquity_motor util ${catkin_LIBRARIES})

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


