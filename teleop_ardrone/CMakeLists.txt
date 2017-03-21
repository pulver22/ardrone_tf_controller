cmake_minimum_required(VERSION 2.8.3)
project(teleop_ardrone)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	sensor_msgs
	spacenav_node
	roscpp
	joy
	)
	
	
catkin_package(
	CATKIN_DEPENDS geometry_msgs sensor_msgs spacenav_node roscpp joy)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/teleop_ardrone_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_ardrone_spacenav src/teleop_ardrone_spacenav.cpp)

target_link_libraries(teleop_ardrone_spacenav
   ${catkin_LIBRARIES}
   spnav
 )
