cmake_minimum_required(VERSION 2.8.3)
project(ardrone_tf_controller)

find_package(catkin REQUIRED COMPONENTS
	tf2
	tf2_ros
	roscpp
	cv_bridge
	geometry_msgs
	std_msgs
	)
	
	
catkin_package(
	CATKIN_DEPENDS tf2 tf2_ros roscpp cv_bridge Toon geometry_msgs std_msgs)


include_directories(
  ${catkin_INCLUDE_DIRS}
  
)

#add_library(DroneController STATIC IMPORTED)
#set_property(TARGET DroneController PROPERTY IMPORTED_LOCATION /home/pulver/catkin_ws4/src/uga_tum_ardrone/src/autopilot/DroneController.h)

#add_executable(ardrone_tf_controller src/ardrone_tf_controller.cpp)
add_executable(send_target_autopilot src/send_target_autopilot.cpp)

#add_dependencies(send_target_autopilot uga_tum_ardrone)


target_link_libraries(send_target_autopilot
   ${catkin_LIBRARIES}
  # DroneController
 )
