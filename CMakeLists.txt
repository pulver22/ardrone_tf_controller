cmake_minimum_required(VERSION 2.8.3)
project(ardrone_tf_controller)

find_package(catkin REQUIRED COMPONENTS
	tf2
	tf2_ros
	roscpp
	)
	
	
catkin_package(
	CATKIN_DEPENDS tf2 tf2_ros roscpp)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ardrone_tf_controller src/ardrone_tf_controller.cpp)


target_link_libraries(ardrone_tf_controller
   ${catkin_LIBRARIES}
 )
