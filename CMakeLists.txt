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

add_executable(ardrone_tf_controller src/ardrone_tf_controller.cpp)
add_executable(frontcamera_autopilot src/frontcamera_autopilot.cpp)
add_executable(bottomcamera_autopilot src/bottomcamera_autopilot.cpp)




target_link_libraries(frontcamera_autopilot 
   ${catkin_LIBRARIES}
 )

 
target_link_libraries(ardrone_tf_controller
${catkin_LIBRARIES}

)

target_link_libraries(bottomcamera_autopilot 
   ${catkin_LIBRARIES}
 )
