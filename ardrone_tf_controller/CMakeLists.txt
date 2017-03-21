cmake_minimum_required(VERSION 2.8.3)
project(ardrone_tf_controller)

find_package(catkin REQUIRED COMPONENTS
	tf2
	tf2_ros
	roscpp
	rospy	
	cv_bridge
	geometry_msgs
	std_msgs
)




catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ardrone_tf_controller
        CATKIN_DEPENDS tf2 tf2_ros roscpp rospy cv_bridge Toon geometry_msgs std_msgs
        )


include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
  
)

add_library(ardrone_tf_controller src/offset.cpp src/utilities.cpp)

add_executable(single_tf_controller src/ardrone_tf_controller.cpp)
add_executable(frontcamera_autopilot src/frontcamera_autopilot.cpp)
add_executable(bottomcamera_autopilot src/bottomcamera_autopilot.cpp)
add_executable(coop_controller src/coop_controller.cpp)
add_executable(reset_filter src/reset_filter.cpp)


target_link_libraries(single_tf_controller ardrone_tf_controller
   ${catkin_LIBRARIES}
)

target_link_libraries(frontcamera_autopilot ardrone_tf_controller
   ${catkin_LIBRARIES}
)

target_link_libraries(bottomcamera_autopilot ardrone_tf_controller
   ${catkin_LIBRARIES}
)

target_link_libraries(coop_controller ardrone_tf_controller
   ${catkin_LIBRARIES}
)

target_link_libraries(reset_filter ardrone_tf_controller
   ${catkin_LIBRARIES}
)


list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -g  -ftest-coverage -fprofile-arcs")

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
