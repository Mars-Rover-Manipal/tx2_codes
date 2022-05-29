cmake_minimum_required(VERSION 3.0.2)
project(urc_2022)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES urc_2022
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

FIND_PACKAGE( OpenCV REQUIRED )                              
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

add_library( planners_header
   include/urc_2022/planners.h
   src/planners.cpp
)
 
add_library( obstacle_header
   include/urc_2022/obstacle.h
   src/obstacle.cpp
)
 
add_library( aruco_header
   include/urc_2022/arucoDetection.h
   src/arucoDetection.cpp
)
 
add_executable(urc_pwm_chala src/socket_pwm_client.cpp )
add_dependencies(urc_pwm_chala ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(urc_pwm_chala
   ${catkin_LIBRARIES}
)

add_executable(urc_chala_node src/urc_chala_node.cpp )
add_dependencies(urc_chala_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} planners_header obstacle_header)

target_link_libraries(urc_chala_node
   ${catkin_LIBRARIES}
   planners_header
   obstacle_header
)
 
add_executable(arucoDetection src/arucoDetection.cpp)
add_dependencies(arucoDetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arucoDetection
    ${catkin_LIBRARIES}
)
  
add_executable(postTravChala src/postTravChala.cpp )
add_dependencies(postTravChala ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(postTravChala
    ${catkin_LIBRARIES}
)
 
 
