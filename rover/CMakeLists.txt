cmake_minimum_required(VERSION 3.0.2)
project(rover)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rviz
  tf
  urdf
  xacro
  roscpp
  pcl_conversions
  pcl_ros
  dynamic_reconfigure
  geometry_msgs
  message_generation
)


add_message_files(
  FILES
  pcl_processing.msg
  Quaternion.msg
  Point.msg
  Centroid.msg
  obstacle_info.msg
  free_sectors.msg
  num_clusters.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover
#  CATKIN_DEPENDS roscpp rospy std_msgs
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(pcl_testing_obb src/pcl_testing_obb.cpp)
add_dependencies(pcl_testing_obb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_testing_obb
    ${catkin_LIBRARIES}
  )

add_executable(pclProcessing src/pclProcessing.cpp src/pclProcessing.h)
add_dependencies(pclProcessing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pclProcessing
    ${catkin_LIBRARIES}
  )
  
add_executable(pclProcessingforRealsense src/pclProcessingforRealsense.cpp)
add_dependencies(pclProcessingforRealsense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pclProcessingforRealsense
    ${catkin_LIBRARIES}
  )

add_executable(pclProcessingforLidar src/pclProcessingforLidar.cpp src/pclProcessing.h)
add_dependencies(pclProcessingforLidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pclProcessingforLidar
    ${catkin_LIBRARIES}
  )

#add_executable(globalPlanner src/globalPlanner.cpp src/globalPlanner.h)
#add_dependencies(globalPlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(globalPlanner
#    ${catkin_LIBRARIES}
#  )

add_executable(pclProcessingforSimulation src/pclProcessingforSimulation.cpp src/pclProcessing.h)
add_dependencies(pclProcessingforSimulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pclProcessingforSimulation
    ${catkin_LIBRARIES}
  )

# add_executable(pclProcessingforSimulation-IRC src/pclProcessingforSimulation-IRC.cpp src/pclProcessing.h)
# add_dependencies(pclProcessingforSimulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(pclProcessingforSimulation
#     ${catkin_LIBRARIES}
#   )

add_executable(pclProcessingforLidar2 src/temp.cpp)
add_dependencies(pclProcessingforLidar2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pclProcessingforLidar2
    ${catkin_LIBRARIES}
  )
  
       



