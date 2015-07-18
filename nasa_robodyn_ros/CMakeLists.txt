cmake_minimum_required(VERSION 2.8.3)
project(nasa_robodyn_ros)

find_package(catkin REQUIRED COMPONENTS roscpp nasa_robodyn_controllers_core nasa_r2_common_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nasa_robodyn_controllers_core nasa_r2_common_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/R2RosArbiter.cpp
  src/R2RosTrajectoryManager.cpp
  src/R2RosTreeId.cpp
  src/R2RosGripper.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(robodyn src/robodyn_node.cpp)
target_link_libraries(robodyn ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} robodyn
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
