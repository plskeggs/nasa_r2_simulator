cmake_minimum_required(VERSION 2.8.3)
project(gazebo_taskboard)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED message_generation std_msgs)
find_package(gazebo REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TaskboardPanelA.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ManipulateNumPad.srv
  ManipulatePowerCover.srv
  ManipulatePowerSwitch.srv
  ManipulateRockerSwitch.srv
  ManipulateSafeToggle.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES r2_gazebo
   CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
#//////////////////////////////////////////////////////////***Gazebo
include(FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Declare a cpp library
add_library(gazebo_taskboard
  src/${PROJECT_NAME}/GazeboTaskboard.cpp
)

add_dependencies(gazebo_taskboard ${catkin_EXPORTED_TARGETS} ${gazebo_taskboard_EXPORTED_TARGETS})