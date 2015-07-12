cmake_minimum_required(VERSION 2.8.3)
project(iss_taskboard_gazebo)

find_package(catkin REQUIRED COMPONENTS message_generation gazebo_ros rostest)
find_package(xacro REQUIRED)
find_package(gazebo REQUIRED)

catkin_destinations()

add_message_files(
  DIRECTORY
    msg
  FILES 
    TaskboardPanelA.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES 
    ManipulateNumPad.srv
    ManipulatePowerCover.srv
    ManipulatePowerSwitch.srv
    ManipulateRockerSwitch.srv
    ManipulateSafeToggle.srv
)

#catkin_python_setup()

generate_messages()

set(urdf_files "")

# Xacro files
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/xacro/*.xacro)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
foreach(it ${xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)
  set(dest_dir "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/models/${basename}")
    set(dest_dir "${CMAKE_CURRENT_SOURCE_DIR}/models/${basename}")
  endif()
  message(" processing xacro file :"${basepath}/${basename})
  set(urdf_file "${dest_dir}/${basename}.urdf")
  message("   produce urdf file :"${urdf_file})

  xacro_add_xacro_file(${it} ${urdf_file})
  set(urdf_files ${urdf_files} ${urdf_file})
endforeach(it)

add_custom_target(iss_taskboard_gazebo_files ALL DEPENDS ${urdf_files})

catkin_add_env_hooks(10.${PROJECT_NAME} SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_package(
  INCLUDE_DIRS "plugins/include"
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime gazebo_ros
  DEPENDS gazebo
)

include_directories(
  plugins/include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  plugins/src/ISSTaskboardPanelA.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

catkin_install_python(
  PROGRAMS
    plugins/test/helper.py
    plugins/test/test_numpad.py
    plugins/test/test_power_cover.py
    plugins/test/test_power_switch.py
    plugins/test/test_rocker_switch_a01.py
    plugins/test/test_toggle_a03_a05.py
    plugins/test/test_toggle_a04.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(T
    plugins/test/test_numpad.test
    plugins/test/test_power_cover.test
    plugins/test/test_power_switch.test
    plugins/test/test_rocker_switch_a01.test
    plugins/test/test_toggle_a03_a05.test
    plugins/test/test_toggle_a04.test)
    add_rostest(${T})
endforeach()

install(DIRECTORY
          launch
          models
          plugins
          urdf
          xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY plugins/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

