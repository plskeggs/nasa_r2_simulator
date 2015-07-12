cmake_minimum_required(VERSION 2.8.3)
project(r2_gazebo)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY
          config
          launch
          media 
          models
          worlds
          xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_env_hooks(10.${PROJECT_NAME} SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
