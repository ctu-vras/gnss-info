# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(gnss_info_msgs)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(FILES
  DOP.msg
  Enums.msg
  SatelliteInfo.msg
  SatelliteObservation.msg
  SatellitePosition.msg
  SatelliteSignal.msg
  SatelliteSkyPosition.msg
  SatellitesList.msg
  SatellitesObservations.msg
  SatellitesPositions.msg
  SignalObservation.msg
  SkyView.msg
)

generate_messages(DEPENDENCIES
  geographic_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geographic_msgs geometry_msgs message_runtime std_msgs
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment and with empty rosdep cache,
  # so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_HOME})
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{ROS_HOME}")
  else()
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{HOME}/.ros")
  endif()

  #catkin_lint: ignore_once env_var
  if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
    roslint_custom(catkin_lint "-W2" .)
    roslint_add_test()
  endif()
endif()
