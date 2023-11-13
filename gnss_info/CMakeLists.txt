# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(gnss_info)

set(CMAKE_CXX_STANDARD 17)

#catkin_lint: ignore_once literal_project_name
find_package(catkin REQUIRED COMPONENTS
  angles
  cras_cpp_common
  geographic_msgs
  gnss_info_msgs
  gnsstk_ros
  roscpp
)

find_package(CURL REQUIRED)
find_package(Eigen3 REQUIRED)
# Provided by cras_cpp_common, produces std::filesystem target using the newest possible implementation
find_package(Filesystem REQUIRED COMPONENTS Final Experimental Boost)
find_package(GNSSTK REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(YAML-CPP REQUIRED)

#catkin_lint: ignore_once literal_project_name
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES satellite_metadata tle_reader
  CATKIN_DEPENDS geographic_msgs gnss_info_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${CURL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_common src/common.cpp)
add_dependencies(${PROJECT_NAME}_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_common PUBLIC ${catkin_LIBRARIES} PRIVATE ${CURL_LIBRARIES} std::filesystem)

add_library(satellite_metadata src/igs_satellite_metadata.cpp)
add_dependencies(satellite_metadata ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(satellite_metadata PUBLIC ${catkin_LIBRARIES} PRIVATE ${PROJECT_NAME}_common gnsstk std::filesystem ${YAML_CPP_LIBRARIES})
if (CXX_FILESYSTEM_TYPE STREQUAL "Boost")
  add_compile_definitions(satellite_metadata PRIVATE -DCXX_FILESYSTEM_IS_BOOST=1)
endif()

add_library(orbital_data src/ethz_satdb_provider.cpp src/orbital_data_manager.cpp src/tle.c)
add_dependencies(orbital_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(orbital_data PUBLIC ${catkin_LIBRARIES} PRIVATE Eigen3::Eigen ${PROJECT_NAME}_common gnsstk jsoncpp_lib std::filesystem)

add_executable(publish_all_satellites nodes/publish_all_satellites.cpp)
target_link_libraries(publish_all_satellites satellite_metadata)

add_executable(publish_sky_view nodes/publish_sky_view.cpp)
target_link_libraries(publish_sky_view orbital_data)

install(TARGETS publish_all_satellites publish_sky_view
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_common orbital_data satellite_metadata
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/sky_plot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY data/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

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
  endif()

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_SRC src/*.cpp include/${PROJECT_NAME}/*.h nodes/*.cpp)

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright,-build/namespaces")
  roslint_cpp(${ROSLINT_SRC})

  roslint_add_test()
endif()
