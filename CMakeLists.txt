#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

cmake_minimum_required(VERSION 3.8)
project(crospi_default_plugins)

message("-------------- ${PROJECT_NAME} compiling ----------------")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(crospi_core REQUIRED)
find_package(crospi_interfaces REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(etasl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# -- START -- Stuff for triple buffer implementation of boost 1.89

# --------------------------------------------------
# Resolve Boost 1.89 root. This is done because in github, docker, and local home might or might not be defined.
# If BOOST_189_ROOT is defined in the environment, use that installation directory instead (e.g. as done manually for github actions).
# --------------------------------------------------
if(DEFINED ENV{BOOST_189_ROOT} AND NOT "$ENV{BOOST_189_ROOT}" STREQUAL "")
  set(_BOOST_1_89_CUSTOM_ROOT "$ENV{BOOST_189_ROOT}")
  message(STATUS "Using Boost 1.89 from BOOST_189_ROOT=${_BOOST_1_89_CUSTOM_ROOT}")
else()
  if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
    set(_BOOST_1_89_CUSTOM_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
    message(STATUS "Using Boost 1.89 from HOME fallback: ${_BOOST_1_89_CUSTOM_ROOT}")
  else()
    message(FATAL_ERROR
      "Boost 1.89 requested, but neither BOOST_189_ROOT nor HOME is defined")
  endif()
endif()


set(BOOST_ROOT "${_BOOST_1_89_CUSTOM_ROOT}")
set(Boost_INCLUDE_DIR "${_BOOST_1_89_CUSTOM_ROOT}/include")
message(STATUS "CMAKE_PREFIX_PATH before: ${CMAKE_PREFIX_PATH}")
set(CMAKE_PREFIX_PATH "${_BOOST_1_89_CUSTOM_ROOT}" ${CMAKE_PREFIX_PATH})
message(STATUS "CMAKE_PREFIX_PATH after: ${CMAKE_PREFIX_PATH}")
message(STATUS "Using Boost version ${Boost_VERSION}")

find_package(Boost 1.89 REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
# -- END -- Stuff for triple buffer implementation of boost 1.89

message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")

set(CMAKE_CXX_STANDARD 17)

set(src_crospi_default_plugins
    src/simulators/simple_kinematic_simulation.cpp

    src/inputhandlers/topicinputhandler.cpp
    src/inputhandlers/twistinputhandler.cpp
    src/inputhandlers/wrenchinputhandler.cpp
    src/inputhandlers/tfinputhandler.cpp
    src/inputhandlers/poseinputhandler.cpp
    src/inputhandlers/vectorinputhandler.cpp

    src/outputhandlers/jointstateoutputhandler.cpp
    # src/outputhandlers/tfoutputhandler.cpp
    src/outputhandlers/topicoutputhandler.cpp
    src/outputhandlers/topicoutputderivativeshandler.cpp
)

add_library(${PROJECT_NAME} 
            SHARED
            ${src_crospi_default_plugins})


set(rosdependencies
  rclcpp
  rclcpp_lifecycle
  crospi_interfaces
  tf2
  tf2_ros
  pluginlib
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  crospi_core
  # ament_index_cpp
  # Eigen3
)


set(dependencies
    etasl::expressiongraph_context 
    # etasl::expressiongraph_solver_qpoases 
    etasl::expressiongraph_context_lua   
    fmt   
    jsoncpp
    crospi_core::crospi_utils
    crospi_core::crospi_node_utils
    crospi_core::robot_interfacing_utils
)

set(ros_targets 
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
    pluginlib::pluginlib
    ${crospi_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(crospi_core plugins.xml)

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${ament_index_cpp_INCLUDE_DIRS}>  # if needed
)


#The following is done because ROS2 deprecated ament_target_dependencies in favor of target_link_libraries, but the former is still required for older (legacy) ROS2 versions. 
if(DEFINED ENV{ROS_DISTRO} AND (
  "$ENV{ROS_DISTRO}" STREQUAL "foxy" OR
  "$ENV{ROS_DISTRO}" STREQUAL "galactic" OR
  "$ENV{ROS_DISTRO}" STREQUAL "humble" OR
  "$ENV{ROS_DISTRO}" STREQUAL "iron" OR
  "$ENV{ROS_DISTRO}" STREQUAL "jazzy"
))
      ament_target_dependencies(${PROJECT_NAME} ${rosdependencies})
      target_link_libraries(${PROJECT_NAME} ${dependencies})
      message(STATUS "Legacy ROS_DISTRO is $ENV{ROS_DISTRO}, using ament_target_dependencies")
else()
    target_link_libraries(${PROJECT_NAME}  ${dependencies} ${ros_targets})
    message(STATUS "Modern ROS_DISTRO detected ($ENV{ROS_DISTRO}), using target_link_libraries")
endif()

ament_export_dependencies(crospi_core)

ament_export_libraries(
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY json_schemas
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()