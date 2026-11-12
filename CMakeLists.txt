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

cmake_minimum_required(VERSION 3.10)
project(crospi_default_plugins)

message("-------------- ${PROJECT_NAME} compiling ----------------")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# crospi_core exports all its dependencies (rclcpp, etasl, Eigen3, pluginlib, tf2, crospi_interfaces, ...),
# so they are found and linked through crospi_core::crospi_core.
find_package(crospi_core REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} SHARED
    src/simulators/simple_kinematic_simulation.cpp
    src/robotdrivers/ros_topics_driver_crospi.cpp

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

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC crospi_core::crospi_core)

pluginlib_export_plugin_description_file(crospi_core plugins.xml)

ament_export_dependencies(crospi_core)

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
