cmake_minimum_required(VERSION 3.8)
project(etasl_ros2_default_plugins)

message("-------------- ${PROJECT_NAME} compiling ----------------")





# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(ament_index_cpp REQUIRED)

find_package(pluginlib REQUIRED)
find_package(etasl_ros2 REQUIRED)
find_package(etasl_interfaces REQUIRED)


find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(etasl REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(pugixml REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(Sphinx REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# -- start --Stuff for triple buffer implementation of boost 1.89
set(BOOST_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local/lib/boost/1.89.0" ${CMAKE_PREFIX_PATH})
find_package(Boost 1.89 REQUIRED)
# find_package(Boost 1.89 REQUIRED COMPONENTS lockfree)
message(STATUS "Using Boost version ${Boost_VERSION}")
# -- end -- Stuff for triple buffer implementation of boost 1.89


# # Add this at the top of your CMakeLists.txt file
# find_package(pybind11 REQUIRED)

# # Add this before the pybind11_add_module calls
# # include_directories(${pybind11_INCLUDE_DIRS})

message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")


include_directories(
   include
  #  $ENV{HOME}/.local/lib/boost/1.89.0/include  # <- point directly to Boost include
   ${Boost_INCLUDE_DIRS}
)


set(CMAKE_CXX_STANDARD 17)

# include_directories(include ${LUA_INCLUDE_DIR})

set(src_etasl_ros2_default_plugins
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
    #Add all plugin src files here
)

add_library(${PROJECT_NAME} 
            SHARED
            ${src_etasl_ros2_default_plugins})


set(rosdependencies
  rclcpp
  rclcpp_lifecycle
  etasl_ros2
  pluginlib
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  etasl_interfaces
  # ament_index_cpp
  # Eigen3
)


set(dependencies
    etasl::expressiongraph_context 
    # etasl::expressiongraph_solver_qpoases 
    etasl::expressiongraph_context_lua   
    fmt   
    jsoncpp
    # etasl_task_utils

)


pluginlib_export_plugin_description_file(etasl_ros2 plugins.xml)

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${ament_index_cpp_INCLUDE_DIRS}>  # if needed
)

target_link_libraries(${PROJECT_NAME} ${dependencies})
ament_target_dependencies(${PROJECT_NAME} ${rosdependencies})  


ament_export_dependencies(etasl_ros2)

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

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME})


# ament_export_include_directories(
#   include
# )


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
