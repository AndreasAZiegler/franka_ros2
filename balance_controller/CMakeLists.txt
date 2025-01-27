cmake_minimum_required(VERSION 3.5)
project(balance_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(forward_command_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ball_tracker_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(balance_controller SHARED
        src/balance_controller.cpp
        )
target_include_directories(balance_controller PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(balance_controller
	forward_command_controller
  pluginlib
  rclcpp
  ball_tracker_msgs
  )

pluginlib_export_plugin_description_file(controller_interface balance_controller.xml)

install(DIRECTORY include/
        DESTINATION include
        )

install(TARGETS balance_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

ament_export_dependencies(
        controller_interface
        control_msgs
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        trajectory_msgs
)
ament_export_include_directories(
        include
)
ament_export_libraries(
        balance_controller
)
ament_package()
