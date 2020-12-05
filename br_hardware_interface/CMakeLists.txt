cmake_minimum_required(VERSION 3.5)
project(br_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tss_msgs REQUIRED)
find_package(br_hardware_interface_msgs REQUIRED)

add_executable(br_hw_node
        src/br_hw_node.cpp
        src/br_hw_interface.cpp
        src/arduino_com.cpp
        src/gpio.cpp
        src/serial.cpp
        src/unix.cpp
        src/pid.cpp
        )

include_directories(br_hw_node
        include)

ament_target_dependencies(br_hw_node
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        tss_msgs
        br_hardware_interface_msgs
)

install(TARGETS br_hw_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
        DESTINATION share/${PROJECT_NAME}/
        )

install(PROGRAMS
        scripts/kinematic_constraints_test.py
        scripts/introspection_util.py
        DESTINATION lib/${PROJECT_NAME}
      )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
