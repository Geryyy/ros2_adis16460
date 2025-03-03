cmake_minimum_required(VERSION 3.8)
project(imu_cpp)

# Enable extra compile options for GCC/Clang compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  
  # Add native optimization for GCC/Clang
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-march=native)
  endif()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(WIRINGPI_LIB wiringPi)

# Add executable
add_executable(imu_publisher src/imu_publisher_all.cpp)

# Manually link the libraries and include paths
target_include_directories(imu_publisher PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${se
