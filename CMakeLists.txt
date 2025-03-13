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
# find_package(concurent REQURIED)

# Add executable
add_executable(imu_publisher src/imu_publisher_all.cpp)

# Manually link the libraries and include paths
target_include_directories(imu_publisher PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add include path for concurrentqueue
include_directories(${CMAKE_SOURCE_DIR}/src/concurrentqueue)



# Link dependencies with proper libraries
target_link_libraries(imu_publisher
  PRIVATE
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${WIRINGPI_LIB}
)

# Set required C and C++ standard versions
target_compile_features(imu_publisher PUBLIC c_std_99 cxx_std_17)

# Set release build type flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# Install targets
install(TARGETS imu_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Run linting if testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
