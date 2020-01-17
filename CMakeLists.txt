cmake_minimum_required(VERSION 3.5)
project(platform_driver_ethercat)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)

pkg_check_modules(SOEM REQUIRED IMPORTED_TARGET soem)

file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)

add_executable(platform_driver_ethercat ${MY_SOURCES})
target_include_directories(platform_driver_ethercat PRIVATE src)
target_link_libraries(platform_driver_ethercat PkgConfig::SOEM Eigen3::Eigen)
ament_target_dependencies(platform_driver_ethercat rclcpp)

install(TARGETS
  platform_driver_ethercat
  DESTINATION lib/${PROJECT_NAME})

ament_package()

