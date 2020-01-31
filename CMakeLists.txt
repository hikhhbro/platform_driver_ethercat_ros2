cmake_minimum_required(VERSION 3.5)
project(platform_driver_ethercat)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)            # hard codes link paths into RUNPATH of installed binary to avoid need for LD_LIBRARY_PATH
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")  # use RPATH instead of RUNPATH as RUNPATH is not searched for indirect dependencies

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(SOEM REQUIRED IMPORTED_TARGET soem)

file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)

add_executable(platform_driver_ethercat ${MY_SOURCES})
target_include_directories(platform_driver_ethercat PRIVATE src)
target_link_libraries(platform_driver_ethercat PkgConfig::SOEM Eigen3::Eigen yaml-cpp)
ament_target_dependencies(platform_driver_ethercat rclcpp rclcpp_lifecycle geometry_msgs rover_msgs sensor_msgs)

install(TARGETS
  platform_driver_ethercat
  DESTINATION lib/${PROJECT_NAME})

install(CODE "execute_process(COMMAND sudo /sbin/setcap CAP_NET_RAW=ep \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/platform_driver_ethercat\")")

ament_package()
