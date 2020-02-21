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
find_package(platform_driver_ethercat_lib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# this should not be necessary as soem is not a direct dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(SOEM REQUIRED IMPORTED_TARGET soem)

# add executable
file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
add_executable(${PROJECT_NAME} ${MY_SOURCES})

# specify include paths and dependencies
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_link_libraries(${PROJECT_NAME} yaml-cpp PkgConfig::SOEM)
ament_target_dependencies(${PROJECT_NAME} platform_driver_ethercat_lib rclcpp rclcpp_lifecycle geometry_msgs rover_msgs sensor_msgs)

# install target at destination
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# set capabilities of executable for raw network interface access
install(CODE "execute_process(COMMAND sudo /sbin/setcap CAP_NET_RAW=ep \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${PROJECT_NAME}\")")

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
