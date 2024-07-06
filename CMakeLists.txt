cmake_minimum_required(VERSION 2.8.3)
project(ros2_wj_716N_lidar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

message(STATUS "${LIB_NAME} Ros version : $ENV{ROS_DISTRO}")
if ($ENV{ROS_DISTRO} MATCHES "iron")
  add_definitions(-DUSE_IRON_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
elseif ($ENV{ROS_DISTRO} MATCHES "humble")
  add_definitions(-DUSE_HUMBLE_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
elseif ($ENV{ROS_DISTRO} MATCHES "galactic")
  add_definitions(-DUSE_GALACTIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
elseif ($ENV{ROS_DISTRO} MATCHES "foxy")
  add_definitions(-DUSE_FOXY_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
elseif ($ENV{ROS_DISTRO} MATCHES "eloquent")
  add_definitions(-DUSE_ELOQUENT_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
elseif ($ENV{ROS_DISTRO} MATCHES "dashing")
  add_definitions(-DUSE_DASHING_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
elseif ($ENV{ROS_DISTRO} MATCHES "noetic")
  add_definitions(-DUSE_ROS_NORTIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE ON)
elseif ($ENV{ROS_DISTRO} MATCHES "melodic")
  add_definitions(-DUSE_ROS_MELODIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE ON)
endif()

include_directories(
    include
)

file(GLOB main_srcs 
    src/*.cpp
)

if(USE_ROS1_MIDDLE_PRICE)
######################## 使用ROS1 作为中间件 ###########################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
    cfg/wj_716N_lidar.cfg

  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wj_716N_lidar
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${main_srcs})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#######################################################################
else()
######################## 使用ROS2 作为中间件 ###########################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
    ${roscpp_INCLUDE_DIRS}
    ${roslib_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${main_srcs})
target_link_libraries(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
		DESTINATION share/${PROJECT_NAME}
)

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
#######################################################################
endif()
