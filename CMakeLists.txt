cmake_minimum_required(VERSION 2.8.3)
project(mavctr)

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  pluginlib
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  angles
  libmavconn
  message_generation
)
add_message_files(
  FILES
  acc.msg
  global_position.msg 
  rc_channels.msg
  attitude_quad.msg
  local_position.msg
  velocity.msg
  battery.msg
  State.msg
  RadioStatus.msg
  imu.msg
  Joy.msg
  position_target_local.msg
  position_target_global.msg
  vfr_hud.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavros
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater pluginlib roscpp tf2_ros  libmavconn message_runtime
  DEPENDS Boost
)


###########
## Build ##
###########


include_directories(include)

add_subdirectory(mavctr)

###########
##install##
###########

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
