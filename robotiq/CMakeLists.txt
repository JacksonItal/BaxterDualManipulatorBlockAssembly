cmake_minimum_required(VERSION 2.8.3)
project(robotiq)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  std_msgs
  genmsg
  message_generation
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Service.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Actions.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)


install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
