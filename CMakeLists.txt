cmake_minimum_required(VERSION 2.8.3)

project(asv_sensors)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nmea_navsat_driver
  lms1xx
  message_generation
  std_msgs
  sensor_msgs
  tf
)

add_message_files(
  FILES
  Serial.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)