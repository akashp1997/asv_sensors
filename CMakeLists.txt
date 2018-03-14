cmake_minimum_required(VERSION 2.8.3)
project(asv_sensors)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nmea_navsat_driver
  lms1xx
  robot_pose_ekf
)