#!/usr/bin/env python
#Import any external libraries here
import serial, termios
import sys

#Import ROS libraries here
import rospy

#Import ROS messages here
import sensor_msgs.msg
import asv_sensors.msg

pub = rospy.Publisher("/imu_data", sensor_msgs.msg.Imu, queue_size=10)
pub_temp = rospy.Publisher("/temperature", sensor_msgs.msg.Temperature, queue_size=10)

def listener():
	rospy.init_node("imu_parser")
	rospy.Subscriber("/imu_raw", asv_sensors.msg.Serial, callback)
	rospy.spin()

def callback(serial_msg):
	global pub
	data = [i for i in serial_msg.data.split("\r\n") if len(i)!=0]
	msg = sensor_msgs.msg.Imu()
	msg.header = serial_msg.header
	#msg.header.stamp.nsecs = int(str(msg.header.stamp.nsecs)[:3]+6*"0")
	msg.orientation.w = float(data[1][18:])
	msg.orientation.x = float(data[2][5:])
	msg.orientation.y = float(data[3][5:])
	msg.orientation.z = float(data[4][5:])
	msg.linear_acceleration.x = float(data[5][14:])/100
	msg.linear_acceleration.y = float(data[6][5:])/100
	msg.linear_acceleration.z = float(data[7][5:])/100
	msg.angular_velocity.x = float(data[8][13:])
	msg.angular_velocity.y = float(data[9][5:])
	msg.angular_velocity.z = float(data[10][5:])
	temp_msg = sensor_msgs.msg.Temperature()
	temp_msg.header = msg.header
	temp_msg.temperature = float(data[11][14:])
	pub.publish(msg)
	pub_temp.publish(temp_msg)

listener()