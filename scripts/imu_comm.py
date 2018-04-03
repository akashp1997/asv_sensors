#!/usr/bin/env python
#Import any external libraries here
import serial, termios
import sys
import math

#Import ROS libraries here
import rospy
import tf.transformations

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
	quat = [float(data[1][18:]),float(data[2][5:]),float(data[3][5:]),float(data[4][5:])]
	msg.orientation.w = quat[0]
	msg.orientation.x = quat[1]
	msg.orientation.y = quat[2]
	msg.orientation.z = quat[3]
	msg.linear_acceleration.x = float(data[5][14:])/100
	msg.linear_acceleration.y = float(data[6][5:])/100
	msg.linear_acceleration.z = float(data[7][5:])/100
	msg.angular_velocity.x = float(data[8][13:])
	msg.angular_velocity.y = float(data[9][5:])
	msg.angular_velocity.z = float(data[10][5:])
	temp_msg = sensor_msgs.msg.Temperature()
	z_error = tf.transformations.euler_from_quaternion(quat)
	expected_z = 9.81*math.cos(z_error[1])*math.cos(z_error[2])
	error = (msg.linear_acceleration.z-expected_z)/expected_z
	msg.linear_acceleration_covariance[::4] = [error]*3
	if msg.linear_acceleration.x<0:
		msg.linear_acceleration_covariance[0] *= -1
	if msg.linear_acceleration.y<0:
		msg.linear_acceleration_covariance[4] *= -1
	if msg.linear_acceleration.z<0:
		msg.linear_acceleration_covariance[8] *= -1
	temp_msg.header = msg.header
	temp_msg.temperature = float(data[11][14:])
	pub.publish(msg)
	pub_temp.publish(temp_msg)

listener()