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
import numpy as np

pub = rospy.Publisher("/imu_data", sensor_msgs.msg.Imu, queue_size=10)
pub_temp = rospy.Publisher("/temperature", sensor_msgs.msg.Temperature, queue_size=10)
count = 1
res = 0

def listener():
	rospy.init_node("imu_parser")
	rospy.Subscriber("/imu_raw", asv_sensors.msg.Serial, callback)
	rospy.spin()

def callback(serial_msg):
	global pub, count,res
	data = [i for i in serial_msg.data.split("\r\n") if len(i)!=0]
	msg = sensor_msgs.msg.Imu()
	msg.header = serial_msg.header
	#msg.header.stamp.nsecs = int(str(msg.header.stamp.nsecs)[:3]+6*"0")
	quat = [-float(data[1][18:]),-float(data[2][5:]),-float(data[3][5:]),-float(data[4][5:])]
	yaw,pitch,roll = tf.transformations.euler_from_quaternion(quat)
	yaw = 2*math.pi-yaw
	if yaw>math.pi:
		yaw -= math.pi
	else:
		yaw += math.pi
	roll = -roll
	pitch = -pitch
	quat = tf.transformations.quaternion_from_euler(yaw,pitch,roll)
	rospy.loginfo("%f %f %f" %(math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
	msg.orientation.w = quat[0]
	msg.orientation.x = quat[1]
	msg.orientation.y = quat[2]
	msg.orientation.z = quat[3]
	msg.linear_acceleration.x = float(data[5][14:])/100
	msg.linear_acceleration.y = float(data[6][5:])/100
	msg.linear_acceleration.z = float(data[7][5:])/100
	msg.angular_velocity.x = -float(data[8][13:])
	msg.angular_velocity.y = -float(data[9][5:])
	msg.angular_velocity.z = -float(data[10][5:])
	temp_msg = sensor_msgs.msg.Temperature()
	z_error = [yaw, pitch, roll]
	expected_z = 9.81*math.cos(z_error[1])*math.cos(z_error[2])
	expected_x = 9.81*math.sin(z_error[1])
	expected_y = 9.81*math.sin(z_error[2])
	if(np.sign(expected_z)!=np.sign(msg.linear_acceleration.z)):
		expected_z *= -1
	if(np.sign(expected_x)!=np.sign(msg.linear_acceleration.x)):
		expected_x *= -1
	if(np.sign(expected_y)!=np.sign(msg.linear_acceleration.y)):
		expected_y *= -1
	#rospy.loginfo(msg.linear_acceleration.z-expected_z)
	msg.linear_acceleration.z = float("%f" % (msg.linear_acceleration.z-expected_z))
	msg.linear_acceleration.x = float("%f" % (msg.linear_acceleration.x-expected_x))
	msg.linear_acceleration.y = float("%f" % (msg.linear_acceleration.y-expected_y))
	#rospy.loginfo(("%f %f %f" % (expected_x, expected_y, expected_z)))
	#res += z_error**2
	#variance = (res/count)**0.5
	#count += 1
	#rospy.loginfo("%.4f %.4f %.4f" % (x_error, y_error, z_error))
	#msg.linear_acceleration_covariance[0] = 0.012
	#msg.linear_acceleration_covariance[4] = msg.linear_acceleration.y
	#msg.linear_acceleration_covariance[8] = msg.linear_acceleration.z
	#rospy.loginfo("%.4f %.4f %.4f" % (msg.linear_acceleration.x-expected_x, msg.linear_acceleration.y-expected_y, msg.linear_acceleration.z-expected_z))
	temp_msg.header = msg.header
	temp_msg.temperature = float(data[11][14:])
	pub.publish(msg)
	pub_temp.publish(temp_msg)

listener()