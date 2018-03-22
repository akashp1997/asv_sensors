#!/usr/bin/env python

import rospy
import numpy
import nmea_msgs.msg

import sensor_msgs.msg
import geometry_msgs.msg

imu_data = sensor_msgs.msg.Imu()

pub = rospy.Publisher("/fix", sensor_msgs.msg.NavSatFix, queue_size=10)
pub_vel = rospy.Publisher("/vel", geometry_msgs.msg.TwistStamped, queue_size=10)
def listener():
	rospy.init_node("gps_parser")
	rospy.Subscriber("/gps_serial", nmea_msgs.msg.Sentence, callback)
	rospy.Subscriber("/imu_data", sensor_msgs.msg.Imu, imu_callback)
	rospy.spin()

def callback(data):
	if data.sentence.startswith("$PGRMF"):
		msg = sensor_msgs.msg.NavSatFix()
		msg.header = imu_data.header
		line = data.sentence.split(",")
		msg.latitude = int(float(line[6])/100)+(float(line[6])%100)/60
		if line[7]=="S":
			msg.latitude *= -1
		msg.longitude = int(float(line[8])/100)+(float(line[8])%100)/60
		if line[9]=="W":
			msg.latitude *= -1
		msg.altitude = numpy.nan
		msg.status.status = int(line[11])-1
		msg.status.service = 1
		msg.position_covariance_type=3
		msg.position_covariance = [0]*9
		for i in [0,4,8]:
			msg.position_covariance[i] = 1

		#rospy.loginfo(msg)
		pub.publish(msg)
	elif data.sentence.startswith("$PGRMV"):
		msg = geometry_msgs.msg.TwistStamped()
		msg.header = imu_data.header
		line = data.sentence.split(",")
		if line[1]=="":
			msg.twist.linear.x = 0
		else:
			msg.twist.linear.x = float(line[1])
		if line[2]=="":
			msg.twist.linear.y = 0.0
		else:
			msg.twist.linear.y = float(line[2])
		if line[1]!="" and line[2]!="":
			#rospy.loginfo(msg)
			pub_vel.publish(msg)

def imu_callback(data):
	global imu_data
	imu_data = data

listener()