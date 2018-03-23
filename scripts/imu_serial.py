#!/usr/bin/env python
#Import any external libraries here
import sys, serial

#Import ROS libraries here
import rospy

#Import ROS messages here
import std_msgs.msg
import asv_sensors.msg
import time

def talker():
	rospy.init_node("imu_serial_node")
	port_name = sys.argv[1].split("=")[1]
	port = serial.Serial(port_name, baudrate=115200)
	pub = rospy.Publisher("/imu_raw", asv_sensors.msg.Serial, queue_size=1000)
	while True:
		port.write("quaternion di. accelp di. gyrop di. temperature di.\r\n")
		lines = []
		while True:
			line = port.readline()
			if "OK" in line:
				msg = asv_sensors.msg.Serial()
				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = "base_link"
				msg.data = "".join(lines)
				pub.publish(msg)
				lines = []
				break
			else:
				lines.append(line)

talker()