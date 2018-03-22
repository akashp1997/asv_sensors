#!/usr/bin/env python
#Import any external libraries here
import serial
import subprocess
import time

#Import ROS libraries here
import rospy

#Import ROS messages here
import sensor_msgs.msg
from asv_sensors.srv import *

class ImuSensor(object):
	def __init__(self, baud=115200, topic_name="/imu_data", temp_topic="/temperature", rate=100):
		self.imu_data = sensor_msgs.msg.Imu()
		self.temp_data = sensor_msgs.msg.Temperature()
		self.baud=baud
		self.port = None
		self.pub = rospy.Publisher(topic_name, sensor_msgs.msg.Imu, queue_size=10)
		self.temp_pub = rospy.Publisher(temp_topic, sensor_msgs.msg.Temperature, queue_size=10)
		self.rate = rospy.Rate(rate)

	def get_port(self):
		rospy.wait_for_service("serial_detect")
		try:
			port_detect = rospy.ServiceProxy('serial_detect', port_update)
			if self.port==None:
				resp = port_detect(0, "")
			else:
				resp = port_detect(0, self.port.port)
			if resp.found==True:
				self.port = serial.Serial(resp.port_name, baudrate=115200)
		except rospy.ServiceException:
			rospy.logfatal("Service call failed.")

	def publish(self):
		while not rospy.is_shutdown():
			try:
				self.get_data()
				self.pub.publish(self.imu_data)
				self.temp_pub.publish(self.temp_data)
				self.rate.sleep()
			except:
				#rospy.logwarn("IMU has been disconnected.")
				self.get_port()

	def get_data(self):
		self.port.write("quaternion di. accelp di. gyrop di. temperature di.\r\n")
		data = self.port.read_until(terminator="OK").strip().split("\r\n")
		#return lines
		self.imu_data.header.frame_id="base_link"
		self.imu_data.header.stamp = rospy.Time.now()
		self.imu_data.orientation.w = float(data[1][18:])
		self.imu_data.orientation.x = float(data[2][5:])
		self.imu_data.orientation.y = float(data[3][5:])
		self.imu_data.orientation.z = float(data[4][5:])
		self.imu_data.linear_acceleration.x = float(data[7][14:])/100
		self.imu_data.linear_acceleration.y = float(data[8][5:])/100
		self.imu_data.linear_acceleration.z = float(data[9][5:])/100
		self.imu_data.angular_velocity.x = float(data[12][13:])
		self.imu_data.angular_velocity.y = float(data[13][5:])
		self.imu_data.angular_velocity.z = float(data[14][5:])
		self.temp_data.header = self.imu_data.header
		self.temp_data.temperature = float(data[17][14:])

if __name__=="__main__":
	rospy.init_node("imu_node")
	imu = ImuSensor(rate=100)
	imu.publish()