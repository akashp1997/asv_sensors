#!/usr/bin/env python
#Import any external libraries here
import serial
import subprocess
import time

#Import ROS libraries here
import rospy

#Import ROS messages here
import nmea_msgs.msg
from asv_sensors.srv import *

class GPSSensor(object):
	def __init__(self, baud=38400, topic_name="/gps_serial", vel_topic_name="/vel", rate=10):
		self.port = None
		self.pub = rospy.Publisher(topic_name, nmea_msgs.msg.Sentence, queue_size=10)
		self.rate = rospy.Rate(rate)

	def get_port(self):
		rospy.wait_for_service("serial_detect")
		try:
			port_detect = rospy.ServiceProxy('serial_detect', port_update)
			if self.port==None:
				resp = port_detect(1, "")
			else:
				resp = port_detect(1, self.port.port)
			if resp.found==True:
				self.port = serial.Serial(resp.port_name, baudrate=38400)
		except rospy.ServiceException:
			rospy.logfatal("Service call failed.")

	def publish(self):
		while not rospy.is_shutdown():
			try:
				line = self.port.readline().strip()
				data = nmea_msgs.msg.Sentence()
				data.header.frame_id = "garmin"
				data.header.stamp = rospy.Time.now()
				data.sentence = line
				self.pub.publish(data)
				#self.pub.publish(self.nmea_data)
			except:
				#rospy.logwarn("GPS has been disconnected.")
				self.get_port()

	#def get_data(self):

		#self.nmea_data.header.stamp = rospy.Time.now()
		#self.pub.publish(self.nmea_data)
		"""if line[0]=="$PGRMF":
			self.nmea_data.header.stamp = rospy.Time.now()
			self.nmea_data.latitude = (float(line[6])/100)+(float(line[6])%1)*10/6
			if line[7]=="S":
				self.nmea_data.latitude *= -1
			self.nmea_data.longitude = (float(line[8])/100)+(float(line[6])%1)*10/6
			if line[9]=="W":
				self.nmea_data.latitude *= -1
			self.nmea_data.status.status = int(line[11])-1
			self.nmea_data.status.service = 1
			self.pub.publish(self.nmea_data)
		elif line[0]=="$PGRMV":
			self.vel_data.header.stamp = rospy.Time.now()
			self.vel_data.header.frame_id = "/garmin"
			self.vel_data.header.linear.x = float(line[1])
			self.vel_data.header.linear.y = float(line[2])
			self.vel_data.header.linear.z = float(line[3])
			self.vel_pub.publish(self.vel_data)"""
			#if line[7]=="N":
			#	self.nmea_data.latitude = float(line[6])
			#else:
			#	self.nmea_data.latitude = -float(line[6])
		#rospy.loginfo(line)


if __name__=="__main__":
	rospy.init_node("gps_serial")
	gps = GPSSensor()
	#gps.detect_port()
	#rospy.loginfo(gps.port)
	gps.publish()