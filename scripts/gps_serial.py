#!/usr/bin/env python
#Import any external libraries here
import serial
import subprocess
import time

#Import ROS libraries here
import rospy

#Import ROS messages here
import nmea_msgs.msg

class GPSSensor(object):
	def __init__(self, baud=38400, topic_name="/gps/sentence", rate=10):
		self.nmea_data = nmea_msgs.msg.Sentence()
		self.nmea_data.header.frame_id = "/garmin"
		self.baud=baud
		self.port = None
		self.pub = rospy.Publisher(topic_name, nmea_msgs.msg.Sentence, queue_size=10)
		self.rate = rospy.Rate(rate)

	def detect_port(self):
		while not rospy.is_shutdown():
			try:
				ports = subprocess.check_output(["ls", "/dev/"]).split("\n")
				ports = ["/dev/"+x for x in ports if "ttyUSB" in x]
				for port_name in ports:
					try:
						port = serial.Serial(port_name, baudrate=38400, timeout=0.1)
						while True:
							line = port.readline()
							if len(line)==0:
								break
							if line.startswith("$"):
								self.port = serial.Serial(port_name, baudrate=38400, timeout=0.1)
								self.port.flush()
								return
						#if "$" in line:
						#	self.port = serial.Serial(port_name, baudrate=38400)
						#	return
					except serial.serialutil.SerialException:
						continue
			except:
				return

	def publish(self):
		while not rospy.is_shutdown():
			try:
				self.get_data()
				#self.pub.publish(self.nmea_data)
			except:
				rospy.logwarn("GPS can be disconnected")
				self.detect_port()
			#self.rate.sleep()

	def get_data(self):
		line = self.port.readline()
		rospy.loginfo(line)


if __name__=="__main__":
	rospy.init_node("gps_serial")
	gps = GPSSensor()
	gps.detect_port()
	#rospy.loginfo(gps.port)
	gps.publish()