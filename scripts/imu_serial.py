#!/usr/bin/env python

#Import any external libraries here
import sys, serial, time

#Import ROS libraries here
import rospy

#Import ROS messages here
import std_msgs.msg
import asv_sensors.msg

"""
class Imu:
	#Set emulate to True, if there is a need to emulate the sensor.
	#If emulate is set to false, there is a need to set the parameter for port number
	def __init__(self, emulate=False, data=None, port_name="/dev/ttyUSB0", baudrate=115200):
		#Will only set the data from the data field only if emulate is set to true
		self.emulate = emulate
		self.imu_data = data
		self.port = port_name
		self.baudrate = baudrate
		#The messages should be of the format asv_sensors.msg._Serial.Serial
		if (type(self.imu_data)!=str):
			raise ValueError("Please provide data of only String format")

	def emulate_data(self):
		#This function will return the current value of emulated data, only if the emulate flag is set to True
		if (self.emulate==True):
			return self.imu_data
		else:
			return ""

	def open_io_port(self):
		#This utility function will open a Serial Port only if the emulate flag is set to True
		if(self.emulate==False):
			self.port = serial.Serial(self.port, baudrate=self.baudrate)
		else:
			del self.port, self.baudrate
			return False

	def get_serial_data(self):
		#This function will access the Serial Port and Get the data from the port and return it
		if(emulate==False):
			port = serial.serial(self.port_name, baudrate=self.baudrate)
			port.write("quaternion di. accelp di. gyrop di. temperature di.\r\n")
			lines = []
			while True:
				line = port.readline()
				if ("OK" in line):
					break
				else:
					lines.append(line)
			self.imu_data = "".join(lines)
			return self.imu_data

	def get_data(self):
		if(self.emulate==True):
			while True:
				print(self.imu_data)
		else:
			while True:
				self.open_io_port()
				while True:
					print(self.get_serial_data())

class ROS_Imu(Imu):
	def __init__(self, emulate=False, data=None, port_name="/dev/ttyUSB0", baudrate=115200, topic_name="/imu_raw", rate=100):
		rospy.init_node("imu_serial_node")
		super(ROS_Imu, self).__init__()
		self.publisher = rospy.Publisher(topic_name)
		self.rate = rospy.Rate(rate)

	def get_data(self):
		if(self.emulate==True):
			self.open_io_port()
		msg = asv_sensors.msg.Serial()
		while not rospy.is_shutdown():
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = "base_link"
			if(self.emulate==True):
				msg.data = self.get_serial_data()
			else:
				msg.data = self.emulate_data()
			self.publisher.publish(msg)
			self.rate.sleep()
		else:
			while not rospy.is_shutdown():
				msg = asv_sensors.msg.Serial()
				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = "base_link"				
				self.publisher.publish()

"""

def talker():
	rospy.init_node("imu_serial_node")
	if rospy.has_param("~port"):
		port_name = rospy.get_param("~port")
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


#obj = ROS_Imu(emulate=True, data="Hello World")
#obj.get_data()