#!/usr/bin/env python
import rospy
import serial

class AutoSerial(object):
	def __init__(self, device_type=0, baud=9600, ran=[0,100], no_count=10):
		"""0 for IMU
		1 for GPS
		2 for Arduino"""
		self.device_type = device_type
		self.baud = baud
		self.range = ran
		self.count = no_count

	def get_port(self):
		self.port_prefix = ["/dev/ttyUSB", "/dev/ttyUSB", "/dev/ttyACM"]
		for no_iter in range(self.count):
			for port_no in range(self.range[0], self.range[1]):
				rate = rospy.Rate(100)
				#Start IMU Test
				try:
					rate.sleep()
					self.port = serial.Serial(self.port_prefix[self.device_type]+str(port_no), baudrate=self.baud)
					self.port.flush()					
					self.port.write("currentTime di.\r\n")
					line = self.port.read_until(terminator="OK")
					if line.split("\n")[1].startswith("currentTime ="):
						self.port.flush()
						self.port.close()
						return self.port_prefix[self.device_type]+str(port_no)
				except serial.serialutil.SerialException:
					continue
				#End IMU Test
		return None