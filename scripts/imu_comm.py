#!/usr/bin/env python
#Import any external libraries here
import serial, termios

#Import ROS libraries here
import rospy

#Import ROS messages here
import sensor_msgs.msg

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

class ImuSensor(object):
	def __init__(self, baud=115200, topic_name="/imu_data", temp_topic="/temperature", rate=100):
		self.imu_data = sensor_msgs.msg.Imu()
		self.temp_data = sensor_msgs.msg.Temperature()
		self.start_port()
		if self.port==None:
			rospy.logwarn("IMU Sensor is disconnected.")
		self.pub = rospy.Publisher(topic_name, sensor_msgs.msg.Imu, queue_size=10)
		self.temp_pub = rospy.Publisher(temp_topic, sensor_msgs.msg.Temperature, queue_size=10)
		self.rate = rate

	def detect_port(self):
		self.port = serial.Serial(AutoSerial(baud=115200).get_port(), baudrate=115200)
		self.port_prefix = ["/dev/ttyUSB", "/dev/ttyUSB", "/dev/ttyACM"]
		while not rospy.is_shutdown():
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

	def publish(self):
		while not rospy.is_shutdown():
			try:
				if not self.port.is_open:
					self.port.open()
				else:
					self.get_data()
					self.pub.publish(self.imu_data)
					self.temp_pub.publish(self.temp_data)
					self.port.close()
			except:
				self.start_port()
			rospy.Rate(self.rate).sleep()

	def get_data(self):
		self.port.flush()
		self.port.write("quaternion di. accelp di. gyrop di. temperature di.\r\n")
		data = self.port.read_until(terminator="OK").strip().split("\r\n")
		self.port.flush()
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

try:
	if __name__=="__main__":
		rospy.init_node("imu_node")
		imu = ImuSensor(rate=100)
		imu.publish()
except KeyboardInterrupt:
	self.pub.publish(sensor_msgs.msg.Imu())
	self.port.close()