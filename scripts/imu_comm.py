#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import serial
from tf.transformations import euler_from_quaternion as e_q
from math import radians

class ImuSensor(object):
	def __init__(self, topic_name="/imu_data", port="/dev/ttyUSB0", rate=100):
		rospy.init_node("imu_node")
		self.imu_data = sensor_msgs.msg.Imu()
		try:
			self.port = serial.Serial(port, baudrate=115200)
		except:
			rospy.logfatal("Serial Port %s not found. Kindly check with the parameters that you have entered inside the launch file." % port)
			quit()
		self.pub = rospy.Publisher(topic_name, sensor_msgs.msg.Imu, queue_size=10)
		self.rate = rate
		self.last_rpy = [0,0,0]
		self.now_rpy = [0,0,0]

	def get_linear_accel(self):
		self.port.write("$PSPA,A\r\n")
		self.port.flush()
		line = self.port.readline()
		line = [float(i[i.find("=")+1:]) for i in line[:line.find("*")].split(",")[1:4]]
		self.imu_data.linear_acceleration.x = line[0]/100
		self.imu_data.linear_acceleration.y = line[1]/100
		self.imu_data.linear_acceleration.z = (line[2]-980)/100
		self.imu_data.angular_velocity_covariance = [-1]*9 # No angular velocity is shown by covariance matrix of 9 -1s

	def set_angular_vel(self):
		self.port.write("$PSPA,PR\r\n")
		self.port.flush()
		line = self.port.readline()
		self.port.write("$xxHDT\r\n")
		self.port.flush()
		yaw = float(self.port.readline().split(",")[1])
		self.now_rpy = [float(i[i.find("=")+1:]) for i in line[:line.find("*")].split(",")[1:3]]+[yaw]
		ang = [radians((self.now_rpy[i]-self.last_rpy[i]))*self.rate for i in range(len(self.now_rpy))]
		rospy.loginfo(ang)
		self.last_rpy = self.now_rpy

	def get_quat(self):	
		self.port.write("$PSPA,QUAT\r\n")
		self.port.flush()
		line = self.port.readline()
		line = [float(i[i.find("=")+1:]) for i in line[:line.find("*")].split(",")[1:5]]
		self.imu_data.orientation.w = line[0]
		self.imu_data.orientation.x = line[1]
		self.imu_data.orientation.y = line[2]
		self.imu_data.orientation.z = line[3]

	def publish(self):
		try:
			while not rospy.is_shutdown():
				self.get_linear_accel()
				self.get_quat()
				self.set_angular_vel()
				self.pub.publish(self.imu_data)
				rospy.Rate(self.rate).sleep()
		except KeyboardInterrupt:
			self.pub.publish(sensor_msgs.msg.Imu())
			quit()

def __main__():
	imu = ImuSensor(rate=30)
	imu.publish()

if __name__=="__main__":
	__main__()
