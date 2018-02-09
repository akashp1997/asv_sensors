#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import serial
import tf
import math

prev_heading = []
heading = 0
imu = sensor_msgs.msg.Imu()
port = serial.Serial("/dev/ttyACM0", baudrate=115200)

def publisher():
	global heading, prev_heading, prev, imu
	rospy.init_node("magnetometer")
	pub = rospy.Publisher("/imu_data", sensor_msgs.msg.Imu, queue_size=10)
	rate = rospy.Rate(100)
	i = 0
	while not rospy.is_shutdown():
		heading = float(port.readline().split(",")[1])
		arr = tf.transformations.quaternion_from_euler(0,0,math.radians(heading))
		imu.header.frame_id = "base_link"
		imu.header.stamp = rospy.Time().now()
		imu.orientation.w = arr[0]
		imu.orientation.x = arr[1]
		imu.orientation.y = arr[2]
		imu.orientation.z = arr[3]
		if (len(prev_heading)>=10):
			imu.angular_velocity.z = math.radians(heading-prev_heading[0])*100
			prev_heading.pop(0)
		pub.publish(imu)
		prev_heading.append(heading)
		rate.sleep()
try:
	publisher()
except:
	port.close()