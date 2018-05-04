#!/usr/bin/env python
import rospy
import socket
import asv_sensors.msg

rospy.init_node("imu_socket")
pub = rospy.Publisher("/imu_raw", asv_sensors.msg.Serial, queue_size=10)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "192.168.1.2"
port = 40000
if(rospy.has_param("~host")):
	host = rospy.get_param("~host")
if(rospy.has_param("~port")):
	port = int(rospy.get_param("~port"))
sock.connect((host, port))
try:
	while True:
		data = sock.recv(1024)
		msg = asv_sensors.msg.Serial()
		msg.data = data
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"
		pub.publish(msg)
except:
	sock.close()
