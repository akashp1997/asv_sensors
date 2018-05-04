#!/usr/bin/env python
import rospy
import socket
import nmea_msgs.msg

rospy.init_node("gps_socket")
pub = rospy.Publisher("/nmea_sentence", nmea_msgs.msg.Sentence, queue_size=10)
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
		data = sock.recv(1024).strip()
		msg = nmea_msgs.msg.Sentence()
		msg.sentence = data
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "garmin"
		pub.publish(msg)
except:
	sock.close()