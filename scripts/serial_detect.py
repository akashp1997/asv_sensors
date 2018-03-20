#!/usr/bin/env python
import rospy

import subprocess
import serial
from asv_sensors.srv import *

def handle_request(req):
	#0 for IMU
	#1 for GPS
	#2 for Camera
	#3 for Arduino
	if req.device_type==0:
		port_name = detect_imu()
	#elif req.device_type==1:
	#	port_name = detect_gps()
	#elif req.device_type==2:
	#	port_name = detect_camera()
	#elif req.device_type==3:
	#	port_name = detect_arduino()
	else:
		return port_updateResponse("", False)
	return port_updateResponse(port_name, True)

def server():
	rospy.init_node("serial_detect_server")
	service = rospy.Service("serial_detect", port_update, handle_request)
	rospy.loginfo("Server ready to update Serial Ports")
	rospy.spin()

def detect_imu():
	

server()