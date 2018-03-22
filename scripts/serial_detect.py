#!/usr/bin/env python
import rospy

import subprocess
import serial
from asv_sensors.srv import *

port_open_list = []

def handle_request(req):
	global port_open_list
	#0 for IMU
	#1 for GPS
	#2 for Camera
	#3 for Arduino
	if req.last_port in port_open_list:
		#port_open_list = list(rospy.get_param("open_com_ports"))
		port_open_list.remove(req.last_port)
		#rospy.set_param("open_com_ports", port_open_list)
	if req.device_type==0:
		port = detect_imu()
	elif req.device_type==1:
		port = detect_gps()
	#elif req.device_type==2:
	#	port_name = detect_camera()
	#elif req.device_type==3:
	#	port_name = detect_arduino()
	else:
		return port_updateResponse("", False)
	return port

def server():
	service = rospy.Service("serial_detect", port_update, handle_request)
	rospy.loginfo("Server ready to update Serial Ports")
	rospy.spin()

def detect_imu():
	ports = subprocess.check_output(["ls", "/dev/"]).split("\n")
	ports = ["/dev/"+x for x in ports if "ttyUSB" in x]
	for port_name in ports:
		if port_name in port_open_list:
			continue
		try:
			port = serial.Serial(port_name, baudrate=115200, timeout=0.1)
			port.write("currentTime di.\r\n")
			line = port.readline().strip()
			if line=="currentTime di.":
				port.close()
				rospy.loginfo("Found IMU at Port: " + port_name)
				port_open_list.append(port_name)
				#rospy.set_param("open_com_ports", port_open_list)
				return port_updateResponse(port_name, True)
		except:
			continue
	return port_updateResponse("", False)

def detect_gps():
	global port_open_list
	ports = subprocess.check_output(["ls", "/dev/"]).split("\n")
	ports = ["/dev/"+x for x in ports if "ttyUSB" in x]
	for port_name in ports:
		if port_name in port_open_list:
			continue
		try:
			port = serial.Serial(port_name, baudrate=38400, timeout=0.1)
			line = port.readline()
			if line.startswith("$"):
				port.close()
				rospy.loginfo("Found GPS at Port: " + port_name)
				port_open_list.append(port_name)
				#rospy.set_param("open_com_ports", port_open_list)
				return port_updateResponse(port_name, True)
		except:
			continue
	return port_updateResponse("", False)


rospy.init_node("serial_detection_server")
#rospy.set_param("open_com_ports", "[]")
server()