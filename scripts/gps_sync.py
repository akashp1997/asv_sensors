#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import geometry_msgs.msg

vel = geometry_msgs.msg.TwistStamped()
fix = sensor_msgs.msg.NavSatFix()
imu = sensor_msgs.msg.Imu()

def listener():
	rospy.init_node("gps_sync")
	rospy.Subscriber("/vel", geometry_msgs.msg.TwistStamped, callback=get_data, callback_args=0)
	rospy.Subscriber("/fix", sensor_msgs.msg.NavSatFix, callback=get_data, callback_args=1)
	rospy.Subscriber("/imu_data", sensor_msgs.msg.Imu, callback=get_data, callback_args=2)
	#publish()
	rospy.spin()

def get_data(data, arg):
	global vel, fix, imu
	if arg==0:
		vel = data
	elif arg==1:
		fix = data
	elif arg==2:
		imu = data
		pub_fix = rospy.Publisher("/fix_synced", sensor_msgs.msg.NavSatFix, queue_size=100)
		fix.header.stamp = imu.header.stamp
		pub_fix.publish(fix)
		pub_vel = rospy.Publisher("/vel_synced", geometry_msgs.msg.TwistStamped, queue_size=100)
		vel.header.stamp = imu.header.stamp
		pub_vel.publish(vel)
	else:
		return

def publish():
	global old_fix, new_fix, new_vel, old_vel
	#pub_vel = rospy.Publisher("/vel_synced", geometry_msgs.msg.TwistStamped, queue_size=100)
	pub_fix = rospy.Publisher("/fix_synced", sensor_msgs.msg.NavSatFix, queue_size=100)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		#old_vel.header.stamp = imu.header.stamp
		fix.header.stamp = imu.header.stamp
		pub_fix.publish(fix)
		rate.sleep()

listener()