#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import utm
import tf
#Install utm from pip

prev_odom = nav_msgs.msg.Odometry()
odom = nav_msgs.msg.Odometry()



def listener():
	rospy.init_node("odom_gen")
	rospy.Subscriber("/fix", sensor_msgs.msg.NavSatFix, callback, callback_args="fix")
	rospy.Subscriber("/vel", geometry_msgs.msg.TwistStamped, callback, callback_args="vel")
	rospy.Subscriber("/imu_data", sensor_msgs.msg.Imu, callback, callback_args="imu")
	publisher()
	rospy.spin()

def callback(data, args):
	global odom
	if args=="fix":
		if data.latitude>84 or data.latitude<-80:
			data.latitude = 0
		if abs(data.longitude)>=180:
			data.longitude = 0
		arr = utm.from_latlon(data.latitude, data.longitude)
		odom.pose.pose.position.x = arr[0]
		odom.pose.pose.position.y = arr[1]
		odom.header = data.header
	elif args=="vel":
		odom.twist.twist = data.twist
	elif args=="imu":
		odom.pose.covariance = data.orientation_covariance*4
		quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		quat = tf.transformations.unit_vector(quat)
		odom.pose.pose.orientation.w = quat[0]
		odom.pose.pose.orientation.x = quat[1]
		odom.pose.pose.orientation.y = quat[2]
		odom.pose.pose.orientation.z = quat[3]
		odom.twist.twist.angular = data.angular_velocity
		odom.twist.covariance = data.angular_velocity_covariance*4
		#Integration with respect to delta time
		lin_accel = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
		time = odom.header.stamp.secs-prev_odom.header.stamp.secs+(odom.header.stamp.nsecs-prev_odom.header.stamp.nsecs)*10**(-9)
		lin_accel = [i*time for i in lin_accel]
		odom.twist.twist.linear.x += lin_accel[0]
		odom.twist.twist.linear.y += lin_accel[1]
		odom.twist.twist.linear.z += lin_accel[2]

	
def publisher():
	global odom
	rate = rospy.Rate(1000)
	tfmsg = geometry_msgs.msg.TransformStamped()
	tfmsg.header.frame_id = "odom"
	tfmsg.child_frame_id = "base_footprint"
	broadcaster = tf.TransformBroadcaster()
	pub = rospy.Publisher("/odom", nav_msgs.msg.Odometry, queue_size=10)
	while not rospy.is_shutdown():
		tfmsg.header.stamp = odom.header.stamp
		tfmsg.transform.translation.x = odom.pose.pose.position.x
		tfmsg.transform.translation.y = odom.pose.pose.position.y
		tfmsg.transform.translation.z = odom.pose.pose.position.z
		tfmsg.transform.rotation = odom.pose.pose.orientation
		broadcaster.sendTransformMessage(tfmsg)
		odom.header.frame_id = "base_link"
		odom.child_frame_id = "base_link"
		pub.publish(odom)
		prev_odom = odom
		rate.sleep()

listener()