#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('vel_parameter')
vel_x = rospy.get_param('~vel_x', 0.2)
vel_rot = rospy.get_param('~vel_rot', 0.5)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

r = rospy.Rate(10.0)
for i in range(20):
	r.sleep()

vel = Twist()
vel.linear.x = vel_x
r = rospy.Rate(10.0)
for i in range(50):
	pub.publish(vel)
	r.sleep()

vel = Twist()
r = rospy.Rate(10.0)
for i in range(10):
	pub.publish(vel)
	r.sleep()

vel = Twist()
vel.linear.x = -vel_x
r = rospy.Rate(10.0)
for i in range(50):
	pub.publish(vel)
	r.sleep()

vel = Twist()
r = rospy.Rate(10.0)
for i in range(10):
	pub.publish(vel)
	r.sleep()

vel = Twist()
vel.angular.z = vel_rot
r = rospy.Rate(10.0)
for i in range(50):
	pub.publish(vel)
	r.sleep()

vel = Twist()
r = rospy.Rate(10.0)
for i in range(10):
	pub.publish(vel)
	r.sleep()

vel = Twist()
vel.angular.z = -vel_rot
r = rospy.Rate(10.0)
for i in range(50):
	pub.publish(vel)
	r.sleep()

vel = Twist()
r = rospy.Rate(10.0)
for i in range(10):
	pub.publish(vel)
	r.sleep()

