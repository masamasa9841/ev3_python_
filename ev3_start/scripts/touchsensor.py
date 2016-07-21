#!/usr/bin/env python

import rpyc
import rospy
from std_msgs.msg import *

if __name__ == "__main__":
	rospy.init_node('ev3_start_touchsensor')
	host="192.168.22.175"
	conn = rpyc.classic.connect(host)
	lego = conn.modules.ev3.lego

	s=lego.TouchSensor()
	r = rospy.Rate(1000)
	pub = rospy.Publisher('touch_sensor', Bool)

	while not rospy.is_shutdown():
		pub.publish(s.is_pushed)
		r.sleep()
