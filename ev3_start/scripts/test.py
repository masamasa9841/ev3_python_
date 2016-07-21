#!/usr/bin/env python

import rospy
import rpyc
import time
from sensor_msgs.msg import *
from std_msgs.msg import *

class Otintin(object):
	def __init__(self):
		self.host = rospy.get_param('~host', "192.168.22.175")
		self.conn = rpyc.classic.connect(self.host)
		self.lego = self.conn.modules.ev3.lego

		self.motorB = self.lego.Motor('B')
		self.motorC = self.lego.Motor('C')
		self.touch_s = self.lego.TouchSensor()

		self.r = rospy.Rate(100)

		rospy.on_shutdown(self.shutdown_hook)

	def motor_hz(self, hz):
		self.motorB.run_forever(hz, regulation_mode = False)
		self.motorC.run_forever(hz, regulation_mode = False)

	def motor_dif(self, b_hz, c_hz):
		self.motorB.run_forever(b_hz, regulation_mode = False)
		self.motorC.run_forever(c_hz, regulation_mode = False)

	def motor_stop(self):
		self.motorB.stop()
		self.motorC.stop()

	def shutdown_hook(self):
		print "shutdown"
		self.motor_stop()

if __name__ == "__main__":
	rospy.init_node('ev3_start_test')
	otin=Otintin()
	try:
		while not rospy.is_shutdown():
			if otin.touch_s.is_pushed == True:
				print "Stop"
				for i in range(10):
					otin.motor_hz(-50)
				for i in range(5):
					otin.motor_dif(-50,50)
				otin.motor_stop()
			else:
				print "Go"
				otin.motor_hz(100)
			otin.r.sleep()
	except rospy.ROSInterruptException:
		print "interrupt"
		pass
