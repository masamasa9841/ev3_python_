#!/usr/bin/env python

import rpyc
import rospy
from std_msgs.msg import *
from ev3.lego import GyroSensor
#import unittest
#from util import get_input

class TestGyroSensor(unittest.TestCase):
    def test_gyro_sensor(self):
        get_input('Attach a GyroSensor then continue')
        d = GyroSensor()
        get_input('test ang')
        print(d.ang)
        print(d.mode)




if __name__ == "__main__":
	rospy.init_node('ev3_start_gyrosensor')
	host = "192.168.22.175"
	conn = rpyc.classic.connect(host)
	lego = conn.modules.ev3.lego

	while not rospy.is_shutdown():
                unittest.main()


