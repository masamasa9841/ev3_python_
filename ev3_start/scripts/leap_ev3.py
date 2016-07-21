#!/usr/bin/env python

import rospy
from leap_motion.msg import leapros
from std_msgs.msg import Float64

class LeapTest(object):
    def __init__(self):
        self.sub = rospy.Subscriber("leapmotion/data", leapros, self.callback,queue_size=1)
        self.pub = rospy.Publisher("oppai_velocity", Float64,queue_size=1)
        self.pub2 = rospy.Publisher("oppai_velocityx", Float64,queue_size=1)
        self.rate = rospy.Rate(100)
        self.pos = 0.0
        self.pos2 = 0.0

    def callback(self, msg):
        self.pos = msg.palmpos.x
        self.pos2= msg.palmpos.z

    def run(self):
        while not rospy.is_shutdown():
            pos = self.pos
            pos2 = self.pos2
            if pos>100:
                pos=100
            elif pos < -100:
                pos=-100
            if pos2 > 100:
                pos2 = 100
            elif pos2 < -100:
                pos2 = -100
            self.pub.publish(pos)
            self.pub2.publish(pos2)
            self.rate.sleep()

    def stop(self):
        self.pub.publish(0.0)
        self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("oppai_leap")
    lt=LeapTest()
    try:
        lt.run()
    except rospy.ROSInterruptException:
        pass
