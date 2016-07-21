#!/usr/bin/env python
import rospy
import rpyc
from sensor_msgs.msg import Joy
def joy_callthrow(joytwist):
    global motorA
    print joytwist.axes[1]
    if joytwist.axes[1] == 0:
        motorA.stop()
    else:
        motorA.run_forever(joytwist.axes[1]*50)

if  __name__ =='__main__':
    host = "192.168.22.175"
    conn = rpyc.classic.connect(host)
    lego = conn.modules.ev3.lego
    motorA = lego.Motor('A')
    rospy.init_node('joy_twist')
    sub = rospy.Subscriber('joy', Joy, joy_callthrow,queue_size=1)
    rospy.spin()
