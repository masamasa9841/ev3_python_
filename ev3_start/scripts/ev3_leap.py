#!/usr/bin/env python
import rospy
import rpyc
import math
from std_msgs.msg import Float64
class MyMotor(object):
    def __init__(self,port,lego):
        self.motor = lego.LargeMotor(port=port)
        self.motor.reset()

    def reset(self):
        self.motor.reset()

    def stop(self):
        self.motor.stop()
    
    def run(self,duty):
        self.motor.run_forever(duty/3, speed_regulation = False)

class IPMotor(object):
    def __init__(self,lego):
        self.motora = MyMotor(port = 'A', lego=lego)
        self.motord = MyMotor(port = 'D', lego=lego)
        self.sub = rospy.Subscriber("oppai_velocity",Float64,self.callback,queue_size=1)
        self.sub2 = rospy.Subscriber("oppai_velocityx",Float64,self.callback2,queue_size=1)
        rospy.on_shutdown(self.shutdown_hook)
        self.hoge = [0.0,0.0]
        self.rate = rospy.Rate(100)
        print self.sub

    def run_a(self,duty):
        self.motora.run(duty)

    def run_d(self,duty):
        self.motord.run(duty)

    def stop(self):
        self.motora.stop()
        self.motord.stop()

    def callback(self,msg):
        self.x = msg.data
    def callback2(self,msg):

        self.z = msg.data
        if abs(self.x)<30:
            self.run_a(self.z)
            self.run_d(self.z)
        elif abs(self.z) < 30 and self.x < 0:
            self.run_a(-self.x)
            self.run_d(self.x)
        elif abs(self.z) < 30 and self.x > 0:
            self.run_a(-self.x)
            self.run_d(self.x)
        print self.x,self.z

    def shutdown_hook(self):
        self.motora.stop()
        self.motord.stop()

if __name__ == '__main__':
    rospy.init_node("oppai_velo")
    host = '192.168.22.175'
    conn = rpyc.classic.connect(host)
    lego = conn.modules.ev3.lego
    i=IPMotor(lego)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        i.stop()
