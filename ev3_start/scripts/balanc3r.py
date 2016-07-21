from ev3.ev3dev import Motor, Tone
from ev3.lego import GyroSensor
import time
import sys
import rospy
import rpyc

class MyMotor(object):
    def __init__(self,port):
        self.motor=Motor(port=port)
        self.motor.reset()

    def reset(self):
        self.motor.reset()

    def run(self,duty):
        self.motor.run_forever(duty, speed_regulation=False)

    def stop(self):
        self.motor.stop()

    def position(self):
        return self.motor.position

class IPMotor(object):
    def __init__(self):
        self.motora=MyMotor(port=Motor.PORT.A)
        self.motord=MyMotor(port=Motor.PORT.D)

    def run_d(self,duty):
        if duty >= 100:
            duty = 100
        if duty <= -100:
            duty = -100
        self.motord.run(duty)

    def run_a(self,duty):
        if duty >= 100:
            duty = 100
        if duty <= -100:
            duty = -100
        self.motora.run(duty)

    def position_a(self):
        return self.motora.position()

    def position_d(self):
        return self.motord.position()

    def reset(self):
        self.motora.reset()
        self.motord.reset()
    def stop(self):
        self.motora.stop()
        self.motord.stop()

class MyGyroSensor(object):
    def __init__(self):
        self.g=GyroSensor()

    def get_value(self):
        return self.g.rate  

class MySpeaker(object):
    def __init__(self):
        self.t = Tone()

    def play(self,frequency=100,milliseconds=100):
        self.t.play(frequency,milliseconds)

    def stop(self):
        self.t.stop()

class Balancer(object):
    def __init__(self):
        self.gyro = MyGyroSensor()
        self.tone = MySpeaker()
        self.motor = IPMotor()
        self.ang = 0.0
        self.enc_index = 0
        self.compare_index = 0
        self.acc_err = 0.0
        self.prev_err = 0.0
        self.old_steering = 0.0
        self.sync_0 = 0.0
        try:
            self.gyro.get_value()
        except:
            pass

    def GyroRate(self):
        return self.gyro.get_value() / float(-4.86)

    def Calibrate(self):
        self.tone.play()
        mean = 0
        for a in range(20):
            mean += self.GyroRate()
            time.sleep(0.01)
        mean /= 20.0
        time.sleep(0.1)
        self.tone.play()
        time.sleep(0.1)
        self.tone.play()
        return mean  

    def Initialize(self,WheelDiameter=42.0,SampleTime=22.0):
        self.dt =0.07#(SampleTime - 2.0) / 1000.0
        self.radius = WheelDiameter / 2000.0
        self.max_index = 7
        self.enc_val = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.motor.reset()
        self.refpos = 0.0
        self.NowOutOFBound = False
        self.PrevOutOFBound = False
        self.OutOFBoundCount = 0
        self.OutOFBound = 0
        ang = 0.0
        self.mean = 0.0
        time.sleep(0.1)
        self.mean = self.Calibrate()
        self.speed = 0.0
        self.steering = 0.0
        self.max_acceraration = 0.0

    def SetConstants(self,x=0.6,y=12.2,z=0.0058,AngularVelocity=1.3,Angle=21.7,WheelSpeed=86.3,WheelPosition=350.0):
        self.gain_motor_position = WheelPosition
        self.gain_motor_speed = WheelSpeed
        self.gain_angle = Angle
        self.gain_angle_velocity = AngularVelocity
        self.kp = x
        self.ki = y
        self.kd = z
        self.hage = time.time()

    def Position(self):
        self.refpos += self.speed * self.dt * 0.02
        return self.refpos
    
    def GetMotorSpeed(self):
        # self.enc_index += 1
        # if self.max_index <= self.enc_index:
        #   self.enc_index = 0
        self.enc_index = self.enc_index + 1 if self.max_index > self.enc_index else 0
        #self.compare_index = self.enc_index + 1
        #if self.max_index <= self.compare_index:
        #    self.compare_index = 0
        self.compare_index = self.enc_index + 1 if self.max_index > self.compare_index else 0
        # self.enc_val[self.enc_index] = (self.motor.position_a() + self.motor.position_d()) / 2.0 
        self.enc_val[self.enc_index] = (self.motor.position_a() + self.motor.position_d()) * 0.5
        Speed = (self.enc_val[self.enc_index] - self.enc_val[self.compare_index]) / (self.max_index * self.dt) 
        return Speed

    def ReadEncoders(self):
        RobotSpeed = self.radius * self.GetMotorSpeed() / 57.3
        RobotPosition =(self.radius * (self.motor.position_a() + self.motor.position_d()) / 2.0) / 57.3
        return RobotSpeed,RobotPosition

    def ReadGyro(self):
        curr_val = self.GyroRate()
        self.mean = (self.mean * (1.0 - self.dt * 0.2)) + (curr_val * self.dt * 0.20)
        angular_velocity = curr_val - self.mean
        self.ang += self.dt * angular_velocity
        return angular_velocity,self.ang

    def CombineSensorValues(self,AngularVelocity,Angle,MotorSpeed,MotorPosition,MotorReferencePosition):
        WeigtedSum = (self.gain_motor_position * (MotorPosition - MotorReferencePosition)) + (self.gain_motor_speed * MotorSpeed) + (self.gain_angle * Angle) + (self.gain_angle_velocity * AngularVelocity)
        return WeigtedSum

    def ReadConstants(self):
        pass

    def PID(self,ReferenceValue,InputValue):
        reference = ReferenceValue
        inp = InputValue
        curr_err = inp - reference
        self.acc_err += curr_err * self.dt
        dir_err = (curr_err - self.prev_err) / self.dt
        self.prev_err = curr_err
        return (self.kp * curr_err) + (self.ki * self.acc_err) + (self.kd * dir_err)

    def Errors(self,AveragePower):
        # if abs(AveragePower) > 100:
        #    self.NowOutOFBound = True
        #else:
        #    self.NowOutOFBound = False
        self.NowOutOFBound = True if abs(AveragePower) > 100 else False
        # if self.NowOutOFBound and self.PrevOutOFBound == True:
        #     self.OutOFBoundCount += 1
        # else:
        #     self.OutOFBoundCount = 0
        self.OutOFBoundCount = OutOFBoundCount + 1 if self.NowOutOFBound and self.PrevOutOFBound == True else 0
        if self.OutOFBoundCount > 20:
            pass
        else:
            self.PrevOutOFBound = self.NowOutOFBound

    def GetSteer(self):
        pass

    def Limit(self,lower,upper):
        if self.steering > upper:
            return upper
        if self.steering < lower:
            return lower
        return self.steering

    def SetMotorPower(self,AveragePower):
        avg_pwr = AveragePower
        new_steering = self.Limit(-50,50)
        if new_steering == 0:
            if not self.old_steering == 0:
                self.sync_0 = self.motor.position_d() - self.motor.position_a()
            extra_pwr = (self.motor.position_d() - self.motor.position_a() - self.sync_0) * 0.05
        else:
            extra_pwr = new_steering * -0.5
        pwr_c = avg_pwr - extra_pwr
        pwr_b = avg_pwr + extra_pwr
        self.old_steering = new_steering
        self.motor.run_a(pwr_b * 0.021 / self.radius)
        self.motor.run_d(pwr_c * 0.021 / self.radius)

    def wait(self):
        self.hoge = time.time()
        if self.hoge - self.hage > self.dt:
            print self.hoge-self.hage
        else:
            time.sleep(self.hoge-self.hage)
        self.hage = time.time()

if __name__ == '__main__':
    balanc = Balancer()
    balanc.Initialize()
    balanc.SetConstants(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]),1.3,21.7,86.3,350.0)
    while True:
        anglevelocity,angle = balanc.ReadGyro()
        robotspeed,robotposition = balanc.ReadEncoders()
        inp = balanc.CombineSensorValues(anglevelocity,angle,robotspeed,robotposition,balanc.Position())
        out = balanc.PID(0,inp)
        balanc.SetMotorPower(out)
        balanc.wait()
