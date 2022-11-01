#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import UInt8
import numpy as np
import signal
import os

import sys

class Tunnel():
    def __init__(self):
        rospy.init_node('tunnel_driver')
        self.pub_lane_following = rospy.Publisher('/core/lane_following', UInt8, queue_size=1)

        # plus
        rospy.Subscriber('/core/tunnel', UInt8, self.Mode, queue_size=1)
        self.pub_mission = rospy.Publisher('/core/obstacle', UInt8, queue_size=1)
        # plus 종로 될 때 정보 넘겨주고
        # lane following 은 미션 터널 플래그 온 되면 커주고,
        # 터널 종료될 때 켜주고

        self.cmd = UInt8()
        self.motor_msg = xycar_motor()
        self.distance = None

        self.Kp = 200
        self.Ki = 1
        self.Kd = 2
        self.error = 0
        self.end = 0.0 
        self.I_control = 0.0
        #self.Time = 0
        self.error_previous = 0.0
        self.PID_output = 0
        self.speed = 4
        self.tunnel_in_flag1 = 0
        self.tunnel_in_flag2 = 0

        self.rate = rospy.Rate(10)

        self.left_cnt = 0
        self.right_cnt = 0
        self.tunnel_flag = 0

        self.Is_fin = True

        self.is_triggered = False

        rospy.loginfo("Tunnel_start")
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.start()
            
            self.rate.sleep()

    def Mode(self, data):
        if data == 1:
            self.is_triggered = True
        elif data == 0:
            self.is_triggered = True

    def signal_handler(self, sig, frame):
        import time
        time.sleep(3)
        os.system('killall -9 python rosout')
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)

    def callback(self, data):
        self.distance = data.ranges


    def start(self, data):
        rospy.Subscriber('/scan', LaserScan, self.callback, queue_size = 1)
        self.motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        rospy.loginfo("start")

        while self.distance is None:
            continue


#delete this after
        self.cmd = 0
        self.pub_lane_following.publish(self.cmd)

        while self.Is_fin:
            #signal.signal(signal.SIGINT, signal_handler)
            if(self.tunnel_flag):
                pass
                #start = time.time()
                #rospy.loginfo("flag")
            else:
                #rospy.loginfo("init")
                self.left_cnt = 0
                self.right_cnt = 0
                self.tunnel_flag = 0
                self.motor_msg.angle = 0
                self.motor_msg.speed = self.speed
                self.motor.publish(self.motor_msg)



            while(self.tunnel_flag):
                self.left_angle = np.min(np.concatenate([self.distance[0:15], self.distance[345:]]))
                self.right_angle = np.min(self.distance[165:195])
                if(self.left_angle > 1.0 and (self.right_angle == float("inf"))):
                    self.right_angle = 0.1

                if(self.right_angle > 1.0 and (self.left_angle == float("inf"))):
                    self.left_angle = 0.1


                self.forward_angle = np.min(self.distance[75:105])
                
                rospy.loginfo("left_angle: {}".format(self.left_angle))
                rospy.loginfo("right_angle: {}".format(self.right_angle))
                rospy.loginfo("forward_angle: {}".format(self.forward_angle))
                
                #signal.signal(signal.SIGINT, signal_handler)
                if(self.motor_msg.angle == 50 or self.motor_msg.angle == -50):
                    self.tunnel_in_flag1 = self.motor_msg.angle

                if(((self.tunnel_in_flag1 == 50) and not(self.motor_msg.angle==50)) or ((self.tunnel_in_flag1 == -50) and not(self.motor_msg.angle==-50))):
                    self.tunnel_in_flag2=1

                if((self.forward_angle > 1.0) and self.tunnel_in_flag2):
                    #if(not(right_angle == float("inf") or left_angle == float("inf"))):

                    self.tunnel_in_flag1 = 0
                    self.tunnel_in_flag2 = 0
                    self.tunnel_flag = 0             
                    self.left_cnt = 0
                    self.right_cnt = 0
                    self.motor_msg.angle = - self.motor_msg.angle/2
                    self.motor_msg.speed = self.speed
                    self.motor.publish(self.motor_msg)


                    self.Is_fin = 0
                    self.cmd = 1
                    self.pub_lane_following.publish(self.cmd)
                    self.pub_mission.publish(self.cmd)

                    #time.sleep(2.0)
                    break


                if(self.forward_angle <0.15):
                    
                    self.motor_msg.angle = 0
                    self.motor_msg.speed = self.speed
                    self.motor.publish(self.motor_msg)

                if(self.right_angle < self.left_angle):
                    self.error = self.right_angle - ( self.right_angle + self.left_angle)/2
                elif(self.left_angle < self.right_angle):
                    self.error = (self.right_angle + self.left_angle)/2 - self.left_angle
                    
                self.P_control = self.Kp * self.error
                #I_control = I_control + (Ki * error * Time)
                #if(Time==0):
                #    Time=1
                #D_control = Kd * (error - error_previous)/Time
                self.PID_output += (self.P_control)
                #PID_output+= (P_control + I_control)
                #PID_output += (P_control + I_control + D_control)
                if(self.PID_output>1000):
                    self.PID_output = 1000
                elif(self.PID_output <-1000):
                    self.PID_output = -1000
                #print(P_control, I_control, D_control)
                #error_previous = error
                #print(Time)
            
                #print("angle",angle)
                rospy.loginfo("PID_output: {}".format(self.PID_output)+" motor_angle: {}".format(int(self.PID_output*0.05)))

                self.motor_msg.angle = int(self.PID_output*0.05)
                if(self.motor_msg.angle>50):
                    self.motor_msg.angle = 50
                elif(self.motor_msg.angle<-50):
                    self.motor_msg.angle = -50
                self.motor_msg.speed = self.speed
                self.motor.publish(self.motor_msg)
                #time.sleep(0.05)
                #end = time.time() 
                #Time = end - start
                #start = end


            for i in range (0,30):
                self.left_angle = np.min(np.concatenate([self.distance[0:15], self.distance[345:]]))
                self.forward_angle = np.min(self.distance[75:105])
                self.right_angle = np.min(self.distance[165:195])
                rospy.loginfo("left_angle: {}".format(self.left_angle))
                rospy.loginfo("right_angle: {}".format(self.right_angle))
                rospy.loginfo("forward_angle: {}".format(self.forward_angle))

                if(self.left_angle < 0.80):
                    self.left_cnt += 1
                    rospy.loginfo("left_cnt: {}".format(self.left_cnt))

                if(self.right_angle < 0.80):
                    self.right_cnt += 1
                    rospy.loginfo("right_cnt: {}".format(self.right_cnt))

                if (self.left_cnt>10) and (self.right_cnt > 10):
                    self.tunnel_flag = 1
                    break
                    
                self.rate.sleep()

if __name__ == '__main__':
    #rospy.init_node('tunnel_driver')
    node2 = Tunnel()
    #node2.start()
