#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
import numpy as np
import signal
import os

motor_msg = xycar_motor()
distance = None

Kp = 200
Ki = 1
Kd = 2
error = 0
end = 0.0 
I_control = 0.0
Time = 0
error_previous = 0.0
PID_output = 0
speed = 4
tunnel_in_flag1 = 0
tunnel_in_flag2 = 0

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    rospy.shutdown()
    os.system('killall -9 python rosout')
    sys.exit(0)


def callback(data):
    global distance, motor_msg
    distance = data.ranges

 
rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

rospy.loginfo("start")

while distance is None:
    signal.signal(signal.SIGINT, signal_handler)
    continue

rate = rospy.Rate(10)

left_cnt = 0
right_cnt = 0
tunnel_flag = 0

#delete this after
motor_msg.angle = 0
motor_msg.speed = speed
motor_pub.publish(motor_msg)

while not rospy.is_shutdown():
    signal.signal(signal.SIGINT, signal_handler)
    if(tunnel_flag):
         start = time.time()
         rospy.loginfo("flag")
    else:
         rospy.loginfo("init")
         left_cnt = 0
         right_cnt = 0
         tunnel_flag = 0
         motor_msg.angle = 0
         motor_msg.speed = speed
         motor_pub.publish(motor_msg)



    while(tunnel_flag):
         left_angle = np.min(np.concatenate([distance[0:15], distance[345:]]))
         right_angle = np.min(distance[165:195])
         if(left_angle>1.0 and (right_angle == float("inf"))):
             right_angle = 0.1

         if(right_angle>1.0 and (left_angle == float("inf"))):
             left_angle = 0.1


         forward_angle = np.min(distance[75:105])
         rospy.loginfo("left_angle: {}".format(left_angle))
         rospy.loginfo("right_angle: {}".format(right_angle))
         rospy.loginfo("forward_angle: {}".format(forward_angle))
         signal.signal(signal.SIGINT, signal_handler)
         if(motor_msg.angle==50 or motor_msg.angle== -50):
             tunnel_in_flag1=motor_msg.angle

         if(((tunnel_in_flag1 == 50) and not(motor_msg.angle==50)) or ((tunnel_in_flag1 == -50) and not(motor_msg.angle==-50))):
             tunnel_in_flag2=1

         if((forward_angle > 1.0) and tunnel_in_flag2):
             #if(not(right_angle == float("inf") or left_angle == float("inf"))):

                 tunnel_in_flag1 = 0
                 tunnel_in_flag2 = 0
                 tunnel_flag = 0             
                 left_cnt = 0
                 right_cnt = 0
                 motor_msg.angle = -motor_msg.angle/2
                 motor_msg.speed = speed
                 motor_pub.publish(motor_msg)
                 time.sleep(2.0)
                 break


         if(forward_angle <0.15):
             
             motor_msg.angle = 0
             motor_msg.speed = speed
             motor_pub.publish(motor_msg)

         if(right_angle<left_angle):
             error = right_angle - (right_angle+left_angle)/2
         elif(left_angle<right_angle):
             error = (right_angle+left_angle)/2 - left_angle
             
         P_control = Kp * error
         #I_control = I_control + (Ki * error * Time)
         #if(Time==0):
         #    Time=1
         #D_control = Kd * (error - error_previous)/Time
         PID_output += (P_control)
         #PID_output+= (P_control + I_control)
         #PID_output += (P_control + I_control + D_control)
         if(PID_output>1000):
             PID_output = 1000
         elif(PID_output <-1000):
             PID_output = -1000
         #print(P_control, I_control, D_control)
         #error_previous = error
         #print(Time)
    
         #print("angle",angle)
         rospy.loginfo("PID_output: {}".format(PID_output)+" motor_angle: {}".format(int(PID_output*0.05)))

         motor_msg.angle = int(PID_output*0.05)
         if(motor_msg.angle>50):
             motor_msg.angle = 50
         elif(motor_msg.angle<-50):
             motor_msg.angle = -50
         motor_msg.speed = speed
         motor_pub.publish(motor_msg)
         #time.sleep(0.05)
         end = time.time() 
         Time = end - start
         start = end



    for i in range (0,30):
        left_angle = np.min(np.concatenate([distance[0:15], distance[345:]]))
        forward_angle = np.min(distance[75:105])
        right_angle = np.min(distance[165:195])
        rospy.loginfo("left_angle: {}".format(left_angle))
        rospy.loginfo("right_angle: {}".format(right_angle))
        rospy.loginfo("forward_angle: {}".format(forward_angle))

        if(left_angle < 0.80):
            left_cnt += 1
            rospy.loginfo("left_cnt: {}".format(left_cnt))

        if(right_angle < 0.80):
            right_cnt += 1
            rospy.loginfo("right_cnt: {}".format(right_cnt))

        if (left_cnt>10) and (right_cnt > 10):
            tunnel_flag = 1
            break
            
        rate.sleep()



