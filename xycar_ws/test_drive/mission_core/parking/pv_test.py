#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tkinter import ON
import rospy, time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, UInt8
from xycar_msgs.msg import xycar_motor
import numpy as np
import signal
import os
import sys

motor_msg = xycar_motor()
distance = None
ultra_msg = None

is_triggered = False
fin = False

def Mode(data):
    global is_triggered
    is_triggered = True

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    rospy.shutdown()
    os.system('killall -9 python rosout')
    sys.exit(0)

def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data 

def callback(data):
    global distance, motor_msg
    distance = data.ranges

 
rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

pub_following_lane = rospy.Publisher('lane_following', UInt8, queue_size=1)

rospy.Subscriber('V_parking', UInt8, Mode, queue_size=1)
pub_mission = rospy.Publisher('Obstacle', UInt8, queue_size=1)


rate = rospy.Rate(10)#1초에 10번 루프 0.1초

while distance is None:
    continue

def start():
    global ultra_msg, motor_msg, distance, fin

    parking_start_flag = False
    parking_success_flag = False
    parking_mission_end = False

    right_lidar_status = [0, 0]
    first_wall_flag = False
    second_wall_flag = False

    while distance is None:
        continue

    print('----- V_parking mission start -----')

    while not fin:
        signal.signal(signal.SIGINT, signal_handler)

        left_angle = np.min(np.concatenate([distance[0:10], distance[350:359]]))
        forward_angle = np.min(distance[75:105])
        right_angle = np.min(distance[170:190])

        print(right_angle)
        
        if right_angle < 0.7:
            right_lidar_status[0] = 1
        else:
            right_lidar_status[0] = 0

        if first_wall_flag is not True:
            if right_lidar_status[0] > right_lidar_status[1]:
                first_wall_flag = True
            else:
                first_wall_flag = False
        else:
            if second_wall_flag is not True:
                if right_lidar_status[0] > right_lidar_status[1]:
                    second_wall_flag = True
                else:
                    second_wall_flag = False

        if second_wall_flag is True:
            parking_start_flag = True
        else:
            parking_start_flag = False

        right_lidar_status[1] = right_lidar_status[0]

        if parking_start_flag is True:
            print('----- parking start -----')
            for i in range (0,20):
                motor_msg.angle = -50
                motor_msg.speed = 4
                motor_pub.publish(motor_msg)
                rate.sleep()
            for i in range (0,10):
                motor_msg.angle = 0
                motor_msg.speed = 0
                motor_pub.publish(motor_msg)
                rate.sleep()
            for i in range(0,20):
                motor_msg.angle = 50
                motor_msg.speed = -4
                motor_pub.publish(motor_msg)
                rate.sleep()
            for i in range (0,10):
                motor_msg.angle = 0
                motor_msg.speed = 0
                motor_pub.publish(motor_msg)
                rate.sleep()           
            while True:
                motor_msg.angle = 0
                motor_msg.speed = -4
                motor_pub.publish(motor_msg)
                if ultra_msg[6] < 8:
                    print('-----parking success-----')
                    motor_msg.angle = 0
                    motor_msg.speed = 0
                    motor_pub.publish(motor_msg)
                    parking_success_flag = True
                    break

        if parking_success_flag is True:
            for i in range (0,20):
                print('stay')
                motor_msg.angle = 0
                motor_msg.speed = 0
                motor_pub.publish(motor_msg)
                rate.sleep()
            print('----- escape -----')
            for i in range (0,10):
                motor_msg.angle = 0
                motor_msg.speed = 4
                motor_pub.publish(motor_msg)
                rate.sleep()

            for i in range (0,20):
                motor_msg.angle = 50
                motor_msg.speed = 4
                motor_pub.publish(motor_msg)
                rate.sleep()
            print('----- parking mission end -----')
            parking_mission_end = True

        if parking_mission_end is True:
            pub_following_lane.publish(UInt8(1))
            pub_mission.publish(UInt8(1))
            fin = True

# while not rospy.is_shutdown() and fin == False:
#     if is_triggered == True:
#         start()
#         is_triggered = False
#         break
#     rate.sleep()

if __name__ == '__main__':
    start()

