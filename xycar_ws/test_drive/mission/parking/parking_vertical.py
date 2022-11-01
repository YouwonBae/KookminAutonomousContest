#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tkinter import ON
import rospy, time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
import numpy as np
import signal

motor_msg = xycar_motor()
distance = None
ultra_msg = None
Kp = 20
Ki = 1
Kd = 2
error = 0
end = 0.0 
I_control = 0.0
Time = 0
error_previous = 0.0


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

while distance is None:
    continue

rate = rospy.Rate(10)#1초에 10번 루프 0.1초

parking_cntl1 = 0
parking_cntr1 = 0
parking_cntl2 = 0
parking_cntr2 = 0
parking_flagl1 = 0
parking_flagr1 = 0
parking_flagl2 = 0
parking_flagr2 = 0
parking_flagl3 = 0
parking_flagr3 = 0

mission_cnt = 0
motor_msg.angle = 0
motor_msg.speed = 3
rospy.loginfo("start")
while not rospy.is_shutdown():
    signal.signal(signal.SIGINT, signal_handler)
    #rospy.loginfo(ultra_msg)#5,x,x,x,1,2,3,4

    left_angle = np.min(np.concatenate([distance[0:15], distance[345:]]))
    forward_angle = np.min(distance[75:105])
    right_angle = np.min(distance[165:195])
    motor_pub.publish(motor_msg)

    if(parking_flagl1 == 0 and parking_flagr1 == 0):
        parking_cntl1 = 0
        parking_cntl2 = 0
        for i in range (0,30):
            left_angle = np.min(np.concatenate([distance[0:15], distance[345:]]))
            forward_angle = np.min(distance[75:105])
            right_angle = np.min(distance[165:195])
            motor_pub.publish(motor_msg)
            if(left_angle < 0.80 and right_angle >0.80):
                rospy.loginfo("cntl")
                parking_cntl1 += 1

            elif(left_angle > 0.80 and right_angle <0.80):
                rospy.loginfo("cntr")
                parking_cntr1 += 1

            if (parking_cntl1>10):
                rospy.loginfo("flagl1")
                parking_flagl1 = 1
                break
                
            elif (parking_cntr1>10):
                rospy.loginfo("flagr1")
                parking_flagr1 = 1 
                break

            rate.sleep()

    if(parking_flagl1 and left_angle > 0.80 and (not parking_flagl2)):
        rospy.loginfo("flagl2")
        parking_flagl2 = 1

        
    elif(parking_flagr1 and right_angle > 0.80 and (not parking_flagr2)):
        rospy.loginfo("flagr2")
        parking_flagr2 = 1

    if(parking_flagl2 and left_angle < 0.80):
        rospy.loginfo("flagl3")
        parking_flagl3 = 1

        
    elif(parking_flagr2 and right_angle < 0.80):
        rospy.loginfo("flagr3")
        parking_flagr3 = 1

    if(mission_cnt > 50):
        rospy.loginfo("init")
        parking_flagl1 = 0
        parking_flagr1 = 0
        parking_flagl2 = 0
        parking_flagr2 = 0
        parking_flagl3 = 0
        parking_flagr3 = 0
    rate.sleep()
    mission_cnt += 1
    if(parking_flagr3 or parking_flagl3):#5(left),x,x,x,1(right),2,3,4
        if(parking_flagl3):
            for i in range (0,int(left_angle * 15)):
                motor_pub.publish(motor_msg)
                rate.sleep()
            for i in range (0,15):
                motor_pub.publish(motor_msg)
                motor_msg.angle = 50
                rate.sleep()
        else:
            for i in range (0,int(right_angle * 15)):
                motor_pub.publish(motor_msg)
                rate.sleep()
            for i in range (0,15):
                motor_pub.publish(motor_msg)
                motor_msg.angle = -50
                rate.sleep()
        rospy.loginfo("start")#5,x,x,x,1,2,3,4
	motor_msg.angle = 0
	motor_msg.speed = -4
	motor_pub.publish(motor_msg)
    
        while(parking_flagl3):
            try:
                    park_cnt = 0
		    left_angle = np.min(np.concatenate([distance[0:15], distance[345:]]))
		    right_angle = np.min(distance[165:195])
		    signal.signal(signal.SIGINT, signal_handler)
		    #rospy.loginfo(ultra_msg)#5,x,x,x,1,2,3,4
		    #rospy.loginfo(left_angle)#5,x,x,x,1,2,3,4
                    if(ultra_msg[0] < 50 and ultra_msg[4] < 50 and ultra_msg[5] < 50 and ultra_msg[6] < 50 and ultra_msg[7] < 50):
                        while(not(ultra_msg[6] < 35 or left_angle < 0.3 or right_angle < 0.3)):
                            if(ultra_msg[5] - ultra_msg[7] < -5):
		                motor_msg.angle = -50
                            elif(ultra_msg[7] - ultra_msg[5] < -5):
		                motor_msg.angle = 50
                            else:
		                motor_msg.angle = 0
		            motor_msg.speed = -4
		            motor_pub.publish(motor_msg)
		        motor_msg.angle = 0
		        motor_msg.speed = 0
		        motor_pub.publish(motor_msg)
		        rospy.loginfo("success")

			break
                        

		    if(ultra_msg[5] < 20 or ultra_msg[6] < 40 or ultra_msg[7] < 20):
                        park_cnt += 1
                        if(park_cnt < 3):
                            pass
                        park_cnt = 0
		        rospy.loginfo(ultra_msg)#5,x,x,x,1,2,3,4
		        rospy.loginfo("check1")
		        motor_msg.angle = 0
		        motor_msg.speed = 4
			for i in range (0,20):
		    		motor_pub.publish(motor_msg)
		    		rate.sleep()
                        if(ultra_msg[5] < ultra_msg[7]):
		            motor_msg.angle = -50
                        else:
		            motor_msg.angle = -50
			for i in range (0,20):
                                
		    	        motor_msg.speed = -4
		    		motor_pub.publish(motor_msg)
		    		rate.sleep()
		        motor_msg.angle = 0

		    elif(left_angle > 1.0 and ultra_msg[6] < 140 and right_angle >1.0):
		        motor_msg.angle = 0
		        motor_msg.speed = -4
		    motor_pub.publish(motor_msg) 

		    rate.sleep()
            except:
                    pass
        while(parking_flagr3):
            try:
                    park_cnt = 0
		    left_angle = np.min(np.concatenate([distance[0:15], distance[345:]]))
		    right_angle = np.min(distance[165:195])
		    signal.signal(signal.SIGINT, signal_handler)
		    #rospy.loginfo(ultra_msg)#5,x,x,x,1,2,3,4
		    #rospy.loginfo(left_angle)#5,x,x,x,1,2,3,4
                    if(ultra_msg[0] < 50 and ultra_msg[4] < 50 and ultra_msg[5] < 50 and ultra_msg[6] < 50 and ultra_msg[7] < 50):
                        while(not(ultra_msg[6] < 35 or left_angle < 0.3 or right_angle < 0.3)):
                            if(ultra_msg[5] - ultra_msg[7] < -5):
		                motor_msg.angle = -50
                            elif(ultra_msg[7] - ultra_msg[5] < -5):
		                motor_msg.angle = 50
                            else:
		                motor_msg.angle = 0
		            motor_msg.speed = -4
		            motor_pub.publish(motor_msg)
		        motor_msg.angle = 0
		        motor_msg.speed = 0
		        motor_pub.publish(motor_msg)
		        rospy.loginfo("success")

			break
                        

		    if(ultra_msg[5] < 20 or ultra_msg[6] < 40 or ultra_msg[7] < 20):
                        park_cnt += 1
                        if(park_cnt < 3):
                            pass
                        park_cnt = 0
		        rospy.loginfo(ultra_msg)#5,x,x,x,1,2,3,4
		        rospy.loginfo("check1")
		        motor_msg.angle = 0
		        motor_msg.speed = 4
			for i in range (0,20):
		    		motor_pub.publish(motor_msg)
		    		rate.sleep()
                        if(ultra_msg[5] < ultra_msg[7]):
		            motor_msg.angle = 50
                        else:
		            motor_msg.angle = 50
			for i in range (0,20):
                                
		    	        motor_msg.speed = -4
		    		motor_pub.publish(motor_msg)
		    		rate.sleep()
		        motor_msg.angle = 0

		    elif(left_angle > 1.0 and ultra_msg[6] < 140 and right_angle >1.0):
		        motor_msg.angle = 0
		        motor_msg.speed = -4
		    motor_pub.publish(motor_msg) 

		    rate.sleep()
            except:
                    pass

        rospy.loginfo("break")
 
        for i in range (0,30):
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            rate.sleep()
 
        for i in range (0,10):
            motor_msg.angle = 0
            motor_msg.speed = 4
            motor_pub.publish(motor_msg)
            rate.sleep()
        if(parking_flagl3):
            for i in range (0,35):
                motor_msg.angle = -50
                motor_msg.speed = 4
                motor_pub.publish(motor_msg)
                rate.sleep()
        else:
            for i in range (0,25):
                motor_msg.angle = 50
                motor_msg.speed = 4
                motor_pub.publish(motor_msg)
                rate.sleep()
        motor_msg.angle = 0
        motor_msg.speed = 0
        motor_pub.publish(motor_msg)
        break
'''
        motor_msg.angle = 0
        motor_msg.speed = 4
        motor_pub.publish(motor_msg)
        time.sleep(2)
        motor_pub.publish(motor_msg)
        time.sleep(2)
        motor_pub.publish(motor_msg)
        time.sleep(2)
        motor_pub.publish(motor_msg)
        time.sleep(2)
        
        motor_msg.angle = 0
        motor_msg.speed = 0
        motor_pub.publish(motor_msg)
        time.sleep(0.5)

        if(parking_flagr3): 
            motor_msg.angle = 50
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            time.sleep(0.5)
            motor_msg.angle = 50
            motor_msg.speed = -4
            motor_pub.publish(motor_msg) 
            time.sleep(6)
            motor_msg.angle = 50
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            time.sleep(0.5)
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            time.sleep(0.5)
            motor_msg.angle = 0
            motor_msg.speed = -4
            motor_pub.publish(motor_msg) 
            time.sleep(2) 
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            time.sleep(3)
            motor_msg.angle = 0
            motor_msg.speed = 4
            motor_pub.publish(motor_msg)
            time.sleep(2) 
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            time.sleep(0.5)
            motor_msg.angle = 50
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            time.sleep(0.5)
            motor_msg.angle = 50
            motor_msg.speed = 4
            motor_pub.publish(motor_msg)
            time.sleep(6)
            motor_msg.angle = 50
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            time.sleep(0.5)
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            time.sleep(0.5)
            motor_msg.angle = 0
            motor_msg.speed = 4
            motor_pub.publish(motor_msg) 

        elif(parking_flagl3): 
            motor_msg.angle = -50
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            time.sleep(0.5)
            motor_msg.angle = -50
            motor_msg.speed = -4
            motor_pub.publish(motor_msg) 
            time.sleep(6)
            motor_msg.angle = -50
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            time.sleep(0.5)
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            time.sleep(0.5)
            motor_msg.angle = 0
            motor_msg.speed = -4
            motor_pub.publish(motor_msg)
            time.sleep(2) 
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            time.sleep(3)
            motor_msg.angle = 0
            motor_msg.speed = 4
            motor_pub.publish(motor_msg)
            time.sleep(2) 
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            time.sleep(0.5)
            motor_msg.angle = -50
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            time.sleep(0.5)
            motor_msg.angle = -50
            motor_msg.speed = 4
            motor_pub.publish(motor_msg)
            time.sleep(6)
            motor_msg.angle = -50
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            time.sleep(0.5)
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) #정방향 조향
            time.sleep(0.5)
            motor_msg.angle = 0
            motor_msg.speed = 4
            motor_pub.publish(motor_msg) #직진
            rospy.loginfo("init")
            parking_flagl1 = 0
            parking_flagr1 = 0
            parking_flagl2 = 0
            parking_flagr2 = 0
            parking_flagl3 = 0
            parking_flagr3 = 0
'''


