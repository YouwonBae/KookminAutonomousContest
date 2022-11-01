#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time #rospy는 ros node 를 작성하려면 import 해야 함.
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray # std_msgs.msg 는 퍼블리시를 위해 Int32MultiArray 메시지 유형을 재사용할 수 있도록 하기 위한 것
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

#target_yaw= 0.0 #PID제어의 목표값입니다.

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
arData = {  1: {"DX":0.0, "DY":0.0, "DZ":0.0,           #arData[KEY]["DX"] = i.pose.pose.position.x 이거 일 때 
            "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0},
            
            2: {"DX":0.0, "DY":0.0, "DZ":0.0, 
            "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0},
            
            8: {"DX":0.0, "DY":0.0, "DZ":0.0, 
            "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
        }

id_list = []
ultra_msg = None
# arData = {"DX":0.0, "DY":0.0, "DZ":0.0, 
#           "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0, "id":0}
roll, pitch, yaw = 0, 0, 0
roll1, pitch1, yaw1 = 0, 0, 0
roll2, pitch2, yaw2 = 0, 0, 0
##test
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() # OpenCV 함수를 사용하기 위한 브릿지 
##test

#test=============================================
# 프로그램에서 사용할 상수 선언부
#test=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기
parking_flag1=0
parking_flag2=0
parking_flag3=0
#test=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 라는 변수에 옮겨 담음.
# 카메라 토픽의 도착을 표시하는 img_ready 값을 True로 바꿈.
#test=============================================
def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True

#=============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
#=============================================
def callback(msg):
    global arData
    global id_list
    id_list = []

    for i in msg.markers:     
        KEY = i.id
        id_list.append(KEY)
        try:
            arData[KEY]["DX"] = i.pose.pose.position.x
            arData[KEY]["DY"] = i.pose.pose.position.y
            arData[KEY]["DZ"] = i.pose.pose.position.z

            arData[KEY]["AX"] = i.pose.pose.orientation.x
            arData[KEY]["AY"] = i.pose.pose.orientation.y
            arData[KEY]["AZ"] = i.pose.pose.orientation.z
            arData[KEY]["AW"] = i.pose.pose.orientation.w
        except:
            print("KEYERROR")

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

# 초음파 콜백 함수 - xycar_ultrasonic 토픽을 처리하는 콜백함수
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data 
#=========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
#=========================================
rospy.init_node('h_drive') # rospy.init_node('노드의 이름을 알려줌') -> rospy에게 노드의 이름을 알려주어서 rospy는 이 정보를 얻을 때까지 ROS master 와 통신을 시작할 수 없음.
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback) # rospy.Subscriber('구독할 노드 토픽의 이름',구독할 토픽의 자료형, 새 메세지가 수신되면 메시지를 첫 번째 인수로 사용하여 콜백이 호출됨.)

motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 ) # pub = rospy.Publisher('퍼블리시 할 대상 토픽',메시지 자료형,큐 사이즈)
rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)

distance1 = 0
distance2 = 0
rate = rospy.Rate(10)#1초에 10번 루프 0.1초 # parking_mission
#test=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#test=============================================
def start():

    global image, img_ready, motor_msg, arData, id_list
 
    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    #motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 

#=========================================
# 메인 루프 
# 끊임없이 루프를 돌면서 
# "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
# 작업을 반복적으로 수행함.
#=========================================
motor_msg = xycar_motor()
while not rospy.is_shutdown(): # ctrl+C 누르기이전까지 계속 반복


    # cv2.imshow("image", image)
    # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
    # try:
    #     print('ultra5:{0} ultra6:{1} ultra7:{2}'.format(ultra_msg[5],ultra_msg[6],ultra_msg[7]))
    # except:
    #     print("NO")
    for key in id_list:
        
        if key==1:
            (roll1,pitch1,yaw1)=euler_from_quaternion((arData[key]["AX"],arData[key]["AY"],arData[key]["AZ"], arData[key]["AW"]))      
            # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
            roll1 = math.degrees(roll1)
            pitch1 = math.degrees(pitch1)
            yaw1 = math.degrees(yaw1)

        if (key==2):
            (roll2,pitch2,yaw2)=euler_from_quaternion((arData[key]["AX"],arData[key]["AY"],arData[key]["AZ"], arData[key]["AW"]))       
            # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
            roll2 = math.degrees(roll2)
            pitch2 = math.degrees(pitch2)
            yaw2 = math.degrees(yaw2)

        # DX값과 DY값을 이용해서 => DX,DZ값을 이용해서 거리값 distance 구하기
        distance1 = math.sqrt(pow(arData[1]["DX"],2) + pow(arData[1]["DZ"],2))
        distance2 = math.sqrt(pow(arData[2]["DX"],2) + pow(arData[2]["DZ"],2))
        # distance와 DX 사이의 각 계산 , theta = arcsin(DZ/distacne) => 이 각이 0이 되면 pitch 값이 0인 것과 동일 
    
        #'pitch : {0.3f} , x : {0.3f}, y : {0.3f}, z : {0.3f}, dis :{0.3f}'.format(pitch, arData["DX"], arData["DY"], arData["DZ"],distance)

        # roll , pitch, yaw, x,y,z 값 테스트
        #print(pitch, arData["DX"], arData["DY"], arData["DZ"],distance)
    #     if (key==1):
    #         print('pitch1 : {0:0.3f} , x : {1:0.3f}, y : {2:0.3f}, z : {3:0.3f}, dis :{4:0.3f}, id:{5}'.format(pitch1, arData[key]["DX"],arData[key]["DY"], arData[key]["DZ"],distance1, key))
        if (key==2):
            print('pitch2 : {0:0.3f} , x : {1:0.3f}, y : {2:0.3f}, z : {3:0.3f}, dis :{4:0.3f}, id:{5}'.format(pitch2, arData[key]["DX"],arData[key]["DY"], arData[key]["DZ"],distance2, key))
    # # # DX값 DY값 Yaw값 구하기
    #dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) +" Yaw:"+ str(round(yaw,1)) => round는 반올림해주는 함수


    # ->> pitch값이 맞음 yaw값이 아님!! -> 주차박스 안착했을 때 좌표 pitch, dis, x, y, z 1.75 , 0.034 , 0.027, 0.020, 0.168 
    # pitch 가 orientation 값 , DX가 x측, DZ가 y축 이라고 생각하면 편함 , 보편적 y축이 태그에서는 수직축임. 

    #주차 flag 시작
    #간판 artag의 artag토픽을 받아오면(태그의 좌표 거리로 판단!) parkingar_flag1 = 1 로 바꿔줌

    # id => 2: 수평주차 포지션 안정화 및 탈출
    # id를 KEY로 받아서 이후에는 KEY조건이랑 좌표 조건 모두 성립하면 하드코딩하는 식으로 코드 짜보기 

    #=> 직진만하고 후진하는 알고리즘
    # 
    if parking_flag1 == 0:
        motor_msg.angle = -2
        motor_msg.speed = 4
        motor_pub.publish(motor_msg)
    if (0.5<=distance2<= 0.55 and parking_flag1==0): # pitch2: 3.945, x: 0.335, y: 0.059, z: 0.443, dis: 0.555, id:2  #artag1을 차량이 1/3정도 지나쳤을 때 artag2
        for _ in range(0,64): # 직진을 좀 더 (+1~3)정도로 조정하고
            motor_msg.angle = -1
            motor_msg.speed = 3
            motor_pub.publish(motor_msg)
            rate.sleep()

        for _ in range(0,34): # 오른쪽으로 핸들 꺾어 후진을 (+-1~3)정도 조정하면 조향 코드빼고 하드코딩만으로도 수평주차 완벽히 가능할지도
            motor_msg.angle = 50
            motor_msg.speed = -3
            motor_pub.publish(motor_msg)
            rate.sleep() # 바퀴를 오른쪽으로 꺾어서 후진하며 주차박스 외곽의 왼쪽의 가운데보다 살짝위에 오른뒷바퀴가 닿으면

        for _ in range(0,36): #35
            motor_msg.angle = -50
            motor_msg.speed = -3
            motor_pub.publish(motor_msg) 
            if(ultra_msg[5] <= 19 or ultra_msg[6] <= 19 or ultra_msg[7] <= 19):
                motor_msg.angle = 0
                motor_msg.speed = 0
                motor_pub.publish(motor_msg)
            rate.sleep() # 바퀴를 왼쪽으로 꺾어서 후진하며 주차박스에 안착할 때까지 후진 

        for _ in range(0,10):
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            rate.sleep() # 후진하고나서 차량 수평 맞추기
        
        parking_flag1 = 1
        parking_flag2 = 1
 
#    # 기존에는 pitch2 값으로 조향하였지만 dx값으로 조향하는 것으로 변경 >> 이게 맞는 것 같음
#     if ( arData[2]["DX"] >0.021 or arData[2]["DX"] < 0.021) and parking_flag3==0 and parking_flag1==1: # 기존에 들어가서 조향보다 좀 더 전후진이 보이도록 하기위해 for문 수정
#         if(arData[2]["DX"] >0.021): # 차가 중앙보다 왼쪽에 위치
#             for i in range(30, -1, -1):
#                 for _ in range(0,8):
#                     motor_msg.angle = i
#                     motor_msg.speed = 3
#                     motor_pub.publish(motor_msg) 
#                     rate.sleep()
#                 for _ in range(0,8):
#                     motor_msg.angle = 0
#                     motor_msg.speed = -3
#                     motor_pub.publish(motor_msg) 
#                     rate.sleep()
#                     if(ultra_msg[5] <= 19 or ultra_msg[6] <= 19 or ultra_msg[7] <= 19):
#                         motor_msg.angle = 0
#                         motor_msg.speed = 0
#                         motor_pub.publish(motor_msg)
#                 if (arData[2]["DX"] <=0.021): 
#                     parking_flag3=1 
#                     motor_msg.angle = 0
#                     motor_msg.speed = 0
#                     break
#         elif(arData[2]["DX"] < 0.021): # 차가 중앙보다 오른쪽을 바라 봄
#             for i in range(-30, 1, 1):
#                 for _ in range(0,8):
#                     motor_msg.angle = i
#                     motor_msg.speed = 3
#                     motor_pub.publish(motor_msg) 
#                     rate.sleep()
#                 for _ in range(0,8):
#                     motor_msg.angle = 0
#                     motor_msg.speed = -3
#                     motor_pub.publish(motor_msg) 
#                     rate.sleep()
#                     if(ultra_msg[5] <= 19 or ultra_msg[6] <= 19 or ultra_msg[7] <= 19):
#                         motor_msg.angle = 0
#                         motor_msg.speed = 0
#                         motor_pub.publish(motor_msg)
#                 if (arData[2]["DX"] >= 0.021): 
#                     parking_flag3=1
#                     motor_msg.angle = 0
#                     motor_msg.speed = 0
#                     break
        
#         parking_flag3=1
#         parking_flag2=1

    if(distance2>=0.21 and parking_flag2==1): #주차박스의 가운데에 위치하도록 태그와의 거리 조정 (+- 0.01 단위로 줄이거나 늘리며 주차박스 정가운데에 안착하도록 하기)
        motor_msg.angle = 0
        motor_msg.speed = 3
        motor_pub.publish(motor_msg) 
        if(distance2<0.21):
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 

    elif parking_flag2==1:
        for _ in range(0,24):
            motor_msg.angle = 0
            motor_msg.speed = 0
            motor_pub.publish(motor_msg) 
            rate.sleep() # 약 3초간 정차

        for _ in range(0,15):
            motor_msg.angle = 0
            motor_msg.speed = -3
            motor_pub.publish(motor_msg) 
            rate.sleep() 
        
        for _ in range(0,30):
            motor_msg.angle = -50
            motor_msg.speed = 4
            motor_pub.publish(motor_msg) 
            rate.sleep()

        for _ in range(0,20):
            motor_msg.angle = 50
            motor_msg.speed = 4
            motor_pub.publish(motor_msg) 
            rate.sleep()    

        for _ in range(0,15):
            motor_msg.angle = 0
            motor_msg.speed = 4
            motor_pub.publish(motor_msg) 
            rate.sleep() # 탈출

        parking_flag1 =2
        parking_flag2 =2

        '''
        while(-70<=pitch2<=70): # 위치 가운데 라인에 차량이 위치하도록 조향
            if(pitch2<-50): pitch2= -50
            elif(pitch2>50): pitch2= 50
            motor_msg.angle = -pitch2
            motor_msg.speed = 0
            motor_pub.publish(motor_msg)
            if(-6<=pitch2<=6):
                motor_msg.angle = 0
                motor_msg.speed = 0
                motor_pub.publish(motor_msg)
                break
        '''
