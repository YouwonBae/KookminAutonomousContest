#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time #rospy는 ros node 를 작성하려면 import 해야 함.
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
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

def Parking_horizon():
    rospy.init_node('h_parking')
    pub_lane_following = rospy.Publisher('lane_following', UInt8, queue_size=1)

    #plus
    rospy.Subscriber('H_parking', UInt8, start, queue_size=1)
    pub_mission = rospy.Publisher('tunnel', UInt8, queue_size=1)
    #plus

    #=============================================
    # 프로그램에서 사용할 변수, 저장공간 선언부
    #=============================================
    roll, pitch, yaw = 0, 0, 0

    ##test
    self.image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
    self.bridge = CvBridge() # OpenCV 함수를 사용하기 위한 브릿지 
    ##test

    self.cmd = UInt8()
    self.motor_msg = xycar_motor()      # 모터 메세지 변수

    #test=============================================
    # 프로그램에서 사용할 상수 선언부
    #test=============================================
    self.CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
    self.WIDTH, self.HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

# def signal_handler(self, sig, frame):
#     import time
#     time.sleep(3)
#     os.system('killall -9 python rosout')
#     sys.exit(0)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
    self.arData = {  1: {"DX":0.0, "DY":0.0, "DZ":0.0,           #arData[KEY]["DX"] = i.pose.pose.position.x 이거 일 때 
            "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0},
            
            2: {"DX":0.0, "DY":0.0, "DZ":0.0, 
            "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0},
            
            8: {"DX":0.0, "DY":0.0, "DZ":0.0, 
            "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
            }

    self.id_list = []
    self.ultra_msg = None

    self.roll, self.pitch, self.yaw = 0, 0, 0

    self.image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
    self.bridge = CvBridge() # OpenCV 함수를 사용하기 위한 브릿지 

    self.parking_flag1=0
    self.parking_flag2=0
    self.parking_flag3=0

    self.KEY = 0

    self.distance1 = 0
    self.distance2 = 0
    self.rate = rospy.Rate(10)#1초에 10번 루프 0.1초 # parking_mission

    self.Is_fin = True

    self.is_triggered = False

    while not rospy.is_shutdown():
        if self.is_triggered == True:
            self.start()
        
        self.rate.sleep()

def Mode(self, data):
    if data == 1:
        self.is_triggered = True
    elif data == 0:
        self.is_triggered = False

#test=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 라는 변수에 옮겨 담음.
# 카메라 토픽의 도착을 표시하는 img_ready 값을 True로 바꿈.
#test=============================================
    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img_ready = True

#=============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
#=============================================
    def callback(self, msg):
        self.id_list = []

        for i in msg.markers:     
            self.KEY = i.id
            self.id_list.append(self.KEY)
            try:
                print(self.KEY)
                self.arData[self.KEY]["DX"] = i.pose.pose.position.x
                self.arData[self.KEY]["DY"] = i.pose.pose.position.y
                self.arData[self.KEY]["DZ"] = i.pose.pose.position.z

                self.arData[self.KEY]["AX"] = i.pose.pose.orientation.x
                self.arData[self.KEY]["AY"] = i.pose.pose.orientation.y
                self.arData[self.KEY]["AZ"] = i.pose.pose.orientation.z
                self.arData[self.KEY]["AW"] = i.pose.pose.orientation.w
            except:
                print("KEYERROR,", self.KEY)

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
    def drive(self, angle, speed):
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        
        self.motor.publish(self.motor_msg)

# 초음파 콜백 함수 - xycar_ultrasonic 토픽을 처리하는 콜백함수
    def ultra_callback(self, data):
        self.ultra_msg = data.data 
#=========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
#=========================================

#test=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#test=============================================
    def start(self, data):

        self.motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 ) # pub = rospy.Publisher('퍼블리시 할 대상 토픽',메시지 자료형,큐 사이즈)
        image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, self.img_callback)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback) # rospy.Subscriber('구독할 노드 토픽의 이름',구독할 토픽의 자료형, 새 메세지가 수신되면 메시지를 첫 번째 인수로 사용하여 콜백이 호출됨.)
        rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, self.ultra_callback)
    
        #=========================================
        # ROS 노드를 생성하고 초기화 함.
        # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
        #=========================================

        print ("----- H Parking -----")
        self.cmd = 0
        self.pub_mission.publish(self.cmd)

        # 첫번째 카메라 토픽이 도착할 때까지 기다림.
        # while not image.size == (WIDTH * HEIGHT * 3):
        #     continue
 
    #=========================================
    # 메인 루프 
    # 끊임없이 루프를 돌면서 
    # "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
        while self.Is_fin: # ctrl+C 누르기이전까지 계속 반복
            
        # cv2.imshow("image", image)
        # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
        # try:
        #     print('ultra5:{0} ultra6:{1} ultra7:{2}'.format(ultra_msg[5],ultra_msg[6],ultra_msg[7]))
        # except:
        #     print("NO")
            for key in self.id_list:
                
                if key==1:
                    (roll1,pitch1,yaw1)=euler_from_quaternion((self.arData[key]["AX"],self.arData[key]["AY"],self.arData[key]["AZ"], self.arData[key]["AW"]))      
                    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
                    roll1 = math.degrees(roll1)
                    pitch1 = math.degrees(pitch1)
                    yaw1 = math.degrees(yaw1)

                if (key==2):
                    (roll2,pitch2,yaw2)=euler_from_quaternion((self.arData[key]["AX"],self.arData[key]["AY"],self.arData[key]["AZ"], self.arData[key]["AW"]))       
                    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
                    roll2 = math.degrees(roll2)
                    pitch2 = math.degrees(pitch2)
                    yaw2 = math.degrees(yaw2)

                # DX값과 DY값을 이용해서 => DX,DZ값을 이용해서 거리값 distance 구하기
                self.distance1 = math.sqrt(pow(self.arData[1]["DX"],2) + pow(self.arData[1]["DZ"],2))
                self.distance2 = math.sqrt(pow(self.arData[2]["DX"],2) + pow(self.arData[2]["DZ"],2))
                # distance와 DX 사이의 각 계산 , theta = arcsin(DZ/distacne) => 이 각이 0이 되면 pitch 값이 0인 것과 동일 
            
                #'pitch : {0.3f} , x : {0.3f}, y : {0.3f}, z : {0.3f}, dis :{0.3f}'.format(pitch, arData["DX"], arData["DY"], arData["DZ"],distance)

                # roll , pitch, yaw, x,y,z 값 테스트
                #print(pitch, arData["DX"], arData["DY"], arData["DZ"],distance)
                # if (key==1):
                #     print('pitch1 : {0:0.3f} , x : {1:0.3f}, y : {2:0.3f}, z : {3:0.3f}, dis :{4:0.3f}, id:{5}'.format(pitch1, self.arData[key]["DX"], self.arData[key]["DY"], self.arData[key]["DZ"], self.distance1, key))
                if (key==2):
                    print('pitch2 : {0:0.3f} , x : {1:0.3f}, y : {2:0.3f}, z : {3:0.3f}, dis :{4:0.3f}, id:{5}'.format(pitch2, self.arData[key]["DX"], self.arData[key]["DY"], self.arData[key]["DZ"], self.distance2, key))
            # DX값 DY값 Yaw값 구하기
            #dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) +" Yaw:"+ str(round(yaw,1)) => round는 반올림해주는 함수


            # ->> pitch값이 맞음 yaw값이 아님!! -> 주차박스 안착했을 때 좌표 pitch, dis, x, y, z 1.75 , 0.034 , 0.027, 0.020, 0.168 
            # pitch 가 orientation 값 , DX가 x측, DZ가 y축 이라고 생각하면 편함 , 보편적 y축이 태그에서는 수직축임. 

            #주차 flag 시작
            #간판 artag의 artag토픽을 받아오면(태그의 좌표 거리로 판단!) parkingar_flag1 = 1 로 바꿔줌

            # id => 2: 수평주차 포지션 안정화 및 탈출
            # id를 KEY로 받아서 이후에는 KEY조건이랑 좌표 조건 모두 성립하면 하드코딩하는 식으로 코드 짜보기 

            #=> 직진만하고 후진하는 알고리즘 
            if (0.5<= self.distance2 <= 0.55 and self.parking_flag1==0): # pitch2: 3.945, x: 0.335, y: 0.059, z: 0.443, dis: 0.555, id:2  #artag1을 차량이 1/3정도 지나쳤을 때 artag2
                self.cmd = 0
                self.pub_lane_following.publish(self.cmd)
                for _ in range(0,60):
                    self.motor_msg.angle = -1
                    self.motor_msg.speed = 3
                    self.motor.publish(self.motor_msg)
                    self.rate.sleep()

                for _ in range(0,36):
                    self.motor_msg.angle = 50
                    self.motor_msg.speed = -3
                    self.motor.publish(self.motor_msg)
                    self.rate.sleep() # 바퀴를 오른쪽으로 꺾어서 후진하며 주차박스 외곽의 왼쪽의 가운데보다 살짝위에 오른뒷바퀴가 닿으면

                for _ in range(0,35):
                    self.motor_msg.angle = -50
                    self.motor_msg.speed = -3
                    self.motor.publish(self.motor_msg) 
                    if( self.ultra_msg[5] < 20 or self.ultra_msg[6] < 20 or self.ultra_msg[7] < 20):
                        self.motor_msg.angle = 0
                        self.motor_msg.speed = 0
                        self.motor.publish(self.motor_msg)
                    self.rate.sleep() # 바퀴를 왼쪽으로 꺾어서 후진하며 주차박스에 안착할 때까지 후진 

                for _ in range(0,10):
                    self.motor_msg.angle = 0
                    self.motor_msg.speed = 0
                    self.motor.publish(self.motor_msg) 
                    self.rate.sleep() # 후진하고나서 차량 수평 맞추기
                
                self.parking_flag1 = 1
                self.parking_flag2 = 1

            #if(pitch2>40 or pitch2<25):
            #    if(pitch2>40): # 차 기준 중앙보다 왼쪽
            #        for i in range(40, 25, -1)
                        

                #elif(pitch2<25): # 차 기준 중앙보다 오른쪽

            if(self.distance2>=0.23 and self.parking_flag2==1):
                self.motor_msg.angle = 0
                self.motor_msg.speed = 3
                self.motor.publish(self.motor_msg) 
                if(self.distance2<0.23):
                    self.motor_msg.angle = 0
                    self.motor_msg.speed = 0
                    self.motor.publish(self.motor_msg)             
            elif self.parking_flag2==1:
                for _ in range(0,15):
                    self.motor_msg.angle = 0
                    self.motor_msg.speed = 0
                    self.motor.publish(self.motor_msg) 
                    self.rate.sleep() # 전진하고나서 정지

                for _ in range(0,15):
                    self.motor_msg.angle = 0
                    self.motor_msg.speed = -3
                    self.motor.publish(self.motor_msg) 
                    self.rate.sleep() # 1.5초간 주차
                
                for _ in range(0,30):
                    self.motor_msg.angle = -50
                    self.motor_msg.speed = 4
                    self.motor.publish(self.motor_msg) 
                    self.rate.sleep()

                for _ in range(0,17):
                    self.motor_msg.angle = 50
                    self.motor_msg.speed = 4
                    self.motor.publish(self.motor_msg) 
                    self.rate.sleep()    

                for _ in range(0,15):
                    self.motor_msg.angle = 0
                    self.motor_msg.speed = 4
                    self.motor.publish(self.motor_msg) 
                    self.rate.sleep() # 탈출

                self.parking_flag1 =2
                self.parking_flag2 =2

                self.cmd = 1
                self.pub_lane_following.publish(self.cmd)

                #plus
                self.pub_mission.publish(self.cmd)
                self.Is_fin = False
                # 프로세서 종료시키기
                
                '''
                while(-70<=pitch2<=70): # 위치 가운데 라인에 차량이 위치하도록 조향
                    if(pitch2<-50): pitch2= -50
                    elif(pitch2>50): pitch2= 50
                    motor_msg.angle = -pitch2
                    motor_msg.speed = 0
                    motor.publish(motor_msg)
                    if(-6<=pitch2<=6):
                        motor_msg.angle = 0
                        motor_msg.speed = 0
                        motor.publish(motor_msg)
                        break
                '''

if __name__ == '__main__':
    rospy.init_node('h_parking') # rospy.init_node('노드의 이름을 알려줌') -> rospy에게 노드의 이름을 알려주어서 rospy는 이 정보를 얻을 때까지 ROS master 와 통신을 시작할 수 없음.
    node1 = Parking_horizon()
    #node1.start()