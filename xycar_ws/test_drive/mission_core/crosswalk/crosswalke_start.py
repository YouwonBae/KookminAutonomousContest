#!/usr/bin/env python
# -*- coding: utf-8 -*-
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import UInt8
from math import *
import signal
import sys
import os
import random

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() # OpenCV 함수를 사용하기 위한 브릿지 
motor = None # 모터 토픽을 담을 변수
img_ready = False # 카메라 토픽이 도착했는지의 여부 표시 

#===========================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기
ROI_ROW = 250   # 차선을 찾을 ROI 영역의 시작 Row값 
ROI_HEIGHT = HEIGHT - ROI_ROW   # ROI 영역의 세로 크기  
L_ROW = ROI_HEIGHT - 120  # 차선의 위치를 찾기 위한 기준선(수평선)의 Row값

cross_flag = False

is_triggered = False
fin = False

rospy.init_node('Crosswalk')

rate = rospy.Rate(10)#1초에 10번 루프 0.1초 # parking_mission

def Mode(data):
    global is_triggered
    is_triggered = True

pub_following_lane = rospy.Publisher('lane_following', UInt8, queue_size=1)

rospy.Subscriber('Crosswalk', UInt8, Mode, queue_size=1)

#=============================================
# White BGR
#=============================================
lower_white = np.array([0, 0, 176])#
upper_white = np.array([179, 70, 255])#
#=============================================
# 횡단보도 정지선만을 인식하기 위한 ROI 설정 함수
#=============================================
def region_of_interest(img, vertices, color3=(255, 255, 255), color1=255):  # ROI 셋팅

    mask = np.zeros_like(img)
    
    if len(img.shape) > 2:  # if color img
        color = color3  
    else:  # if gray img
        color = color1
        
    cv2.fillPoly(mask, np.array([vertices], dtype=np.int32), color) # int64가 디폴트 값이어서 오류가 났음. int32로 지정해주어 오류 해결.
    # vertices에 정한 점들로 이뤄진 ROI 설정부분을 위 대입한 color로 채움
    ROI_img = cv2.bitwise_and(img, mask)
     # 이미지와 <color로 채워진 ROI>를 합침
    
    return ROI_img

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 라는 변수에 옮겨 담음.
# 카메라 토픽의 도착을 표시하는 img_ready 값을 True로 바꿈.
#=============================================
def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True
    
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

#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================
def start():

    global image, img_ready, motor, cross_flag, fin

    speed = 3 #기본 속도

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    image_pub = rospy.Publisher("webcam_img",Image, queue_size=1)

    print ("----- crosswalk -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================

    while not fin:
        # 카메라 토픽이 도착할때까지 잠시 기다림
        while img_ready == False:
            continue
            
        img = image.copy()  # 이미지처리를 위한 카메라 원본이미지 저장 # 이미지를 읽음
        img_ready = False  # 카메라 토픽이 도착하면 콜백함수 안에서 True로 바뀜
	
        # 횡단보도 파트
        height, width = img.shape[:2] # 이미지의 높이와 너비를 불러옴
        vertices = np.array([(width/8, height / 4 ), (width/8, height), (width*0.75, height), (width*0.75,  height / 4)], dtype=np.int32)  # ROI 꼭짓점 설정
        cv2.rectangle(img, ((0, height / 4)), (int(width), int(height)), (255, 0, 0), 2) # ROI 시각화
        
        ROI_img_line = region_of_interest(img, vertices)
        # vertices에 정한 점들 기준으로 ROI 이미지 생성
        ROI_img_hsv = cv2.cvtColor(ROI_img_line,cv2.COLOR_BGR2HSV) # (입력 이미지, 색상 변환 코드)
        # White BGR to HSV
        white_hsv = cv2.inRange(ROI_img_hsv, lower_white, upper_white)
        # ROI_img_hsv의 이미지에서 범위 내의 픽셀들만 흰색, 나머지 검은색
        white_bitwise = cv2.bitwise_and(img, img, mask = white_hsv)
        # 바이너리 이미지를 마스크로 사용하여 원본이미지에서 범위 값에 해당하는 영상부분을 획득
        gray_line_img = cv2.cvtColor(white_bitwise, cv2.COLOR_BGR2GRAY)
        # 그레이스케일 이미지로 변경
        ret, line_threshold = cv2.threshold(gray_line_img, 100, 255, cv2.THRESH_BINARY) 
        #(img , threshold_value, value, flag)

        contours, hierarchy = cv2.findContours(line_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #contours, hierarchy = cv.findContours(img_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            contour = contours[0] 
            # 첫번째 contour에 대한 이미지를 구함 
            cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)  
            # Green으로 draw
        
        n_all_pix = float(np.sum(img>=0))          # ROI 안에 전체 픽셀
        #print(n_all_pix)                  #cv2.countNonZero()
        
        n_white_pix = float(np.sum(img>=200))      # ROI 안에 흰색 부분 픽셀
        #print(n_white_pix)
        
        _percent = float((n_white_pix / n_all_pix) * 100)
        print (_percent) #

        #rate = rospy.Rate(10)
        # 0 or 1인 것은 false or true로 하여 int형으로 저장 되는 것을 방지하기 위함.
        # int형으로 저장되면 다시 바꿔줘야하기 때문.
        if (_percent >= 10 and cross_flag is False):  # 흰색에 비율이 60퍼센트가 넘으면 
            # 5초간 정차하고 3초 이내에 출발.
            pub_following_lane.publish(UInt8(0))

            for _ in range(60): # (0,30)이나 (30)이나 결과적으로 같은 코드. 
            # 5초보다 여유 있게 5.3으로 설정하였음.
            # 차선과 센서 장치의 거리는 50mm
                #print('----- stay -----')
                speed = 0
                angle = -3
                drive(angle, speed)
                rate.sleep()

            cross_flag = True
            #print('----- keep going -----')

            for _ in range(20):
                angle = -3
                speed = 3
                drive(angle, speed)
                rate.sleep()

            #print('----- crosswalk end -----')
            fin = False
            pub_following_lane.publish(UInt8(1))
        
        # image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8")) #

        #angle = 0
        #speed = 3        
        #drive(angle, speed)
        
#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    #rospy.init_node('h_drive')
    start()

# while not rospy.is_shutdown() and fin == False:
#    if is_triggered == True:
#        start()
#        is_triggered = False
#        break
#    rate.sleep()
