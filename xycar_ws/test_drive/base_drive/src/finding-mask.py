#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Convenient way for choosing right Colour Mask to Detect needed Object
#
# Algorithm:
# Reading RGB image --> Converting to HSV --> Getting Mask
#
# Result:
# min_blue, min_green, min_red = 21, 222, 70
# max_blue, max_green, max_red = 176, 255, 255

from pickle import TRUE
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

class find_HSV():
    def __init__(self):
        # =============================================
        # 프로그램에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.image = np.empty(shape=[0])    # 카메라 이미지를 담을 변수
        self.bridge = CvBridge()            # OpenCV 함수를 사용하기 위한 브릿지 
        self.img_ready = False              # 카메라 토픽이 도착했는지의 여부 표시

        # =============================================
        # 프로그램에서 사용할 상수 선언부
        # =============================================
        self.CAM_FPS = 30                   # 카메라 FPS - 초당 30장의 사진을 보냄
        self.WIDTH, self.HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
        self.HALF_WIDTH = 320               # 카메라 이미지 가로 넓이의 절반

        # =============================================
        # 시작시 사용 변수 초기화
        # =============================================

        # =============================================
        # Bird Eye View 파라미터 값
        # =============================================

        self.top_x = 245
        self.top_y = 30
        self.bottom_x = 320
        self.bottom_y = 20

        # =============================================
        # 빛의 영향을 덜 받는 HSV 파라미터 값 지정
        # =============================================
        self.hue_white_l = 0
        self.hue_white_h = 180 # 180

        self.satu_white_l = 0
        self.satu_white_h = 70

        self.light_white_l = 160
        self.light_white_h = 255

        # Giving name to the window with Track Bars
        # And specifying that window is resizable

        cv2.namedWindow('Track Bars', cv2.WINDOW_NORMAL)

        # Defining Track Bars for convenient process of choosing colours
        # For minimum range
        cv2.createTrackbar('min_H', 'Track Bars', 0, 180, self.do_nothing)
        cv2.createTrackbar('min_S', 'Track Bars', 0, 255, self.do_nothing)
        cv2.createTrackbar('min_V', 'Track Bars', 0, 255, self.do_nothing)

        # For maximum range
        cv2.createTrackbar('max_H', 'Track Bars', 0, 180, self.do_nothing)
        cv2.createTrackbar('max_S', 'Track Bars', 0, 255, self.do_nothing)
        cv2.createTrackbar('max_V', 'Track Bars', 0, 255, self.do_nothing)

    def get_HSV(self, image):

        # Converting Original Image to HSV
        image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        cv2.namedWindow('Original Image', cv2.WINDOW_NORMAL)
        cv2.imshow('Original Image', image)


        # Defining loop for choosing right Colours for the Mask

        # Defining variables for saving values of the Track Bars
        # For minimum range
        min_H = cv2.getTrackbarPos('min_H', 'Track Bars')
        min_S = cv2.getTrackbarPos('min_S', 'Track Bars')
        min_V = cv2.getTrackbarPos('min_V', 'Track Bars')

        # For maximum range
        max_H = cv2.getTrackbarPos('max_H', 'Track Bars')
        max_S = cv2.getTrackbarPos('max_S', 'Track Bars')
        max_V = cv2.getTrackbarPos('max_V', 'Track Bars')

        # Implementing Mask with chosen colours from Track Bars to HSV Image
        # Defining lower bounds and upper bounds for thresholding
        mask = cv2.inRange(image_HSV,
                        (min_H, min_S, min_V),
                        (max_H, max_S, max_V))

        # Showing Binary Image with implemented Mask
        # Giving name to the window with Mask
        # And specifying that window is resizable
        cv2.namedWindow('Binary Image with Mask', cv2.WINDOW_NORMAL)
        cv2.imshow('Binary Image with Mask', mask)

        # Destroying all opened windows

#        print('min_H, min_S, min_V = {0}, {1}, {2}'.format(min_H, min_S,
#                                                                min_V))
#        print('max_H, max_S, max_V = {0}, {1}, {2}'.format(max_H, max_S,
#                                                                max_V))


    # =============================================
    # 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
    # 그 처리시간을 줄이기 위한 함수
    # =============================================

    def signal_handler(self, sig, frame):
        import time
        time.sleep(3)
        os.system('killall -9 python rosout')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Preparing Track Bars
    # Defining empty function
    def do_nothing(self, x):
        pass

    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img_ready = True

    def start(self):

        # 위에서 선언한 변수를 start() 안에서 사용하고자 함

        # =========================================
        # ROS 노드를 생성하고 초기화 함.
        # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
        # =========================================
        rospy.init_node('find_hsv')
        image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, self.img_callback)

        print("----- Xycar self driving -----")

        # 첫번째 카메라 토픽이 도착할 때까지 기다림.
        while not self.image.size == (self.WIDTH * self.HEIGHT * 3):
            continue

        # =========================================
        # 메인 루프 
        # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
        # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
        # 작업을 반복적으로 수행함.
        # =========================================
        while not rospy.is_shutdown():

            while self.img_ready == False:
                continue
            
            top_x = self.top_x
            top_y = self.top_y
            bottom_x = self.bottom_x
            bottom_y = self.bottom_y

            height = 350

            # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
            img = self.image.copy()
            self.img_ready = False

            # 가우시안 블러 처리하여 노이즈를 줄이고, 부드럽게 만듦
            cv_image_original = cv2.GaussianBlur(img, (5, 5), 0)

            # 원근 변환의 좌표 파라미터 값을 정하고, 매칭시킬 포인터를 지정시킴 
            pts_src = np.array([[self.HALF_WIDTH - top_x, height - top_y], [self.HALF_WIDTH + top_x, height - top_y],
                                [self.HALF_WIDTH + bottom_x, height + bottom_y], [self.HALF_WIDTH - bottom_x, height + bottom_y]])
            pts_dst = np.array([[0, 0], [self.WIDTH, 0], [self.WIDTH, self.HEIGHT], [0, self.HEIGHT]])

            # 정해준 파라미터 값들로 매칭시킴
            h, status = cv2.findHomography(pts_src, pts_dst)
            cv_image_homography = cv2.warpPerspective(cv_image_original, h, (self.WIDTH, self.HEIGHT))

            self.get_HSV(cv_image_homography)

            # 디버깅용 사다리꼴 시각화

            img = cv2.line(img, (self.HALF_WIDTH - top_x, height - top_y), (self.HALF_WIDTH + top_x, height - top_y), (0, 0, 255), 1)
            img = cv2.line(img, (self.HALF_WIDTH - bottom_x, height + bottom_y), (self.HALF_WIDTH + bottom_x, height + bottom_y), (0, 0, 255), 1)
            img = cv2.line(img, (self.HALF_WIDTH + bottom_x, height + bottom_y), (self.HALF_WIDTH + top_x, height - top_y), (0, 0, 255), 1)
            img = cv2.line(img, (self.HALF_WIDTH - bottom_x, height + bottom_y), (self.HALF_WIDTH - top_x, height - top_y), (0, 0, 255), 1)

            # 디버깅을 위해 모니터에 이미지를 디스플레이

            cv2.imshow("CAM View", img)
            cv2.waitKey(1)


# =============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
# =============================================
if __name__ == '__main__':
    node = find_HSV()
    node.start()
