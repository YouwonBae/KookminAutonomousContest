#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import
#선언부
# =============================================
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

STRAIGHT = 0
LEFT = 1
RIGHT = 2
NOT_EXIST = 3

class Driving():
    def __init__(self):
        # =============================================
        # 프로그램에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.image = np.empty(shape=[0])    # 카메라 이미지를 담을 변수
        self.bridge = CvBridge()            # OpenCV 함수를 사용하기 위한 브릿지 
        self.motor = None                   # 모터 토픽을 담을 변수
        self.img_ready = False              # 카메라 토픽이 도착했는지의 여부 표시

        self.motor_msg = xycar_motor()      # 모터 메세지 변수

        # =============================================
        # 프로그램에서 사용할 상수 선언부
        # =============================================
        self.CAM_FPS = 30                   # 카메라 FPS - 초당 30장의 사진을 보냄
        self.WIDTH, self.HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
        self.HALF_WIDTH = 320               # 카메라 이미지 가로 넓이의 절반

        # =============================================
        # 시작시 사용 변수 초기화
        # =============================================
        self.Init = True                    # 처음 시작에만 Init 함수 On 설정 
        self.lasterror = 0                  # 직전 angle error 값 초기화
        self.lastangle = 0
        self.mid_x = 320                    # 왼쪽, 오른쪽 차선 구분 ROI 영역 변수 처음의 mid_x 값은 화면의 중심

        self.reliability_left_line = 100    # 왼쪽 차선에 대한 신뢰도
        self.reliability_right_line = 100   # 오른쪽 차선에 대한 신뢰도
        self.center = 320                   # 카메라 이미지의 중심
        
        self.FRACTION_NUM = 2000#3300            # 차선이 정상적으로 검출됐는지 확인하는 기준(픽셀)
        self.MAX_FRACTION_NUM = 13000#6500       # 이 값이 넘어가면 오류를 검출한 것으로 판정
        
        self.TOWARD_CENTER = 325                 # 한쪽 차선만 검출됐을 때 중심을 향하게 하는 값

        self.left_base = 30
        self.right_base = 610

        self.mid_right = 400
        self.mid_left = 240

        self.mid_right_bef = 400
        self.mid_left_bef = 240

        self.lane_left_fit_bef = None
        self.lane_right_fit_bef = None

        self.left_fit = [1.0e-06, 1.0e-06, 20]
        self.right_fit = [1.0e-06, 1.0e-06, 620]

        self.oppsite_ROI = 525  # for straight # 535
        self.side_ROI = 50      # for straight

        # self.side_ROI = 100 이었음

        self.curve_oppsite = 525

        self.Is_curve = STRAIGHT

        # ploty 는 화면의 길이만큼 오름차순 숫자를 가지고 있는 리스트 0 ~ 479
        ploty = np.linspace(0, 479, 480)

        self.left_fitx = self.left_fit[0] * ploty ** 2 + self.left_fit[1] * ploty + self.left_fit[2]
        self.right_fitx = self.right_fit[0] * ploty ** 2 + self.right_fit[1] * ploty + self.right_fit[2]

        self.left_fraction = 0
        self.right_fraction = 0

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

        self.left_light_white_l = 160
        self.right_light_white_l = 160

        self.light_white_h = 255

        self.min_v = 139
        self.max_v = 178

        self.straight_cnt = 2

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

    # =============================================
    # 콜백함수 - 카메라 토픽을 처리하는 콜백함수
    # 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
    # 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
    # =============================================
    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img_ready = True

    # =============================================
    # 모터 토픽을 발행하는 함수  
    # 입력으로 받은 angle과 speed 값을 
    # 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
    # =============================================
    def drive(self, angle, speed):

        #motor_msg = xycar_motor()
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed

        self.motor.publish(self.motor_msg)

    # =============================================
    # BGR -> HSV 변환 후 범위를 지정하여 inRange
    # 처음 차선을 구분하는 ROI는 화면 픽셀의 중심인 320
    # inRange된 영역에서 차선이 검출된 길이를 계산하여
    # 값에 따라 신뢰성을 높이거나 낮춤
    # =============================================

    def InitmaskLane(self, image, left_or_right):
        img = image.copy()

        # 화면 중심점을 기준으로 왼쪽 차선의 경우 오른쪽 부분을 0으로 만들어 주고,
        # 오른쪽 차선의 경우 왼쪽 부분을 0으로 만들어 준다.
        if left_or_right == 'left':
            img[:, self.HALF_WIDTH:, ] = 0
        elif left_or_right == 'right':
            img[:, :self.HALF_WIDTH, ] = 0

        # 빛의 영향을 덜 받는 HSV 이미지로 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hue_l = self.hue_white_l
        hue_h = self.hue_white_h
        satu_l = self.satu_white_l
        satu_h = self.satu_white_h

        left_l = self.left_light_white_l
        right_l = self.right_light_white_l

        light_h = self.light_white_h

        if left_or_right == 'left':
            lower_white = np.array([hue_l, satu_l, left_l])
        elif left_or_right == 'right':
           lower_white = np.array([hue_l, satu_l, right_l])
        upper_white = np.array([hue_h, satu_h, light_h])

        # 설정한 파라미터 값으로 특정 색상 영역 추출
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 차선이 검출되는 영역의 크기, 길이를 계산하는 변수
        fraction_num = np.count_nonzero(mask)
        how_much_short = 0

        # 화면의 세로 길이에서 검출된 선의 길이를 계산
        for i in range(0, self.HEIGHT):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1

        how_much_short = self.HEIGHT - how_much_short

        # 선의 길이가 짧으면 신뢰성을 낮춰 조향하는데 사용하지 않도록 설정
        # 선의 길이가 길어지면 신뢰성을 높여 조향하는데 사용하도록 설정
        if left_or_right == 'left':
            if how_much_short > 80:
                if self.reliability_left_line >= 25:
                    self.reliability_left_line = 25
            elif how_much_short <= 120:
                if self.reliability_left_line <= 99:
                    self.reliability_left_line += 25

        elif left_or_right == 'right':
            if how_much_short > 80:
                if self.reliability_right_line >= 25:
                    self.reliability_right_line = 25
            elif how_much_short <= 120:
                if self.reliability_right_line <= 99:
                    self.reliability_right_line += 25

        # 차선이 검출되는 영역의 크기, 특정 색상으로 추출된 이미지 값 반환
        return fraction_num, mask

    # =============================================
    # BGR -> HSV 변환 후 범위를 지정하여 inRange
    # 차선을 구분하는 ROI는 검출된 차선으로 부터 차선 폭의 절반 만큼 증가, 감소한 영역 (210)
    # inRange된 영역에서 차선이 검출된 길이를 계산하여
    # 값에 따라 신뢰성을 높이거나 낮춤
    # =============================================
    def maskLane(self, image, lane_fit, left_or_right):
        img = image.copy() # 이미지를 복사하여 가져옴

        # ploty 는 화면의 길이만큼 오름차순 숫자를 가지고 있는 배열 0 ~ 479
        # ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])

        # lane_fitx 는 검출된 차선의 포물선에서의 x 좌표 (인덱스는 y)
        # lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        # 신뢰도에 따라 영역(ROI)을 재설정
        # 2차 포물선의 첫 번째 x 좌표(이미지 상에서 위쪽)로 ROI 영역을 재설정한다.

        self.mid_left_bef = self.mid_left
        self.mid_right_bef = self.mid_right

        if self.Is_curve in (LEFT, RIGHT):
            if left_or_right == 'left' and self.Is_curve == RIGHT:
                self.mid_left = int(lane_fit + self.side_ROI)  # 임의의 수,,
                self.mid_right = int(lane_fit + self.curve_oppsite) # 임의의 수,,

            elif left_or_right == 'right' and self.Is_curve == LEFT:
                self.mid_left = int(lane_fit - self.curve_oppsite) # 여유값 80
                self.mid_right = int(lane_fit - self.side_ROI)

        elif self.Is_curve == STRAIGHT:
            if left_or_right == 'left':
                if self.reliability_left_line > 50:
                    self.mid_left = int(lane_fit + self.side_ROI)  # 임의의 수,,
                    self.mid_right = int(lane_fit + self.oppsite_ROI) # 임의의 수,,

            elif left_or_right == 'right':
                if self.reliability_right_line > 50:
                    self.mid_left = int(lane_fit - self.oppsite_ROI) # 여유값 80
                    self.mid_right = int(lane_fit - self.side_ROI)

        elif self.Is_curve == NOT_EXIST:
            self.mid_left = 160
            self.mid_right = 480

   
        # ROI 영역의 최대, 최소크기 설정
        if self.mid_right > 640:
            self.mid_right = 640
        elif self.mid_right < 240:
            self.mid_right = 240
        
        if self.mid_left < 0:
            self.mid_left = 0
        elif self.mid_left > 400:
            self.mid_left = 400

        # ROI 영역에 맞춰 차선의 반대 영역은 0으로 초기화하여 인식하지 않도록 설정
        if left_or_right == 'left':
            img[:, self.mid_left:, ] = 0

        elif left_or_right == 'right':
            img[:, :self.mid_right, ] = 0

        # 빛의 영향을 덜 받는 HSV 이미지로 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hue_l = self.hue_white_l
        hue_h = self.hue_white_h
        satu_l = self.satu_white_l
        satu_h = self.satu_white_h

        left_l = self.left_light_white_l
        right_l = self.right_light_white_l
        
        light_h = self.light_white_h

        if left_or_right == 'left':
            lower_white = np.array([hue_l, satu_l, left_l])
        elif left_or_right == 'right':
            lower_white = np.array([hue_l, satu_l, right_l])
    
        upper_white = np.array([hue_h, satu_h, light_h])

        # 설정한 파라미터 값으로 특정 색상 영역 추출
        mask = cv2.inRange(hsv, lower_white, upper_white)

        
        # if left_or_right == 'left':
        #     cv2.imshow("left", mask)

        # elif left_or_right == 'right':
        #    cv2.imshow("right", mask)

        # 차선이 검출되는 영역의 크기, 길이를 계산하는 변수
        fraction_num = np.count_nonzero(mask)
        how_much_short = 0

        # 화면의 세로 길이에서 검출된 선의 길이를 계산
        for i in range(0, self.HEIGHT):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1

        how_much_short = self.HEIGHT - how_much_short

        # 선의 길이가 짧으면 신뢰성을 낮춰 조향하는데 사용하지 않도록 설정
        # 선의 길이가 길어지면 신뢰성을 높여 조향하는데 사용하도록 설정
        if left_or_right == 'left':
            if self.Is_curve == RIGHT:
                if how_much_short > 120:
                    if self.reliability_left_line >= 25:
                        self.reliability_left_line = 25
                elif how_much_short <= 120:
                    if self.reliability_left_line <= 99:
                        self.reliability_left_line += 25

            elif how_much_short > 80:
                if self.reliability_left_line >= 25:
                    self.reliability_left_line = 25
            elif how_much_short <= 80:
                if self.reliability_left_line <= 99:
                    self.reliability_left_line += 25

        elif left_or_right == 'right':
            if self.Is_curve == LEFT:
                if how_much_short > 120:
                    if self.reliability_right_line >= 25:
                        self.reliability_right_line = 25
                elif how_much_short <= 120:
                    if self.reliability_right_line <= 99:
                        self.reliability_right_line += 25

            elif how_much_short > 80:
                if self.reliability_right_line >= 25:
                    self.reliability_right_line = 25
            elif how_much_short <= 80:
                if self.reliability_right_line <= 99:
                    self.reliability_right_line += 25

        # 차선이 검출되는 영역의 크기, 특정 색상으로 추출된 이미지 값 반환
        return fraction_num, mask

    # =============================================
    # 이전의 곡선의 방정식 값을 수정하는 함수
    # 이전의 곡선의 방정식에서 margin 값의 범위에서 새로운 곡선을 탐색
    # =============================================
    def fit_from_lines(self, lane_fit, image, left_or_right):

        # 검출된 차선의 이미지로부터 변수 초기화
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # 새 곡선을 찾는 범위 값
        margin_base = 22#20
        margin_oppo = 16

        # lane_inds는 margin 범위 내에 선이 있는지 확인

        if left_or_right == 'left':
            lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin_oppo)) & (
                    nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin_base)))
        elif left_or_right == 'right':
            lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin_base)) & (
                    nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin_oppo)))

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # polyfit으로 2차 곡선을 찾아 계수를 반환
        lane_fit = np.polyfit(y, x, 2)

        # ploty 는 화면의 길이만큼 오름차순 숫자를 가지고 있는 리스트 0 ~ 479
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])

        # lane_fitx 는 검출된 차선의 포물선에서의 x 좌표 (인덱스는 y)
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
        
        if left_or_right == 'left':
            if abs(lane_fit[0]) > 0.00035 or abs(lane_fit[1] > 0.8):
                self.left_base = self.left_fitx[0]
                return self.left_fitx, self.left_fit
            else:
                self.left_base = lane_fitx[0]

        elif left_or_right == 'right':
            if abs(lane_fit[0]) > 0.00035 or abs(lane_fit[1] > 0.8):
                self.right_base = self.right_fitx[0]
                return self.right_fitx, self.right_fit
            else:
                self.right_base = lane_fitx[0]

        # 포물선의 x 좌표 리스트, 포물선의 계수 반환
        return lane_fitx, lane_fit

    # =============================================
    # 왼쪽, 오른쪽 차선에 대한 2차 방정식을 만드는 함수
    # 이미지를 윈도우 영역의 크기만큼 분해하여 탐색
    # =============================================
    def sliding_window(self, img_w, left_or_right):

        print("sliding!!")
        
        # 받아온 이미지를 높이를 반으로 나눈 inRange 단색 이미지를 
        # 세로축에서 감지된 픽셀 수를 가로 직선 리스트로 나타냄
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        # 이미지를 그리기 위한 배경 생성
        out_img = np.dstack((img_w, img_w, img_w)) * 255

        # 선 검출 ROI영역을 기준으로 midpoint 설정 
        #midpoint = self.mid_x

        # 차선에 따라 histogram 최대값의 좌표를 lane_base로 설정
        try:
            if left_or_right == 'left':
                lane_base = np.argmax(histogram[:self.mid_left]) # 가운데 라인을 피하기 위한 추가 수식
            elif left_or_right == 'right':
                lane_base = np.argmax(histogram[self.mid_right:]) + (self.mid_right) # 가운데 라인을 피하기 위한 추가 수식

        #  histogram을 구할 수 없을 때 lane_base는 각 차선에서 검출될 수 있는 가장 끝 지점으로 설정
        except:
            if left_or_right == 'left':
                lane_base = 32
            elif left_or_right == 'right':
                lane_base = 607

        # sliding windows의 분해능 설정
        nwindows = 40

        # 각 window 영역의 높이를 설정
        window_height = np.int(img_w.shape[0] / nwindows)

        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        x_current = lane_base
        margin = 24
        minpix = 32

        lane_inds = []

        # 설정한 window 분해능만큼 실행
        for window in range(nwindows):
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            # 디버깅용 sliding window 시각화
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            # window 영역에서 0이 아닌 값을 가져옴
            good_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (
                    nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]

            lane_inds.append(good_lane_inds)

            # window 안에서 감지된 픽셀 수가 32 이상일 때 window의 x축 기준을 x 좌표 평균으로 재조정함
            if len(good_lane_inds) > minpix:
                x_current = np.int(np.mean(nonzerox[good_lane_inds]))

        lane_inds = np.concatenate(lane_inds)

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # window 영역에서 검출한 좌표로 2차 곡선을 찾아 계수를 반환
        try:
            lane_fit = np.polyfit(y, x, 2)
            if left_or_right == 'left':
                self.lane_left_fit_bef = lane_fit
            elif left_or_right == 'right':
                self.lane_right_fit_bef = lane_fit
        except:
            if left_or_right == 'left':
                if self.lane_left_fit_bef is not None:
                    lane_fit = self.lane_left_fit_bef
                else:
                    lane_fit = [0, 0, 30]
            elif left_or_right == 'right':
                if self.lane_right_fit_bef is not None:
                    lane_fit = self.lane_right_fit_bef
                else:
                    lane_fit = [0, 0, 610]

        # ploty 는 화면의 길이만큼 오름차순 숫자를 가지고 있는 리스트 0 ~ 479
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])

        # lane_fitx 는 검출된 차선의 포물선에서의 x 좌표 (인덱스는 y)
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        # 포물선의 x 좌표 리스트, 포물선의 계수 반환
        
        if left_or_right == 'left':
            self.left_base = lane_fitx[0]

        elif left_or_right == 'right':
            self.right_base = lane_fitx[0]
        
        return lane_fitx, lane_fit

    # =============================================
    # 각 차선에 대해 검출되는 픽셀수에 따라 2차 함수의 계수를 갱신할지 정하고
    # 이미지에서 2차 곡선을 찾아 angle 값을 계산해주는 함수를 호출하는 함수
    # =============================================

    ## self.move_avg 에 대해서 사용할 필요가 없다면 지우자

    def cbFindLane(self, image, left_or_right):

        # 코드를 처음 시행할 때 카메라의 중심점을 기준으로 양 차선을 구분짓는다.
        if self.Init == True:
            if left_or_right == 'left':
                self.left_fraction, left_img = self.InitmaskLane(image, left_or_right)
            elif left_or_right == 'right':
                self.right_fraction, right_img = self.InitmaskLane(image, 'right')
                self.Init = False

        # 이후에는 ROI를 유동적으로 변화시켜 차선 인식에 대한 오류를 줄임
        else:
            if left_or_right == 'left':
                self.left_fraction, left_img = self.maskLane(image, self.left_base, 'left') # self.left_fit no

            elif left_or_right == 'right':
                self.right_fraction, right_img = self.maskLane(image, self.right_base, 'right')

        if self.Is_curve in (LEFT, RIGHT):
            if self.left_fraction + self.right_fraction >= (self.MAX_FRACTION_NUM ) * 2.3:
                if left_or_right == 'right':
                    if self.Is_curve == LEFT:
                        self.right_light_white_l += 2
                        if self.right_light_white_l >= self.max_v:
                            self.right_light_white_l = self.max_v
                elif left_or_right == 'left':
                    if self.Is_curve == RIGHT:
                        self.left_light_white_l += 2
                        if self.left_light_white_l >= self.max_v:
                            self.left_light_white_l = self.max_v
            
            elif self.left_fraction + self.right_fraction <= self.FRACTION_NUM:
                if left_or_right == 'right':
                    self.left_light_white_l -= 2
                    if self.left_light_white_l <= self.min_v:
                        self.left_light_white_l = self.min_v
                elif left_or_right == 'left':
                    self.right_light_white_l -= 2
                    if self.right_light_white_l <= self.min_v:
                        self.right_light_white_l = self.min_v

        elif self.Is_curve != NOT_EXIST:
            if left_or_right == 'left':
                if self.left_fraction >= self.MAX_FRACTION_NUM * 1.2:
                    self.left_light_white_l += 2
                    if self.left_light_white_l >= self.max_v:
                        self.left_light_white_l = self.max_v

            elif left_or_right == 'right':
                if self.right_fraction >= self.MAX_FRACTION_NUM * 1.2:
                    self.right_light_white_l += 2
                    if self.right_light_white_l >= self.max_v:
                        self.right_light_white_l = self.max_v

            if self.left_fraction <= self.FRACTION_NUM and self.right_fraction <= self.FRACTION_NUM * 1.5:
                if left_or_right == 'left':
                    self.left_light_white_l -= 2
                    if self.left_light_white_l <= self.min_v:
                        self.left_light_white_l = self.min_v

            if self.right_fraction <= self.FRACTION_NUM and self.left_fraction <= self.FRACTION_NUM * 1.5:
                if left_or_right == 'right':
                    self.right_light_white_l -= 2
                    if self.right_light_white_l <= self.min_v:
                        self.right_light_white_l = self.min_v            

        
        else:
            if left_or_right == 'left':
                self.left_light_white_l -= 2
                if self.left_light_white_l <= self.min_v:
                    self.left_light_white_l = self.min_v
            elif left_or_right == 'right':
                self.right_light_white_l -= 2
                if self.right_light_white_l <= self.min_v:
                    self.right_light_white_l = self.min_v
        




        # 차선이 정상적으로 검출 됐을 때 실행(픽셀 기준을 넘었을 때)
        # 각 차선에 2차 함수 계수를 가지고 있을 때 fit_from_lines 함수를 통해 2차 곡선을 기준으로 재탐색

        if left_or_right == 'left':
            try:                
                #2.6
                if self.MAX_FRACTION_NUM * 2.6> self.left_fraction > self.FRACTION_NUM and self.reliability_left_line > 50:
                #if self.left_fraction > self.FRACTION_NUM and self.reliability_left_line > 50:
                    self.left_fitx, self.left_fit = self.fit_from_lines(self.left_fit, left_img, 'left')
                if self.left_fraction > self.mid_left * 446:
                    print("BEF")
                    self.mid_left = self.mid_left_bef
                    self.mid_right = self.mid_right_bef
            except:
                print("er")
                if self.MAX_FRACTION_NUM > self.left_fraction > self.FRACTION_NUM:
                    self.left_fitx, self.left_fit = self.sliding_window(left_img, 'left')

            # 차선이 정상적으로 검출 됐을 때 실행(픽셀 기준을 넘었을 때)
            # 각 차선에 2차 함수 계수가 없을 때 sliding_window 함수를 통해 ROI를 기준으로 2차 곡선 탐색

        elif left_or_right == 'right':
            try:
                if self.MAX_FRACTION_NUM * 2.6 > self.right_fraction > self.FRACTION_NUM and self.reliability_right_line > 50:
                #if self.right_fraction > self.FRACTION_NUM and self.reliability_right_line > 50:
                    self.right_fitx, self.right_fit = self.fit_from_lines(self.right_fit, right_img, 'right')
                #if self.right_fraction > ( 640 - self.mid_right ) * 459:
                #    self.mid_left = self.mid_left_bef
                #    self.mid_right = self.mid_right_bef                    
            except:
                print("er")
                if self.MAX_FRACTION_NUM > self.right_fraction > self.FRACTION_NUM:
                    self.right_fitx, self.right_fit = self.sliding_window(right_img, 'right')

        # 양쪽 차선의 검출된 픽셀값에 따라 추종 라인을 생성
        # 디버깅을 위한 final 이미지와 추종하는 각도 값을 반환
        return True

    # =============================================
    # 각 라인에 대한 신뢰도와 2차 곡선에 따라서 추종라인을 그려주는 함수
    # 양쪽 차선에 대해 신뢰도 기준이 만족되면 두 차선의 평균값으로 추종라인을 그린다
    # 한쪽 차선에 대해서만 신뢰도 기준이 만족되면 한쪽 차선이 정상적으로 검출됐을 때 차선의 중심이 되는 값을 더하거나 빼서 추종라인을 그린다
    # =============================================
    def make_lane(self, image):
        # 이미지의 크기만큼 0으로 채워져 있는 공간 생성
        warp_zero = np.zeros((image.shape[0], image.shape[1], 1), dtype=np.uint8)

        # 디버깅을 위한 출력화면을 그릴 수 있는 공간 생성
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

        # ploty 는 화면의 길이만큼 오름차순 숫자를 가지고 있는 리스트 0 ~ 479
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])

        # 차선이 정상적으로 검출 됐을 때 실행(픽셀 기준을 넘었을 때)
        # pts_left, pts_right 는 2차 곡선의 x 좌표와 화면의 높이만큼의 리스트 정보를 가지고 있는 numpy.array
        # 디버깅을 위해 pts_left, pts_right 를 color_warp_lines 캔버스에 그림
        #if self.MAX_FRACTION_NUM * 1.4 > self.left_fraction > self.FRACTION_NUM and self.reliability_left_line > 50:
        if self.left_fraction > self.FRACTION_NUM and self.reliability_left_line > 50:
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)
        #if self.MAX_FRACTION_NUM * 1.4 > self.right_fraction > self.FRACTION_NUM and self.reliability_right_line > 50:
        if self.right_fraction > self.FRACTION_NUM and self.reliability_right_line > 50:
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)

        # 추종 x 좌표를 가지고 있는 것으로 설정
        self.is_center_x_exist = True

        # 신뢰도와 정상차선의 픽셀을 기준으로 실행
        # 두 차선이 검출될 때, 한쪽 차선만 검출될 때, 검출되지 않을 때 각각 다른 동작을 수행

        # 두 차선의 신뢰도 기준을 만족했을 때
        if self.reliability_left_line > 50 and self.reliability_right_line > 50:
            # 두 차선이 검출될 때 추종라인은 두 차선의 평균값으로 계산
            #if self.MAX_FRACTION_NUM > self.left_fraction > self.FRACTION_NUM and self.MAX_FRACTION_NUM > self.right_fraction > self.FRACTION_NUM:
            if self.left_fraction > self.FRACTION_NUM and self.right_fraction > self.FRACTION_NUM and self.Is_curve == STRAIGHT:
                centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
                pts = np.hstack((pts_left, pts_right))
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                # 디버깅을 위해 추종라인을 color_warp_lines 캔버스에 그림, 두 차선 영역을 색칠하여 디버깅이 용이하도록 함
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255),
                              thickness=12)

            # 한쪽 차선만 정상적으로 검출됐을 때 차선의 중심이 되는 값을 더하거나 빼서 추종라인을 계산
            #elif self.MAX_FRACTION_NUM > self.left_fraction > self.FRACTION_NUM and (self.right_fraction <= self.FRACTION_NUM or self.right_fraction >= self.MAX_FRACTION_NUM):
            
            elif self.Is_curve == RIGHT:
            #elif self.left_fraction > self.FRACTION_NUM and self.right_fraction <= self.FRACTION_NUM:
                try:
                    #centerx = np.subtract(self.right_fitx, self.TOWARD_CENTER)
                    centerx = np.add(self.left_fitx, self.TOWARD_CENTER)
                except:
                    print("no attribute 'left_fitx'")
                    centerx = np.full(self.HEIGHT, self.HALF_WIDTH)
                
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255),
                              thickness=12)

            #elif (self.left_fraction >= self.MAX_FRACTION_NUM or self.left_fraction <= self.FRACTION_NUM) and self.MAX_FRACTION_NUM > self.right_fraction > self.FRACTION_NUM:
            elif self.Is_curve == LEFT:
            #elif self.left_fraction <= self.FRACTION_NUM and self.right_fraction > self.FRACTION_NUM:
                try:
                    #centerx = np.add(self.left_fitx, self.TOWARD_CENTER)
                    centerx = np.subtract(self.right_fitx, self.TOWARD_CENTER)
                except:
                    print("no attribute 'right_fitx'")
                    centerx = np.full(self.HEIGHT, self.HALF_WIDTH)

                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255),
                              thickness=12)

            # 갑작스럽게 차선 인식이 실패했을 때 중심으로 향하도록 만듦
            else:
                centerx = np.full(self.HEIGHT, self.HALF_WIDTH)

        # 한쪽 차선만 신뢰도 기준을 만족했을 때
        # 신뢰도를 만족한 차선에서 중심이 되는 값을 더하거나 빼서 추종라인을 계산
        elif self.reliability_left_line <= 50 and self.reliability_right_line > 50:
            centerx = np.subtract(self.right_fitx, self.TOWARD_CENTER)
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        elif self.reliability_left_line > 50 and self.reliability_right_line <= 50:
            centerx = np.add(self.left_fitx, self.TOWARD_CENTER)
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        # 두 차선에 대해서 신뢰도 기준을 만족하지 못할 때 실행
        # 핸들을 중심으로 서서히 풀며, 라인을 다시 찾도록 ROI를 화면의 중심선상으로 만듦
        else:
            self.is_center_x_exist = False
            self.center = self.center + (self.HALF_WIDTH - self.center)
            
            self.Is_curve = NOT_EXIST
            #self.mid_x = self.HALF_WIDTH
            #self.mid_left = 200
            #self.mid_right = 440

        # 디버깅을 위해 final 이미지에 추종라인, 영역을 그림
        final = cv2.addWeighted(image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        # center_x가 존재할 때 추종라인의 특정지점의 x 좌표로 조향
        if self.is_center_x_exist == True:
            self.center = centerx.item(230)

        # 중심점에서 떨어진 위치를 기준으로 angle값을 계산
        error = self.center - self.HALF_WIDTH

        Kp = 0.34 # 비례 이득 값 # 5 : 0.4,  6 : 0.45        11 : 0.2
        Kd = 0.27 # 0.25   # 미분 이득 값 # 5 : 0.65 6 : 0.65     11 : 0.65

        # =========================================
        # 핸들조향각 값인 angle값 정하기.
        # 차선의 위치 정보를 이용해서 angle값을 설정함.
        # PD 제어법을 사용하여 부드러운 조향이 가능하도록 설정함
        # 최대, 최소 값을 넘어가지 않도록 설정
        # =========================================
        angle = Kp * error - Kd * ( error - self.lasterror )

        self.lasterror = error

        if angle > 50:
            angle = 50
        elif angle < -50:
            angle = -50

        # 디버깅을 위한 이미지, 추종 각을 반환
        return final, angle

    # =============================================
    # 실질적인 메인 함수 
    # 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
    # 차선의 위치를 파악한 후에 조향각을 결정하고,
    # 최종적으로 모터 토픽을 발행하는 일을 수행함. 
    # =============================================
    def start(self):

        # 위에서 선언한 변수를 start() 안에서 사용하고자 함

        # =========================================
        # ROS 노드를 생성하고 초기화 함.
        # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
        # =========================================
        rospy.init_node('driving')
        self.motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
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

            # BGR -> HSV 하얀선 라인 검출
            self.cbFindLane(cv_image_homography, 'left')
            self.cbFindLane(cv_image_homography, 'right')
            
            #0.000028
            if (self.Is_curve != LEFT and abs(self.left_fit[0] * self.left_fit[1]) > 0.00003 and self.reliability_left_line >= 50):
                if self.left_fit[0] > 0:
                    self.Is_curve = RIGHT
                else:
                    self.Is_curve = LEFT
            elif (self.Is_curve != RIGHT and abs(self.right_fit[0] * self.right_fit[1]) > 0.00003 and self.reliability_right_line >= 50):
                if self.right_fit[0] > 0:
                    self.Is_curve = RIGHT
                else:
                    self.Is_curve = LEFT
            elif self.left_fit[1] < -0.35 and self.reliability_left_line >= 50: #0.35
                self.Is_curve = RIGHT
            elif self.right_fit[1] > 0.35 and self.reliability_right_line >= 50:
                self.Is_curve = LEFT
            elif self.reliability_right_line >= 50 or self.reliability_left_line >= 50:
                self.Is_curve = STRAIGHT
            else:
                self.Is_curve = NOT_EXIST

            # debug
            '''
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            hue_l = self.hue_white_l
            hue_h = self.hue_white_h
            satu_l = self.satu_white_l
            satu_h = self.satu_white_h
            light_l = self.light_white_l
            light_h = self.light_white_h

            lower_white = np.array([hue_l, satu_l, light_l])
            upper_white = np.array([hue_h, satu_h, light_h])

            # 설정한 파라미터 값으로 특정 색상 영역 추출
            mask = cv2.inRange(hsv, lower_white, upper_white)
            cv2.imshow("hsv", mask)
            '''

            
            # try:
            #     print("left_fit :", self.left_fit, "right_fit :", self.right_fit)
            # except:
            #     pass

            final, angle = self.make_lane(cv_image_homography)

            #print("IS_CURVE : ", self.Is_curve, "left_l :", self.left_light_white_l, "right_l :", self.right_light_white_l, 'left F : ', self.left_fraction, 'right F : ', self.right_fraction)
            # # #print("LEFT :", abs(self.left_fit[0] * self.left_fit[1]), "RIGHT :", abs(self.right_fit[0] * self.right_fit[1]))
            #print("mid_left :", self.mid_left, "mid_right :", self.mid_right)


            # Bird View + HSV 하얀선 이미지 디스플레이
            # cv2.imshow("Bird View", final)
            #cv2.imshow("Bird View", cv_image_homography)

            # 디버깅용 사다리꼴 시각화

            # img = cv2.line(img, (self.HALF_WIDTH - top_x, height - top_y), (self.HALF_WIDTH + top_x, height - top_y), (0, 0, 255), 1)
            # img = cv2.line(img, (self.HALF_WIDTH - bottom_x, height + bottom_y), (self.HALF_WIDTH + bottom_x, height + bottom_y), (0, 0, 255), 1)
            # img = cv2.line(img, (self.HALF_WIDTH + bottom_x, height + bottom_y), (self.HALF_WIDTH + top_x, height - top_y), (0, 0, 255), 1)
            # img = cv2.line(img, (self.HALF_WIDTH - bottom_x, height + bottom_y), (self.HALF_WIDTH - top_x, height - top_y), (0, 0, 255), 1)

            # # # 디버깅을 위해 모니터에 이미지를 디스플레이

            # cv2.imshow("CAM View", img)
            # cv2.waitKey(1)

            # =========================================
            # 차량의 속도 값인 speed값 정하기.
            # 직선 코스에서는 빠른 속도로 주행하고 
            # 회전구간에서는 느린 속도로 주행하도록 설정함.
            # =========================================

            # 우선 테스트를 위해 느린속도(10값)로 설정
            
            self.straight_cnt -= abs(angle) // 10

            if self.Is_curve == STRAIGHT:
                self.straight_cnt += 1
                if self.straight_cnt >= 2:
                    self.straight_cnt = 2
            
            else:
                self.straight_cnt = -5

            if self.straight_cnt <= -5:
                self.straight_cnt = -5

            speed = 10 + self.straight_cnt
            #speed = 4

            # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
            self.drive(angle, speed)


# 모터함수, drive 함수 주석

# =============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
# =============================================
if __name__ == '__main__':
    node = Driving()
    node.start()
