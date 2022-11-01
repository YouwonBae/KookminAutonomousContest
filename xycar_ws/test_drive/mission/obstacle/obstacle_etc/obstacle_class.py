#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import
#선언부
# =============================================
import numpy as np
import math
import rospy, rospkg, time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

import signal
import sys
import os

class Driving_auto():
    def __init__(self):
        # =============================================
        # 토픽 발행 또는 호출 시 사용할 변수, 저장공간 선언부
        # =============================================
        self.image = np.empty(shape=[0])    # 카메라 이미지를 담을 변수
        self.motor = None                   # 모터 토픽을 담을 변수
        self.distance = np.zeros(shape=(360), dtype=np.float64) #라이다 데이터를 담을 변수
        self.ultra_msg = []
        # =============================================
        # main에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.left_ultra = 0
        self.right_ultra = 0
        self.obstacle_flag = [0, 0]
        self.none_obs_delay = 4     # 장애물이 탐지되지 않을경우 obstacle mode를 종료할 시간(단위: 초)
        # =============================================
        # camera 사용할 변수, 저장공간 선언부
        # =============================================
        self.WIDTH, self.HEIGHT = 640, 480
        # =============================================
        # recoveryAngle에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.recover_time = 0.7     # 장애물이 측면에서 사라질때 각도를 회복하는 시간(단위: 초)
        self.recover_angle = 30     # 장애물이 측면에서 사라질때 회복할 각도(단위: 도)
        # =============================================
        # lidarDriving에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.Kp = 0.30
        self.lasterror = 0
        self.right_sepdist =250     # 장애물과 이격할 거리(오른쪽, 단위: mm)
        self.left_sepdist = 170     # 장애물과 이격할 거리(왼쪽, 단위: mm)
        # =============================================
        # ultraSenDir에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.side_ultra_threshold = 20  # 초음파센서 탐지 최대 거리(단위: cm)
        self.left_ultra_status = [0, 0]
        self.right_ultra_status = [0, 0]
        self.left_steer_flag = False
        self.right_steer_flag = False
        # =============================================
        # obstacleData에서 사용할 변수, 저장공간 선언부
        # =============================================
        # hardware angle
        self.hardware_midangle = 90     # 라이다센서에서 출력되는 정면각도의 rawdata(단위: 도)
        # lidar ROI setup
        self.mid_angle = 180            # 소프트웨어에서 처리할 정면각도 설정(단위: 도)
        self.start_angle = 140          # 라이다 ROI시작 각도(단위: 도)
        self.end_angle = 220            # 라이다 ROI종료 각도(단위: 도)
        self.maxdist = 0.70             # 라이다 ROI시작 거리(단위: m)
        self.mindist = 0.05             # 라이다 ROI종료 거리(단위: m)
        # obstacle filter setup
        self.linear_distance = 0.06     # 하나의 장애물로 판단할 최대거리(단위: m)
        self.linear_angle = 4           # 하나의 장애물로 판단할 최대각도(단위: 도)
        self.obstacle_minsize = 0.06    # 장애물로 판단할 최소크기(단위: m)
        self.obstacle_maxsize = 0.35    # 장애물로 판단할 최대크기(단위: m)

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
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(data, "bgr8")
    # =============================================
    # 라이다 토픽을 호출하는 함수
    # =============================================
    def lidar_callback(self, data):
        self.distance = data.ranges
    # =============================================
    # 초음파 토픽을 호출하는 함수
    # =============================================
    def ultra_callback(self, data):
        self.ultra_msg = data.data
    # =============================================
    # 모터 토픽을 발행하는 함수  
    # 입력으로 받은 angle과 speed 값을 
    # 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
    # =============================================
    def drive(self, angle, speed):
        motor_msg = xycar_motor()
        motor_msg.angle = angle
        motor_msg.speed = speed
        self.motor.publish(motor_msg)
    # =============================================
    # 장애물 회피 후 각도를 회복하는 함수
    # =============================================
    def recoveryAngle(self, direction):
        max_time_end = time.time() + self.recover_time
        while True:
            print("recovery")
            if direction == 0:  # direction == 0 : left
                angle = -1 * self.recover_angle
            else:               # direction == 1 : right
                angle = self.recover_angle

            speed = 4

            self.drive(angle, speed)

            if time.time() > max_time_end:
                break
    # =============================================
    # 가장 가까운 정면장애물의 대표데이터와
    # 정면장애물의 방향상태를 통해 조향각을 결정 하는
    # 함수
    # =============================================
    def lidarDriving(self, fobs_data, fobs_dir):
        error = 0

        Kp = self.Kp   # 비례 이득 값 # 0.4
        Kd = 0.65   # 미분 이득 값 # 0.65

        nearest_obstacle = np.argmin( fobs_data[:, 0] )
        nearest_pos = fobs_data[nearest_obstacle, 2]

        if fobs_dir == 1:        #front right obstacle
            error = nearest_pos - self.right_sepdist
        elif fobs_dir == 2:      #front left obstacle
            error = self.left_sepdist + nearest_pos
        else:                   #first obstacle
            if nearest_pos > 0:             #right
                error = nearest_pos - self.right_sepdist
            else:                           #left
                error = self.left_sepdist + nearest_pos 

        angle = Kp * error + Kd * (error - self.lasterror)
        self.lasterror = error

        if angle > 50:
            angle = 50
        elif angle < -50:
            angle = -50
        
        return angle
    # =============================================
    # 초음파데이터를 통해 정면장애물의 방향상태를 결
    # 정 하는 함수
    # =============================================
    def ultraSenDir(self, left_ultra, right_ultra):
        dist_standard = self.side_ultra_threshold # 초음파센서 탐지 최대거리 설정

        # ultra sensor
        if 1 < left_ultra < dist_standard:
            self.left_ultra_status[0] = 1
        else:
            self.left_ultra_status[0] = 0
        
        if 1 < right_ultra < dist_standard:
            self.right_ultra_status[0] = 1
        else:
            self.right_ultra_status[0] = 0
        
        if True:
            print('left ultra:%d right ultra:%d' %(self.left_ultra_status[0], self.right_ultra_status[0]))
        
        #left steer case
        if  self.left_ultra_status[0] > self.left_ultra_status[1]:
            self.left_steer_flag = True
            self.right_steer_flag = False
            self.recoveryAngle(0)

        #right steer case
        if  self.right_ultra_status[0] > self.right_ultra_status[1]:
            self.right_steer_flag = True
            self.left_steer_flag = False
            self.recoveryAngle(1)

        #flag
        self.left_ultra_status[1] = self.left_ultra_status[0]
        self.right_ultra_status[1] = self.right_ultra_status[0]
        
        #obstacle direction
        if self.left_steer_flag is True and self.right_steer_flag is False:       # right obstacle
            fobs_dir = 1
        elif self.left_steer_flag is False and self.right_steer_flag is True:     # left obstacle
            fobs_dir = 2
        else: fobs_dir = 0

        return fobs_dir
    # =============================================
    # 라이다데이터를 가공하여 정면장애물 들의
    # 대표데이터를 계산하고 정면장애물의 방향상태를
    # 이용 하여 조향거리를 산출 하는 함수
    # =============================================
    def obstacleData(self, raw_distance, fobs_dir):
        # lidar array
        pointcloud = []
        objcloud = np.empty((0,2), int)
        linear_classes = np.empty((0,3), int)
        fobs_data = np.empty((0,4), int)
        cloud_cnt = 1
        # lidar ROI setup
        mid_angle = self.mid_angle
        start_angle = self.start_angle
        end_angle = self.end_angle
        maxdist = self.maxdist
        mindist = self.mindist
        # obstacle filter setup
        linear_distance = self.linear_distance
        linear_angle = self.linear_angle
        obstacle_minsize = self.obstacle_minsize
        obstacle_maxsize = self.obstacle_maxsize

        angle_err = self.hardware_midangle - mid_angle

        for i in range(start_angle, end_angle):
            i += angle_err
            if i > 359: i -= 360
            elif i < 0: i += 360

            if mindist <= raw_distance[i] <= maxdist:
                pointcloud = np.append(pointcloud, [raw_distance[i]])
                objcloud = np.append(objcloud, [[raw_distance[i], i - angle_err]], axis = 0)
            elif raw_distance[i] > maxdist:
                pointcloud = np.append(pointcloud, [np.inf])
            else:
                pointcloud = np.append(pointcloud, [0])
            
        for i in range(len(objcloud) - 2):
            i += 1
            if abs(objcloud[i - 1, 0] - objcloud[i, 0]) < linear_distance and abs(objcloud[i - 1, 1] - objcloud[i, 1]) < linear_angle:
                linear_classes = np.append(linear_classes, [[ cloud_cnt, objcloud[i, 0], objcloud[i, 1] ]], axis = 0)
                if abs(objcloud[i, 0] - objcloud[i+1, 0]) < linear_distance and abs(objcloud[i, 1] - objcloud[i+1, 1]) < linear_angle: None
                else: cloud_cnt += 1

        #print('\nclassed data')
        #print(linear_classes)
        if False:
            print('\nraw data')
            print(pointcloud)
            print('\nobject data')
            print(objcloud)
            print('\nclassed data')
            print(linear_classes)

        for cnt in range(cloud_cnt):
            #initialize
            obstacle_size = 0
            min_angle = 360
            max_angle = 0
            for i, linear_class in enumerate(linear_classes): # check object's features
                if linear_classes[i, 0] == cnt + 1:
                    if i < len(linear_classes) - 1 and linear_classes[i + 1, 0] == cnt + 1:
                        l_now = linear_classes[i, 1]
                        l_next = linear_classes[i + 1, 1]
                        inc_angle = linear_classes[i + 1, 2] - linear_classes[i, 2]
                        obj_piece = math.sqrt((l_now * (math.sin(math.pi * (float(inc_angle) / 180)))) ** 2 
                                                + abs(l_next - l_now * (math.cos(math.pi * (float(inc_angle) / 180)))) ** 2)
                        obstacle_size += obj_piece
                    if linear_classes[i, 2] < min_angle:
                        min_angle = linear_classes[i, 2]
                        min_dist = linear_classes[i, 1]
                    if linear_classes[i, 2] >= max_angle:
                        max_angle = linear_classes[i, 2]
                        max_dist = linear_classes[i, 1]

            # get object_data from a obstacle in linear class's
            if obstacle_minsize <= obstacle_size <=  obstacle_maxsize:
                obstacle_angle = np.mean(linear_classes[np.where(linear_classes[:, 0] == cnt + 1), 2])

                # decide left or right
                if fobs_dir == 1:        # right obstacle (left ultra detection end)
                    represent_dist = min_dist
                    represent_angle = min_angle

                elif fobs_dir == 2:      # left obstacle  (right ultra detection end)
                    represent_dist = max_dist
                    represent_angle = max_angle
                else:                   #                (no ultra_sensor detection)
                    if obstacle_angle > mid_angle:  # right obstacle
                        represent_dist = min_dist
                        represent_angle = min_angle
                    else:                           # left obstacle
                        represent_dist = max_dist
                        represent_angle = max_angle

                # calculate steer_dist (left: negative, right: positive)
                if represent_angle < mid_angle:
                    sin_angle = mid_angle - represent_angle
                    steer_dist = -1 * represent_dist * (math.sin(math.pi * (float(sin_angle) / 180)))
                else:
                    sin_angle = represent_angle - mid_angle
                    steer_dist = represent_dist * (math.sin(math.pi * (float(sin_angle) / 180)))
                
                fobs_data = np.append(fobs_data, [[ represent_dist*1000, represent_angle, int(steer_dist*1000), obstacle_size*1000]], axis = 0 ) 
            else: None

        return fobs_data
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
        rospy.Subscriber("/usb_cam/image_raw/", Image, self.img_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size = 1)
        rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, self.ultra_callback)

        print("----- Xycar self driving -----")

        # 첫번째 카메라 토픽이 도착할 때까지 기다림.
        while not self.image.size == (self.WIDTH * self.HEIGHT * 3):
            continue
        while self.distance is None:
            continue
        while self.ultra_msg is None:
            continue

        # =========================================
        # 메인 루프 
        # 작업을 반복적으로 수행함.
        # =========================================
        obs_table = ['none', 'right', 'left']
        none_obs_time = 0
        time_flag = False
        start_time = time.time()

        while not rospy.is_shutdown():
            # 초음파 처리를 위해 거리값 복사 저장
            for i in range(len(self.ultra_msg)):
                if i == 0: self.left_ultra = self.ultra_msg[i]
                if i == 4: self.right_ultra = self.ultra_msg[i]

            # get obstacle data
            fobs_dir = self.ultraSenDir(self.left_ultra, self.right_ultra)
            print('obstacle direction: %s' %obs_table[fobs_dir])
            fobs_data = self.obstacleData(self.distance, fobs_dir)
            print('\nresult[ distance, angle, vertical distance, obstacle size ]')
            print(fobs_data)

            # check obstacle existence
            if len(fobs_data) is not 0:
                self.obstacle_flag[0] = 1
            else: 
                self.obstacle_flag[0] = 0

            now_time = time.time() - start_time

            # check obstacle status
            if self.obstacle_flag[0] < self.obstacle_flag[1]:
                none_obs_time = time.time() - start_time
                time_flag = True
            else: None
            self.obstacle_flag[1] = self.obstacle_flag[0]

            # reinitialize
            if time_flag is True and now_time - none_obs_time > self.none_obs_delay:
                none_obs_time = 0
                time_flag = False
                self.left_ultra_status = [0, 0]
                self.right_ultra_status = [0, 0]
                self.left_steer_flag = False
                self.right_steer_flag = False

            # wether lidar driving or image driving
            if self.obstacle_flag[0] == 1:                      # lidar driving angle
                angle = self.lidarDriving(fobs_data, fobs_dir)
            else:                                               # image driving angle
                angle = 0
            # =========================================
            # 차량의 속도 값인 speed값 정하기.
            # =========================================
            speed = 4
            # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
            self.drive(angle, speed)

# =============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
# =============================================
if __name__ == '__main__':
    node = Driving_auto()
    node.start()
