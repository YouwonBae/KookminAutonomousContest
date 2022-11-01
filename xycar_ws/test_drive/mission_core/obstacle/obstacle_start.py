#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import
#선언부
# =============================================
from tkinter.tix import IMAGETEXT
import numpy as np
import math
import rospy, rospkg, time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import UInt8

import signal
import sys
import os

# =============================================
# core 선언부
# =============================================
is_triggered = False
fin = False

def Mode(data):
    global is_triggered
    is_triggered = True

rospy.init_node('Obstacle')
pub_following_lane = rospy.Publisher('lane_following', UInt8, queue_size=1)

rospy.Subscriber('Obstacle', UInt8, Mode, queue_size=1)
pub_mission = rospy.Publisher('Crosswalk', UInt8, queue_size=1)

rate = rospy.Rate(10)#1초에 10번 루프 0.1초 # parking_mission
# =============================================
# 토픽 발행 또는 호출 시 사용할 변수, 저장공간 선언부
# =============================================
image = np.empty(shape=[0])    # 카메라 이미지를 담을 변수
motor = None                   # 모터 토픽을 담을 변수
distance = np.zeros(shape=(360), dtype=np.float64) #라이다 데이터를 담을 변수
ultra_msg = []
# =============================================
# main에서 사용할 변수, 저장공간 선언부
# =============================================
left_ultra = 0
right_ultra = 0
obstacle_flag = [0, 0]
none_obs_delay = 3.5    # 장애물이 탐지되지 않을경우 obstacle mode를 종료할 시간(단위: 초)
obstacle_mission_end = False
obstacle_mission_start = False
# =============================================
# camera 사용할 변수, 저장공간 선언부
# =============================================
WIDTH, HEIGHT = 640, 480
# =============================================
# recoveryAngle에서 사용할 변수, 저장공간 선언부
# =============================================
uldevi_delay = 0.06     # 오른쪽 왼쪽 초음파센서 위치 편차를 보상하는 딜레이(단위: 초)
recovery_cnt = 0
recover_time = 0.5      # 장애물이 측면에서 사라질때 각도를 회복하는 시간(단위: 초)
recover_angle = 35      # 장애물이 측면에서 사라질때 회복할 각도(단위: 도)
# =============================================
# lidarDriving에서 사용할 변수, 저장공간 선언부
# =============================================
Kp = 0.30
lasterror = 0
right_sepdist =290     # 장애물과 이격할 거리(오른쪽, 단위: mm)
left_sepdist = 170     # 장애물과 이격할 거리(왼쪽, 단위: mm)
# =============================================
# ultraSenDir에서 사용할 변수, 저장공간 선언부
# =============================================
side_ultra_threshold = 20  # 초음파센서 탐지 최대 거리(단위: cm)
left_ultra_status = [0, 0]
right_ultra_status = [0, 0]
left_steer_flag = False
right_steer_flag = False
# =============================================
# obstacleData에서 사용할 변수, 저장공간 선언부
# =============================================
# hardware angle
hardware_midangle = 90     # 라이다센서에서 출력되는 정면각도의 rawdata(단위: 도)
# lidar ROI setup
mid_angle = 180            # 소프트웨어에서 처리할 정면각도 설정(단위: 도)
start_angle = 145          # 라이다 ROI시작 각도(단위: 도)
end_angle = 215            # 라이다 ROI종료 각도(단위: 도)
maxdist = 0.70             # 라이다 ROI시작 거리(단위: m)
mindist = 0.05             # 라이다 ROI종료 거리(단위: m)
# obstacle filter setup
linear_distance = 0.06     # 하나의 장애물로 판단할 최대거리(단위: m)
linear_angle = 4           # 하나의 장애물로 판단할 최대각도(단위: 도)
obstacle_minsize = 0.06    # 장애물로 판단할 최소크기(단위: m)
obstacle_maxsize = 0.35    # 장애물로 판단할 최대크기(단위: m)

# =============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
# =============================================
def signal_handler(sig, frame):
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
def img_callback(data):
    global image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
# =============================================
# 라이다 토픽을 호출하는 함수
# =============================================
def lidar_callback(data):
    global distance
    distance = data.ranges
# =============================================
# 초음파 토픽을 호출하는 함수
# =============================================
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data
# =============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
# =============================================
def drive(angle, speed):
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
# =============================================
# 장애물 회피 후 각도를 회복하는 함수
# =============================================
def recoveryAngle(direction):
    global uldevi_delay, recover_time, recover_angle, recovery_cnt, none_obs_delay
    
    recovery_cnt = recovery_cnt + 1
    if direction == 1:
        max_time_end = time.time() + uldevi_delay
        while True:
            print('deviation')
            speed = 4
            angle = 0
            drive(angle, speed)
            if time.time() > max_time_end:
                break
    if recovery_cnt == 3:
        none_obs_delay = 0.5
        max_time_end = time.time() + recover_time + 0.15
    else:
        max_time_end = time.time() + recover_time
    
    while True:
        print("recovery")
        if direction == 0:  # direction == 0 : left
            angle = -1 * recover_angle
        else:               # direction == 1 : right
            angle = recover_angle

        speed = 4
        drive(angle, speed)

        if time.time() > max_time_end:
            break
# =============================================
# 가장 가까운 정면장애물의 대표데이터와
# 정면장애물의 방향상태를 통해 조향각을 결정 하는
# 함수
# =============================================
def lidarDriving(fobs_data, fobs_dir):
    global Kp, right_sepdist, left_sepdist, lasterror
    
    error = 0

    #             비례 이득 값 # 0.4
    Kd = 0.65   # 미분 이득 값 # 0.65

    nearest_obstacle = np.argmin( fobs_data[:, 0] )
    nearest_pos = fobs_data[nearest_obstacle, 2]

    if fobs_dir == 1:        #front right obstacle
        error = nearest_pos - right_sepdist
    elif fobs_dir == 2:      #front left obstacle
        error = left_sepdist + nearest_pos
    else:                   #first obstacle
        if nearest_pos > 0:             #right
            error = nearest_pos - right_sepdist
        else:                           #left
            error = left_sepdist + nearest_pos 

    angle = Kp * error + Kd * (error - lasterror)
    lasterror = error

    if angle > 50:
        angle = 50
    elif angle < -50:
        angle = -50
    
    return angle
# =============================================
# 초음파데이터를 통해 정면장애물의 방향상태를 결
# 정 하는 함수
# =============================================
def ultraSenDir(left_ultra, right_ultra):
    global side_ultra_threshold, left_ultra_status, right_ultra_status, left_steer_flag, right_steer_flag

    # ultra sensor
    if 1 < left_ultra < side_ultra_threshold: # 초음파센서 탐지 최대거리 설정
        left_ultra_status[0] = 1
    else:
        left_ultra_status[0] = 0
    
    if 1 < right_ultra < side_ultra_threshold:
        right_ultra_status[0] = 1
    else:
        right_ultra_status[0] = 0
    
    if False:
        print('left ultra:%d right ultra:%d' %(left_ultra_status[0], right_ultra_status[0]))
    
    #left steer case
    if  left_ultra_status[0] > left_ultra_status[1]:
        left_steer_flag = True
        right_steer_flag = False
        recoveryAngle(0)

    #right steer case
    if  right_ultra_status[0] > right_ultra_status[1]:
        right_steer_flag = True
        left_steer_flag = False
        recoveryAngle(1)

    #flag
    left_ultra_status[1] = left_ultra_status[0]
    right_ultra_status[1] = right_ultra_status[0]
    
    #obstacle direction
    if left_steer_flag is True and right_steer_flag is False:       # right obstacle
        fobs_dir = 1
    elif left_steer_flag is False and right_steer_flag is True:     # left obstacle
        fobs_dir = 2
    else: fobs_dir = 0

    return fobs_dir
# =============================================
# 라이다데이터를 가공하여 정면장애물 들의
# 대표데이터를 계산하고 정면장애물의 방향상태를
# 이용 하여 조향거리를 산출 하는 함수
# =============================================
def obstacleData(raw_distance, fobs_dir):
    global mid_angle, start_angle, end_angle, maxdist, mindist, linear_distance, linear_angle, obstacle_minsize, obstacle_maxsize, hardware_midangle
    # lidar array
    pointcloud = []
    objcloud = np.empty((0,2), int)
    linear_classes = np.empty((0,3), int)
    fobs_data = np.empty((0,4), int)
    cloud_cnt = 1

    angle_err = hardware_midangle - mid_angle

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
def start():
    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image, distance, ultra_msg, left_ultra, right_ultra, obstacle_flag, none_obs_delay
    global left_ultra_status, right_ultra_status, left_steer_flag, right_steer_flag, obstacle_mission_end, obstacle_mission_start
    global fin

    # =========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    # =========================================
    #rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size = 1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)

    print("----- Obstacle -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
    while distance is None:
        continue
    while ultra_msg is None:
        continue

    # =========================================
    # 메인 루프 
    # 작업을 반복적으로 수행함.
    # =========================================
    obs_table = ['none', 'right', 'left']
    none_obs_time = 0
    time_flag = False
    start_time = time.time()

    while not fin:
        # 초음파 처리를 위해 거리값 복사 저장
        for i in range(len(ultra_msg)):
            if i == 0: left_ultra = ultra_msg[i]
            if i == 4: right_ultra = ultra_msg[i]

        # get obstacle data
        fobs_dir = ultraSenDir(left_ultra, right_ultra)
        #print('obstacle direction: %s' %obs_table[fobs_dir])
        fobs_data = obstacleData(distance, fobs_dir)
        #print('\nresult[ distance, angle, vertical distance, obstacle size ]')
        #print(fobs_data)

        # check obstacle existence
        if len(fobs_data) is not 0:
            obstacle_flag[0] = 1
            pub_following_lane.publish(UInt8(0))
            obstacle_mission_start = True
        else: 
            obstacle_flag[0] = 0

        now_time = time.time() - start_time

        # check obstacle status
        if obstacle_flag[0] < obstacle_flag[1]:
            none_obs_time = time.time() - start_time
            time_flag = True
        else: None
        obstacle_flag[1] = obstacle_flag[0]

        # reinitialize
        if time_flag is True and now_time - none_obs_time > none_obs_delay:
            none_obs_time = 0
            time_flag = False
            recovery_cnt = 0
            left_ultra_status = [0, 0]
            right_ultra_status = [0, 0]
            left_steer_flag = False
            right_steer_flag = False
            obstacle_mission_end = True
            obstacle_mission_start = False

        # wether lidar driving or image driving
        if obstacle_mission_start is True:
            if obstacle_flag[0] == 1:                           # lidar driving angle
                print('----- obstacle detected -----')
                speed = 4
                angle = lidarDriving(fobs_data, fobs_dir)
                drive(angle, speed)
            else:
                print('----- obstacle disappear -----')
                speed = 4
                angle = 0
                drive(angle, speed)
        elif obstacle_mission_end is True:                                               # image driving angle
            print("----- Obstacle mission end -----")
            angle = 0
            speed = 4
            drive(angle, speed)
            pub_following_lane.publish(UInt8(1))
            pub_mission.publish(1)
            fin = True
        else: None 
            #angle = 0
        # =========================================
        # 차량의 속도 값인 speed값 정하기.
        # =========================================
        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        #drive(angle, speed)

# =============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
# =============================================
# while not rospy.is_shutdown() and fin == False:
#     if is_triggered == True:
#         start()
#         is_triggered = False
#         break
#     rate.sleep()

if __name__ == '__main__':
    start()
