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
        # =============================================
        # main에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.wall_flag = False
        self.tunnel_flag = False
        self.angle = 0
        # =============================================
        # camera 사용할 변수, 저장공간 선언부
        # =============================================
        self.WIDTH, self.HEIGHT = 640, 480
        # =============================================
        # lidarDriving에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.Kp = 0.30
        self.lasterror = 0
        self.entry_dist = 0.30           # 진입으로 판단할 벽과의 거리(오른쪽, 단위: m)
        # =============================================
        # wallData에서 사용할 변수, 저장공간 선언부
        # =============================================
        # hardware angle
        self.hardware_midangle = 90     # 라이다센서에서 출력되는 정면각도의 rawdata(단위: 도)
        # lidar ROI setup
        self.mid_angle = 180            # 소프트웨어에서 처리할 정면각도 설정(단위: 도)
        self.start_angle = 135          # 라이다 ROI시작 각도(단위: 도)
        self.end_angle = 225            # 라이다 ROI종료 각도(단위: 도)
        self.maxdist = 1.00             # 라이다 ROI시작 거리(단위: m)
        self.mindist = 0.05             # 라이다 ROI종료 거리(단위: m)
        # wall filter setup
        self.linear_distance = 0.06     # 하나의 벽으로 판단할 최대거리(단위: m)
        self.linear_angle = 4           # 하나의 벽으로 판단할 최대각도(단위: 도)
        self.wall_minsize = 0.20        # 벽으로 판단할 최소크기(단위: m)
        self.wall_maxsize = 1.00        # 벽으로 판단할 최대크기(단위: m)
        # =============================================
        # entryTunnel에서 사용할 변수, 저장공간 선언부
        # =============================================
        self.tunnel_mission_end = False # 터널 탈출 후 종료를 알리는 flag

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
    # 모터 토픽을 발행하는 함수  
    # 입력으로 받은 angle과 speed 값을 
    # 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
    # =============================================
    def drive(self, angle, speed):
        self.motor_msg = xycar_motor()
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        self.motor.publish(self.motor_msg)
    # =============================================
    # 가장 가까운 좌우 벽의 대표데이터를 통해 조향각을 결정 하는
    # 함수
    # =============================================
    def lidarDriving(self, wall_data):
        # 터널주행을 시작할 벽과의 거리
        entry_dist = self.entry_dist
        # 조향 error값 초기화
        error = 0
        # pd제어
        Kp = self.Kp   # 비례 이득 값 # 0.4
        Kd = 0.65   # 미분 이득 값 # 0.65

        leftest_wall = np.argmin( wall_data[:, 1] )
        leftwall_pos = wall_data[leftest_wall, 2]
        leftwall_dist = wall_data[leftest_wall, 0]

        rightest_wall = np.argmax( wall_data[:, 1] )
        rightwall_pos = wall_data[rightest_wall, 2]
        righttwall_dist = wall_data[rightwall_pos, 0]

        if leftwall_dist < entry_dist and righttwall_dist < entry_dist:
            self.tunnel_flag = True
        else:
            self.tunnel_flag = False

        error = rightwall_pos + leftwall_pos

        angle = Kp * error + Kd * (error - self.lasterror)
        self.lasterror = error

        if angle > 50:
            angle = 50
        elif angle < -50:
            angle = -50
        
        return angle
    # =============================================
    # 라이다데이터를 가공하여 벽들의
    # 대표데이터를 계산하여 조향거리를 산출 하는 함수
    # =============================================
    def wallData(self, raw_distance):
        # lidar array
        pointcloud = []
        objcloud = np.empty((0,2), int)
        linear_classes = np.empty((0,3), int)
        wall_data = np.empty((0,4), int)
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
        wall_minsize = self.wall_minsize
        wall_maxsize = self.wall_maxsize

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
            wall_size = 0
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
                        wall_size += obj_piece
                    if linear_classes[i, 2] < min_angle:
                        min_angle = linear_classes[i, 2]
                        min_dist = linear_classes[i, 1]
                    if linear_classes[i, 2] >= max_angle:
                        max_angle = linear_classes[i, 2]
                        max_dist = linear_classes[i, 1]

            # get object_data from a obstacle in linear class's
            if wall_minsize <= wall_size <=  wall_maxsize:
                wall_angle = np.mean(linear_classes[np.where(linear_classes[:, 0] == cnt + 1), 2])

                # decide left or right
                if wall_angle > mid_angle:  # right obstacle
                    represent_dist = max_dist
                    represent_angle = max_angle
                else:                           # left obstacle
                    represent_dist = min_dist
                    represent_angle = min_angle

                # calculate vertical_dist (left: negative, right: positive)
                if represent_angle < mid_angle:
                    sin_angle = mid_angle - represent_angle
                    vertical_dist = -1 * represent_dist * (math.sin(math.pi * (float(sin_angle) / 180)))
                else:
                    sin_angle = represent_angle - mid_angle
                    vertical_dist = represent_dist * (math.sin(math.pi * (float(sin_angle) / 180)))
                
                wall_data = np.append(wall_data, [[ represent_dist*1000, represent_angle, int(vertical_dist*1000), wall_size*1000]], axis = 0 ) 
            else: None

        return wall_data
    # =============================================
    # 터널 진입 후 동작할 알고리즘 함수
    # =============================================
    def entryTunnel(self, raw_distance):
        Kp = 200
        error = 0
        speed = 4
        tunnel_in_flag1 = 0
        tunnel_in_flag2 = 0

        while True:
            left_angle = np.min(np.concatenate([raw_distance[0:15], raw_distance[345:]]))
            right_angle = np.min(raw_distance[165:195])

            if(left_angle > 1.0 and (right_angle == float("inf"))):
                right_angle = 0.1

            if(right_angle > 1.0 and (left_angle == float("inf"))):
                left_angle = 0.1

            forward_angle = np.min(raw_distance[75:105])
            rospy.loginfo("left_angle: {}".format(left_angle))
            rospy.loginfo("right_angle: {}".format(right_angle))
            rospy.loginfo("forward_angle: {}".format(forward_angle))
            signal.signal(signal.SIGINT, self.signal_handler)

            if(self.motor_msg.angle == 50 or self.motor_msg.angle == -50):
                self.tunnel_in_flag1 = self.motor_msg.angle

            if(((tunnel_in_flag1 == 50) and not (self.motor_msg.angle == 50)) or ((tunnel_in_flag1 == -50) and not (self.motor_msg.angle == -50))):
                tunnel_in_flag2 = 1

            if((forward_angle > 1.0) and tunnel_in_flag2):
                tunnel_in_flag1 = 0
                tunnel_in_flag2 = 0
                self.tunnel_flag = False
                self.tunnel_mission_end = True
                self.motor_msg.angle = -(self.motor_msg.angle / 2)
                self.motor_msg.speed = speed
                self.motor.publish(self.motor_msg)
                time.sleep(2.0)
                break

            if(forward_angle < 0.15):        
                self.motor_msg.angle = 0
                self.motor_msg.speed = speed
                self.motor.publish(self.motor_msg)

            if(right_angle < left_angle):
                error = right_angle - (right_angle + left_angle)/2
            elif(left_angle < right_angle):
                error = (right_angle + left_angle)/2 - left_angle

            PID_output += (Kp * error)

            if(PID_output > 1000):
                PID_output = 1000
            elif(PID_output < -1000):
                PID_output = -1000

            rospy.loginfo("PID_output: {}".format(PID_output) + " motor_angle: {}".format(int( PID_output * 0.05 )))

            self.motor_msg.angle = int(PID_output * 0.05)
            if(self.motor_msg.angle > 50):
                self.motor_msg.angle = 50
            elif(self.motor_msg.angle < -50):
                self.motor_msg.angle = -50

            self.motor_msg.speed = speed
            self.motor.publish(self.motor_msg)
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
        rospy.Subscriber("/usb_cam/image_raw/", Image, self.img_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size = 1)
        self.motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        print("----- Xycar self driving -----")

        # 첫번째 카메라 토픽이 도착할 때까지 기다림.
        while not self.image.size == (self.WIDTH * self.HEIGHT * 3):
            signal.signal(signal.SIGINT, self.signal_handler)
            continue
        while self.distance is None:
            signal.signal(signal.SIGINT, self.signal_handler)
            continue

        # =========================================
        # 메인 루프 
        # 작업을 반복적으로 수행함.
        # =========================================
        start_time = time.time()

        while not rospy.is_shutdown():
            signal.signal(signal.SIGINT, self.signal_handler)
            # get obstacle data
            wall_data = self.wallData(self.distance)
            print('\nresult[ distance, angle, vertical distance, obstacle size ]')
            print(wall_data)

            # check obstacle existence
            if len(wall_data) is not 0:
                self.wall_flag = True
            else: 
                self.wall_flag = False

            # wether lidar driving or image driving
            if self.tunnel_flag is True:                    # in tunnel
                 self.entryTunnel(self.distance)
            elif self.wall_flag is True:                    # lidar driving angle
                angle = self.lidarDriving(wall_data)
            else:                                           # image driving angle
                angle = 0
            self.angle = angle
            # =========================================
            # 차량의 속도 값인 speed값 정하기.
            # =========================================
            self.speed = 4
            # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
            self.drive(self.angle, self.speed)

# =============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
# =============================================
if __name__ == '__main__':
    node = Driving_auto()
    node.start()
