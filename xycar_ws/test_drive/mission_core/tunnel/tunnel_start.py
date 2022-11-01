#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import
#선언부
# =============================================
import math
import os
import signal
import sys
import time
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import UInt8

# =============================================
# core 선언부
# =============================================
is_triggered = False
fin = False

def Mode(data):
    global is_triggered
    is_triggered = True

rospy.init_node('Tunnel')
pub_following_lane = rospy.Publisher('lane_following', UInt8, queue_size=1)

rospy.Subscriber('Tunnel', UInt8, Mode, queue_size=1)
pub_mission = rospy.Publisher('V_parking', UInt8, queue_size=1)

rate = rospy.Rate(10)#1초에 10번 루프 0.1초 # parking_mission

# =============================================
# 토픽 발행 또는 호출 시 사용할 변수, 저장공간 선언부
# =============================================
image = np.empty(shape=[0])    # 카메라 이미지를 담을 변수
motor = None                   # 모터 토픽을 담을 변수
distance = np.zeros(shape=(360), dtype=np.float64) #라이다 데이터를 담을 변수
# =============================================
# main에서 사용할 변수, 저장공간 선언부
# =============================================
wall_flag = False
tunnel_flag = False
# =============================================
# camera 사용할 변수, 저장공간 선언부
# =============================================
WIDTH, HEIGHT = 640, 480
# =============================================
# lidarDriving에서 사용할 변수, 저장공간 선언부
# =============================================
Kp = 0.30
Kd = 0.65
lasterror = 0
entry_dist = 550           # 진입으로 판단할 벽과의 거리(오른쪽, 단위: mm)
# =============================================
# wallData에서 사용할 변수, 저장공간 선언부
# =============================================
# hardware angle
hardware_midangle = 90     # 라이다센서에서 출력되는 정면각도의 rawdata(단위: 도)
# lidar ROI setup
mid_angle = 180            # 소프트웨어에서 처리할 정면각도 설정(단위: 도)
start_angle = 95           # 라이다 ROI시작 각도(단위: 도)
end_angle = 265            # 라이다 ROI종료 각도(단위: 도)
maxdist = 1.00             # 라이다 ROI시작 거리(단위: m)
mindist = 0.05             # 라이다 ROI종료 거리(단위: m)
# wall filter setup
linear_distance = 0.06     # 하나의 벽으로 판단할 최대거리(단위: m)
linear_angle = 4           # 하나의 벽으로 판단할 최대각도(단위: 도)
wall_minsize = 0.10        # 벽으로 판단할 최소크기(단위: m)
wall_maxsize = 1.00        # 벽으로 판단할 최대크기(단위: m)
# =============================================
# entryTunnel에서 사용할 변수, 저장공간 선언부
# =============================================
tunnel_mission_end = False # 터널 탈출 후 종료를 알리는 flag
start_flag = False
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
# 가장 가까운 좌우 벽의 대표데이터를 통해 조향각을 결정 하는
# 함수
# =============================================
def lidarDriving(right_wall_data, left_wall_data):
    global entry_dist, Kp, Kd, tunnel_flag, lasterror

    # 조향 error값 초기화
    error = 0

    # pd제어
    # 비례 이득 값 # 0.4
    # 미분 이득 값 # 0.65

    leftmaxangle_wall = np.argmax( left_wall_data[:, 1] )
    leftminangle_wall = np.argmin( left_wall_data[:, 1] )
    leftwall_minpos = left_wall_data[leftminangle_wall, 2]
    leftwall_mindist = left_wall_data[leftminangle_wall, 0]

    rightminangle_wall = np.argmin( right_wall_data[:, 1] )
    rightmaxangle_wall = np.argmax( right_wall_data[:, 1] )
    rightwall_maxpos = right_wall_data[rightmaxangle_wall, 2]
    rightwall_maxdist = right_wall_data[rightmaxangle_wall, 0]

    if leftwall_mindist < entry_dist and rightwall_maxdist < entry_dist: # 터널주행을 시작할 벽과의 거리
        tunnel_flag = True 
    else:
        tunnel_flag = False

    error = rightwall_maxdist - leftwall_mindist

    angle = Kp * error + Kd * (error - lasterror)
    lasterror = error

    if angle > 50:
        angle = 50
    elif angle < -50:
        angle = -50

    angle = -3

    return angle
# =============================================
# 라이다데이터를 가공하여 벽들의
# 대표데이터를 계산하여 조향거리를 산출 하는 함수
# =============================================
def wallData(raw_distance):
    global mid_angle, start_angle, end_angle, maxdist, mindist, linear_distance, linear_angle, wall_minsize, wall_maxsize, hardware_midangle
    # lidar array
    pointcloud = []
    objcloud = np.empty((0,2), int)
    linear_classes = np.empty((0,3), int)
    right_wall_data = np.empty((0,4), int)
    left_wall_data = np.empty((0,4), int)
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
            
            if wall_angle > mid_angle:
                right_wall_data = np.append(right_wall_data, [[ represent_dist*1000, represent_angle, int(vertical_dist*1000), wall_size*1000]], axis = 0 ) 
            else:
                left_wall_data = np.append(left_wall_data, [[ represent_dist*1000, represent_angle, int(vertical_dist*1000), wall_size*1000]], axis = 0 ) 

        else: None

    return right_wall_data, left_wall_data
# =============================================
# 터널 진입 후 동작할 알고리즘 함수
# =============================================
def entryTunnel(raw_distance):
    global rate, distance, start_flag, tunnel_flag, tunnel_mission_end

    print('-------enter tunnel--------')
    speed = 4
    angle = 0

    Kp = 200
    error = 0
    PID_output = 0

    tunnel_in_flag1 = 0
    tunnel_in_flag2 = 0
    left_cnt = 0
    right_cnt = 0

    while(tunnel_flag is True):
        left_cnt = 0
        right_cnt = 0
        angle = 0
        speed = 4
        drive(angle, speed)

        if start_flag is True:
            for i in range(0, 38):
                print("----- Escape tunnel -----")
                tunnel_mission_end = True                
                tunnel_flag = False

                angle = -50
                speed = 4
                drive(angle, speed)
                rate.sleep()
 
            angle = -3
            speed = 4
            drive(angle, speed)               
            
            break

        '''
        while start_flag is True:
            print("----- Corner in tunnel -----")
            escape_angle = np.min(distance[80:115])
            left_angle = np.min(np.concatenate([distance[0:15], distance[345:359]]))
            right_angle = np.min(distance[165:195])
            
            if(left_angle>=2.0):
                left_angle = 2.0
            if(right_angle>=2.0):
                right_angle = 2.0
            if(escape_angle>=2.0):
                escape_angle = 2.0

            #rospy.loginfo("left_angle: {}".format(left_angle))
            #rospy.loginfo("right_angle: {}".format(right_angle))
            rospy.loginfo("escape_angle: {}".format(escape_angle))

            if( 1.2 < escape_angle <= 2.0 ):
                print("----- Escape tunnel -----")
                tunnel_mission_end = True                
                tunnel_flag = False

                angle = 0
                speed = 4
                drive(angle, speed)

                break

            error = 0.2 - left_angle

            P_control = Kp * error
            PID_output += (P_control)
            if(PID_output>1000):
                PID_output = 1000
            elif(PID_output <-1000):
                PID_output = -1000

            if left_angle > 0.5:
                angle = -45
            elif right_angle < 0.3:
                angle = -30
            else:
                angle = int(PID_output*0.05)

            if(angle>50):
                angle = 50
            elif(angle<-50):
                angle = -50

            speed = 4
            drive(angle, speed)
        '''

        while start_flag is not True:
            print("----- Straight in tunnel -----")
            left_angle = np.min(np.concatenate([distance[0:15], distance[345:359]]))
            right_angle = np.min(distance[165:195])
            #rospy.loginfo("left_angle: {}".format(left_angle))
            #rospy.loginfo("right_angle: {}".format(right_angle))
            #rospy.loginfo("forward_angle: {}".format(forward_angle))
            
            angle = 0
            speed = 4
            drive(angle, speed)

            if(left_angle < 1.00):
                left_cnt = 1

            if(right_angle < 1.00):
                right_cnt = 1

            #rospy.loginfo("left_cnt: {}".format(left_cnt))
            #rospy.loginfo("right_cnt: {}".format(right_cnt))

            if (left_cnt == 1) and (right_cnt == 1):
                for i in range(0, 19):
                    angle = 0
                    speed = 4
                    drive(angle, speed)
                    rate.sleep()

                start_flag = True
                break
# =============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
# =============================================
def start():
    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image, distance, wall_flag, tunnel_flag, fin
    # =========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    # =========================================
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size = 1)
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    print("----- Tunnel -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        signal.signal(signal.SIGINT, signal_handler)
        continue
    while distance is None:
        signal.signal(signal.SIGINT, signal_handler)
        continue

    # =========================================
    # 메인 루프 
    # 작업을 반복적으로 수행함.
    # =========================================
    while not fin:
        signal.signal(signal.SIGINT, signal_handler)
        # get obstacle data
        right_wall_data, left_wall_data = wallData(distance)

        if False:
            print('\nresult[ distance, angle, vertical distance, obstacle size ]')
            print('\nright')
            print(right_wall_data)
            print('\nleft')
            print(left_wall_data)

        # check obstacle existence
        if len(left_wall_data) is not 0 and len(right_wall_data) is not 0:
            wall_flag = True
        else: 
            wall_flag = False

        # wether lidar driving or image driving
        if tunnel_flag is True:                    # in tunnel
            entryTunnel(distance)
        elif wall_flag is True:                    # lidar driving angle
            print('----- Tunnel mission start -----')
            pub_following_lane.publish(UInt8(0))
            angle = lidarDriving(right_wall_data, left_wall_data)
            speed = 4
            drive(angle, speed)
        elif tunnel_mission_end is True:
            print("----- Tunnel mission end -----")
            angle = 0
            speed = 4
            drive(angle, speed)
            pub_following_lane.publish(UInt8(1))
            pub_mission.publish(1)
            fin = True
        else: None

        # =========================================
        # 차량의 속도 값인 speed값 정하기.
        # =========================================
        #speed = 4
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
