#!/usr/bin/env python
# -*- coding: utf-8 -*- 2
#=============================================
# 본 프로그램은 2025 제8회 국민대 자율주행 경진대회에서
# 예선과제를 수행하기 위한 파일입니다. 
# 예선과제 수행 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from xycar_msgs.msg import laneinfo
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped
import matplotlib.pyplot as plt

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
ranges = None  # 라이다 데이터를 담을 변수
motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
Fix_Speed = 20 #원래 10  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
lane_data = None  # 차선 정보를 담을 변수
k_p = new_speed/12
k_para = 10
k_lateral = 5

# Stanley 제어를 위한 변수들
lane_width = 260  # 차선 폭 (픽셀 단위, BEV 이미지의 너비)
max_steer = 50.0  # 최대 조향각
left_x = 0.0
right_x = 0.0
left_slope = 0.0
right_slope = 0.0

#=============================================
# 라이다 스캔정보로 그림을 그리기 위한 변수
#=============================================
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
   
#=============================================
# 콜백함수 - 라이다 토픽을 받아서 처리하는 콜백함수
#=============================================
def lidar_callback(data):
    global ranges    
    ranges = data.ranges[0:360]

#=============================================
# 콜백함수 - 차선 정보를 받아서 처리하는 콜백함수
#=============================================
def lane_callback(data):
    global lane_data, left_x, right_x, left_slope, right_slope
    lane_data = data
    left_x = data.left_x
    right_x = data.right_x
    left_slope = data.left_slope
    right_slope = data.right_slope

#=============================================
# 모터로 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)

#=============================================
# Stanley 제어 함수
#=============================================
def stanley_control():
    global left_x, right_x, left_slope, right_slope
    try:
        if left_x == 130 and right_x == -130:
            return
        elif left_x == 130:  # 차선이 하나만 잡히는 경우 
            lateral_err = (0.5 - (right_x/lane_width))  # 픽셀 단위로 계산
            heading_err = right_slope
        elif right_x == -130:
            lateral_err = (-0.5 + (left_x/lane_width))  # 픽셀 단위로 계산
            heading_err = left_slope
        else:  # 일반적인 주행
            lateral_err = ((left_x/(left_x + right_x)) - 0.5)  # 픽셀 단위로 계산
            heading_err = (left_slope + right_slope)/2

        k = 1  # stanley_상수
        velocity_profile = 30  # 속도값 km/h
                
        steer = heading_err + np.arctan2(k*lateral_err,((velocity_profile/3.6)))  # stanley control

        steer = max(-max_steer,min(max_steer,steer)) * 0.5  # scaling

        throttle = 0.6
        
        cmd_vel = Vector3Stamped()
        cmd_vel.vector.x = throttle
        cmd_vel.vector.y = -np.degrees(steer)*1/28  # -0.1~0.1
        motor.publish(cmd_vel)
        
        rospy.loginfo(f'\nlateral error : {lateral_err}\nheading error : {heading_err}\nsteer : {steer}\npubsteer : {cmd_vel.vector.y}')
    except ZeroDivisionError as e:
        rospy.loginfo(e)

#=============================================
# 실질적인 메인 함수 
#=============================================
def start():
    global motor, image, ranges
    
    print("Start program --------------")

    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    rospy.Subscriber("lane_info", laneinfo, lane_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
        
    #=========================================
    # 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("/scan", LaserScan)
    print("Lidar Ready ----------")
    
    #=========================================
    # 라이다 스캔정보에 대한 시각화 준비를 합니다.
    #=========================================
    plt.ion()
    plt.show()
    print("Lidar Visualizer Ready ----------")
    
    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    #=========================================
    # 메인 루프 
    #=========================================
    while not rospy.is_shutdown():
        if image.size != 0:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cv2.imshow("original", image)
            cv2.imshow("gray", gray)

        if ranges is not None:            
            angles = np.linspace(0,2*np.pi, len(ranges))+np.pi/2
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            lidar_points.set_data(x, y)
            fig.canvas.draw_idle()
            plt.pause(0.01)  
        
        # Stanley 제어 실행
        stanley_control()
        time.sleep(0.1)
        
        cv2.waitKey(1)

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
