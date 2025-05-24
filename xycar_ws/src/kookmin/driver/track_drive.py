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
import matplotlib.pyplot as plt

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
ranges = None  # 라이다 데이터를 담을 변수
motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
Fix_Speed = 30 #원래 10  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
lane_data = None  # 차선 정보를 담을 변수
k_p = Fix_Speed/25
k_para = 20
k_lateral = 5
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
    global lane_data
    lane_data = data
    #rospy.loginfo("left: %.2f, right: %.2f", left, right)
	
#=============================================
# 모터로 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)

#=============================================
# 차선 정보를 기반으로 조향각을 계산하는 함수
#=============================================
def kanayama_control():
    global lane_data
    if lane_data is None:
        return 0.0, Fix_Speed  # 기본 조향각 0, 속도 Fix_Speed

    # 좌우 차선 x, slope 가져오기
    left_x = lane_data.left_x
    right_x = lane_data.right_x
    left_slope = lane_data.left_slope
    right_slope = lane_data.right_slope
    lane_width = 3.5  # m
    
    # 파라미터
    K_y = 1
    K_phi = 3
    L = 0.5
    v_r = Fix_Speed
    
    # lateral_err, heading_err 계산 (주신 코드 참고)
    if left_x == 130 and right_x == 130:
        rospy.logwarn("Both lanes lost, skipping control.")
        return 0.0, Fix_Speed
    elif left_x == 130:
        #lateral_err = (0.5 - (right_x / 220.0)) * lane_width
        #lateral_err = -(0.5 - (right_x / 150.0)) * lane_width
        lateral_err = -(0.5 - (right_x / 150.0)) * lane_width
        heading_err = right_slope
    elif right_x == 130:
        #lateral_err = (-0.5 + (left_x / 220.0)) * lane_width
        #lateral_err = (0.5 - (left_x / 150.0)) * lane_width
        lateral_err = (0.5 - (left_x / 150.0)) * lane_width
        heading_err = left_slope
    else:
        lateral_err = -(left_x / (left_x + right_x) - 0.5) * lane_width
        heading_err = 0.5 * (left_slope + right_slope)
    heading_err = 0.5 * (left_slope + right_slope)
    heading_err *= -1

    # 각속도 w 계산
    v = v_r * (math.cos(heading_err))**2
    w = v_r * (K_y * lateral_err + K_phi * math.sin(heading_err))

    # 조향각 delta 계산 (라디안)
    delta = math.atan2(w * L, v)

    # 필요시 각도를 degree 단위로 변환 가능 (현재는 radian 그대로 사용)
    steering_angle = math.degrees(delta)

    return steering_angle, v


def calculate_steering_angle():
    if lane_data is None:
        return 0.0
    
    # 왼쪽과 오른쪽 차선의 기울기를 이용하여 조향각 계산
    left_slope = lane_data.left_slope
    right_slope = lane_data.right_slope
    
    # 두 차선의 기울기 평균을 사용
    avg_slope = (left_slope + right_slope) / 2.0
    
    # 기울기를 조향각으로 변환 (라디안 -> 각도)
    steering_angle = -1*math.degrees(avg_slope)
    
    # 조향각 계산
    #dx = lane_data.left_x
    #dy = -lane_data.right_x
    #normalized_para_angle = -((steering_angle>0)-(steering_angle<0))*(steering_angle/50)**2
    #steering_angle = steering_angle + math.atan(x)
    #normalrized_lateral_offset = ((dx>dy)*dx + (dy>dy)*dy)/320
    #steering_angle = k_p*steering_angle - k_lateral*((dx>dy+100)*(normalrized_lateral_offset) + (dy>dx+100)*(normalrized_lateral_offset))
    #steering_angle = k_p*steering_angle
    steering_angle = k_p*steering_angle
    # 조향각 제한 (-50 ~ 50도)
    steering_angle = max(min(steering_angle, 50.0), -50.0)
    rospy.loginfo("angle: %.2f", steering_angle)
    return steering_angle

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
        
        # 차선 정보를 기반으로 조향각 계산
        steering_angle, _ = kanayama_control()
        
        # 계산된 조향각으로 주행
        drive(angle=steering_angle, speed=Fix_Speed)
        time.sleep(0.1)
        
        cv2.waitKey(1)

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
