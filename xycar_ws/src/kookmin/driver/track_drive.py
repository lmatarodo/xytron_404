#!/usr/bin/env python
# -*- coding: utf-8 -*- 2
#=============================================
# 본 프로그램은 2025 제8회 국민대 자율주행 경진대회에서
# 예선과제를 수행하기 위한 파일입니다. 
# 예선과제 수행 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import XycarMotor, laneinfo
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

#=============================================
# 프로그램에서 사용할 변수
#=============================================
image = np.empty(shape=[0])
ranges = None
motor = None
motor_msg = XycarMotor()
Fix_Speed = 30
bridge = CvBridge()
lane_data = None
k_p = Fix_Speed / 25
light_go = False  # ← 초록불 인식 후 출발 유지

#=============================================
# 라이다 시각화 준비
#=============================================
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')

#=============================================
# 카메라 콜백
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 신호등 검출 함수
#=============================================
def detect_traffic_light(img):
    h, w = img.shape[:2]
    roi = img[0:int(h*0.2), int(w*0.35):int(w*0.65)]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_r1 = np.array([0, 120, 70]);   upper_r1 = np.array([10, 255, 255])
    lower_r2 = np.array([170, 120, 70]); upper_r2 = np.array([180, 255, 255])
    mask_r = cv2.bitwise_or(cv2.inRange(hsv, lower_r1, upper_r1),
                            cv2.inRange(hsv, lower_r2, upper_r2))

    lower_y = np.array([15, 100, 100]); upper_y = np.array([35, 255, 255])
    mask_y = cv2.inRange(hsv, lower_y, upper_y)

    lower_g = np.array([40, 50, 50]); upper_g = np.array([90, 255, 255])
    mask_g = cv2.inRange(hsv, lower_g, upper_g)

    r_cnt = cv2.countNonZero(mask_r)
    y_cnt = cv2.countNonZero(mask_y)
    g_cnt = cv2.countNonZero(mask_g)
    total = roi.shape[0] * roi.shape[1]

    if r_cnt > 0.01 * total:
        return 'red'
    elif y_cnt > 0.01 * total:
        return 'yellow'
    elif g_cnt > 0.01 * total:
        return 'green'
    else:
        return 'none'

#=============================================
# 라이다 콜백
#=============================================
def lidar_callback(data):
    global ranges
    ranges = data.ranges[0:360]

#=============================================
# 차선 콜백
#=============================================
def lane_callback(data):
    global lane_data
    lane_data = data

#=============================================
# 모터 제어 함수
#=============================================
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)

#=============================================
# 차선 제어 함수
#=============================================
def kanayama_control():
    global lane_data
    if lane_data is None:
        return 0.0, Fix_Speed

    left_x = lane_data.left_x
    right_x = lane_data.right_x
    left_slope = lane_data.left_slope
    right_slope = lane_data.right_slope
    lane_width = 3.5

    K_y = 1
    K_phi = 3
    L = 0.5
    v_r = Fix_Speed

    if left_x == 130 and right_x == 130:
        rospy.logwarn("Both lanes lost")
        return 0.0, Fix_Speed
    elif left_x == 130:
        lateral_err = -(0.5 - (right_x / 150.0)) * lane_width
        heading_err = right_slope
    elif right_x == 130:
        lateral_err = (0.5 - (left_x / 150.0)) * lane_width
        heading_err = left_slope
    else:
        lateral_err = -(left_x / (left_x + right_x) - 0.5) * lane_width
        heading_err = 0.5 * (left_slope + right_slope)

    heading_err *= -1
    v = v_r * (math.cos(heading_err))**2
    w = v_r * (K_y * lateral_err + K_phi * math.sin(heading_err))
    delta = math.atan2(w * L, v)
    return math.degrees(delta), v

#=============================================
# 메인 실행 함수
#=============================================
def start():
    global motor, image, ranges, lane_data, light_go

    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    rospy.Subscriber("lane_info", laneinfo, lane_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

    rospy.wait_for_message("/usb_cam/image_raw", Image)
    rospy.wait_for_message("/scan", LaserScan)
    print("Camera & Lidar Ready")

    plt.ion()
    plt.show()

    while not rospy.is_shutdown():
        if image is None or image.size == 0:
            continue

        steering_angle, _ = kanayama_control()
        tl_state = detect_traffic_light(image)

        # 신호등 상태 기반 출발/정지 플래그 제어
        if tl_state == 'green':
            light_go = True
        elif tl_state in ('red', 'yellow'):
            light_go = False
        # 'none'이면 이전 상태 유지

        rospy.loginfo(f"[TL] state: {tl_state}, go: {light_go}")
        speed = Fix_Speed if light_go else 0.0
        drive(angle=steering_angle, speed=speed)

        # 디버깅 출력
        cv2.imshow("camera", image)
        cv2.waitKey(1)

        if ranges is not None:
            angles = np.linspace(0, 2*np.pi, len(ranges)) + np.pi/2
            x = np.array(ranges) * np.cos(angles)
            y = np.array(ranges) * np.sin(angles)
            lidar_points.set_data(x, y)
            plt.pause(0.01)

#=============================================
if __name__ == '__main__':
    start()
