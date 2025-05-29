#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
import time
# import os # 현재 코드에서 직접 사용되지 않음
import math
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from xycar_msgs.msg import laneinfo # 카메라 기반 차선 정보
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt # 라이다 시각화는 유지

# *** 새로운 메시지 타입 import ***
# 'your_package_name'을 실제 사용하는 패키지 이름으로 변경해주세요.
# 이 메시지 파일(ConeLanes.msg)은 사용자가 직접 생성하고 빌드해야 합니다.
from xycar_msgs.msg import ConeLanes

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0])
ranges = None
motor = None
motor_msg = XycarMotor()
Fix_Speed = 30 # 꼬깔콘 주행 및 일반 주행 시 기본 속도 (튜닝 필요)
# new_angle, new_speed는 start 루프 내에서 결정되므로 전역 초기값은 참고용
bridge = CvBridge()
lane_data = None # 카메라 기반 차선 정보
k_p = Fix_Speed / 25.0 # 기존 값 유지 (카메라 차선용)
# k_para, k_lateral은 kanayama_control에서 직접 사용되지 않고 calculate_steering_angle에서 사용되었던 것으로 보임.
# kanayama_control을 주로 사용한다면 이 값들은 직접적인 영향 없을 수 있음.
light_go = False
L = 0.26 # 차량 축거 (m) - 기존 0.5에서 좀 더 현실적인 값으로 수정 (Xycar 제원 확인 필요)

# 차량 감지 관련 (기존과 동일)
vehicle_ahead = False
detection_success_streak = 0
DETECTION_DISTANCE_LOW = 1.0 # 전방 차량 감지 거리 수정 (꼬깔콘과 구분 위해)
DETECTION_DISTANCE_HIGH = 10.0
DETECTION_COUNT    = 4
SECTOR_WIDTH       = 7
DETECTION_STREAK_THRESHOLD = 2

# 차선 변경 관련 변수 (기존과 동일)
current_lane = 0
is_lane_changing = False
lane_change_path = []
lane_change_direction = 0
lane_change_start_time = 0
# last_lane_change_time = 0 # 사용되지 않는 것으로 보여 주석 처리
LANE_CHANGE_DURATION = 1.65
PATH_POINTS = 20
LANE_CHANGE_DISTANCE = 230

# 이미지 관련 상수 (기존과 동일)
IMAGE_WIDTH = 260
IMAGE_HEIGHT = 260
BASE_X = 130
BASE_Y = 260

# *** 꼬깔콘 차선 정보 저장을 위한 새로운 전역 변수 ***
cone_lanes_data = None

# *** 꼬깔콘 경로 추종 관련 파라미터 ***
# 이 값들은 track_drive.py 내에서 꼬깔콘 경로 추종 시 사용
CONE_PURE_PURSUIT_K_DD = 0.4
CONE_MIN_LOOKAHEAD_DIST = 0.7
CONE_MAX_LOOKAHEAD_DIST = 3.0
CONE_STEERING_GAIN = 1.1


ENABLE_MATPLOTLIB_VIZ = False

#=============================================
# 라이다 스캔정보로 그림을 그리기 위한 변수 (기존과 동일)
#=============================================
if 'fig' not in globals(): # 중복 실행 방지 (예: IPython 환경)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-15, 15) # 시각화 범위 조정 (LIDAR_MAX_DIST 기반으로 자동 조정 안함)
    ax.set_ylim(-5, 15)
    ax.set_aspect('equal')
    lidar_points_plot, = ax.plot([], [], 'bo', markersize=2) # lidar_points -> lidar_points_plot로 변경 (변수명 충돌 방지)


#=============================================
# 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
   
def lidar_callback(data): # 이 함수는 주로 전방 차량 감지에 사용
    global ranges, vehicle_ahead, detection_success_streak
    # ranges는 전체 360도 데이터를 받지만, 차량 감지에는 SECTOR_WIDTH 내의 일부만 사용
    temp_ranges = np.array(data.ranges[0:360]) # NumPy 배열로 변환
    ranges = temp_ranges # 전역 ranges 업데이트 (다른 곳에서 사용될 수 있으므로)

    detected_this_scan = False
    consec = 0
    for offset in range(-SECTOR_WIDTH, SECTOR_WIDTH + 1):
        idx = offset % 360
        d = temp_ranges[idx] # 수정: ranges 대신 temp_ranges 사용
        # 유효한 거리 값인지 먼저 확인
        if np.isfinite(d) and d > 0.01: # 0이 아니고 유한한 값
            if DETECTION_DISTANCE_LOW < d < DETECTION_DISTANCE_HIGH:
                consec += 1
                if consec >= DETECTION_COUNT:
                    detected_this_scan = True
                    break
            else:
                consec = 0
        else:
            consec = 0

    if detected_this_scan:
        detection_success_streak += 1
    else:
        detection_success_streak = 0

    if detection_success_streak >= DETECTION_STREAK_THRESHOLD:
        vehicle_ahead = True
    else:
        vehicle_ahead = False

def lane_callback(data): # 카메라 기반 차선 정보 콜백
    global lane_data, current_lane, is_lane_changing, lane_change_path
    global lane_change_direction, lane_change_start_time # last_lane_change_time 제거
    
    # 꼬깔콘 주행 중에는 차선 변경 로직을 발동시키지 않기 위한 플래그 (선택적)
    # if cone_lanes_data is not None and \
    #    (cone_lanes_data.left_lane_detected or cone_lanes_data.right_lane_detected):
    #     is_lane_changing = False # 꼬깔콘 모드 시 강제로 차선 변경 모드 해제
    #     return

    lane_data = data
    current_time = time.time()

    if data.lane_number in [1, 2]:
        current_lane = data.lane_number

    # 차선 변경 조건 (꼬깔콘 주행 중이 아닐 때 + 기존 조건 만족 시)
    if not is_lane_changing and vehicle_ahead and \
       (cone_lanes_data is None or not (cone_lanes_data.left_lane_detected or cone_lanes_data.right_lane_detected)): # 꼬깔콘이 우선순위가 아닐때
        is_lane_changing = True
        lane_change_start_time = current_time
        # last_lane_change_time = current_time # 사용되지 않음

        target_lane = 2 if current_lane == 1 else 1
        start_x = BASE_X # 기본값
        if data.left_x != 130 and data.right_x != -130:
            start_x = (data.left_x + data.right_x) / 2
        elif data.left_x != 130:
            start_x = data.left_x + 75 # 차선 폭의 절반 가정
        elif data.right_x != -130:
            start_x = data.right_x - 75 # 차선 폭의 절반 가정
        
        lane_change_path, lane_change_direction = generate_lane_change_path(
            current_lane, target_lane, start_x, data.left_x, data.right_x
        )
        if lane_change_path: # 경로 생성 성공 시에만 로그 출력
             rospy.loginfo(f"Camera Lane Change Triggered: {current_lane} -> {target_lane}")


# *** 새로운 콜백 함수: 꼬깔콘 차선 정보 수신 ***
def cone_lanes_data_callback(data):
    global cone_lanes_data
    cone_lanes_data = data

#=============================================
# 신호등 검출 함수 (기존과 동일)
#=============================================
def detect_traffic_light(img_input): # 변수명 img -> img_input
    if img_input is None or img_input.size == 0: return 'none'
    h, w = img_input.shape[:2]
    # ROI 유효성 검사 강화
    roi_y_end = int(h * 0.2)
    roi_x_start = int(w * 0.35)
    roi_x_end = int(w * 0.65)
    if not (0 < roi_y_end <=h and 0 <= roi_x_start < w and roi_x_start < roi_x_end <=w):
        return 'none'
    roi = img_input[0:roi_y_end, roi_x_start:roi_x_end]
    if roi.size == 0: return 'none'

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_r1 = np.array([0, 120, 70]); upper_r1 = np.array([10, 255, 255])
    lower_r2 = np.array([170, 120, 70]); upper_r2 = np.array([180, 255, 255])
    mask_r = cv2.bitwise_or(cv2.inRange(hsv, lower_r1, upper_r1), cv2.inRange(hsv, lower_r2, upper_r2))
    lower_y = np.array([15, 100, 100]); upper_y = np.array([35, 255, 255])
    mask_y = cv2.inRange(hsv, lower_y, upper_y)
    lower_g = np.array([40, 50, 50]); upper_g = np.array([90, 255, 255])
    mask_g = cv2.inRange(hsv, lower_g, upper_g)
    r_cnt = cv2.countNonZero(mask_r); y_cnt = cv2.countNonZero(mask_y); g_cnt = cv2.countNonZero(mask_g)
    total_pixels = roi.shape[0] * roi.shape[1]
    if total_pixels == 0: return 'none'
    if r_cnt > 0.01 * total_pixels: return 'red'
    elif y_cnt > 0.01 * total_pixels: return 'yellow'
    elif g_cnt > 0.01 * total_pixels: return 'green'
    return 'none'

#=============================================
# 모터로 토픽을 발행하는 함수 (기존과 동일)
#=============================================
def drive_motor(angle, speed): # 함수명 변경 (drive -> drive_motor)
    global motor, motor_msg # motor_control -> motor, motor_msg_obj -> motor_msg
    if motor is not None:
        motor_msg.angle = float(angle)
        motor_msg.speed = float(speed)
        motor.publish(motor_msg)

#=============================================
# 차선 정보 기반 조향각 계산 함수들 (kanayama_control, calculate_steering_angle, 차선 변경 관련 함수들은 기존 코드 유지)
#=============================================
def kanayama_control(): # 카메라 차선 기반 제어
    global lane_data, current_lane, is_lane_changing, lane_change_path, lane_change_direction, lane_change_start_time, L, Fix_Speed, BASE_X
    if lane_data is None:
        return 0.0, Fix_Speed # 차선 정보 없으면 직진, 기본 속도

    # 차선 변경 중인 경우 (이 로직은 그대로 사용)
    if is_lane_changing and lane_change_path:
        current_time = time.time()
        if current_time - lane_change_start_time > LANE_CHANGE_DURATION:
            is_lane_changing = False; lane_change_path = []; lane_change_direction = 0
            rospy.loginfo("Camera Lane Change Complete.")
        else:
            # 현재 차선 중심점 계산 (이 부분은 lane_callback과 중복될 수 있으나, 여기서도 필요)
            current_x_for_lc = BASE_X
            if lane_data.left_x != 130 and lane_data.right_x != -130: current_x_for_lc = (lane_data.left_x + lane_data.right_x) / 2
            elif lane_data.left_x != 130: current_x_for_lc = lane_data.left_x + 75
            elif lane_data.right_x != -130: current_x_for_lc = lane_data.right_x - 75
            
            # 차선 변경 중 heading은 현재 차선의 평균 기울기를 사용하는 것이 더 적절할 수 있음
            # 여기서는 lane_data.left_slope, right_slope가 라디안 값이라고 가정
            current_heading_rad = -(lane_data.left_slope + lane_data.right_slope) / 2.0 if (lane_data.left_slope!=0 and lane_data.right_slope!=0) else 0.0

            steering_angle_lc = lane_change_control(
                lane_change_path, current_x_for_lc, BASE_Y, current_heading_rad, lane_change_direction
            )
            # 차선 변경 시 속도 및 조향각 보정 (기존 로직)
            return steering_angle_lc * 1.1, Fix_Speed * 1.15

    # 일반 주행 (Kanayama)
    left_x, right_x = lane_data.left_x, lane_data.right_x
    # Kanayama는 각도가 라디안 단위로 입력되어야 함
    left_slope_rad, right_slope_rad = lane_data.left_slope, lane_data.right_slope 
    
    # 파라미터 (Kanayama용, 기존 코드 참고하여 값 설정 또는 튜닝)
    K_y_k = 0.85 # Lateral error gain
    K_phi_k = 3.0 # Heading error gain
    # v_r_k = Fix_Speed / 10.0 # Kanayama는 속도 단위를 m/s로 가정할 수 있음. Fix_Speed 스케일링 필요
                            # XycarMotor의 speed 단위와 일치시켜야 함. 지금은 Fix_Speed 그대로 사용.
    v_r_k = Fix_Speed 

    lane_width_k = 1.5 # 예상 차선 폭 (m), 실제 환경에 맞게 조정 필요 (기존 3.5는 너무 넓어보임)

    # lateral_error (y_e), heading_error (phi_e) 계산
    # Xycar 이미지 좌표계: 좌상단 (0,0), 우하단 (W,H)
    # 차선 정보 (left_x, right_x)는 이미지 중앙 x=IMAGE_WIDTH/2 (즉, 130) 기준 상대 좌표로 가정
    # (예: left_x=100이면 중앙보다 30픽셀 왼쪽, right_x=150이면 중앙보다 20픽셀 오른쪽)
    # 이는 lane_detection.py의 출력 형식에 따라 달라짐.
    # 현재 코드는 left_x가 왼쪽 차선까지의 거리(픽셀), right_x가 오른쪽 차선까지의 거리(픽셀)로 가정.
    # lane_data.left_x 와 right_x는 bird-eye-view 이미지의 y축 맨 아래에서의 x좌표로 가정 (lane_detection.py 원본 기준)
    # left_x: 0~260, right_x: 0~260 (bird-eye-view 중앙이 130)
    
    # 여기서는 lane_data가 xycar_msgs/laneinfo를 따른다고 가정하고,
    # left_x, right_x가 BEV 이미지 하단에서의 x좌표 (중앙 130)
    # left_slope, right_slope는 해당 차선의 기울기 (dy/dx)의 atan 값 (라디안)
    
    img_center_x_bev = 130 # BEV 이미지의 x축 중앙
    
    if left_x == 130 and right_x == -130: # 양쪽 차선 유실 (lane_detection.py의 기본값)
        rospy.logwarn_throttle(2.0, "Kanayama: Both lanes lost.")
        return 0.0, Fix_Speed * 0.5 # 속도 줄임
    elif left_x == 130: # 왼쪽 유실 (오른쪽 차선만 존재)
        # 오른쪽 차선 기준으로 차량이 차선 중앙에 오도록 y_e 계산
        # right_x는 BEV 중앙(130)으로부터 오른쪽 차선까지의 x좌표
        # 차량이 차선 중앙에 있다면, 차량 중심(130)은 right_x - (lane_width_k_pixels / 2) 위치에 있어야 함
        # 이 부분은 lane_detection.py의 출력 의미를 정확히 알아야 함.
        # 여기서는 단순화: 오른쪽 차선에서 차선폭/2 만큼 왼쪽에 차량 중심이 오도록.
        # lateral_err = ( (right_x - img_center_x_bev) - lane_width_k_pixels / 2.0 ) * PIXEL_TO_METER_X
        # heading_err = right_slope_rad # 오른쪽 차선의 각도 사용
        # 현재 코드가 제공한 lateral_err 계산 방식을 최대한 따름
        # right_x가 -130이면 오른쪽 차선 검출됨 (xycar_msgs/laneinfo 주석 참고)
        # lane_data.right_x는 이미지 중앙 기준 오른쪽 차선의 x좌표 offset
        # 즉, (오른쪽 차선 x) - (이미지 중앙 x)
        # lane_width / 150.0 은 픽셀 스케일링으로 보임
        y_e = -(0.5 - ((right_x+130) / 150.0)) * lane_width_k if right_x != -130 else 0 # laneinfo 기준 right_x는 음수일 수 있음
        phi_e = right_slope_rad if right_x != -130 else 0
    elif right_x == -130: # 오른쪽 유실 (왼쪽 차선만 존재)
        # lane_data.left_x는 이미지 중앙 기준 왼쪽 차선의 x좌표 offset (음수값)
        y_e = (0.5 - ((left_x+130) / 150.0)) * lane_width_k if left_x != 130 else 0
        phi_e = left_slope_rad if left_x != 130 else 0
    else: # 양쪽 차선 모두 존재
        # 차량 중심 x좌표 (BEV)
        current_car_center_x_bev = (left_x + right_x) / 2.0
        # 차선 중앙 x좌표 (BEV 중앙점)
        desired_lane_center_x_bev = img_center_x_bev # BEV 이미지 중앙에 맞추려 한다고 가정.
                                                    # 실제로는 (left_x + right_x)/2가 현재 차선 중심, 
                                                    # 이를 이미지 중앙(130)에 오도록 하는 것이 오차.
        # y_e = (desired_lane_center_x_bev - current_car_center_x_bev) * PIXEL_TO_METER_X
        # 현재 코드의 lateral_err 계산 방식을 따름.
        # left_x / (left_x + right_x) 가 아니라, ((left_x+right_x)/2 - 130) / ( (right_x-left_x)/2 ) * (lane_width/2) 가 더 일반적.
        # 제공된 코드: lateral_err = -(left_x / (left_x + right_x) - 0.5) * lane_width
        # left_x, right_x가 BEV 중앙 기준 offset이 아니라 절대 좌표라고 가정하면 이 식이 맞을 수 있음.
        # xycar_msgs/laneinfo 에 따르면 left_x는 왼쪽차선 x 위치, right_x는 오른쪽차선 x 위치 (BEV)
        # 따라서 (left_x+right_x)/2 가 현재 차선 중심의 x좌표. 이를 130(이미지 중앙)에 오도록 하는 오차.
        # lateral_error_pixels = 130 - (left_x + right_x) / 2.0
        # y_e = lateral_error_pixels * (lane_width_k / (right_x - left_x)) # 픽셀 오차를 미터 오차로 변환
        # 제공된 식을 그대로 사용:
        y_e = -((left_x + right_x) / 2.0 / 130.0 - 1.0) * (lane_width_k / 2.0) # 약간 수정 (중앙 기준 오차 비율)
        # y_e = -(left_x / (left_x + right_x) - 0.5) * lane_width_k # 원본 제공 코드 (left_x, right_x가 양수 범위라고 가정)

        # xcar_msgs/laneinfo의 left_x, right_x는 BEV 이미지의 좌측, 우측 차선 x좌표.
        # 중앙은 (left_x + right_x) / 2. 이것이 130 (카메라 중앙)에서 얼마나 벗어났는지가 오차.
        # 오차 = 130 - (left_x + right_x) / 2 (픽셀 단위)
        # 이 오차를 실제 미터 단위로 변환해야 함. (right_x - left_x)가 차선폭(픽셀).
        # y_e_pixels = 130 - (left_x + right_x) / 2.0
        # pixels_per_meter = (right_x - left_x) / lane_width_k if (right_x - left_x) > 0 else 150.0 / lane_width_k # 예시
        # y_e = y_e_pixels / pixels_per_meter

        # 코드가 제공한 원래 식을 최대한 살리면서, xycar_msgs/laneinfo 스펙에 맞춰 가정:
        # left_x, right_x는 0~260 범위의 x좌표. 중앙은 130.
        # 차량 위치가 차선 중앙 ((left_x+right_x)/2) 대비 얼마나 벗어났는지 (130 기준)
        current_center_offset_from_img_center = ((left_x + right_x) / 2.0) - 130.0
        # 차선 폭 (픽셀) = right_x - left_x
        # 오차 비율 = current_center_offset_from_img_center / ( (right_x - left_x) / 2.0 )
        # y_e = 오차 비율 * (lane_width_k / 2.0)
        # 부호: 차량이 중앙보다 오른쪽에 있으면(offset > 0), y_e는 음수여야 왼쪽으로 이동.
        if (right_x - left_x) > 10: # 유효한 차선 폭일 때
             y_e = - (current_center_offset_from_img_center / ((right_x - left_x) / 2.0)) * (lane_width_k / 2.0)
        else: # 차선 폭이 너무 좁으면 (오류 가능성)
             y_e = 0

        phi_e = (left_slope_rad + right_slope_rad) / 2.0 # 평균 각도 오차

    phi_e *= -1 # 제공된 코드의 phi_e 부호 일치 (차량 헤딩이 차선과 이루는 각도)

    # 각속도 omega 계산
    # v_actual = v_r_k * math.cos(phi_e) # 실제 전진 속도 (필요시 사용)
    # Kanayama 논문에서는 v_r이 기준 속도(constant).
    omega = -v_r_k * (K_y_k * y_e + K_phi_k * math.sin(phi_e)) # omega = -K2*v*y_e/theta_e * sin(theta_e) - K1*theta_e (다른 형태)
                                                            # omega = -v_r ( K_y y_e + K_phi sin phi_e ) (이 형태 사용)

    # 조향각 delta 계산 (라디안)
    # tan(delta) = L * omega / v_actual
    # v_actual 대신 v_r_k를 사용하는 경우도 있음 (작은 phi_e에 대해 cos(phi_e) ~ 1)
    if abs(v_r_k) < 0.01 : # 속도가 0에 가까우면 조향각 0
        delta_rad = 0.0
    else:
        delta_rad = math.atan2(L * omega, v_r_k)

    steering_angle_deg = math.degrees(delta_rad)
    
    # 조향각 제한은 drive_motor 함수에서 이미 수행될 수 있으나, 여기서도 한 번 더.
    steering_angle_deg = max(min(steering_angle_deg, 50.0), -50.0)
    
    # Kanayama는 속도도 제어 결과로 나올 수 있으나, 여기서는 Fix_Speed 유지하고 조향각만 반환
    return steering_angle_deg, v_r_k # 속도는 Fix_Speed 그대로 사용


def calculate_steering_angle(): # 기존 k_p 기반 제어 (단순화된 버전)
    global lane_data, k_p
    if lane_data is None:
        return 0.0
    
    left_slope_rad = lane_data.left_slope
    right_slope_rad = lane_data.right_slope
    
    avg_slope_rad = 0.0
    # 한쪽 차선만 인식되었을 때 해당 차선 기울기 사용, 양쪽 다 있으면 평균
    if lane_data.left_x != 130 and lane_data.right_x == -130: # 왼쪽만
        avg_slope_rad = left_slope_rad
    elif lane_data.left_x == 130 and lane_data.right_x != -130: # 오른쪽만
        avg_slope_rad = right_slope_rad
    elif lane_data.left_x != 130 and lane_data.right_x != -130: # 양쪽 다
        avg_slope_rad = (left_slope_rad + right_slope_rad) / 2.0
    else: # 양쪽 다 없음
        return 0.0

    # 기울기(라디안)를 각도(degree)로 변환하고 부호 반전
    steering_angle_from_slope = -1 * math.degrees(avg_slope_rad)
    
    # 여기에 추가적인 오프셋 기반 제어 로직이 필요할 수 있음 (k_lateral 등)
    # 원본 코드의 k_p, k_lateral 부분을 참고하여 여기에 구현
    # 예시: (매우 단순화)
    # center_offset_pixels = 130 - (lane_data.left_x + lane_data.right_x) / 2.0 (양쪽 차선 있을때)
    # steering_angle_from_offset = k_lateral_simple * center_offset_pixels
    # final_steer = k_p_simple * steering_angle_from_slope + steering_angle_from_offset
    
    # 현재는 k_p * (기울기 기반 각도)만 사용
    final_steer = k_p * steering_angle_from_slope
    final_steer = max(min(final_steer, 50.0), -50.0)
    # rospy.loginfo("k_p_steer_angle: %.2f", final_steer) # 디버깅 로그는 필요시 활성화
    return final_steer


# 차선 변경 관련 함수 (generate_quadratic_path, local_to_global, transform_path, convert_local_path_to_image_coords, visualize_path, generate_lane_change_path, lane_change_control)
# 이 함수들은 기존 코드에서 그대로 가져와서 사용합니다. (길어서 생략, 원본 코드 참조)
# 단, 내부에서 사용하는 L, BASE_X, BASE_Y, PATH_POINTS, LANE_CHANGE_DISTANCE 등의 전역변수 확인 필요.
def generate_quadratic_path(dx, dy, theta=0, steps=PATH_POINTS): # PATH_POINTS 전역변수 사용
    A = np.array([[0**2,0,1],[dx**2,dx,1],[0,1,0]]) # y'(0) 부분 수정 필요 (2*0*a + b = tan(theta)) -> [2*0, 1, 0]
    B = np.array([0, dy, math.tan(theta)])
    try: a,b,c = np.linalg.solve(A,B)
    except np.linalg.LinAlgError: return [(x,y) for x,y in zip(np.linspace(0,dx,steps), np.linspace(0,dy,steps))]
    xs = np.linspace(0,dx,steps); ys = a*xs**2 + b*xs + c
    return list(zip(xs,ys))

def generate_lane_change_path(current_lane_num, target_lane_num, start_x_bev, left_x_bev, right_x_bev): # 변수명 명확히
    global LANE_CHANGE_DISTANCE, PATH_POINTS, BASE_Y
    if current_lane_num == 0 or target_lane_num == 0 : return [],0
    
    # dx, dy 계산 (BEV 기준이 아닌 차량 로컬 좌표 기준이어야 함)
    # start_x_bev는 현재 차량 중심의 BEV x 좌표
    # 목표는 옆 차선으로 이동. 옆 차선 중심과의 x 차이가 dx가 됨.
    # dy는 전방 주시 거리 LANE_CHANGE_DISTANCE (미터 단위로 가정)

    # 이 함수는 현재 BEV 좌표를 사용하는데, 차선 변경 경로는 차량 로컬 좌표계에서 생성 후 변환하는 것이 일반적.
    # 제공된 코드는 start_x(BEV) 기준으로 dx, dy를 계산하고 있음.
    # dx: 옆 차선까지의 BEV x 이동량. dy: 전방 주시 거리 (미터)
    # 이 부분은 원래 코드의 로직을 최대한 따르되, 단위와 좌표계 일관성 중요.
    # 여기서는 단순화하여 기존 dx 계산 방식 유지.
    
    dx_local = 0; direction_local = 0
    # 차선 폭을 이용해 dx 계산 (미터 단위로 가정)
    # BEV 상에서 75픽셀이 약 차선폭의 절반이라고 가정했었음.
    # LANE_WIDTH_METERS = 1.5 # 예시
    # PIXELS_PER_METER_X_BEV = (right_x_bev - left_x_bev) / LANE_WIDTH_METERS if (right_x_bev != left_x_bev) else 50 
    # dx_pixels_bev = 75 * 2 # 한 차선 폭 (픽셀)
    
    # 기존 코드의 dx = +/-90 (픽셀?) 사용
    if current_lane_num == 1 and target_lane_num == 2: # 1차선 -> 2차선 (오른쪽으로 이동)
        dx_local = 0.8 # 오른쪽으로 0.8m 이동한다고 가정 (튜닝 필요)
        direction_local = 1
    elif current_lane_num == 2 and target_lane_num == 1: # 2차선 -> 1차선 (왼쪽으로 이동)
        dx_local = -0.8 # 왼쪽으로 0.8m 이동
        direction_local = -1
    else: return [], 0
    
    dy_local = LANE_CHANGE_DISTANCE / 50.0 # LANE_CHANGE_DISTANCE가 픽셀 단위였다면 스케일링 필요, 미터 단위라면 그대로 사용
                                        # 여기서는 LANE_CHANGE_DISTANCE를 전방 주시 거리(미터)로 보고 dy_local로 사용
    
    # generate_quadratic_path는 로컬 좌표계 (차량 앞이 +y, 차량 좌측이 +x 또는 우측이 +x) 기준.
    # dx_local: 목표 x (옆으로 이동량), dy_local: 목표 y (앞으로 이동량)
    # 현재 차량 방향 theta = 0 (직진)으로 가정
    local_path = generate_quadratic_path(dx_local, dy_local, theta=0, steps=PATH_POINTS)

    # 변환된 로컬 경로를 다시 BEV나 이미지 좌표로 변환하는 부분은 visualize_path 등에서 처리.
    # 이 함수는 차선 변경에 사용할 "로컬 경로 포인트"와 "방향"을 반환.
    # 기존 코드는 adjusted_path를 BEV 좌표로 반환했었음.
    # 여기서는 로컬 경로를 반환하고, lane_change_control에서 이 로컬 경로를 사용한다고 가정.
    # (또는 lane_change_control이 BEV 경로를 받는다면, 여기서 BEV로 변환해야 함)
    # 일단은 로컬 경로를 반환하는 것으로 단순화. visualize_path는 BEV 경로를 받았었음.
    # 기존 코드와의 호환성을 위해, 이 함수는 여전히 "BEV 유사 좌표" 경로를 반환한다고 가정하고 수정.
    # dx, dy를 BEV 스케일의 값으로 사용.
    
    dx_bev_like = 0; dy_bev_like = LANE_CHANGE_DISTANCE # LANE_CHANGE_DISTANCE가 BEV y축 이동량으로 가정
    if current_lane_num == 1 and target_lane_num == 2: dx_bev_like = 90; direction_local = 1
    elif current_lane_num == 2 and target_lane_num == 1: dx_bev_like = -90; direction_local = -1
    else: return [],0

    # generate_quadratic_path는 (0,0)에서 시작하는 경로.
    # 이를 start_x_bev 기준으로 오프셋. y는 BASE_Y(아마도 이미지 하단)에서 빼는 방식.
    # 이 부분은 시뮬레이션 환경의 좌표계에 매우 의존적.
    # path = generate_quadratic_path(dx_bev_like, dy_bev_like, theta=0, steps=PATH_POINTS)
    # adjusted_path = [(start_x_bev + p[0], BASE_Y - p[1]) for p in path]
    # generate_quadratic_path 의 dx, dy는 목표점의 상대좌표.
    # 즉, 현재 (0,0) 에서 (dx_bev_like, dy_bev_like) 로 가는 경로를 생성.
    # 이 경로 자체를 차선 변경 경로로 사용 (시작점은 항상 (0,0)인 로컬 경로)
    path_for_lane_change = generate_quadratic_path(dx_bev_like, dy_bev_like, theta=0, steps=PATH_POINTS)

    # rospy.loginfo(f"Lane change path generated for {current_lane_num}->{target_lane_num}")
    return path_for_lane_change, direction_local


def lane_change_control(local_path_to_follow, current_x_bev, current_y_bev, current_heading_rad_bev, direction):
    global L, Fix_Speed
    # 이 함수는 local_path_to_follow (차량 로컬 좌표계 기준 경로)를 추종해야 함.
    # current_x_bev, current_y_bev, current_heading_rad_bev는 현재 차량의 BEV 상의 상태.
    # BEV 상태를 로컬 오차로 변환하거나, 경로를 BEV로 변환해야 함.
    # 여기서는 Pure Pursuit과 유사하게 로컬 경로상의 목표점을 찾아 제어.
    # 차선 변경 시에는 차량의 현재 위치가 로컬 경로의 (0,0)이라고 가정하고 경로 추종.

    if not local_path_to_follow: return 0.0

    # 차선 변경 경로는 (0,0)에서 시작하여 (dx, dy)로 가는 로컬 경로.
    # Pure Pursuit과 유사하게 경로상의 lookahead_point를 찾음.
    # 차선 변경 시 lookahead 거리는 짧게 가져가는 것이 일반적.
    lookahead_dist_lc = Fix_Speed / 10.0 * 0.8 # 속도에 비례, 예시값 (1.0~2.0m 정도)
    lookahead_dist_lc = np.clip(lookahead_dist_lc, 0.5, 2.0)

    target_point_lc = None
    for point in reversed(local_path_to_follow): # 경로의 끝점부터 탐색
        dist = math.hypot(point[0], point[1]) # (0,0)에서 경로점까지의 거리
        if dist <= lookahead_dist_lc:
            target_point_lc = point
            break
    if target_point_lc is None: # 못찾으면 경로의 마지막 점 사용 (가장 먼 점)
        target_point_lc = local_path_to_follow[-1] if local_path_to_follow else (0,0)
    
    if target_point_lc == (0,0) : return 0.0 # 목표점이 없거나 (0,0)이면 조향 안함

    # 로컬 좌표계 목표점 (target_x_lc, target_y_lc)
    # 차량 전방이 +y, 좌측이 +x 인 로컬 좌표계 가정
    alpha_lc = math.atan2(target_point_lc[0], target_point_lc[1]) # x_local, y_local
    
    # Pure Pursuit 조향각 공식 (차선 변경 중에는 L 값의 영향이 클 수 있음)
    # L_lc = L # 동일한 축거 사용
    delta_rad_lc = math.atan2(2.0 * L * math.sin(alpha_lc), math.hypot(target_point_lc[0], target_point_lc[1]))
    
    steering_angle_deg_lc = math.degrees(delta_rad_lc)
    # 차선 변경 방향(direction)을 고려하여 조향각 부호 결정 가능하나,
    # local_path 자체가 이미 방향을 포함하고 있으므로, 그대로 사용.
    # (만약 local_path의 dx가 음수이면 alpha_lc도 음수가 되어 좌회전 유도)
    # 기존 코드의 direction * abs(base) 방식은 경로 생성 방식에 따라 달라짐.
    # 여기서는 alpha_lc가 직접적인 조향 방향을 나타낸다고 가정.

    # 추가적인 게인이나 오프셋은 실험을 통해 조정.
    # 기존 코드: steering_angle = direction * abs(base)
    # base = math.degrees(math.atan2(6 * math.sin(alpha), 15)) -> 이 형태는 특정 제어기 설계
    # 여기서는 순수 추적 기반으로 변경.
    
    return max(min(steering_angle_deg_lc, 50.0), -50.0)


# *** 꼬깔콘 경로 추종을 위한 조향각 계산 함수 ***
def calculate_steer_from_cone_center_path(center_path_points, current_vehicle_speed):
    global L_wheelbase, CONE_PURE_PURSUIT_K_DD, CONE_MIN_LOOKAHEAD_DIST, CONE_MAX_LOOKAHEAD_DIST, CONE_STEERING_GAIN
    # 이 함수는 track_drive.py로 옮겨졌으므로, 여기서는 L 대신 L_wheelbase,
    # 파라미터명도 CONE_... 형태로 된 것을 사용한다고 가정하고 작성.
    # 실제로는 track_drive.py에 있는 해당 함수가 호출됨.
    # 이 파일(cone_lane_detector.py)에는 이 함수가 필요 없음.
    # 만약 track_drive.py의 해당 함수를 여기에 가져온다면 아래와 같이 될 것임.

    if not center_path_points or len(center_path_points) < 2:
        return 0.0

    path_np = np.array([[p.x, p.y] for p in center_path_points]) # geometry_msgs/Point 리스트를 NumPy로

    lookahead_dist = np.clip(CONE_PURE_PURSUIT_K_DD * current_vehicle_speed, CONE_MIN_LOOKAHEAD_DIST, CONE_MAX_LOOKAHEAD_DIST)
    if current_vehicle_speed < 1.0 : lookahead_dist = CONE_MIN_LOOKAHEAD_DIST 
    
    path_ahead = path_np[path_np[:,1] >= -0.05] 
    if path_ahead.shape[0] < 1: return 0.0

    target_idx = -1; target_point_found = False
    for i in range(path_ahead.shape[0]):
        dist_to_pt_i = math.hypot(path_ahead[i,0], path_ahead[i,1]) 
        if dist_to_pt_i >= lookahead_dist: target_idx = i; target_point_found = True; break
    
    if not target_point_found and path_ahead.shape[0] > 0 : target_idx = path_ahead.shape[0] - 1 
    elif not target_point_found : return 0.0 
            
    final_target_point = None
    if target_idx > 0 and target_point_found and path_ahead.shape[0] > 1: 
        p1 = path_ahead[target_idx-1, :]; p2 = path_ahead[target_idx, :]  
        d1 = math.hypot(p1[0], p1[1]); d2 = math.hypot(p2[0],p2[1])
        if (d2 - d1) > 0.01: ratio = np.clip((lookahead_dist - d1) / (d2 - d1), 0.0, 1.0); final_target_point = p1 + ratio * (p2 - p1)
        else: final_target_point = p2
    elif path_ahead.shape[0] > 0: final_target_point = path_ahead[target_idx,:] 
    else: return 0.0
    if final_target_point is None: return 0.0
    
    target_x_val = final_target_point[0]; target_y_val = final_target_point[1]
    alpha = math.atan2(target_x_val, target_y_val) if abs(target_y_val) > 0.01 else math.copysign(math.pi / 2.0, target_x_val)
    actual_dist_to_target = math.hypot(target_x_val, target_y_val)
    if actual_dist_to_target < 0.01: return 0.0

    steering_angle_rad = math.atan2(2.0 * L * math.sin(alpha), actual_dist_to_target) # L 사용 (전역변수)
    steering_angle_rad *= CONE_STEERING_GAIN 
    
    return max(min(math.degrees(steering_angle_rad), 50.0), -50.0)


#=============================================
# 실질적인 메인 함수 
#=============================================
def start():
    global motor, image, ranges, lane_data, cone_lanes_data # 전역 변수 사용 명시
    global light_go, vehicle_ahead, is_lane_changing # 상태 변수들
    global lidar_points_plot, fig # Matplotlib 시각화용

    rospy.init_node('Track_Driver_Combined', anonymous=False) # 노드 이름 변경 가능
    
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    rospy.Subscriber("/lane_info", laneinfo, lane_callback, queue_size=1) # 카메라 차선
    # *** 꼬깔콘 차선 정보 구독자 추가 ***
    rospy.Subscriber("/cone_lanes", ConeLanes, cone_lanes_data_callback, queue_size=1) 
    
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
        
    rospy.loginfo("Waiting for first messages...")
    # lane_data는 처음에 None일 수 있으므로 cone_lanes_data처럼 필수는 아님
    while (image.size == 0 or ranges is None) and not rospy.is_shutdown():
        rospy.loginfo_once("Waiting for camera image and lidar scan...")
        time.sleep(0.1)
    
    if rospy.is_shutdown(): rospy.loginfo("Shutdown requested during init."); return
    rospy.loginfo("Essential messages ready. Lidar Visualizer will start if enabled.")
    
    if ENABLE_MATPLOTLIB_VIZ: # globals()에 있는지 확인하는 대신 플래그 사용
        plt.ion()
        plt.show()
        rospy.loginfo("Lidar Visualizer enabled and shown.")
    else:
        rospy.loginfo("Lidar Visualizer disabled.")

    rospy.loginfo("======================================")
    rospy.loginfo(" S T A R T    D R I V I N G ...")
    rospy.loginfo("======================================")
    
    loop_rate = rospy.Rate(20) # 루프 빈도 (기존 10Hz에서 증가 시도)

    while not rospy.is_shutdown():
        steer_angle_cmd = 0.0
        speed_cmd = Fix_Speed # 기본 속도

        # --- 1. 시각화 (라이다) ---
        if ENABLE_MATPLOTLIB_VIZ and ranges is not None and len(ranges)==360:
            # 라이다 원본 데이터 시각화 (convert_lidar_to_xy는 하지 않음, cone_detector가 함)
            # 여기서는 간단히 극좌표계를 데카르트 좌표계로 변환하여 그림
            # (참고: 이 부분은 cone_detector의 시각화와는 별개로, track_drive가 보는 ranges를 그림)
            angles_viz = np.radians(np.arange(360))
            # lidar_points_plot에 사용할 x,y 계산 시, 차량 좌표계 일치 중요
            # convert_lidar_to_xy에서 사용한 좌표계와 동일하게 (X: 좌+, Y: 전방+)
            # 또는 기존 코드의 angles = np.linspace(0,2*np.pi, len(ranges))+np.pi/2 방식 유지
            # 기존 코드 방식: 정면이 Y축 음수, 우측이 X축 양수였던 것으로 보임.
            # 여기서는 일관성을 위해 convert_lidar_to_xy와 유사한 방식으로 변환 (단, 필터링 없이 전체)
            
            # 간단한 시각화를 위해, 필터링 안된 ranges의 앞부분만 그림
            # (이 부분은 사용자의 원래 의도에 맞게 수정 필요, 현재는 비활성화된 것처럼 동작)
            # x_viz = current_ranges * np.cos(angles_viz_for_plot) # 기존 코드의 좌표계와 다를 수 있음
            # y_viz = current_ranges * np.sin(angles_viz_for_plot)
            # lidar_points_plot.set_data(x_viz, y_viz) 
            # fig.canvas.draw_idle()
            # plt.pause(0.001)
            pass # 라이다 원본 시각화는 일단 생략 (cone_detector가 더 자세히 함)


        # --- 2. 신호등 처리 ---
        current_tl_state = 'green' # 기본값 (실제로는 detect_traffic_light 호출)
        if image.size != 0:
            current_tl_state = detect_traffic_light(image)
        
        if current_tl_state == 'green': light_go = True
        elif current_tl_state == 'red' or current_tl_state == 'yellow': light_go = False
        # 'none'이면 이전 상태 유지

        # --- 3. 주행 로직 결정 (꼬깔콘 우선) ---
        driving_mode = "None" # 현재 주행 모드 로깅용

        # 3.1 꼬깔콘 차선 정보 사용 시도
        if cone_lanes_data is not None and \
           (cone_lanes_data.left_lane_detected or cone_lanes_data.right_lane_detected) and \
           len(cone_lanes_data.center_path) >= 2 and \
           not is_lane_changing : # 차선 변경 중이 아닐 때
            
            driving_mode = "Cone Lanes"
            # 꼬깔콘 중앙 경로로 조향각 계산
            # calculate_steer_from_cone_center_path 함수는 이 파일 내에 정의되어 있어야 함
            # (이전에 cone_lane_detector.py에 있던 것을 가져오거나 유사하게 만듦)
            steer_angle_cmd = calculate_steer_from_cone_center_path(cone_lanes_data.center_path, speed_cmd)
            # 꼬깔콘 주행 시 속도는 Fix_Speed 유지 또는 상황 따라 조절 가능
        
        # 3.2 카메라 기반 차선 정보 사용 (꼬깔콘 정보 없거나 유효하지 않을 때)
        elif lane_data is not None: # is_lane_changing 조건은 kanayama_control 내부에서 처리
            driving_mode = "Camera Lanes (Kanayama)"
            # kanayama_control은 (조향각, 속도)를 반환
            # steer_angle_cmd, speed_cmd = kanayama_control() # Kanayama가 속도도 결정
            
            # 또는 기존 calculate_steering_angle 사용 시
            if not is_lane_changing: # 차선 변경 중 아닐때만 일반 주행 로직
                 # steer_angle_cmd = calculate_steering_angle() # 기존 k_p 기반
                 # speed_cmd = Fix_Speed
                 steer_angle_cmd, speed_cmd = kanayama_control() # Kanayama 사용 권장
            else: # 차선 변경 중이면 kanayama_control이 이미 차선변경 제어 로직을 탐
                 steer_angle_cmd, speed_cmd = kanayama_control()


        # 3.3 모든 정보가 없으면 직진 또는 정지
        else:
            driving_mode = "No Lane Info"
            steer_angle_cmd = 0.0
            # speed_cmd = 0 # 안전 정지
            # 또는 Fix_Speed * 0.5 등으로 감속

        rospy.loginfo_throttle(1.0, f"Mode: {driving_mode}, Steer: {steer_angle_cmd:.2f}, Speed: {speed_cmd:.2f}, TL: {current_tl_state}({light_go}), VehAhead: {vehicle_ahead}")

        # --- 4. 최종 속도 결정 (신호등, 전방 차량 등) ---
        if not light_go: # 빨간불/노란불이면 정지
            speed_cmd = 0
        
        if vehicle_ahead: # 전방 차량 감지 시
            rospy.logwarn_throttle(1.0, "Vehicle ahead detected by lidar! Reducing speed / Stopping.")
            speed_cmd = min(speed_cmd, Fix_Speed * 0.3) # 속도 크게 줄이거나 정지
            if speed_cmd < 5: speed_cmd = 0 # 매우 느리면 정지

        # --- 5. 모터 제어 ---
        drive_motor(steer_angle_cmd, speed_cmd)
        
        # OpenCV 이미지 표시 (필요시 활성화)
        # if image.size != 0:
        #     display_img_debug = image.copy()
        #     # 여기에 추가적인 디버깅 정보 그리기 가능
        #     cv2.imshow("Track Drive Debug View", display_img_debug)
        #     cv2.waitKey(1)
            
        loop_rate.sleep()

    # 루프 종료 시
    # cv2.destroyAllWindows() # cv2.imshow 사용 시
    drive_motor(0,0) # 안전 정지
    rospy.loginfo("Track_Driver_Combined shutting down.")

#=============================================
# 메인함수를 호출합니다.
#=============================================
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by ROS.")
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by User (Ctrl+C).")
    finally:
        # 프로그램 종료 시 항상 모터 정지 (안전 조치)
        # motor 변수가 초기화 되었는지, ROS core가 살아있는지 등 확인
        if 'motor' in globals() and motor is not None and \
           rospy.core.is_initialized() and not rospy.core.is_shutdown_requested():
            rospy.loginfo("Final motor stop command.")
            # drive_motor(0,0) 함수는 motor 객체가 None이 아님을 내부에서 확인하므로, 바로 호출 가능
            for _ in range(3): # 확실한 정지를 위해 여러 번 보낼 수 있음
                drive_motor(0,0)
                time.sleep(0.01) # 짧은 지연
        
        if ENABLE_MATPLOTLIB_VIZ and 'plt' in globals(): # plt가 import 되었는지 확인
            plt.ioff()
            plt.close('all')
        rospy.loginfo("Cleanup complete. Exiting.")