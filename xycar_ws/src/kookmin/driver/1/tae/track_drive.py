#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================
# 본 프로그램은 2025 제8회 국민대 자율주행 경진대회에서
# 예선과제를 수행하기 위한 파일입니다. 
# 예선과제 수행 용도로만 사용가능하며 외부유출은 금지됩니다.
# =============================================
import numpy as np
import cv2, rospy, time, math
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import XycarMotor, laneinfo, ConeLanes # ConeLanes 추가
from cv_bridge import CvBridge
# import matplotlib.pyplot as plt # 라이다 시각화는 cone_lane_detector에서 하므로 여기서는 제거 또는 주석처리

# =============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
# =============================================
image_raw = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
lidar_ranges = None  # 라이다 데이터를 담을 변수 (차량 감지용)
motor_control = None  # 모터노드
motor_msg_obj = XycarMotor()  # 모터 토픽 메시지

# --- 주행 파라미터 ---
FIX_SPEED_NORMAL = 37       # 일반 차선 주행 속도
CONE_DRIVE_MAX_SPEED = 3 # 라바콘 주행 모드 최대 속도
WHEELBASE = 0.5             # 차량 축거 (m), Pure Pursuit용

# --- 제어 관련 변수 ---
current_steering_angle = 0  # 현재 조향각 (디버깅용)
current_target_speed = FIX_SPEED_NORMAL  # 현재 목표 속도

bridge = CvBridge()
lane_info_data = None       # 카메라 기반 차선 정보를 담을 변수
cone_lanes_data = None    # 라이다 기반 라바콘 차선 정보를 담을 변수 (새로 추가)

# --- 신호등 관련 ---
traffic_light_go = False  # 초록불 인식 후 출발 유지 플래그

# --- 차량 감지 관련 (기존 코드 유지) ---
vehicle_ahead_flag = False
detection_success_streak = 0
DETECTION_DISTANCE_LOW = 10.0 #원래 5
DETECTION_DISTANCE_HIGH = 40.0
DETECTION_COUNT = 4
SECTOR_WIDTH = 7
DETECTION_STREAK_THRESHOLD = 2

# --- 차선 변경 관련 (기존 코드 유지, 일부 수정 가능성) ---
current_driving_lane = 0  # 현재 주행 차선 번호 (0: 불확실, 1: 1차선, 2: 2차선)
is_lane_changing_now = False  # 차선 변경 중인지 여부
lane_change_generated_path = []  # 차선 변경 경로
lane_change_direction_val = 0  # 차선 변경 방향 (1: 우회전, -1: 좌회전)
lane_change_start_timestamp = 0  # 차선 변경 시작 시간
# last_lane_change_time = 0 # 사용되지 않는다면 제거 가능
LANE_CHANGE_DURATION_SEC = 1.65 # 차선 변경 소요 시간 (초)
PATH_POINTS_COUNT = 20 # 경로 생성 시 샘플링할 포인트 수
LANE_CHANGE_LOOKAHEAD_DIST = 230 # 차선 변경 시 전방 주시 거리 (BEV 픽셀 단위였던 것으로 추정)

# --- 차선 감지 실패 처리 관련 (기존 코드 유지) ---
# right_lane_missing_time = 0
# left_lane_missing_time = 0
# LANE_MISSING_THRESHOLD_SEC = 2.0
# last_right_lane_detected_bool = True
# last_left_lane_detected_bool = True

# 이미지 관련 상수 (BEV 기준)
BEV_IMAGE_WIDTH = 260
BEV_IMAGE_HEIGHT = 260
BEV_BASE_X_CENTER = BEV_IMAGE_WIDTH / 2.0 # 130
BEV_BASE_Y_BOTTOM = BEV_IMAGE_HEIGHT      # 260

# =============================================
# 콜백함수
# =============================================
def usbcam_callback(data):
    global image_raw
    image_raw = bridge.imgmsg_to_cv2(data, "bgr8")

def lidar_scan_callback(data): # 함수 이름 변경 (lidar_callback -> lidar_scan_callback)
    global lidar_ranges, vehicle_ahead_flag, detection_success_streak
    lidar_ranges = data.ranges[0:360] # 전체 360도 사용

    # 차량 감지 로직 (기존과 동일)
    detected_this_scan = False
    consecutive_points = 0
    for offset in range(-SECTOR_WIDTH, SECTOR_WIDTH + 1):
        dist = lidar_ranges[offset % 360]
        if DETECTION_DISTANCE_LOW < dist < DETECTION_DISTANCE_HIGH:
            consecutive_points += 1
            if consecutive_points >= DETECTION_COUNT:
                detected_this_scan = True
                break
        else:
            consecutive_points = 0

    if detected_this_scan:
        detection_success_streak += 1
    else:
        detection_success_streak = 0

    if detection_success_streak >= DETECTION_STREAK_THRESHOLD:
        vehicle_ahead_flag = True
    else:
        vehicle_ahead_flag = False

def lane_info_callback(data): # 카메라 기반 차선 정보 콜백
    global lane_info_data, current_driving_lane, is_lane_changing_now
    global lane_change_generated_path, lane_change_direction_val, lane_change_start_timestamp
    
    lane_info_data = data
    
    current_time_sec = time.time()

    if data.lane_number in [1, 2]:
        current_driving_lane = data.lane_number

    # 차선 변경 조건 (차량 감지 시) - 이 로직은 차선 주행 모드에서만 유효해야 함
    # 라바콘 주행 중에는 차선 변경 안 함
    if not is_driving_in_cone_mode() and not is_lane_changing_now and vehicle_ahead_flag:
        is_lane_changing_now = True
        lane_change_start_timestamp = current_time_sec
        
        target_lane = 2 if current_driving_lane == 1 else 1
        
        # 현재 차선 중심점 계산 (BEV 이미지 기준)
        # lane_info_data.left_x, right_x는 차량 중심(x=0) 기준 상대 좌표
        # left_x: 음수, right_x: 양수
        # BEV 이미지 x좌표로 변환: BEV_BASE_X_CENTER + 상대좌표
        
        # start_x_bev: 현재 차량의 BEV 이미지상 x 위치 (차선 중심)
        start_x_bev = BEV_BASE_X_CENTER 
        if lane_info_data.left_x != (BEV_BASE_X_CENTER) and lane_info_data.right_x != (-BEV_BASE_X_CENTER): # 양쪽 다 유효하면
             # left_x는 (중심 - 왼쪽차선x), right_x는 (오른쪽차선x - 중심)
             # 왼쪽차선_bev_x = BEV_BASE_X_CENTER - lane_info_data.left_x
             # 오른쪽차선_bev_x = BEV_BASE_X_CENTER + lane_info_data.right_x
             # 중심_bev_x = (왼쪽차선_bev_x + 오른쪽차선_bev_x) / 2
             # = BEV_BASE_X_CENTER + (lane_info_data.right_x - lane_info_data.left_x) / 2
             # 이 계산은 laneinfo 메시지의 left_x, right_x 정의에 따라 달라짐.
             # 기존 코드에서는 left_x가 130-실제x, right_x가 실제x-130 이었음.
             # 수정된 lane_detection.py에서는 left_x = 130 - fit_x, right_x = fit_x - 130
             # 따라서 (fit_left + fit_right)/2 를 구하려면 (130-left_x + 130+right_x)/2 = 130 + (right_x-left_x)/2
            start_x_bev = BEV_BASE_X_CENTER + (lane_info_data.right_x - lane_info_data.left_x) / 2.0

        elif lane_info_data.left_x != (BEV_BASE_X_CENTER) : # 왼쪽만 유효
            # 왼쪽차선_bev_x = BEV_BASE_X_CENTER - lane_info_data.left_x
            # 오른쪽은 차선폭(예:150픽셀)만큼 더함
            start_x_bev = (BEV_BASE_X_CENTER - lane_info_data.left_x) + 75 
        elif lane_info_data.right_x != (-BEV_BASE_X_CENTER) : # 오른쪽만 유효
            # 오른쪽차선_bev_x = BEV_BASE_X_CENTER + lane_info_data.right_x
            start_x_bev = (BEV_BASE_X_CENTER + lane_info_data.right_x) - 75
        else: # 둘 다 유효하지 않으면 중앙으로 가정
            start_x_bev = BEV_BASE_X_CENTER

        lane_change_generated_path, lane_change_direction_val = generate_lane_change_path_bev(
            current_driving_lane, target_lane,
            start_x_bev, # BEV 이미지 상의 현재 x 위치
            BEV_BASE_X_CENTER - lane_info_data.left_x, # BEV 왼쪽 차선 x
            BEV_BASE_X_CENTER + lane_info_data.right_x  # BEV 오른쪽 차선 x
        )
        rospy.loginfo(f"Lane change initiated: {current_driving_lane} -> {target_lane}")


def cone_lanes_info_callback(data): # 라이다 기반 라바콘 차선 정보 콜백 (새로 추가)
    global cone_lanes_data
    cone_lanes_data = data

# =============================================
# 유틸리티 및 제어 함수
# =============================================
def detect_traffic_light(img_bgr): # 기존 함수 유지
    if img_bgr is None or img_bgr.size == 0:
        return 'none'
    h, w = img_bgr.shape[:2]
    # ROI를 이미지 상단 중앙으로 좀 더 넓게 잡음
    roi = img_bgr[0:int(h*0.25), int(w*0.30):int(w*0.70)]
    if roi.size == 0: return 'none'
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_r1 = np.array([0, 120, 70]); upper_r1 = np.array([10, 255, 255])
    lower_r2 = np.array([170, 120, 70]); upper_r2 = np.array([180, 255, 255])
    mask_r = cv2.bitwise_or(cv2.inRange(hsv, lower_r1, upper_r1),
                            cv2.inRange(hsv, lower_r2, upper_r2))

    lower_y = np.array([15, 100, 100]); upper_y = np.array([35, 255, 255])
    mask_y = cv2.inRange(hsv, lower_y, upper_y)

    lower_g = np.array([40, 50, 50]); upper_g = np.array([90, 255, 255]) # 초록색 범위 조정
    mask_g = cv2.inRange(hsv, lower_g, upper_g)

    # 픽셀 수 대신 면적으로 판단 (Contour 사용이 더 정확하나, 일단 픽셀 수 유지)
    min_pixel_threshold = int(roi.shape[0] * roi.shape[1] * 0.005) # ROI 영역의 0.5%

    r_cnt = cv2.countNonZero(mask_r)
    y_cnt = cv2.countNonZero(mask_y)
    g_cnt = cv2.countNonZero(mask_g)

    if r_cnt > min_pixel_threshold: return 'red'
    elif y_cnt > min_pixel_threshold: return 'yellow'
    elif g_cnt > min_pixel_threshold: return 'green'
    else: return 'none'

def send_motor_command(angle_deg, speed_val):
    motor_msg_obj.angle = float(angle_deg)
    motor_msg_obj.speed = float(speed_val)
    if motor_control:
        motor_control.publish(motor_msg_obj)

# --- 차선 변경 경로 생성 및 제어 (BEV 좌표계 기준) ---
def generate_quadratic_path_bev(dx_bev, dy_bev, theta_rad=0, steps=PATH_POINTS_COUNT):
    """ 2차 곡선 경로 생성 (BEV 이미지 좌표계 기준)
        dx_bev, dy_bev: 목표점의 상대 BEV 픽셀 좌표 (시작점 (0,0) 기준)
        theta_rad: 시작 방향 (BEV y축 기준, 반시계 +, 현재는 0으로 가정)
    """
    A = np.array([
        [0**2, 0, 1],      # y(0) = 0
        [dx_bev**2, dx_bev, 1],  # y(dx_bev) = dy_bev
        [2*0, 1, 0]        # y'(0) = tan(theta_rad)
    ])
    B = np.array([0, dy_bev, math.tan(theta_rad)])
    
    try:
        a, b, c = np.linalg.solve(A, B)
    except np.linalg.LinAlgError: # 해가 없으면 직선 경로
        rospy.logwarn("Quadratic path solve failed, generating linear path.")
        xs_bev = np.linspace(0, dx_bev, steps)
        ys_bev = np.linspace(0, dy_bev, steps)
        return list(zip(xs_bev, ys_bev))
    
    xs_bev = np.linspace(0, dx_bev, steps)
    ys_bev = a * xs_bev**2 + b * xs_bev + c
    return list(zip(xs_bev, ys_bev))

def generate_lane_change_path_bev(current_lane_num, target_lane_num, current_x_bev, left_lane_x_bev, right_lane_x_bev):
    if current_lane_num == 0 or target_lane_num == 0: return [], 0
    
    # 목표 x 위치 계산 (BEV 이미지 기준)
    # 차선 폭을 대략 150픽셀로 가정 (BEV_IMAGE_WIDTH / 2 보다 약간 작게)
    lane_width_pixels_bev = 150 # 이 값은 튜닝 필요
    
    # dx_bev: 현재 x에서 목표 x까지의 변화량 (BEV 픽셀)
    # dy_bev: 전방 주시 거리 (BEV 픽셀)
    dx_bev = 0
    direction = 0

    if current_lane_num == 1 and target_lane_num == 2: # 1차선 -> 2차선 (오른쪽으로 이동)
        # 목표 x는 현재 오른쪽 차선 너머 (또는 현재 위치 + 차선폭)
        # target_x_bev = right_lane_x_bev + lane_width_pixels_bev / 2 # 너무 멀 수 있음
        dx_bev = lane_width_pixels_bev * 0.6 # 차선폭의 60% 정도 이동 (튜닝)
        direction = 1 # 우회전
    elif current_lane_num == 2 and target_lane_num == 1: # 2차선 -> 1차선 (왼쪽으로 이동)
        # target_x_bev = left_lane_x_bev - lane_width_pixels_bev / 2
        dx_bev = -lane_width_pixels_bev * 0.6
        direction = -1 # 좌회전
    else:
        return [], 0
        
    dy_bev = LANE_CHANGE_LOOKAHEAD_DIST # 전방 주시 거리 (BEV 픽셀)
    
    # 경로 생성 (시작점을 (0,0)으로 하는 상대 경로)
    relative_path_bev = generate_quadratic_path_bev(dx_bev, dy_bev, theta_rad=0, steps=PATH_POINTS_COUNT)
    
    # 실제 BEV 이미지 좌표로 변환 (시작점을 current_x_bev, BEV_BASE_Y_BOTTOM 으로)
    # y좌표는 이미지 하단에서 위로 올라가므로 dy_bev를 빼줌
    absolute_path_bev = [(current_x_bev + rel_x, BEV_BASE_Y_BOTTOM - rel_y) for rel_x, rel_y in relative_path_bev]
    
    if absolute_path_bev:
        rospy.loginfo(f"Lane change path (BEV): Start {absolute_path_bev[0]}, End {absolute_path_bev[-1]}, Dir: {direction}")
    return absolute_path_bev, direction


def lane_change_control_bev(path_bev, current_x_bev, current_y_bev, current_angle_rad, direction_val):
    """ 차선 변경 제어 (BEV 좌표계 기준 경로 추종) """
    if not path_bev: return 0.0
    
    # 현재 위치에서 가장 가까운 경로 포인트 찾기 (BEV)
    min_dist_sq = float('inf')
    closest_idx = 0
    for i, (px_bev, py_bev) in enumerate(path_bev):
        dist_sq = (px_bev - current_x_bev)**2 + (py_bev - current_y_bev)**2
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_idx = i
    
    # 전방 주시 포인트 (BEV)
    # look_ahead_idx = min(closest_idx + 5, len(path_bev) - 1) # 경로 포인트 개수에 따라 조절
    look_ahead_idx = min(closest_idx + int(PATH_POINTS_COUNT * 0.3), len(path_bev) - 1) # 경로의 30% 앞
    target_x_bev, target_y_bev = path_bev[look_ahead_idx]
    
    # 조향각 계산 (Pure Pursuit과 유사)
    # BEV 좌표를 차량 로컬 좌표로 변환해야 하나, 여기서는 간략화
    # 차량 현재 방향(current_angle_rad)은 BEV y축 기준이라고 가정 (0이면 위쪽)
    # alpha: 현재 차량 방향에서 목표점까지의 각도차
    # BEV y축은 아래로 갈수록 값이 커짐. 차량 전방은 BEV y축 위쪽(-y 방향)
    # atan2(delta_y, delta_x)
    # target_y_bev는 current_y_bev (BEV_BASE_Y_BOTTOM) 보다 작아야 전방.
    alpha_bev = math.atan2( (current_y_bev - target_y_bev), (target_x_bev - current_x_bev) ) - current_angle_rad
                                                                                        # current_angle_rad는 현재 0으로 가정
    
    # lookahead_dist_bev = math.sqrt((target_x_bev - current_x_bev)**2 + (target_y_bev - current_y_bev)**2)
    # steering_rad = math.atan2(2.0 * WHEELBASE_PIXELS * math.sin(alpha_bev), lookahead_dist_bev) # WHEELBASE도 픽셀 단위로?
    
    # 단순 비례 제어 사용 가능성 (차선 변경은 저속에서 부드럽게)
    # target_x_bev 와 current_x_bev의 차이를 이용
    error_x_bev = target_x_bev - current_x_bev
    
    # Kp 값은 튜닝 필요
    # 방향(direction_val)을 고려하여 조향각 부호 결정
    # steering_deg = direction_val * abs(error_x_bev) * 0.1 # 예시 Kp
    
    # 기존 코드의 로직을 최대한 활용 (각도 기반)
    # look_ahead_distance는 픽셀 단위. 실제 거리로 변환 필요하나, 일단 비례적으로.
    # L 값도 픽셀 단위로 변환하거나, 각도만 사용.
    # 여기서는 alpha_bev를 직접 조향각에 반영하는 형태로 단순화.
    # (차선 변경 시에는 차량의 현재 yaw가 크게 변하지 않는다고 가정)
    steering_deg = math.degrees(alpha_bev) * 0.5 # Kp 값 0.5 (튜닝)
    steering_deg *= direction_val # 방향 적용

    steering_deg = max(min(steering_deg, 30.0), -30.0) # 차선 변경 시 조향각 제한 (부드럽게)
    return steering_deg

# --- Kanayama 제어 (차선 주행 모드) ---
def kanayama_lane_control():
    global lane_info_data, FIX_SPEED_NORMAL, is_lane_changing_now, lane_change_generated_path, lane_change_direction_val
    
    if lane_info_data is None:
        rospy.logwarn_throttle(1.0, "Kanayama: No lane_info_data received.")
        return 0.0, FIX_SPEED_NORMAL

    # 차선 변경 중인 경우
    if is_lane_changing_now and lane_change_generated_path:
        current_time_sec = time.time()
        if current_time_sec - lane_change_start_timestamp > LANE_CHANGE_DURATION_SEC:
            is_lane_changing_now = False
            lane_change_generated_path = []
            lane_change_direction_val = 0
            rospy.loginfo("Lane change completed.")
        else:
            # 현재 차량 위치 (BEV 이미지 중앙 하단)
            current_x_bev = BEV_BASE_X_CENTER
            current_y_bev = BEV_BASE_Y_BOTTOM 
            current_angle_rad = 0 # 차량 전방이 BEV y축 위쪽이라고 가정

            steering_angle_deg = lane_change_control_bev(
                lane_change_generated_path,
                current_x_bev, current_y_bev,
                current_angle_rad, # 현재 차량의 yaw (BEV 기준, 여기서는 0으로 가정)
                lane_change_direction_val
            )
            # 차선 변경 시 속도 약간 증가 또는 유지
            control_speed = FIX_SPEED_NORMAL * 1.1 
            control_speed = min(control_speed, 45) # 최대 45
            return steering_angle_deg, control_speed

    # 일반 차선 주행 (카나야마)
    # lane_info_data의 left_x, right_x, left_slope, right_slope 사용
    # left_x: 차량중심(0)에서 왼쪽차선까지 x거리 (오른쪽이 +). 즉, (0 - 왼쪽차선x_로컬) = -왼쪽차선x_로컬
    # right_x: 차량중심(0)에서 오른쪽차선까지 x거리. 즉, (오른쪽차선x_로컬 - 0) = 오른쪽차선x_로컬
    # left_slope, right_slope: 라디안 단위
    
    left_x_local = -lane_info_data.left_x  # 차량 로컬 좌표계에서 왼쪽 차선의 x 위치 (음수 예상)
    right_x_local = lane_info_data.right_x # 차량 로컬 좌표계에서 오른쪽 차선의 x 위치 (양수 예상)
    left_slope_rad = lane_info_data.left_slope
    right_slope_rad = lane_info_data.right_slope
    
    # 카나야마 파라미터 (기존 값 유지 또는 튜닝)
    K_y = 0.85  # 측면 오차에 대한 게인
    K_phi = 3.0 # 헤딩 오차에 대한 게인
    v_ref = FIX_SPEED_NORMAL # 기준 속도

    lane_width_estimate = 3.5 # m 단위, 실제 차선 폭 추정치 (필요시 사용)

    lateral_error = 0.0
    heading_error_rad = 0.0

    # left_x_local은 음수, right_x_local은 양수여야 정상.
    # lane_info_data.left_x는 (130 - 실제x_bev), right_x는 (실제x_bev - 130)
    # left_x == 130 이면 왼쪽 차선 BEV x=0 (이미지 왼쪽 끝) -> 매우 왼쪽으로 치우침
    # right_x == -130 이면 오른쪽 차선 BEV x=0 (이미지 왼쪽 끝) -> 매우 왼쪽으로 치우침 (이상)
    # right_x == 130 이면 오른쪽 차선 BEV x=260 (이미지 오른쪽 끝) -> 매우 오른쪽으로 치우침
    # left_x == -130 이면 왼쪽 차선 BEV x=260 (이미지 오른쪽 끝) -> 매우 오른쪽으로 치우침 (이상)

    # lane_detection.py에서 left_x, right_x가 130을 기준으로 계산되었으므로,
    # 한쪽 차선이 안보일 때의 값은 130 또는 -130이 아님. (해당 차선 fit_x가 0 또는 259가 됨)
    # left_x = 130 - left_fitx[-1]
    # right_x = right_fitx[-1] - 130
    # left_fitx[-1]가 0에 가까우면 left_x는 130에 가까움 (왼쪽 차선이 이미지 왼쪽에 붙음)
    # right_fitx[-1]가 259에 가까우면 right_x는 130에 가까움 (오른쪽 차선이 이미지 오른쪽에 붙음)

    # 한쪽 차선만 보이는 경우 처리 (기존 로직 단순화)
    # 양쪽 차선이 모두 유효하게 보인다고 가정하고 중심 오차 계산
    # 차량 중심은 x=0. 목표는 (left_x_local + right_x_local) / 2.0
    # lateral_error = 0 - (left_x_local + right_x_local) / 2.0
    # = -(left_x_local + right_x_local) / 2.0
    
    # lane_info_data.left_x와 right_x를 직접 사용 (이 값들이 이미 중심에서의 거리로 계산됨)
    # left_x는 중심에서 왼쪽 차선까지의 거리 (양수), right_x는 중심에서 오른쪽 차선까지의 거리 (양수)
    # 로 메시지 형식을 가정한다면:
    # lateral_error = (lane_info_data.right_x - lane_info_data.left_x) / 2.0 
    # (오른쪽으로 치우치면 양수, 왼쪽으로 치우치면 음수) -> 목표는 0
    # 하지만 현재 메시지 left_x = (중심-왼쪽), right_x = (오른쪽-중심)
    # 따라서 lateral_error = -( (130-left_fitx) + (right_fitx-130) ) / 2 (픽셀단위)
    # = -(right_fitx - left_fitx) / 2 -> 이것은 차선 중앙의 x좌표. 목표는 0.
    # 즉, lateral_error = 0 - ( (BEV_BASE_X_CENTER - lane_info_data.left_x) + (BEV_BASE_X_CENTER + lane_info_data.right_x) ) / 2.0
    # 이것은 BEV 이미지 좌표 기준. 실제 m 단위로 변환 필요.
    # 여기서는 lane_info_data의 left_x, right_x가 이미 m 단위로 변환되었다고 가정하고 사용.
    # (실제로는 lane_detection.py에서 픽셀 단위로 계산 후 발행)
    # 이 부분은 laneinfo 메시지의 left_x, right_x 단위와 기준을 명확히 해야 함.
    # 일단 기존 코드의 계산 방식을 따름:
    # lateral_err = -(left_x / (left_x + right_x) - 0.5) * lane_width (left_x, right_x가 양수일 때)
    # 현재 left_x = (중심-왼쪽차선), right_x = (오른쪽차선-중심)
    # 두 값의 부호가 다를 수 있음.
    
    # 단순화: 차량 중심(x=0)에서 차선 중앙((left_x_local + right_x_local)/2)까지의 오차
    # left_x_local = -lane_info_data.left_x, right_x_local = lane_info_data.right_x
    # target_center_x_local = (right_x_local - lane_info_data.left_x) / 2.0
    # lateral_error = 0 - target_center_x_local = -(right_x_local - lane_info_data.left_x) / 2.0
    # (오른쪽으로 치우치면 right_x_local이 커지고 left_x_local(음수) 절대값이 작아져서 lateral_error 음수 -> 왼쪽으로 조향)
    
    # heading_error_rad = 0.5 * (left_slope_rad + right_slope_rad) # 평균 기울기
    # heading_error_rad *= -1 # 부호 반전 (기존 코드 참고)

    # 더 직관적인 방식: 목표 x = 0, 현재 x = (오른쪽차선x + 왼쪽차선x)/2
    # left_x_m, right_x_m: 차량 중심 기준 실제 거리 (m)
    # 이 값들이 lane_info_data에 있다고 가정. (실제로는 변환 필요)
    # lateral_error = 0 - (left_x_m + right_x_m) / 2.0
    # heading_error_rad = -math.atan( (left_slope_rad + right_slope_rad) / 2.0 ) # 평균 기울기의 각도

    # 임시로, lane_info_data의 left_x, right_x가 BEV 픽셀 중심에서의 거리라고 가정하고 사용
    # 실제로는 m 단위로 변환해야 함. 픽셀-미터 변환 비율 필요.
    # 여기서는 단위를 무시하고 값의 비율로만 제어한다고 가정 (튜닝으로 커버)
    pixel_to_meter_ratio_x = 0.02 # 예시: 1픽셀 = 0.02m (횡방향) - 튜닝필요
    
    # left_x, right_x는 중심에서 각 차선까지의 거리 (양수 값으로 가정)
    # 하지만 메시지는 (중심-왼쪽), (오른쪽-중심) 형태.
    # 차선 중앙의 x좌표 (차량 중심 기준) = ( (오른쪽차선x) + (왼쪽차선x) ) / 2
    # = ( (BEV_BASE_X_CENTER + lane_info_data.right_x) + (BEV_BASE_X_CENTER - lane_info_data.left_x) ) / 2 - BEV_BASE_X_CENTER
    # = (lane_info_data.right_x - lane_info_data.left_x) / 2.0  (픽셀 단위)
    center_offset_pixels = (lane_info_data.right_x - lane_info_data.left_x) / 2.0
    lateral_error = -center_offset_pixels * pixel_to_meter_ratio_x # m 단위, 목표는 0이므로 부호 반전

    heading_error_rad = -0.5 * (left_slope_rad + right_slope_rad) # 평균 기울기 (부호 확인 필요)

    # 각속도 w 계산
    # v = v_ref * math.cos(heading_error_rad) # 전방 속도 성분 (heading_error가 작다고 가정하면 v_ref와 거의 같음)
    v = v_ref # 단순화
    omega = v_ref * (K_y * lateral_error + K_phi * math.sin(heading_error_rad)) # 각속도 (rad/s)

    # 조향각 delta 계산 (rad)
    if abs(v) < 0.1: # 속도가 매우 작으면 조향 안함
        delta_rad = 0.0
    else:
        delta_rad = math.atan2(omega * WHEELBASE, v)
    
    steering_angle_deg = math.degrees(delta_rad)
    steering_angle_deg = max(min(steering_angle_deg, 50.0), -50.0) # 조향각 제한

    return steering_angle_deg, v_ref


# --- Pure Pursuit 제어 (라바콘 주행 모드) ---
def pure_pursuit_cone_control():
    global cone_lanes_data, WHEELBASE, CONE_DRIVE_MAX_SPEED
    
    if cone_lanes_data is None or \
       len(cone_lanes_data.raw_left_cones) < 2 or \
       len(cone_lanes_data.raw_right_cones) < 2:
        rospy.logwarn_throttle(1.0, "PurePursuit: Not enough cone data for control.")
        return None, None # 제어 불가, fallback 유도

    # raw_left_cones와 raw_right_cones를 numpy 배열로 변환 (x, y 좌표)
    # cone_lanes_data의 포인트들은 차량 로컬 좌표계 (x:좌우, y:전후)
    raw_left_cones_np = np.array([[p.x, p.y] for p in cone_lanes_data.raw_left_cones])
    raw_right_cones_np = np.array([[p.x, p.y] for p in cone_lanes_data.raw_right_cones])

    # (0,0)에서 가장 가까운 두 점 찾기
    # 왼쪽 체인
    left_distances_sq = np.sum(raw_left_cones_np**2, axis=1) # 원점에서의 거리 제곱
    sorted_left_indices = np.argsort(left_distances_sq)
    a1 = raw_left_cones_np[sorted_left_indices[0]]
    a2 = raw_left_cones_np[sorted_left_indices[1]]

    # 오른쪽 체인
    right_distances_sq = np.sum(raw_right_cones_np**2, axis=1)
    sorted_right_indices = np.argsort(right_distances_sq)
    b1 = raw_right_cones_np[sorted_right_indices[0]]
    b2 = raw_right_cones_np[sorted_right_indices[1]]

    # 중간점 c1, c2 계산
    c1 = (a1 + b1) / 2.0
    c2 = (a2 + b2) / 2.0

    # 목표점 계산 (c1과 c2 사이 3:1 내분점, c1에 가까움)
    # target_point = c1 * 0.75 + c2 * 0.25
    # 사용자의 "c1과 c2 사이 1대 5 지점"은 c1에서 5, c2에서 1만큼 떨어진 지점 (c1에 가까움)
    target_point = (1 * c1 + 5 * c2) / 6.0
    
    target_x_local, target_y_local = target_point[0], target_point[1]

    # Pure Pursuit 조향각 계산
    # 차량 현재 위치 (0,0), 현재 방향 +y축 (yaw=0)
    # alpha: 차량 전방(+y) 기준 목표점까지의 각도
    alpha_rad = math.atan2(target_x_local, target_y_local) # atan2(x,y) for angle from +y axis

    lookahead_distance = math.hypot(target_x_local, target_y_local)
    
    if lookahead_distance < 0.04: # 목표점이 매우 가까우면 직진
        steering_angle_rad = 0.0
    else:
        steering_angle_rad = math.atan2(2.0 * WHEELBASE * math.sin(alpha_rad), lookahead_distance)

    steering_angle_deg = math.degrees(steering_angle_rad)
    steering_angle_deg = max(min(steering_angle_deg, 50.0), -50.0) # 조향각 제한

    return steering_angle_deg, CONE_DRIVE_MAX_SPEED

# --- 현재 주행 모드 판단 함수 ---
def is_driving_in_cone_mode():
    global lane_info_data, cone_lanes_data
    if lane_info_data is not None and lane_info_data.cone_detected_flag: # 카메라가 라바콘 감지
        if cone_lanes_data is not None and \
            hasattr(cone_lanes_data, 'raw_left_cones') and \
            hasattr(cone_lanes_data, 'raw_right_cones') and \
           len(cone_lanes_data.raw_left_cones) >= 2 and \
           len(cone_lanes_data.raw_right_cones) >= 2: # 라이다도 양쪽 라바콘 체인 감지
        
            return True
    return False

# =============================================
# 메인 실행 함수
# =============================================
def start_driving():
    global motor_control, image_raw, lidar_ranges, traffic_light_go
    global lane_info_data, cone_lanes_data # 콜백에서 업데이트될 변수들
    global current_steering_angle, current_target_speed # 디버깅 및 현재 상태용

    rospy.init_node('Track_Driver_Node')
    
    # 구독자 설정
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_scan_callback, queue_size=1) # 콜백 함수 이름 변경됨
    rospy.Subscriber("/lane_info", laneinfo, lane_info_callback, queue_size=1)
    rospy.Subscriber("/cone_lanes", ConeLanes, cone_lanes_info_callback, queue_size=1) # 새로 추가
    
    # 발행자 설정
    motor_control = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
    
    rospy.loginfo("Waiting for first messages...")
    try:
        rospy.wait_for_message("/usb_cam/image_raw/", Image, timeout=15.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=5.0)
        rospy.wait_for_message("/lane_info", laneinfo, timeout=5.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout waiting for messages: {e}")
        return

    rospy.loginfo("All topics subscribed. Starting main loop.")
    
    rate = rospy.Rate(20) # 루프 실행 빈도 (Hz)

    while not rospy.is_shutdown():
        # 모든 데이터가 수신되었는지 확인
        if image_raw.size == 0 or lidar_ranges is None or lane_info_data is None: # cone_lanes_data는 모드에 따라 필요
            rospy.logwarn_throttle(1.0, "Waiting for all sensor data...")
            rate.sleep()
            continue

        # 1. 주행 모드 결정
        perform_cone_driving = is_driving_in_cone_mode()

        # 2. 모드에 따른 제어 함수 호출
        if perform_cone_driving:
            rospy.loginfo_throttle(0.5, "Mode: Cone Driving (Pure Pursuit)")
            angle_cmd, speed_cmd = pure_pursuit_cone_control()
            if angle_cmd is None: # Pure Pursuit 제어 실패 시 차선 주행으로 fallback
                rospy.logwarn("Pure Pursuit control failed, falling back to lane following.")
                perform_cone_driving = False # 플래그 변경
                # angle_cmd, speed_cmd = kanayama_lane_control() # 여기서 바로 호출 또는 다음 루프에서 처리
        
        # if not perform_cone_driving: # 차선 주행 모드 (위에서 fallback 되었거나 원래 차선모드)
        #     rospy.loginfo_throttle(0.5, "Mode: Lane Following (Kanayama)")
        #     angle_cmd, speed_cmd = kanayama_lane_control()
        # 이부분 수정: angle_cmd, speed_cmd가 None일 경우를 대비하여 초기화
        if not perform_cone_driving:
            rospy.loginfo_throttle(0.5, "Mode: Lane Following (Kanayama)")
            angle_cmd, speed_cmd = kanayama_lane_control()
        elif angle_cmd is None: # Pure Pursuit 실패 후 Fallback
             rospy.loginfo_throttle(0.5, "Mode: Lane Following (Kanayama) - Fallback")
             angle_cmd, speed_cmd = kanayama_lane_control()


        current_steering_angle = angle_cmd
        current_target_speed = speed_cmd

        # 3. 신호등 상태 확인 및 최종 속도 결정
        tl_state = detect_traffic_light(image_raw)
        if tl_state == 'green':
            traffic_light_go = True
        elif tl_state in ('red', 'yellow'):
            traffic_light_go = False
        # 'none'이면 이전 상태 유지

        final_speed_command = current_target_speed if traffic_light_go else 0.0
        
        rospy.loginfo_throttle(0.2, f"Ctrl: Angle={current_steering_angle:.2f}, Speed={final_speed_command:.2f} (TL: {tl_state}, Go: {traffic_light_go})")

        # 4. 모터 명령 전송
        send_motor_command(current_steering_angle, final_speed_command)
        
        # (선택) 디버깅용 이미지 표시
        # display_debug_image() # 별도 함수로 분리 권장

        rate.sleep()

if __name__ == '__main__':
    try:
        start_driving()
    except rospy.ROSInterruptException:
        rospy.loginfo("Track Driving node terminated.")
    finally:
        # 종료 시 모터 정지 명령
        if motor_control:
             send_motor_command(0.0, 0.0)
             rospy.loginfo("Sent stop command to motor.")