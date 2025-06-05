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

# =============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
# =============================================
image_raw = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
lidar_ranges = None  # 라이다 데이터를 담을 변수 (차량 감지용)
motor_control = None  # 모터노드
motor_msg_obj = XycarMotor()  # 모터 토픽 메시지

# --- 주행 파라미터 ---
FIX_SPEED_NORMAL = 37       # 일반 차선 주행 속도
CONE_DRIVE_MAX_SPEED = 9 
WHEELBASE = 0.5             # 차량 축거 (m), Pure Pursuit용
EXPECTED_LANE_WIDTH_APPROX_METERS = 3.5

# --- 제어 관련 변수 ---
current_steering_angle = 0  # 현재 조향각 (디버깅용)
current_target_speed = FIX_SPEED_NORMAL  # 현재 목표 속도

bridge = CvBridge()
lane_info_data = None       # 카메라 기반 차선 정보를 담을 변수
cone_lanes_data = None    # 라이다 기반 라바콘 차선 정보를 담을 변수

# --- 신호등 관련 ---
traffic_light_go = False  # 초록불 인식 후 출발 유지 플래그

# --- 차량 감지 관련 ---
vehicle_ahead_flag = False
detection_success_streak = 0
DETECTION_DISTANCE_LOW = 5.0 
DETECTION_DISTANCE_HIGH = 40.0 
DETECTION_COUNT = 4 
SECTOR_WIDTH = 7 
DETECTION_STREAK_THRESHOLD = 2

# --- 차선 변경 관련 ---
current_driving_lane = 0  
is_lane_changing_now = False  
lane_change_generated_path = []  
lane_change_direction_val = 0  
lane_change_start_timestamp = 0  
LANE_CHANGE_DURATION_SEC = 1.65 
PATH_POINTS_COUNT = 20 
LANE_CHANGE_LOOKAHEAD_DIST = 230 

# 이미지 관련 상수 (BEV 기준)
BEV_IMAGE_WIDTH = 260
BEV_IMAGE_HEIGHT = 260
BEV_BASE_X_CENTER = BEV_IMAGE_WIDTH / 2.0 
BEV_BASE_Y_BOTTOM = BEV_IMAGE_HEIGHT      

# --- 라바콘 주행 관련 변수 ---
cone_mode_activated_time = None 
last_cone_seen_time = None       # [추가] 마지막으로 콘 체인을 정상 인식한 시점 기록
ONE_LANE_DRIVING_TIMEOUT_SEC = 30.0 
MIN_CONES_FOR_LANE_SEGMENT = 2 
TARGET_Y_FOR_CONE_PURSUIT = 2.85 # 라바콘 Pure Pursuit 목표 y 값 (미터 단위)

# [추가] 최초 2초 동안 적용할 배율 및 유지 시간
CONE_PURSUIT_MULTIPLIER = 2          # 콘 모드 진입 직후 곱할 배율 (원하는 값으로 조정 가능)
CONE_PURSUIT_MULTIPLIER_DURATION_SEC = 5.0  # 배율을 유지할 기간 (초 단위)

# --- [새로 추가] 라바콘 제어 실패 시 이전 값 유지 관련 ---
last_successful_cone_steering = 0.0
last_successful_cone_speed = CONE_DRIVE_MAX_SPEED # 초기값은 기본 콘 주행 속도
cone_control_failure_count = 0
MAX_CONSECUTIVE_CONE_FAILURES = 8 # 연속 실패 허용 횟수 (이후 차선주행 전환 또는 정지 고려)

# --- [추가] Pure Pursuit 가속 제한 상수 ---
MAX_CONE_ACCELERATION = 0.5 # 한 루프(1/20초)당 최대 증가 속도 (m/s)

# =============================================
# 콜백함수
# =============================================
def usbcam_callback(data):
    global image_raw
    image_raw = bridge.imgmsg_to_cv2(data, "bgr8")

def lidar_scan_callback(data):
    global lidar_ranges, vehicle_ahead_flag, detection_success_streak
    lidar_ranges = data.ranges[0:360] 

    detected_this_scan = False
    consecutive_points = 0
    for i in range(-SECTOR_WIDTH, SECTOR_WIDTH + 1):
        dist = lidar_ranges[i % 360] 
        if DETECTION_DISTANCE_LOW < dist < DETECTION_DISTANCE_HIGH:
            consecutive_points += 1
            if consecutive_points >= DETECTION_COUNT:
                detected_this_scan = True
                break
        else:
            consecutive_points = 0

    if detected_this_scan:
        detection_success_streak = max(DETECTION_STREAK_THRESHOLD, detection_success_streak + 1)
    else:
        detection_success_streak = max(0, detection_success_streak -1)

    if detection_success_streak >= DETECTION_STREAK_THRESHOLD:
        vehicle_ahead_flag = True
    else:
        vehicle_ahead_flag = False


def lane_info_callback(data): 
    global lane_info_data, current_driving_lane, is_lane_changing_now
    global lane_change_generated_path, lane_change_direction_val, lane_change_start_timestamp
    
    lane_info_data = data
    current_time_sec = time.time()

    if data.lane_number in [1, 2]:
        current_driving_lane = data.lane_number

    if not is_driving_in_cone_mode_now() and not is_lane_changing_now and vehicle_ahead_flag:
        is_lane_changing_now = True
        lane_change_start_timestamp = current_time_sec
        target_lane = 2 if current_driving_lane == 1 else 1
        
        start_x_bev = BEV_BASE_X_CENTER 
        if lane_info_data.left_x != (BEV_BASE_X_CENTER) and lane_info_data.right_x != (-BEV_BASE_X_CENTER):
            start_x_bev = BEV_BASE_X_CENTER + (lane_info_data.right_x - lane_info_data.left_x) / 2.0
        elif lane_info_data.left_x != (BEV_BASE_X_CENTER) : 
            start_x_bev = (BEV_BASE_X_CENTER - lane_info_data.left_x) + 75 
        elif lane_info_data.right_x != (-BEV_BASE_X_CENTER) : 
            start_x_bev = (BEV_BASE_X_CENTER + lane_info_data.right_x) - 75
        else: 
            start_x_bev = BEV_BASE_X_CENTER

        lane_change_generated_path, lane_change_direction_val = generate_lane_change_path_bev(
            current_driving_lane, target_lane, start_x_bev, 
            BEV_BASE_X_CENTER - lane_info_data.left_x, 
            BEV_BASE_X_CENTER + lane_info_data.right_x  
        )
        rospy.loginfo(f"Lane change initiated: {current_driving_lane} -> {target_lane}")

def cone_lanes_info_callback(data): 
    global cone_lanes_data
    cone_lanes_data = data

# =============================================
# 유틸리티 및 제어 함수
# =============================================
def detect_traffic_light(img_bgr): 
    if img_bgr is None or img_bgr.size == 0: return 'none'
    h, w = img_bgr.shape[:2]
    roi = img_bgr[0:int(h*0.25), int(w*0.30):int(w*0.70)]
    if roi.size == 0: return 'none'
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_r1 = np.array([0, 120, 70]); upper_r1 = np.array([10, 255, 255])
    lower_r2 = np.array([170, 120, 70]); upper_r2 = np.array([180, 255, 255])
    mask_r = cv2.bitwise_or(cv2.inRange(hsv, lower_r1, upper_r1), cv2.inRange(hsv, lower_r2, upper_r2))
    lower_y = np.array([15, 100, 100]); upper_y = np.array([35, 255, 255])
    mask_y = cv2.inRange(hsv, lower_y, upper_y)
    lower_g = np.array([40, 50, 50]); upper_g = np.array([90, 255, 255]) 
    mask_g = cv2.inRange(hsv, lower_g, upper_g)
    min_pixel_threshold = int(roi.shape[0] * roi.shape[1] * 0.005) 
    r_cnt = cv2.countNonZero(mask_r); y_cnt = cv2.countNonZero(mask_y); g_cnt = cv2.countNonZero(mask_g)
    if r_cnt > min_pixel_threshold: return 'red'
    elif y_cnt > min_pixel_threshold: return 'yellow'
    elif g_cnt > min_pixel_threshold: return 'green'
    else: return 'none'

def send_motor_command(angle_deg, speed_val):
    motor_msg_obj.angle = float(angle_deg)
    motor_msg_obj.speed = float(speed_val)
    if motor_control:
        motor_control.publish(motor_msg_obj)

def generate_quadratic_path_bev(dx_bev, dy_bev, theta_rad=0, steps=PATH_POINTS_COUNT):
    A = np.array([[0**2, 0, 1], [dx_bev**2, dx_bev, 1], [2*0, 1, 0]])
    B = np.array([0, dy_bev, math.tan(theta_rad)])
    try: a, b, c = np.linalg.solve(A, B)
    except np.linalg.LinAlgError: 
        rospy.logwarn("Quadratic path solve failed, generating linear path.")
        xs_bev = np.linspace(0, dx_bev, steps); ys_bev = np.linspace(0, dy_bev, steps)
        return list(zip(xs_bev, ys_bev))
    xs_bev = np.linspace(0, dx_bev, steps)
    ys_bev = a * xs_bev**2 + b * xs_bev + c
    return list(zip(xs_bev, ys_bev))

def generate_lane_change_path_bev(current_lane_num, target_lane_num, current_x_bev, left_lane_x_bev, right_lane_x_bev):
    if current_lane_num == 0 or target_lane_num == 0: return [], 0
    lane_width_pixels_bev = 150; dx_bev = 0; direction = 0
    if current_lane_num == 1 and target_lane_num == 2: dx_bev = lane_width_pixels_bev * 0.6; direction = 1 
    elif current_lane_num == 2 and target_lane_num == 1: dx_bev = -lane_width_pixels_bev * 0.6; direction = -1 
    else: return [], 0
    dy_bev = LANE_CHANGE_LOOKAHEAD_DIST 
    relative_path_bev = generate_quadratic_path_bev(dx_bev, dy_bev, theta_rad=0, steps=PATH_POINTS_COUNT)
    absolute_path_bev = [(current_x_bev + rel_x, BEV_BASE_Y_BOTTOM - rel_y) for rel_x, rel_y in relative_path_bev]
    if absolute_path_bev: rospy.loginfo(f"Lane change path (BEV): Start {absolute_path_bev[0]}, End {absolute_path_bev[-1]}, Dir: {direction}")
    return absolute_path_bev, direction

def lane_change_control_bev(path_bev, current_x_bev, current_y_bev, current_angle_rad, direction_val):
    if not path_bev: return 0.0
    min_dist_sq = float('inf'); closest_idx = 0
    for i, (px_bev, py_bev) in enumerate(path_bev):
        dist_sq = (px_bev - current_x_bev)**2 + (py_bev - current_y_bev)**2
        if dist_sq < min_dist_sq: min_dist_sq = dist_sq; closest_idx = i
    look_ahead_idx = min(closest_idx + int(PATH_POINTS_COUNT * 0.3), len(path_bev) - 1) 
    target_x_bev, target_y_bev = path_bev[look_ahead_idx]
    alpha_bev = math.atan2( (current_y_bev - target_y_bev), (target_x_bev - current_x_bev) ) - current_angle_rad
    steering_deg = math.degrees(alpha_bev) * 0.5 * direction_val
    steering_deg = max(min(steering_deg, 30.0), -30.0) 
    return steering_deg

def kanayama_lane_control():
    global lane_info_data, FIX_SPEED_NORMAL, is_lane_changing_now, lane_change_generated_path, lane_change_direction_val
    if lane_info_data is None:
        rospy.logwarn_throttle(1.0, "Kanayama: No lane_info_data received.")
        return 0.0, FIX_SPEED_NORMAL

    if is_lane_changing_now and lane_change_generated_path:
        current_time_sec = time.time()
        if current_time_sec - lane_change_start_timestamp > LANE_CHANGE_DURATION_SEC:
            is_lane_changing_now = False; lane_change_generated_path = []; lane_change_direction_val = 0
            rospy.loginfo("Lane change completed.")
        else:
            steering_angle_deg = lane_change_control_bev(lane_change_generated_path, BEV_BASE_X_CENTER, BEV_BASE_Y_BOTTOM, 0, lane_change_direction_val)
            control_speed = min(FIX_SPEED_NORMAL * 1.1, 45) 
            return steering_angle_deg, control_speed

    K_y = 0.85; K_phi = 3.0; v_ref = FIX_SPEED_NORMAL; pixel_to_meter_ratio_x = 0.02 
    center_offset_pixels = (lane_info_data.right_x - lane_info_data.left_x) / 2.0
    lateral_error = -center_offset_pixels * pixel_to_meter_ratio_x 
    heading_error_rad = -0.5 * (lane_info_data.left_slope + lane_info_data.right_slope) 
    omega = v_ref * (K_y * lateral_error + K_phi * math.sin(heading_error_rad)) 
    delta_rad = math.atan2(omega * WHEELBASE, v_ref) if abs(v_ref) > 0.1 else 0.0
    steering_angle_deg = max(min(math.degrees(delta_rad), 50.0), -50.0) 
    return steering_angle_deg, v_ref

# --- [수정됨] Pure Pursuit 제어 (라바콘 주행 모드) - 강건성 향상 ---
def pure_pursuit_cone_control():
    global cone_lanes_data, WHEELBASE, CONE_DRIVE_MAX_SPEED, TARGET_Y_FOR_CONE_PURSUIT
    global cone_mode_activated_time, CONE_PURSUIT_MULTIPLIER, CONE_PURSUIT_MULTIPLIER_DURATION_SEC

    if cone_lanes_data is None:
        rospy.logwarn_throttle(1.0, "PurePursuit: No cone_lanes_data received.")
        return None, None

    center_path_np = np.array([[p.x, p.y] for p in cone_lanes_data.center_path]) if cone_lanes_data.center_path else np.array([])

    if center_path_np.shape[0] < 1: # 경로 점이 하나도 없으면 제어 불가
        rospy.logwarn_throttle(1.0, "PurePursuit: Center_path is empty.")
        return None, None
    
    # (1) 현재 콘 모드 활성화 이후 경과 시간 계산
    effective_target_y = TARGET_Y_FOR_CONE_PURSUIT
    if cone_mode_activated_time is not None:
        elapsed = time.time() - cone_mode_activated_time
        if elapsed < CONE_PURSUIT_MULTIPLIER_DURATION_SEC:
            # 활성화 후 n초 이내라면 배율 적용
            effective_target_y = TARGET_Y_FOR_CONE_PURSUIT * CONE_PURSUIT_MULTIPLIER
            rospy.loginfo_throttle(1.0,
                f"PurePursuit: Cone mode active {elapsed:.2f}s (<= {CONE_PURSUIT_MULTIPLIER_DURATION_SEC}s), "
                f"using TARGET_Y = {effective_target_y:.2f}")
        else:
            # 2초 경과 후에는 원래 값 사용
            effective_target_y = TARGET_Y_FOR_CONE_PURSUIT

    target_point = None

    # 2) effective_target_y 와의 교차점 찾기
    if center_path_np.shape[0] >= 2: # 최소 2개의 점이 있어야 선분으로 교차점 계산 가능
        for i in range(len(center_path_np) - 1):
            p1 = center_path_np[i]; p2 = center_path_np[i+1]
            y1, y2 = p1[1], p2[1]; x1, x2 = p1[0], p2[0]

            if (y1 <= effective_target_y <= y2) or (y2 <= effective_target_y <= y1):
                if abs(y2 - y1) < 1e-6: 
                    if abs(y1 - effective_target_y) < 1e-6:
                        target_x_local = (x1 + x2) / 2.0
                        target_point = np.array([target_x_local, effective_target_y])
                        break 
                else:
                    target_x_local = x1 + (x2 - x1) * (effective_target_y - y1) / (y2 - y1)
                    target_point = np.array([target_x_local, effective_target_y])
                    break 
    
    # 3) 교차점 없을 때 방어 로직 (기존 그대로)
    if target_point is None:
        rospy.logwarn_throttle(1.0, f"PurePursuit: No direct intersection with y={effective_target_y:.2f}. Attempting fallback.")
        if center_path_np.size > 0:
            idx_furthest_y = np.argmax(center_path_np[:, 1])
            furthest_point_on_path = center_path_np[idx_furthest_y]

            MIN_Y_RATIO_FOR_FALLBACK = 0.5
            MIN_Y_ABS_FOR_FALLBACK = 1.0 # 최소 1미터는 앞에 있어야 함
            if furthest_point_on_path[1] > max(effective_target_y * MIN_Y_RATIO_FOR_FALLBACK, MIN_Y_ABS_FOR_FALLBACK) :
                target_point = furthest_point_on_path
                rospy.loginfo_throttle(1.0, f"PurePursuit Fallback: Using furthest point on path: ({target_point[0]:.2f}, {target_point[1]:.2f})")
            else:
                rospy.logwarn_throttle(1.0, f"PurePursuit Fallback: Furthest point ({furthest_point_on_path[0]:.2f}, {furthest_point_on_path[1]:.2f}) is too close or behind. Control failed.")
                return None, None
        else: # center_path_np가 비어있는 경우는 이미 위에서 처리됨
            return None, None

    if target_point is None: # 모든 방어 로직 후에도 목표점을 못 찾으면 실패
        rospy.logerr_throttle(1.0, "PurePursuit: CRITICAL - Failed to determine target_point even after fallbacks.")
        return None, None

    rospy.loginfo_throttle(1.0, f"PurePursuit: Target point: ({target_point[0]:.2f}, {target_point[1]:.2f})")
    target_x_local, target_y_local = target_point[0], target_point[1]

    if abs(target_y_local) < 0.1: # 목표 y가 너무 작으면 (0.01에서 증가) 직진 또는 이전 값 사용 고려
        rospy.logwarn_throttle(1.0, "PurePursuit: Target y is too small, potential instability.")
        # 이 경우, alpha_rad를 0으로 하거나, 이전 제어값을 사용하는 로직을 메인 루프에서 처리
        # 여기서는 일단 alpha=0으로 계산 진행
        alpha_rad = 0.0
    else:
        alpha_rad = math.atan2(target_x_local, target_y_local) 

    lookahead_distance = math.hypot(target_x_local, target_y_local)
    
    if lookahead_distance < 0.1: # 목표점이 매우 가까우면 (0.01에서 증가)
        steering_angle_rad = 0.0 # 직진
    else:
        steering_angle_rad = math.atan2(2.0 * WHEELBASE * math.sin(alpha_rad), lookahead_distance)

    steering_angle_deg = 6* math.degrees(steering_angle_rad) 
    steering_angle_deg = max(min(steering_angle_deg, 50.0), -50.0) 
    
    # --- ◀ 여기서부터 감속 로직을 curvature 기반에서 steering angle 기반으로 변경 ▶ ---
    max_steering_deg = 50.0                   # Pure Pursuit에서 clamp한 최대조향각과 동일하게 설정
    reduction_gain = 0.9                      # steering_angle_deg == max_steering_deg일 때, 속도를 70% 줄이겠다는 의미
    min_speed_factor = 0.9                    # 너무 급감속되지 않도록 최소 배율 설정

    raw_reduction = (abs(steering_angle_deg) / max_steering_deg) * reduction_gain
    speed_reduction_factor = max(min_speed_factor, 1.0 - raw_reduction)

    final_speed = CONE_DRIVE_MAX_SPEED * speed_reduction_factor
    # --- ▲ 감속 로직 끝 ▲ ---
    
    # --- [추가] 가속 제한 로직 ---
    global last_successful_cone_speed, MAX_CONE_ACCELERATION
    if last_successful_cone_speed is not None:
        # 이전 속도보다 final_speed가 많이 증가하지 않도록 제한
        max_allowed_speed = last_successful_cone_speed + MAX_CONE_ACCELERATION
        final_speed = min(final_speed, max_allowed_speed)
    # --- ▲ 가속 제한 끝 ▲ ---
    
    return steering_angle_deg, final_speed


def is_driving_in_cone_mode_now():
    global lane_info_data, cone_lanes_data, cone_mode_activated_time
    global last_cone_seen_time
    camera_sees_cones = False
    if lane_info_data is not None and hasattr(lane_info_data, 'cone_detected_flag') and lane_info_data.cone_detected_flag:
        camera_sees_cones = True

    lidar_sees_enough_cones_for_control = False
    if cone_lanes_data is not None:
        raw_left_cones_np = np.array([[p.x, p.y] for p in cone_lanes_data.raw_left_cones]) if cone_lanes_data.raw_left_cones else np.array([])
        raw_right_cones_np = np.array([[p.x, p.y] for p in cone_lanes_data.raw_right_cones]) if cone_lanes_data.raw_right_cones else np.array([])
        center_path_np = np.array([[p.x, p.y] for p in cone_lanes_data.center_path]) if cone_lanes_data.center_path else np.array([])
        left_ok = raw_left_cones_np.shape[0] >= MIN_CONES_FOR_LANE_SEGMENT
        right_ok = raw_right_cones_np.shape[0] >= MIN_CONES_FOR_LANE_SEGMENT
        center_ok = center_path_np.shape[0] >= 1 # pure_pursuit_cone_control에서 최소 1개 필요 (방어 로직 고려)
        time_in_cone_mode = float('inf')
        if cone_mode_activated_time is not None: time_in_cone_mode = time.time() - cone_mode_activated_time
        if (left_ok and right_ok and center_ok) or \
           ((left_ok or right_ok) and center_ok and time_in_cone_mode <= ONE_LANE_DRIVING_TIMEOUT_SEC) or \
           (center_ok and time_in_cone_mode <= ONE_LANE_DRIVING_TIMEOUT_SEC): # 중앙 경로만 있어도 일정 시간 동안은 콘 모드 시도
            lidar_sees_enough_cones_for_control = True

    현재_인식 = camera_sees_cones and lidar_sees_enough_cones_for_control
    now = time.time()

    if 현재_인식:
        # 콘 체인을 정상 인식한 시점 업데이트
        last_cone_seen_time = now
        if cone_mode_activated_time is None:
            cone_mode_activated_time = now
            rospy.loginfo(f"Cone driving mode ACTIVATED at {cone_mode_activated_time:.2f}")
        return True
    else:
        # 콘 체인이 끊겼더라도 마지막 인식 후 3초 이내면 여전히 콘 모드 유지
        if last_cone_seen_time is not None and (now - last_cone_seen_time) < 6.0:          ###라바콘 주행 pure pursuit  유지
            rospy.loginfo_throttle(1.0,
                f"Cone driving mode 유지: 마지막 인식 {now - last_cone_seen_time:.2f}s 전 (3s 이하)")
            return True
        # 3초가 지나면 콘 모드 해제
        if cone_mode_activated_time is not None:
            rospy.loginfo(f"Cone driving mode DEACTIVATED (Prev activation: {cone_mode_activated_time:.2f})")
        cone_mode_activated_time = None
        last_cone_seen_time = None
        return False

# =============================================
# 메인 실행 함수
# =============================================
def start_driving():
    global motor_control, image_raw, lidar_ranges, traffic_light_go
    global lane_info_data, cone_lanes_data 
    global current_steering_angle, current_target_speed
    global last_successful_cone_steering, last_successful_cone_speed, cone_control_failure_count # 추가된 전역 변수

    rospy.init_node('Track_Driver_Node')
    
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_scan_callback, queue_size=1) 
    rospy.Subscriber("/lane_info", laneinfo, lane_info_callback, queue_size=1)
    rospy.Subscriber("/cone_lanes", ConeLanes, cone_lanes_info_callback, queue_size=1) 
    
    motor_control = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
    
    rospy.loginfo("Waiting for first messages...")
    try:
        rospy.wait_for_message("/usb_cam/image_raw/", Image, timeout=15.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=5.0) # 차량 감지에 필요
        # lane_info와 cone_lanes는 상황에 따라 없을 수 있으므로, 타임아웃 시 경고만 하고 진행
        if not rospy.wait_for_message("/lane_info", laneinfo, timeout=2.0):
            rospy.logwarn("Timeout waiting for /lane_info. Proceeding, but lane following might not work initially.")
        # if not rospy.wait_for_message("/cone_lanes", ConeLanes, timeout=2.0):
        #     rospy.logwarn("Timeout waiting for /cone_lanes. Proceeding, but cone driving might not work initially.")
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout waiting for essential messages (Image/Scan): {e}")
        return # 이미지나 스캔 없이는 주행 불가

    rospy.loginfo("Essential topics subscribed or timeout handled. Starting main loop.")
    rate = rospy.Rate(20) 

    while not rospy.is_shutdown():
        if image_raw.size == 0 or lidar_ranges is None : 
            rospy.logwarn_throttle(1.0, "Waiting for image_raw or lidar_ranges data...")
            rate.sleep()
            continue

        perform_cone_driving = is_driving_in_cone_mode_now()
        angle_cmd, speed_cmd = None, None

        if perform_cone_driving:
            rospy.loginfo_throttle(0.5, "Attempting Mode: Cone Driving")
            if cone_lanes_data is not None:
                temp_angle, temp_speed = pure_pursuit_cone_control()
                if temp_angle is not None and temp_speed is not None:
                    angle_cmd = temp_angle
                    speed_cmd = temp_speed
                    last_successful_cone_steering = angle_cmd
                    last_successful_cone_speed = speed_cmd
                    cone_control_failure_count = 0 # 성공 시 카운터 리셋
                    rospy.loginfo_throttle(0.5, "Succeeded: Cone Driving Mode")
                else: # Pure Pursuit 연산 실패
                    cone_control_failure_count += 1
                    rospy.logwarn(f"Pure Pursuit (cone) control failed. Count: {cone_control_failure_count}/{MAX_CONSECUTIVE_CONE_FAILURES}")
                    if cone_control_failure_count <= MAX_CONSECUTIVE_CONE_FAILURES:
                        angle_cmd = last_successful_cone_steering
                        speed_cmd = last_successful_cone_speed
                        rospy.logwarn(f"Using last successful cone control values: Angle={angle_cmd:.2f}, Speed={speed_cmd:.2f}")
                    else: # 연속 실패 횟수 초과
                        rospy.logerr(f"Persistent cone control failure. Falling back from cone mode.")
                        perform_cone_driving = False # 차선 주행 모드로 강제 전환
                        cone_control_failure_count = 0 # 다음 콘 모드 진입을 위해 리셋
            else:
                rospy.logwarn_throttle(1.0, "Cone mode indicated, but no cone_lanes_data. Attempting fallback or using old values.")
                cone_control_failure_count +=1 # 데이터 없는 것도 실패로 간주
                if cone_control_failure_count <= MAX_CONSECUTIVE_CONE_FAILURES:
                    angle_cmd = last_successful_cone_steering
                    speed_cmd = last_successful_cone_speed
                else:
                    perform_cone_driving = False
                    cone_control_failure_count = 0
        
        if not perform_cone_driving: # 차선 주행 모드 (위에서 fallback 되었거나 원래 차선모드)
            rospy.loginfo_throttle(0.5, "Attempting Mode: Lane Following")
            cone_control_failure_count = 0 # 차선 주행 모드 진입 시 콘 실패 카운트 리셋
            if lane_info_data is not None:
                angle_cmd_lane, speed_cmd_lane = kanayama_lane_control()
                if angle_cmd is None: # 콘 주행에서 넘어왔고, 콘 주행이 실패한 경우
                    angle_cmd = angle_cmd_lane
                    speed_cmd = speed_cmd_lane
                rospy.loginfo_throttle(0.5, "Mode: Lane Following (or fallback)")
            else: 
                 rospy.logwarn_throttle(1.0, "Lane following mode, but no lane_info_data. Control may be erratic.")
                 if angle_cmd is None: # 콘 주행도 실패했고, 차선 정보도 없으면 정지
                    angle_cmd = 0.0
                    speed_cmd = 0.0


        if angle_cmd is None or speed_cmd is None: 
            rospy.logerr_throttle(1.0, "CRITICAL: No valid control command generated after all logic. Stopping vehicle.")
            current_steering_angle = 0.0
            current_target_speed = 0.0
        else:
            current_steering_angle = angle_cmd
            current_target_speed = speed_cmd

        tl_state = detect_traffic_light(image_raw)
        if tl_state == 'green': traffic_light_go = True
        elif tl_state in ('red', 'yellow'): traffic_light_go = False
        
        final_speed_command = current_target_speed
        
        # is_actually_in_cone_driving_state는 perform_cone_driving으로 대체 가능 (위에서 이미 로직 반영됨)
        if perform_cone_driving and angle_cmd is not None and cone_control_failure_count <= MAX_CONSECUTIVE_CONE_FAILURES : 
             rospy.loginfo_throttle(0.2, f"Ctrl (Cone): Angle={current_steering_angle:.2f}, Speed={final_speed_command:.2f} (TL ignored: {tl_state}, Failures: {cone_control_failure_count})")
        else: 
            if not traffic_light_go: final_speed_command = 0.0
            rospy.loginfo_throttle(0.2, f"Ctrl (Lane/Fallback): Angle={current_steering_angle:.2f}, Speed={final_speed_command:.2f} (TL: {tl_state}, Go: {traffic_light_go})")

        send_motor_command(current_steering_angle, final_speed_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        start_driving()
    except rospy.ROSInterruptException:
        rospy.loginfo("Track Driving node terminated.")
    except Exception as e:
        rospy.logerr(f"Unhandled exception in start_driving: {e}")
    finally:
        if motor_control: 
             rospy.loginfo("Sending stop command to motor before exiting.")
             send_motor_command(0.0, 0.0)
        rospy.loginfo("Track Driving node shutdown complete.")
