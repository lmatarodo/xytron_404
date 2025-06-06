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
from xycar_msgs.msg import ConeLanes       # ConeLanes 메시지 import 추가
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0])          # 카메라 이미지를 담을 변수
ranges = None                        # 라이다 데이터를 담을 변수
motor = None                         # 모터노드
motor_msg = XycarMotor()             # 모터 토픽 메시지
Fix_Speed = 37                       # 모터 속도 고정 상수값 (원래 10)
bridge = CvBridge()                  # OpenCV 함수를 사용하기 위한 브릿지
lane_data = None                     # 차선 정보를 담을 변수
light_go = False                     # 초록불 인식 후 출발 유지
L = 0.5

# 차선 감지 관련 변수
current_lane = 0                     # 현재 차선 번호 (0: 불확실, 1: 1차선, 2: 2차선)
is_lane_changing = False             # 차선 변경 중인지 여부
lane_change_path = []                # 차선 변경 경로
lane_change_direction = 0            # 차선 변경 방향 (1: 우회전, -1: 좌회전)
lane_change_start_time = 0           # 차선 변경 시작 시간
last_lane_change_time = 0            # 마지막 차선 변경 시간

# 속도 증가 관련 전역 변수
is_speed_boosted = False
speed_boost_start_time = 0
speed_boost_duration = 10.0          # 10초 동안 속도 증가 유지
has_speed_boosted = False            # 전체 구간에서 속도 증가를 한 번만 하기 위한 플래그

# 차선 감지 누락 대비 변수
last_left_lane_detected = True
last_right_lane_detected = True
left_lane_missing_time = 0
right_lane_missing_time = 0
LANE_MISSING_THRESHOLD = 2.0         # 차선 미감지 시 차선 변경 시도 대기 시간 (초)

# 차선 변경 관련 상수
LANE_CHANGE_DURATION = 2.0           # 차선 변경 소요 시간 (초)
PATH_POINTS = 20                     # 경로 생성 시 샘플링할 포인트 수
LANE_CHANGE_DISTANCE = 300           # 차선 변경 시 전방 주시 거리

# 라바콘 제어 관련 변수
cone_path_data = None
lateral_error_from_topic = 0.0
target_point_detected = False
target_point_from_topic = None
target_heading_from_topic = 0.0
target_cone_speed = 10              # 라바콘 구간 목표 속도

# 안정화(Stabilizer) 관련 변수
is_stabilizing = False
stabilizer_start_time = 0.0
STABILIZER_DURATION = 5              # 안정화 지속 시간 (초)
STABILIZER_SPEED = 10                # 안정화 모드 시 고정 속도

# 차량 감지 관련 변수
vehicle_ahead = False
detection_success_streak = 0
DETECTION_DISTANCE_LOW = 5.0
DETECTION_DISTANCE_HIGH = 40.0
DETECTION_COUNT = 8                  # 연속으로 이만큼 포인트가 잡히면 차량으로 판정
SECTOR_WIDTH = 7                     # 앞 0번 인덱스 기준으로 ±7 인덱스 조사
DETECTION_STREAK_THRESHOLD = 2       # 스캔이 연속으로 성공해야 할 횟수

# 이미지 관련 상수
IMAGE_WIDTH = 260
IMAGE_HEIGHT = 260
BASE_X = 130
BASE_Y = 260

# 라이다 시각화를 위한 변수
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')

sim_start_time = None  # 시뮬레이터 시작 시각 저장용

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
    global ranges, vehicle_ahead, detection_success_streak
    ranges = data.ranges[0:360]

    # 1) 이번 스캔에서 공간적으로 연속된 DETECTION_COUNT 점이 감지됐는지 판정
    detected_this_scan = False
    consec = 0
    for offset in range(-SECTOR_WIDTH, SECTOR_WIDTH + 1):
        d = ranges[offset % 360]
        if DETECTION_DISTANCE_LOW < d < DETECTION_DISTANCE_HIGH:
            consec += 1
            if consec >= DETECTION_COUNT:
                detected_this_scan = True
                break
        else:
            consec = 0

    # 2) 연속 성공 카운터 갱신
    if detected_this_scan:
        detection_success_streak += 1
    else:
        detection_success_streak = 0

    # 3) DETECTION_STREAK_THRESHOLD번 연속 스캔 성공 시에만 플래그 ON
    if detection_success_streak >= DETECTION_STREAK_THRESHOLD:
        vehicle_ahead = True
    else:
        vehicle_ahead = False

#=============================================
# 콜백함수 - 차선 토픽 처리
#=============================================
def lane_callback(data):
    global lane_data, current_lane, is_lane_changing, lane_change_path
    global lane_change_direction, lane_change_start_time, last_lane_change_time
    global has_speed_boosted, is_speed_boosted
    lane_data = data

    current_time = time.time()

    # 실측 lane 번호 업데이트
    if data.lane_number in [1, 2]:
        current_lane = data.lane_number

    # 차선 변경 조건: 차량 앞에 있으면 차선 변경 시작 (한 번만 속도 증가 로직도 트리거)
    if not is_lane_changing and vehicle_ahead and not is_speed_boosted:
        is_lane_changing = True
        lane_change_start_time = current_time
        last_lane_change_time = current_time

        target_lane = 2 if current_lane == 1 else 1

        # 현재 차선 중심점 계산
        if data.left_x != 130 and data.right_x != -130:
            start_x = (data.left_x + data.right_x) / 2
        elif data.left_x != 130:
            start_x = data.left_x + 75
        elif data.right_x != -130:
            start_x = data.right_x - 75
        else:
            start_x = BASE_X  # 기본값으로 이미지 중앙 사용

        lane_change_path, lane_change_direction = generate_lane_change_path(
            current_lane, target_lane,
            start_x, data.left_x, data.right_x
        )
        rospy.loginfo(f"차선 변경 시작: {current_lane} → {target_lane}")

#=============================================
# 콜백함수 - 라바콘 경로 토픽 처리
#=============================================
def conelane_callback(data):
    global cone_path_data, lateral_error_from_topic, target_point_detected
    global target_point_from_topic, target_heading_from_topic

    # 중앙 경로 저장
    if data.center_path:
        cone_path_data = data.center_path
    else:
        cone_path_data = None

    # 토픽에서 직접 사용할 값들 전역 변수에 저장
    lateral_error_from_topic = data.lateral_error
    target_point_detected = data.target_point_detected
    target_heading_from_topic = data.target_heading if data.target_point_detected else 0.0
    target_point_from_topic = data.target_point if data.target_point_detected else None

#=============================================
# 신호등 검출 함수
#=============================================
def detect_traffic_light(img):
    h, w = img.shape[:2]
    roi = img[0:int(h * 0.2), int(w * 0.35):int(w * 0.65)]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_r1 = np.array([0, 120, 70]);   upper_r1 = np.array([10, 255, 255])
    lower_r2 = np.array([170, 120, 70]); upper_r2 = np.array([180, 255, 255])
    mask_r = cv2.bitwise_or(
        cv2.inRange(hsv, lower_r1, upper_r1),
        cv2.inRange(hsv, lower_r2, upper_r2)
    )

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
# 모터로 토픽을 발행하는 함수
#=============================================
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)

#=============================================
# 차선 정보를 기반으로 조향각 및 속도 계산하는 함수
#=============================================
def kanayama_control():
    global lane_data, current_lane, is_lane_changing
    global lane_change_path, lane_change_direction, lane_change_start_time
    global L, last_left_lane_detected, last_right_lane_detected
    global left_lane_missing_time, right_lane_missing_time
    global is_speed_boosted, speed_boost_start_time, has_speed_boosted

    # lane_data가 수신되지 않았다면 정지 상태 (조향 0, 기본 속도)
    if lane_data is None:
        return 0.0, Fix_Speed

    # 1) 차선 변경 중인 경우
    if is_lane_changing and lane_change_path:
        current_time = time.time()
        # 차선 변경 완료 조건
        if current_time - lane_change_start_time > LANE_CHANGE_DURATION:
            is_lane_changing = False
            lane_change_path = []
            lane_change_direction = 0
            rospy.loginfo("차선 변경 완료")
            # 차선 변경 완료 후 속도 증가 시작 (한 번만)
            if not has_speed_boosted:
                is_speed_boosted = True
                speed_boost_start_time = current_time
                has_speed_boosted = True
                rospy.loginfo("속도 증가 시작: 1.2배 속도로 가속")
                return 0.0, Fix_Speed * 1.2  # 가속 시작
        else:
            # 차선 변경 중에는 고정 속도로 유지하며 path 따라 조향만 수행
            if lane_data.left_x != 130 and lane_data.right_x != -130:
                current_x = (lane_data.left_x + lane_data.right_x) / 2
            elif lane_data.left_x != 130:
                current_x = lane_data.left_x + 75
            elif lane_data.right_x != -130:
                current_x = lane_data.right_x - 75
            else:
                current_x = BASE_X

            steering_angle = lane_change_control(
                lane_change_path,
                current_x, BASE_Y,
                math.radians(-(lane_data.left_slope + lane_data.right_slope) / 2),
                lane_change_direction
            )
            return steering_angle, Fix_Speed

    # 2) 속도 증가 중
    if is_speed_boosted:
        current_time = time.time()
        if current_time - speed_boost_start_time > speed_boost_duration:
            is_speed_boosted = False
            rospy.loginfo("속도 증가 종료: 정상 속도로 복귀")
        else:
            remaining = speed_boost_duration - (current_time - speed_boost_start_time)
            rospy.loginfo(f"속도 증가 중: {remaining:.1f}초 남음")
            steering_angle, _ = calculate_normal_steering()
            return steering_angle, Fix_Speed * 1.2

    # 3) 일반 주행 로직
    return calculate_normal_steering()

#=============================================
# 정상적인 조향각 및 속도 계산 로직 (차선 미감지 보정 포함)
#=============================================
def calculate_normal_steering():
    global lane_data, L, is_speed_boosted
    global last_left_lane_detected, last_right_lane_detected
    global left_lane_missing_time, right_lane_missing_time

    FixSpeed = Fix_Speed
    left_x = lane_data.left_x
    right_x = lane_data.right_x
    left_slope = lane_data.left_slope
    right_slope = lane_data.right_slope
    lane_width = 3.5

    # 카나야마 제어 파라미터
    K_y = 0.95
    K_phi = 3.2
    v_r = FixSpeed

    # 속도 증가 중일 때 제어 파라미터 약간 변경
    if is_speed_boosted:
        K_y = 0.85
        K_phi = 2.8

    current_time = time.time()

    # 3-1) 양쪽 차선 모두 감지되지 않을 때
    if left_x == 130 and right_x == -130:
        rospy.logwarn("Both lanes lost, 조향각 0 반환")
        return 0.0, FixSpeed

    # 3-2) 왼쪽 차선만 없는 경우
    elif left_x == 130:
        if not last_left_lane_detected:
            if left_lane_missing_time == 0:
                left_lane_missing_time = current_time
            elif current_time - left_lane_missing_time >= LANE_MISSING_THRESHOLD:
                # 2초 이상 왼쪽 차선이 없으면 오른쪽으로 조향 시도
                steering_angle = 15.0 if is_speed_boosted else 20.0
                rospy.loginfo(f"왼쪽 차선 2초 미감지: 오른쪽으로 조향 (각도: {steering_angle:.1f}도)")
                return steering_angle, v_r
        else:
            left_lane_missing_time = current_time
            last_left_lane_detected = False

        lateral_err = -(0.5 - (right_x / 150.0)) * lane_width
        heading_err = right_slope

    # 3-3) 오른쪽 차선만 없는 경우
    elif right_x == -130:
        if not last_right_lane_detected:
            if right_lane_missing_time == 0:
                right_lane_missing_time = current_time
            elif current_time - right_lane_missing_time >= LANE_MISSING_THRESHOLD:
                # 2초 이상 오른쪽 차선이 없으면 왼쪽으로 조향 시도
                steering_angle = -15.0 if is_speed_boosted else -20.0
                rospy.loginfo(f"오른쪽 차선 2초 미감지: 왼쪽으로 조향 (각도: {steering_angle:.1f}도)")
                return steering_angle, v_r
        else:
            right_lane_missing_time = current_time
            last_right_lane_detected = False

        lateral_err = (0.5 - (left_x / 150.0)) * lane_width
        heading_err = left_slope

    # 3-4) 양쪽 차선 모두 감지된 경우
    else:
        last_left_lane_detected = True
        last_right_lane_detected = True
        left_lane_missing_time = 0
        right_lane_missing_time = 0

        # 0으로 나누는 상황 방지
        if abs(left_x + right_x) < 0.1:
            rospy.logwarn("차선 위치가 중앙에 너무 가까움, 조향각 0 반환")
            return 0.0, v_r

        lateral_err = -(left_x / (left_x + right_x) - 0.5) * lane_width
        heading_err = 0.5 * (left_slope + right_slope)

    heading_err *= -1
    v = v_r * math.cos(heading_err)
    w = v_r * (K_y * lateral_err + K_phi * math.sin(heading_err))
    delta = math.atan2(w * L, v)
    steering_angle = math.degrees(delta)

    # 속도 증가 중일 때 조향 제한 강화
    if is_speed_boosted:
        steering_angle = max(min(steering_angle, 20.0), -20.0)
    # else:
    #     steering_angle = max(min(steering_angle, 50.0), -50.0)

    return steering_angle, v

#=============================================
# 라바콘 제어(카나야마) 함수
#=============================================
def cone_kanayama_control():
    global L, target_point_detected, lateral_error_from_topic, target_heading_from_topic

    if not target_point_detected:
        rospy.logwarn("Target point not detected by cone perception node. Slowing down.")
        return 0.0, target_cone_speed / 2.0

    # 라바콘 카나야마 제어 파라미터
    K_y = 0.0
    K_phi = 5.0
    v_r = target_cone_speed

    lateral_err = lateral_error_from_topic
    heading_err = target_heading_from_topic

    v = v_r * math.cos(heading_err)
    w = v_r * (K_y * lateral_err + K_phi * math.sin(heading_err))
    delta = math.atan2(w * L, v)
    steering_angle = math.degrees(delta)
    steering_angle = max(min(steering_angle, 100.0), -100.0)

    rospy.loginfo(f"[Cone Control] lat_err: {lateral_err:.2f}, head_err(deg): {math.degrees(heading_err):.2f}, angle: {steering_angle:.2f}")
    return steering_angle, v

#=============================================
# 안정화 제어 함수 (Stabilizer)
#=============================================
def stabilizer_control():
    global is_stabilizing, stabilizer_start_time, STABILIZER_DURATION, STABILIZER_SPEED

    if time.time() - stabilizer_start_time > STABILIZER_DURATION:
        is_stabilizing = False
        rospy.loginfo("--- 안정화 모드 종료 ---")
        return kanayama_control()[0], kanayama_control()[1]
    else:
        rospy.loginfo(f"--- Stability Mode Active ({time.time() - stabilizer_start_time:.1f}s / {STABILIZER_DURATION}s) ---")
        steering_angle, _ = kanayama_control()
        return steering_angle, STABILIZER_SPEED

#=============================================
# 2차 곡선 경로 생성 함수
#=============================================
def generate_quadratic_path(dx, dy, theta=0, steps=50):
    A = np.array([
        [0**2, 0, 1],        # y(0) = 0
        [dx**2, dx, 1],      # y(dx) = dy
        [2 * 0, 1, 0]        # y'(0) = tan(theta)
    ])
    B = np.array([0, dy, math.tan(theta)])

    try:
        a, b, c = np.linalg.solve(A, B)
    except np.linalg.LinAlgError:
        # 해를 찾을 수 없는 경우 직선 경로 생성
        return [(x, y) for x, y in zip(np.linspace(0, dx, steps),
                                       np.linspace(0, dy, steps))]

    xs = np.linspace(0, dx, steps)
    ys = a * xs**2 + b * xs + c
    return list(zip(xs, ys))

#=============================================
# 전역 좌표계 변환 함수 (로컬→글로벌)
#=============================================
def local_to_global(x0, y0, theta, dx, dy):
    gx = x0 + dx * math.cos(theta) - dy * math.sin(theta)
    gy = y0 + dx * math.sin(theta) + dy * math.cos(theta)
    return gx, gy

#=============================================
# 차선 변경 경로 생성 함수
#=============================================
def generate_lane_change_path(current_lane, target_lane, start_x, left_x, right_x):
    if current_lane == 0 or target_lane == 0:
        return [], 0

    # 차선 중심점 계산
    if left_x != 130 and right_x != -130:
        center_x = (left_x + right_x) / 2
    elif left_x != 130:
        center_x = left_x + 75
    elif right_x != -130:
        center_x = right_x - 75
    else:
        return [], 0

    # 차선 변경 방향 결정
    if current_lane == 1 and target_lane == 2:
        dx = 150
        dy = LANE_CHANGE_DISTANCE
        direction = 1
    elif current_lane == 2 and target_lane == 1:
        dx = -150
        dy = LANE_CHANGE_DISTANCE
        direction = -1
    else:
        return [], 0

    path = generate_quadratic_path(dx, dy, theta=0, steps=PATH_POINTS)
    adjusted_path = [(start_x + dx_i, BASE_Y - dy) for dx_i, dy in path]

    if adjusted_path:
        rospy.loginfo(f"생성된 경로: 시작점 {adjusted_path[0]}, 끝점 {adjusted_path[-1]}")
        rospy.loginfo(f"차선 변경 방향: {'우회전' if direction > 0 else '좌회전'}, 이동 거리: {abs(dx)}")
        rospy.loginfo(f"시작 x좌표: {start_x}, 차선 중심: {center_x}")

    return adjusted_path, direction

#=============================================
# 차선 변경 제어 함수
#=============================================
def lane_change_control(path, current_x, current_y, current_angle, direction):
    if not path:
        return 0.0

    min_dist = float('inf')
    closest_idx = 0
    for i, (x, y) in enumerate(path):
        dist = math.hypot(x - current_x, y - current_y)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    look_ahead_idx = min(closest_idx + 20, len(path) - 1)
    target_x, target_y = path[look_ahead_idx]

    alpha = math.atan2(target_y - current_y, target_x - current_x) - current_angle
    base = math.degrees(math.atan2(6 * math.sin(alpha), 15))
    steering_angle = direction * abs(base)
    steering_angle = max(min(steering_angle, 50.0), -50.0)

    return steering_angle

#=============================================
# 경로 시각화 함수 (이미지 상)
#=============================================
def visualize_path(image, path):
    if not path:
        return image

    dx0, dy0 = path[0]
    adjusted = [(dx - dx0, dy - dy0) for dx, dy in path]
    image_coords = [(int(BASE_X + dx), int(BASE_Y - dy)) for dx, dy in adjusted]

    # 클리핑
    image_coords = [
        (max(0, min(x, IMAGE_WIDTH - 1)), max(0, min(y, IMAGE_HEIGHT - 1)))
        for x, y in image_coords
    ]
    points = np.array(image_coords, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [points], False, (0, 255, 0), 2)
    if image_coords:
        cv2.circle(image, image_coords[0], 5, (0, 0, 255), -1)   # 시작점
        cv2.circle(image, image_coords[-1], 5, (255, 0, 0), -1) # 끝점
        rospy.loginfo(f"경로 시작 (이미지 좌표): {image_coords[0]}, 끝: {image_coords[-1]}")
        rospy.loginfo(f"원본 경로 시작: {path[0]}, 끝: {path[-1]}")
    return image

#=============================================
# 메인 함수: ROS 노드 초기화 및 루프
#=============================================
def start():
    global motor, image, ranges, lane_data, prev_cone_detected_flag, is_stabilizing, stabilizer_start_time
    global is_speed_boosted, speed_boost_start_time, has_speed_boosted, cone_mode_enabled, has_stabilized
    global is_lane_changing, lane_change_path, lane_change_direction, lane_change_start_time, current_lane
    global sim_start_time

    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    rospy.Subscriber("lane_info", laneinfo, lane_callback, queue_size=1)
    rospy.Subscriber("cone_lanes", ConeLanes, conelane_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    rospy.loginfo("Camera Ready")
    rospy.wait_for_message("/scan", LaserScan)
    rospy.loginfo("Lidar Ready")

    plt.ion()
    plt.show()
    rospy.loginfo("Lidar Visualizer Ready")

    rospy.loginfo("====================================")
    rospy.loginfo("   S T A R T    D R I V I N G ...")
    rospy.loginfo("====================================")

    prev_cone_detected_flag = False
    sim_start_time = time.time()  # 시뮬레이터 시작 시각 저장

    while not rospy.is_shutdown():
        # 1) 카메라 이미지 표시 및 차선 변경 경로 시각화
        if image.size != 0:
            display_img = image.copy()
            if is_lane_changing and lane_change_path:
                display_img = visualize_path(display_img, lane_change_path)
            # cv2.imshow("Frame", display_img)
            # cv2.waitKey(1)

        # 2) 라이다 시각화
        if ranges is not None:
            angles = np.linspace(0, 2 * np.pi, len(ranges)) + np.pi / 2
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            lidar_points.set_data(x, y)
            fig.canvas.draw_idle()
            plt.pause(0.01)

            # 3) 제어 로직 분기
            steering_angle, v = 0.0, Fix_Speed
            if is_stabilizing:
                steering_angle, v = stabilizer_control()
            elif lane_data is not None and lane_data.cone_detected_flag:
                rospy.loginfo("--- Cone Driving Mode ---")
                steering_angle, v = cone_kanayama_control()
            else:
                rospy.loginfo("--- Lane Driving Mode ---")
                steering_angle, v = kanayama_control()

            # 4) 신호등 상태에 따른 정지/출발 판단
            tl_state = detect_traffic_light(image)
            if tl_state == 'green':
                light_go = True
            elif tl_state in ('red', 'yellow'):
                light_go = False
            else:
                # 'none'이면 이전 상태 유지
                pass

            speed = v if light_go else 0.0
            drive(angle=steering_angle, speed=speed)

            # 5) 라바콘 → 차선 모드 전환 시 안정화 모드 트리거
            if lane_data is not None:
                # 1분(60초) 이내에만 안정화 모드 진입 허용
                if (prev_cone_detected_flag and not lane_data.cone_detected_flag
                    and not is_stabilizing
                    and (time.time() - sim_start_time < 90)):
                    is_stabilizing = True
                    stabilizer_start_time = time.time()
                    rospy.loginfo("--- 라바콘 구간 탈출! 안정화 모드 시작 ---")
                prev_cone_detected_flag = lane_data.cone_detected_flag

        time.sleep(0.1)

    # 종료 시 정리
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    start()
