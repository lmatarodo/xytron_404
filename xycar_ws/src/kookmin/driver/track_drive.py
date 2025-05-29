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
from xycar_msgs.msg import ConeLanes
import matplotlib.pyplot as plt

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
ranges = None  # 라이다 데이터를 담을 변수
motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
Fix_Speed = 37 #원래 10  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
lane_data = None  # 차선 정보를 담을 변수
k_p = Fix_Speed/25
k_para = 20
k_lateral = 5
light_go = False  # ← 초록불 인식 후 출발 유지
L = 0.5

#차량 감지 관련
vehicle_ahead = False
detection_success_streak = 0
DETECTION_DISTANCE_LOW = 5.0
DETECTION_DISTANCE_HIGH = 40.0
DETECTION_COUNT    = 4      # 연속으로 이만큼 포인트가 잡히면 차량으로 판정
SECTOR_WIDTH       = 7      # 앞 0번 인덱스 기준으로 ±2 인덱스 조사
DETECTION_STREAK_THRESHOLD = 2     # 스캔이 연속으로 성공해야 할 횟수

# 차선 번호 관련 변수
current_lane = 0  # 현재 차선 번호 (0: 불확실, 1: 1차선, 2: 2차선)
is_lane_changing = False  # 차선 변경 중인지 여부
lane_change_path = []  # 차선 변경 경로
lane_change_direction = 0  # 차선 변경 방향 (1: 우회전, -1: 좌회전)
lane_change_start_time = 0  # 차선 변경 시작 시간
last_lane_change_time = 0  # 마지막 차선 변경 시간
#LANE_CHANGE_INTERVAL = 6.0  #폐기 # 차선 변경 간격 (초)
LANE_CHANGE_DURATION = 1.65  #3.0  차선 변경 소요 시간 (초)
PATH_POINTS = 20  #30 # 경로 생성 시 샘플링할 포인트 수
LANE_CHANGE_DISTANCE = 230 #200  # 차선 변경 시 전방 주시 거리

# 이미지 관련 상수
IMAGE_WIDTH = 260  # 이미지 너비
IMAGE_HEIGHT = 260  # 이미지 높이
BASE_X = 130  # 이미지 중앙 x좌표
BASE_Y = 260  # 이미지 하단 y좌표

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
    global ranges, vehicle_ahead, detection_success_streak
    ranges = data.ranges[0:360]

    # 1) 이번 스캔에서 공간적으로 연속된 DETECTION_COUNT 점이 감지됐는지 판정
    detected_this_scan = False
    consec = 0
    for offset in range(-SECTOR_WIDTH, SECTOR_WIDTH+1):
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

    # 3) 3번 연속 스캔 성공 시에만 플래그 ON
    if detection_success_streak >= DETECTION_STREAK_THRESHOLD:
        vehicle_ahead = True
    else:
        vehicle_ahead = False

def lane_callback(data):
    global lane_data, current_lane, is_lane_changing, lane_change_path
    global lane_change_direction, lane_change_start_time, last_lane_change_time
    lane_data = data
    
    # 현재 시간 확인
    current_time = time.time()

    # 실측 lane 번호 업데이트
    if data.lane_number in [1, 2]:
        current_lane = data.lane_number

    # 차선 변경 조건
    if not is_lane_changing and vehicle_ahead:
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
    global lane_data, current_lane, is_lane_changing
    global lane_change_path, lane_change_direction, lane_change_start_time
    global L
    if lane_data is None:
        return 0.0, Fix_Speed

    # 차선 변경 중인 경우
    if is_lane_changing and lane_change_path:
        current_time = time.time()
        
        # 차선 변경 완료 조건 확인
        if current_time - lane_change_start_time > LANE_CHANGE_DURATION:
            is_lane_changing = False
            lane_change_path = []
            lane_change_direction = 0
            rospy.loginfo("차선 변경 완료")
        else:
            # 현재 차선 중심점 계산
            if lane_data.left_x != 130 and lane_data.right_x != -130:
                current_x = (lane_data.left_x + lane_data.right_x) / 2
            elif lane_data.left_x != 130:
                current_x = lane_data.left_x + 75
            elif lane_data.right_x != -130:
                current_x = lane_data.right_x - 75
            else:
                current_x = BASE_X  # 기본값으로 이미지 중앙 사용

            # 차선 변경 제어
            steering_angle = lane_change_control(
                lane_change_path,
                current_x, BASE_Y,  # 실제 차량 위치 사용
                math.radians(-(lane_data.left_slope + lane_data.right_slope)/2),
                lane_change_direction
            )
            increased_speed = Fix_Speed*1.15
            steering_angle = steering_angle*1.1
            return steering_angle, increased_speed

    # 기존의 kanayama_control 로직
    left_x = lane_data.left_x
    right_x = lane_data.right_x
    left_slope = lane_data.left_slope
    right_slope = lane_data.right_slope
    lane_width = 3.5
    
    # 파라미터
    K_y = 0.85
    K_phi = 3.0
    v_r = Fix_Speed
    
    # lateral_err, heading_err 계산 (주신 코드 참고)
    if left_x == 130 and right_x == -130:
        rospy.logwarn("Both lanes lost, skipping control.")
        return 0.0, Fix_Speed
    elif left_x == 130:
        lateral_err = -(0.5 - (right_x / 150.0)) * lane_width
        heading_err = right_slope
    elif right_x == -130:
        lateral_err = (0.5 - (left_x / 150.0)) * lane_width
        heading_err = left_slope
    else:
        lateral_err = -(left_x / (left_x + right_x) - 0.5) * lane_width
        heading_err = 0.5 * (left_slope + right_slope)
    heading_err *= -1

    # 각속도 w 계산
    v = v_r * math.cos(heading_err)
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
# 폴리노미얼 경로 생성 함수
#=============================================
def generate_quadratic_path(dx, dy, theta=0, steps=50):
    """
    2차 곡선 경로 생성
    dx, dy: 목표점 상대 좌표
    theta: 시작 방향 (라디안)
    steps: 경로 포인트 수
    """
    # y = ax^2 + bx + c
    # 조건: (0, 0), (dx, dy), y'(0) = tan(theta)
    A = np.array([
        [0**2, 0, 1],  # y(0) = 0
        [dx**2, dx, 1],  # y(dx) = dy
        [2*0, 1, 0]  # y'(0) = tan(theta)
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

def local_to_global(x0, y0, theta, dx, dy):
    """로컬 좌표를 전역 좌표로 변환"""
    gx = x0 + dx * math.cos(theta) - dy * math.sin(theta)
    gy = y0 + dx * math.sin(theta) + dy * math.cos(theta)
    return gx, gy

def transform_path(path, x0, y0, theta):
    """경로를 전역 좌표계로 변환"""
    return [local_to_global(x0, y0, theta, dx, dy) for dx, dy in path]

def convert_local_path_to_image_coords(path, base_x=BASE_X, base_y=BASE_Y):
    """경로의 시작점을 항상 이미지 중앙에 오도록 좌표 전체 이동"""
    if not path:
        return []

    dx0, dy0 = path[0]  # 시작점 기준
    image_coords = []

    for dx, dy in path:
        # 경로를 전체적으로 시작점 기준으로 평행이동
        rel_dx = dx - dx0
        rel_dy = dy - dy0
        x = int(base_x + rel_dx)
        y = int(base_y - rel_dy)

        # 이미지 범위 클리핑
        x = max(0, min(x, IMAGE_WIDTH - 1))
        y = max(0, min(y, IMAGE_HEIGHT - 1))
        image_coords.append((x, y))

    return image_coords

def visualize_path(image, path):
    """경로를 이미지에 시각화"""
    if not path:
        return image

    # 시작점 기준 상대 좌표로 평행 이동
    dx0, dy0 = path[0]
    adjusted_path = [(dx - dx0, dy - dy0) for dx, dy in path]

    # 이미지 좌표계로 변환 (기준점은 항상 이미지 하단 중앙)
    image_coords = []
    for dx, dy in adjusted_path:
        x = int(BASE_X + dx)
        y = int(BASE_Y - dy)
        x = max(0, min(x, IMAGE_WIDTH - 1))
        y = max(0, min(y, IMAGE_HEIGHT - 1))
        image_coords.append((x, y))

    # 경로 포인트를 numpy 배열로 변환
    points = np.array(image_coords, dtype=np.int32).reshape((-1, 1, 2))

    # 경로 그리기
    cv2.polylines(image, [points], False, (0, 255, 0), 2)

    # 시작점과 끝점 표시
    if image_coords:
        cv2.circle(image, image_coords[0], 5, (0, 0, 255), -1)  # 시작점
        cv2.circle(image, image_coords[-1], 5, (255, 0, 0), -1)  # 끝점
        rospy.loginfo(f"경로 시작점 (이미지 좌표): {image_coords[0]}, 끝점: {image_coords[-1]}")
        rospy.loginfo(f"원본 경로 시작점: {path[0]}, 끝점: {path[-1]}")

    return image

def generate_lane_change_path(current_lane, target_lane, start_x, left_x, right_x):
    """차선 변경 경로 생성"""
    if current_lane == 0 or target_lane == 0:
        return [], 0
    
    # 차선 중심점 계산
    if left_x != 130 and right_x != -130:
        center_x = (left_x + right_x) / 2
    elif left_x != 130:
        center_x = left_x + 75  # 차선 폭 가정
    elif right_x != -130:
        center_x = right_x - 75  # 차선 폭 가정
    else:
        return [], 0
    
    # 차선 변경 방향 결정
    if current_lane == 1 and target_lane == 2:  # 1차선 → 2차선
        dx = 90  #150# 오른쪽으로 이동
        dy = LANE_CHANGE_DISTANCE  # 전방 거리
        direction = 1  # 우회전
    elif current_lane == 2 and target_lane == 1:  # 2차선 → 1차선
        dx = -90  #-150# 왼쪽으로 이동
        dy = LANE_CHANGE_DISTANCE  # 전방 거리
        direction = -1  # 좌회전
    else:
        return [], 0
    
    # 경로 생성
    path = generate_quadratic_path(dx, dy, theta=0, steps=PATH_POINTS)
    
    # 경로를 실제 시작점 기준으로 변환
    adjusted_path = [(start_x + dx, BASE_Y - dy) for dx, dy in path]
    
    # 디버깅을 위한 경로 정보 출력
    if adjusted_path:
        rospy.loginfo(f"생성된 경로: 시작점 {adjusted_path[0]}, 끝점 {adjusted_path[-1]}")
        rospy.loginfo(f"차선 변경 방향: {'우회전' if direction > 0 else '좌회전'}, 이동 거리: {abs(dx)}")
        rospy.loginfo(f"시작 x좌표: {start_x}, 차선 중심: {center_x}")
    
    return adjusted_path, direction

def lane_change_control(path, current_x, current_y, current_angle, direction):
    global L
    """차선 변경 제어"""
    if not path:
        return 0.0
    
    # 현재 위치에서 가장 가까운 경로 포인트 찾기
    min_dist = float('inf')
    closest_idx = 0
    
    for i, (x, y) in enumerate(path):
        dist = math.sqrt((x - current_x)**2 + (y - current_y)**2)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    
    # 전방 주시 포인트 찾기
    look_ahead_idx = min(closest_idx + 20, len(path) - 1)
    target_x, target_y = path[look_ahead_idx]
    
    # 조향각 계산
    alpha = math.atan2(target_y - current_y, target_x - current_x) - current_angle
    base = math.degrees(math.atan2(6 * math.sin(alpha), 15))
    steering_angle = direction * abs(base)
    
    # 조향각 제한 (-50 ~ 50도)
    steering_angle = max(min(steering_angle, 50.0), -50.0)
    
    return steering_angle

# #=============================================
# # 1) 콘 구역 판단 함수 (min_points=2)
# #=============================================
# def find_cone_section(ranges,
#                       y_threshold=7.0,
#                       min_points=2,
#                       fov_deg=45):
#     count = 0
#     for i, r in enumerate(ranges):
#         if r <= 0.0 or np.isinf(r):
#             continue
#         angle = math.radians(i if i <= 180 else i - 360)
#         y = r * math.cos(angle)
#         if 0 < y < y_threshold and abs(math.degrees(angle)) <= fov_deg:
#             count += 1
#             if count >= min_points:
#                 return True
#     return False

# #=============================================
# # 2) 콘 구역 내 최대 gap 찾기
# #=============================================
# def find_cone_gap(ranges,
#                   y_threshold=7.0,
#                   x_clip=100.0):
#     xs = []
#     for i, r in enumerate(ranges):
#         if r <= 0.0 or np.isinf(r):
#             continue
#         angle = math.radians(i if i <= 180 else i - 360)
#         y = r * math.cos(angle)
#         if 0 < y < y_threshold:
#             x = r * math.sin(angle)
#             if abs(x) <= x_clip:
#                 xs.append(x)
#     if len(xs) < 2:
#         return 0.0
#     xs.sort()
#     max_gap, best_idx = 0.0, 0
#     for j in range(len(xs)-1):
#         gap = xs[j+1] - xs[j]
#         if gap > max_gap:
#             max_gap, best_idx = gap, j
#     x_mid = (xs[best_idx] + xs[best_idx+1]) / 2.0
#     return math.degrees(math.atan2(x_mid, y_threshold/2.0))


#=============================================
# 실질적인 메인 함수 
#=============================================
def start():
    global motor, image, ranges
    prev_angle = 0.0
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
            # 원본 이미지 복사
            display_img = image.copy()
            
            # 차선 변경 경로 시각화
            if is_lane_changing and lane_change_path:
                display_img = visualize_path(display_img, lane_change_path)
            
            # 이미지 표시
            #cv2.imshow("original", display_img)
            #cv2.imshow("gray", cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))

        if ranges is not None:            
            angles = np.linspace(0,2*np.pi, len(ranges))+np.pi/2
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            lidar_points.set_data(x, y)
            fig.canvas.draw_idle()
            plt.pause(0.01)

            steering_angle, v = kanayama_control()
            tl_state = detect_traffic_light(image)
            # 신호등 상태 기반 출발/정지 플래그 제어
            if tl_state == 'green':
                light_go = True
            elif tl_state in ('red', 'yellow'):
                light_go = False
            # 'none'이면 이전 상태 유지
            #light_go = True
            #rospy.loginfo(f"[TL] state: {tl_state}, go: {light_go}")
            speed = v if light_go else 0.0
            drive(angle=steering_angle, speed=speed)
            
            # 계산된 조향각으로 주행
            time.sleep(0.1)
            
            cv2.waitKey(1)

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()