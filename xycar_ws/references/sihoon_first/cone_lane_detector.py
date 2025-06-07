#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from xycar_msgs.msg import ConeLanes # 사용자 정의 메시지
import std_msgs.msg
import matplotlib.pyplot as plt

# =============================================
# 파라미터 
# =============================================
LIDAR_ANGLE_RANGE_HALF = 95 
LIDAR_MAX_DIST = 12.0 
CONE_CLUSTER_RADIUS = 0.3 

CHAIN_INITIAL_SEED_Y_MAX = 4.0   
CHAIN_MAX_NEXT_CONE_DIST = 5.0   # 필터링 조건
CHAIN_MAX_HEADING_CHANGE_RAD = np.radians(80) # 필터링 조건
CHAIN_MIN_POINTS_FOR_LANE = 2 # 최소 2개의 콘이 있어야 차선으로 인정

EXPECTED_LANE_WIDTH_APPROX = 3.5 
X_SEED_CENTER_THRESHOLD = 0.05

# === 새로운 덧셈 방식 비용 함수 가중치 ===
ADD_COST_WEIGHT_DISTANCE = 1.0  # 거리제곱 오차에 대한 가중치
ADD_COST_WEIGHT_ANGLE = 10.0   # 각도변화량 오차에 대한 가중치
ADD_COST_WEIGHT_LATERAL = 2.0  # 측면 거리 절대값에 대한 가중치

# [새로 추가됨] 다음 콘 연결을 위한 최대 허용 비용
MAX_ACCEPTABLE_CONNECTION_COST = 20.0 
# ==========================================

ENABLE_MATPLOTLIB_VIZ = True 

if ENABLE_MATPLOTLIB_VIZ:
    fig, ax = plt.subplots(figsize=(10, 10)) 
    if LIDAR_ANGLE_RANGE_HALF <= 90:
        plot_x_max_abs = LIDAR_MAX_DIST * math.sin(math.radians(LIDAR_ANGLE_RANGE_HALF))
    else:
        plot_x_max_abs = LIDAR_MAX_DIST
    ax.set_xlim(-(plot_x_max_abs + 0.5), plot_x_max_abs + 0.5)
    ax.set_ylim(-0.5, LIDAR_MAX_DIST + 0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Cone Lane Detector Viz (Max Cost Threshold)")
    ax.set_xlabel("X (m) - Lateral")
    ax.set_ylabel("Y (m) - Forward")
    ax.grid(True)
    raw_lidar_points_plot, = ax.plot([], [], 'ko', markersize=3, label='Clustered Pts (All)')
    left_lane_plot, = ax.plot([], [], 'r.-', markersize=5, linewidth=1.5, label='Left Lane Fit')
    right_lane_plot, = ax.plot([], [], 'b.-', markersize=5, linewidth=1.5, label='Right Lane Fit')
    center_path_plot, = ax.plot([], [], 'g--', linewidth=1.5, label='Center Path')
    raw_left_lane_points_plot, = ax.plot([], [], 'rx', markersize=8, markeredgewidth=2, label='Raw Left Cones')  
    raw_right_lane_points_plot, = ax.plot([], [], 'bx', markersize=8, markeredgewidth=2, label='Raw Right Cones') 
    ax.legend(fontsize='small', loc='upper right')

# =============================================
# 유틸리티 함수
# =============================================
def convert_lidar_to_xy(current_ranges_data):
    """라이다 데이터를 XY 좌표로 변환"""
    if current_ranges_data is None or len(current_ranges_data) != 360: return np.array([]) 
    # 관심 각도 범위 설정 (0도 전방 기준, 시계방향 양수)
    # 0 ~ LIDAR_ANGLE_RANGE_HALF (오른쪽)
    # 360-LIDAR_ANGLE_RANGE_HALF ~ 360 (왼쪽)
    indices_left = np.arange(0, LIDAR_ANGLE_RANGE_HALF + 1) 
    indices_right = np.arange(360 - LIDAR_ANGLE_RANGE_HALF, 360) 
    valid_angular_indices = np.unique(np.concatenate((indices_left, indices_right)))
    
    selected_ranges = current_ranges_data[valid_angular_indices]
    selected_angles_rad = np.radians(valid_angular_indices.astype(float)) 
    
    # 유효한 거리 값만 필터링 (inf, nan, LIDAR_MAX_DIST 초과 제외)
    valid_value_indices = np.where(np.isfinite(selected_ranges) & (selected_ranges > 0.1) & (selected_ranges < LIDAR_MAX_DIST))[0] # 0.1m 이상 추가
    if len(valid_value_indices) == 0: return np.array([])
    
    final_ranges = selected_ranges[valid_value_indices]
    final_angles_rad = selected_angles_rad[valid_value_indices]
    
    # XY 좌표 변환 (차량 좌표계: X-좌우, Y-전후)
    # 라이다 각도: 0도 전방, 반시계방향 증가 -> 차량 좌표계 각도: 0도 전방(+Y), 시계방향 증가(+X)
    # x = range * sin(angle_rad_lidar) -> 차량 좌표계에서는 -sin 사용 (오른쪽이 +X)
    # y = range * cos(angle_rad_lidar)
    x_coords = final_ranges * (-1) * np.sin(final_angles_rad) # sin의 부호 반전으로 오른쪽이 +x가 되도록 함
    y_coords = final_ranges * np.cos(final_angles_rad)
    
    return np.column_stack((x_coords, y_coords))

def cluster_lidar_points(points, cluster_radius_sq):
    """간단한 거리 기반 클러스터링 (DBSCAN 유사)"""
    if points.shape[0] == 0: return np.array([])
    n_points = points.shape[0]
    processed_indices = np.zeros(n_points, dtype=bool)
    cluster_centers = []
    
    for i in range(n_points):
        if processed_indices[i]: continue
        
        current_cluster_members_indices = [i]
        processed_indices[i] = True
        queue = [i]
        head = 0
        
        while head < len(queue):
            seed_idx = queue[head]; head += 1
            for j in range(n_points): # 모든 점에 대해 거리 검사
                if processed_indices[j]: continue
                if np.sum((points[seed_idx] - points[j])**2) < cluster_radius_sq:
                    current_cluster_members_indices.append(j)
                    processed_indices[j] = True
                    queue.append(j)
        
        if current_cluster_members_indices: # 클러스터 멤버가 있으면
            cluster_centers.append(np.mean(points[current_cluster_members_indices], axis=0))
            
    return np.array(cluster_centers)

def fit_polynomial_to_lane(lane_points_xy_data, degree=1):
    """주어진 점들에 대해 다항식 피팅 (y에 대한 x값)"""
    min_pts_for_degree = degree + 1 
    if lane_points_xy_data is None or lane_points_xy_data.shape[0] < min_pts_for_degree: return None
    
    x_values = lane_points_xy_data[:, 0] # x 좌표들
    y_values = lane_points_xy_data[:, 1] # y 좌표들 (독립 변수)
    
    try:
        # y를 독립변수로 하여 x = p(y) 형태의 다항식 피팅
        poly_coeffs = np.polyfit(y_values, x_values, degree) 
    except Exception: # 예: RankWarning 등
        poly_coeffs = None
    return poly_coeffs

def _calculate_angle_diff(vec1, vec2):
    """두 벡터 사이의 각도 차이 계산 (-pi ~ pi)"""
    angle1 = math.atan2(vec1[1], vec1[0])
    angle2 = math.atan2(vec2[1], vec2[0])
    diff = angle2 - angle1
    while diff > math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    return diff

# =============================================
# 핵심 차선 감지 로직
# =============================================
def detect_lanes_from_cones(xy_coords_data_raw):
    """클러스터링된 콘으로부터 좌우 차선 및 중앙 경로 추정"""
    clustered_cones = cluster_lidar_points(xy_coords_data_raw, CONE_CLUSTER_RADIUS**2)
    num_total_cones = clustered_cones.shape[0]
    
    sampled_left_lane_coords = [] # 피팅된 왼쪽 차선 샘플링 포인트
    sampled_right_lane_coords = []# 피팅된 오른쪽 차선 샘플링 포인트
    center_path_coords = []       # 추정된 중앙 경로 샘플링 포인트
    
    left_lane_detected_flag = False
    right_lane_detected_flag = False
    left_poly_deg = 0  # 왼쪽 차선 피팅 차수
    right_poly_deg = 0 # 오른쪽 차선 피팅 차수

    raw_left_cones_for_plot = np.array([])  # 실제 왼쪽 체인 콘 (시각화 및 Pure Pursuit용)
    raw_right_cones_for_plot = np.array([]) # 실제 오른쪽 체인 콘 (시각화 및 Pure Pursuit용)

    if num_total_cones < CHAIN_MIN_POINTS_FOR_LANE: # 전체 콘 개수가 너무 적으면 처리 안함
        return np.array(sampled_left_lane_coords), np.array(sampled_right_lane_coords), \
               np.array(center_path_coords), left_lane_detected_flag, \
               right_lane_detected_flag, left_poly_deg, right_poly_deg, \
               raw_left_cones_for_plot, raw_right_cones_for_plot

    is_cone_used = np.zeros(num_total_cones, dtype=bool) # 각 콘이 체인에 사용되었는지 여부
    
    # 내부 함수: 하나의 차선 체인(연속된 콘들)을 구축
    def build_one_lane_chain(start_cone_idx, all_cones, used_mask):
        if start_cone_idx is None or start_cone_idx < 0 or start_cone_idx >= len(all_cones) or used_mask[start_cone_idx]:
            return [] # 시작 콘이 유효하지 않으면 빈 리스트 반환
        
        current_chain = [all_cones[start_cone_idx]] # 현재 체인에 시작 콘 추가
        used_mask[start_cone_idx] = True            # 사용됨으로 표시
        
        last_added_cone = all_cones[start_cone_idx]
        previous_segment_vector = None # 이전 콘과 현재 콘을 잇는 벡터 (방향성 판단용)
        
        while True: # 가장 적합한 다음 콘을 계속 찾아 연결
            best_next_cone_idx = -1
            min_cost = float('inf') # 최소 비용 초기화

            # 모든 사용되지 않은 콘에 대해 다음 콘 후보로 검토
            for i in range(num_total_cones):
                if used_mask[i]: continue # 이미 사용된 콘은 제외
                
                candidate_cone = all_cones[i]
                vector_to_candidate = candidate_cone - last_added_cone # 현재 마지막 콘에서 후보 콘까지의 벡터
                dist_sq_to_candidate = np.sum(vector_to_candidate**2)  # 거리 제곱
                lateral_distance_to_candidate = abs(vector_to_candidate[0]) # 측면 거리 (x축 변화량)

                # 너무 멀리 있는 콘은 후보에서 제외
                if dist_sq_to_candidate >= CHAIN_MAX_NEXT_CONE_DIST**2: continue
                
                angle_change = 0.0 # 이전 세그먼트 대비 현재 세그먼트의 각도 변화량
                if previous_segment_vector is not None: # 체인에 콘이 2개 이상 있을 때부터 각도 변화 계산 가능
                    angle_change = _calculate_angle_diff(previous_segment_vector, vector_to_candidate)
                    # 각도 변화가 너무 크면 후보에서 제외 (급격한 꺾임 방지)
                    if abs(angle_change) > CHAIN_MAX_HEADING_CHANGE_RAD:
                        continue 
                
                # 비용 함수 계산 (거리가 가깝고, 각도 변화가 적고, 측면 이동이 적을수록 좋음)
                current_candidate_cost = 0.0
                current_candidate_cost += ADD_COST_WEIGHT_DISTANCE * dist_sq_to_candidate
                if previous_segment_vector is not None: 
                    current_candidate_cost += ADD_COST_WEIGHT_ANGLE * abs(angle_change) # 각도 변화량 절대값
                current_candidate_cost += ADD_COST_WEIGHT_LATERAL * lateral_distance_to_candidate
                
                # 현재까지의 최소 비용보다 작으면 업데이트
                if current_candidate_cost < min_cost:
                    min_cost = current_candidate_cost
                    best_next_cone_idx = i
            
            # 찾은 최적 후보의 비용이 허용치를 넘으면 연결 중단
            if best_next_cone_idx != -1: 
                if min_cost > MAX_ACCEPTABLE_CONNECTION_COST:
                    best_next_cone_idx = -1 # 비용이 너무 크면, 못 찾은 것으로 처리
            
            if best_next_cone_idx != -1: # 적합한 다음 콘을 찾았으면
                new_cone_to_add = all_cones[best_next_cone_idx]
                current_chain.append(new_cone_to_add)
                used_mask[best_next_cone_idx] = True
                
                if len(current_chain) > 1: # 체인에 콘이 2개 이상이면 이전 세그먼트 벡터 업데이트
                    previous_segment_vector = new_cone_to_add - last_added_cone 
                last_added_cone = new_cone_to_add # 마지막 추가된 콘 업데이트
            else: # 더 이상 연결할 적합한 콘이 없으면 종료
                break 
        return current_chain

    # 차선 체인 생성을 위한 시작 콘(seed cone) 찾기
    # y좌표가 CHAIN_INITIAL_SEED_Y_MAX 이하인 콘들 중에서 탐색
    seed_candidates_in_y_zone = clustered_cones[clustered_cones[:, 1] <= CHAIN_INITIAL_SEED_Y_MAX]
    if seed_candidates_in_y_zone.shape[0] == 0: # 해당 영역에 콘이 없으면 모든 콘을 후보로
        seed_candidates_in_y_zone = clustered_cones 
    
    left_seed_idx_in_clustered_cones = None
    right_seed_idx_in_clustered_cones = None

    if seed_candidates_in_y_zone.shape[0] > 0:
        # 왼쪽 시드: x < -THRESHOLD 이고, y가 가장 작고, 그 다음 x가 가장 작은(가장 왼쪽) 콘
        left_seed_pool = seed_candidates_in_y_zone[seed_candidates_in_y_zone[:, 0] < -X_SEED_CENTER_THRESHOLD]
        # 오른쪽 시드: x > THRESHOLD 이고, y가 가장 작고, 그 다음 x가 가장 큰(가장 오른쪽) 콘
        right_seed_pool = seed_candidates_in_y_zone[seed_candidates_in_y_zone[:, 0] > X_SEED_CENTER_THRESHOLD]

        if left_seed_pool.shape[0] > 0:
            # y 오름차순, x 오름차순 정렬
            sorted_left_seeds = left_seed_pool[np.lexsort((left_seed_pool[:, 0], left_seed_pool[:, 1]))]
            # 정렬된 첫번째 콘이 왼쪽 시드
            temp_idx = np.where((clustered_cones == sorted_left_seeds[0]).all(axis=1))[0]
            if temp_idx.size > 0: left_seed_idx_in_clustered_cones = temp_idx[0]

        if right_seed_pool.shape[0] > 0:
            # y 오름차순, x 내림차순(-x 오름차순) 정렬
            sorted_right_seeds = right_seed_pool[np.lexsort((-right_seed_pool[:, 0], right_seed_pool[:, 1]))] 
            # 정렬된 첫번째 콘이 오른쪽 시드
            temp_idx = np.where((clustered_cones == sorted_right_seeds[0]).all(axis=1))[0]
            if temp_idx.size > 0: right_seed_idx_in_clustered_cones = temp_idx[0]

    # 좌우 차선 체인 구축
    final_left_lane_pts = build_one_lane_chain(left_seed_idx_in_clustered_cones, clustered_cones, is_cone_used)
    final_right_lane_pts = build_one_lane_chain(right_seed_idx_in_clustered_cones, clustered_cones, is_cone_used)
    
    raw_left_cones_for_plot = np.array(final_left_lane_pts)
    raw_right_cones_for_plot = np.array(final_right_lane_pts)

    left_poly_coeffs = None
    right_poly_coeffs = None

    # 왼쪽 차선 피팅
    if raw_left_cones_for_plot.shape[0] >= CHAIN_MIN_POINTS_FOR_LANE:
        degree_left = min(3, raw_left_cones_for_plot.shape[0] - 1) # 최대 3차, 점 개수에 따라 조절
        if degree_left > 0 : 
            left_poly_coeffs = fit_polynomial_to_lane(raw_left_cones_for_plot, degree=degree_left)
            if left_poly_coeffs is not None: 
                left_lane_detected_flag = True
                left_poly_deg = degree_left
    
    # 오른쪽 차선 피팅
    if raw_right_cones_for_plot.shape[0] >= CHAIN_MIN_POINTS_FOR_LANE:
        degree_right = min(3, raw_right_cones_for_plot.shape[0] - 1) # 최대 3차
        if degree_right > 0: 
            right_poly_coeffs = fit_polynomial_to_lane(raw_right_cones_for_plot, degree=degree_right)
            if right_poly_coeffs is not None: 
                right_lane_detected_flag = True
                right_poly_deg = degree_right
    
    # 피팅된 차선 및 중앙 경로 샘플링
    path_sample_y_values = np.linspace(0.1, LIDAR_MAX_DIST * 0.9, 15) # y값 샘플링 구간
    for y_val_sample in path_sample_y_values:
        x_left_on_poly = np.polyval(left_poly_coeffs, y_val_sample) if left_poly_coeffs is not None else None
        x_right_on_poly = np.polyval(right_poly_coeffs, y_val_sample) if right_poly_coeffs is not None else None
        
        if x_left_on_poly is not None: 
            sampled_left_lane_coords.append([x_left_on_poly, y_val_sample])
        if x_right_on_poly is not None: 
            sampled_right_lane_coords.append([x_right_on_poly, y_val_sample])
        
        # 중앙 경로 계산
        if x_left_on_poly is not None and x_right_on_poly is not None:
            # 양쪽 차선 폭이 적절한 경우에만 중앙 경로 생성
            if EXPECTED_LANE_WIDTH_APPROX*0.2 < (x_right_on_poly-x_left_on_poly) < EXPECTED_LANE_WIDTH_APPROX*2.5 : 
                center_path_coords.append([(x_left_on_poly+x_right_on_poly)/2.0, y_val_sample])
        elif left_poly_coeffs is not None and x_left_on_poly is not None: # 왼쪽만 있을 때
            center_path_coords.append([x_left_on_poly + EXPECTED_LANE_WIDTH_APPROX/2.0, y_val_sample])
        elif right_poly_coeffs is not None and x_right_on_poly is not None: # 오른쪽만 있을 때
            center_path_coords.append([x_right_on_poly - EXPECTED_LANE_WIDTH_APPROX/2.0, y_val_sample])
    
    return np.array(sampled_left_lane_coords), np.array(sampled_right_lane_coords), \
           np.array(center_path_coords), left_lane_detected_flag, \
           right_lane_detected_flag, left_poly_deg, right_poly_deg, \
           raw_left_cones_for_plot, raw_right_cones_for_plot # 원본 콘 체인 반환 추가

# =============================================
# ROS 노드 메인 로직
# =============================================
class ConeLaneDetector:
    def __init__(self):
        rospy.init_node('cone_lane_detector_node', anonymous=False)
        
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        self.cone_lanes_publisher = rospy.Publisher('/cone_lanes', ConeLanes, queue_size=1) 
        
        self.current_lidar_ranges = None
        self.node_rate = rospy.Rate(10) # Hz

        if ENABLE_MATPLOTLIB_VIZ:
            plt.ion() # 대화형 모드 켜기
            plt.show() # 창 표시
        rospy.loginfo("Cone Lane Detector Node Initialized (Publisher Enabled).")

    def lidar_callback(self, msg):
        # 라이다 데이터는 0~359도 순서로 들어옴
        self.current_lidar_ranges = np.array(msg.ranges[0:360])

    def run(self):
        rospy.loginfo("Cone Lane Detector Node Running...")
        while not rospy.is_shutdown():
            if self.current_lidar_ranges is not None:
                xy_raw_points = convert_lidar_to_xy(self.current_lidar_ranges)
                
                s_left_coords_np, s_right_coords_np, center_coords_np, \
                l_found, r_found, l_deg, r_deg, \
                raw_left_cones_np, raw_right_cones_np = detect_lanes_from_cones(xy_raw_points)
                
                cone_lanes_msg = ConeLanes()
                cone_lanes_msg.header = std_msgs.msg.Header()
                cone_lanes_msg.header.stamp = rospy.Time.now()
                cone_lanes_msg.header.frame_id = "base_link" # 또는 라이다 센서 프레임

                # 피팅된 왼쪽 차선 정보
                cone_lanes_msg.left_lane_detected = l_found
                if s_left_coords_np.size > 0:
                    cone_lanes_msg.left_lane_points = [Point(x=p[0], y=p[1], z=0.0) for p in s_left_coords_np]
                else:
                    cone_lanes_msg.left_lane_points = []
                cone_lanes_msg.left_lane_degree = l_deg

                # 피팅된 오른쪽 차선 정보
                cone_lanes_msg.right_lane_detected = r_found
                if s_right_coords_np.size > 0:
                    cone_lanes_msg.right_lane_points = [Point(x=p[0], y=p[1], z=0.0) for p in s_right_coords_np]
                else:
                    cone_lanes_msg.right_lane_points = []
                cone_lanes_msg.right_lane_degree = r_deg
                
                # 중앙 경로 정보
                if center_coords_np.size > 0:
                    cone_lanes_msg.center_path = [Point(x=p[0], y=p[1], z=0.0) for p in center_coords_np]
                else:
                    cone_lanes_msg.center_path = []

                # === [수정됨] 원본 클러스터링된 콘 정보 추가 ===
                if raw_left_cones_np.size > 0:
                    cone_lanes_msg.raw_left_cones = [Point(x=p[0], y=p[1], z=0.0) for p in raw_left_cones_np]
                else:
                    cone_lanes_msg.raw_left_cones = []
                
                if raw_right_cones_np.size > 0:
                    cone_lanes_msg.raw_right_cones = [Point(x=p[0], y=p[1], z=0.0) for p in raw_right_cones_np]
                else:
                    cone_lanes_msg.raw_right_cones = []
                # ==========================================
                
                self.cone_lanes_publisher.publish(cone_lanes_msg)

                # 시각화 업데이트
                if ENABLE_MATPLOTLIB_VIZ:
                    clustered_viz = cluster_lidar_points(xy_raw_points, CONE_CLUSTER_RADIUS**2) # 시각화용 클러스터링 다시 호출
                    if clustered_viz.shape[0] > 0:
                        raw_lidar_points_plot.set_data(clustered_viz[:,0], clustered_viz[:,1])
                    else:
                        raw_lidar_points_plot.set_data([],[])
                    
                    if l_found and s_left_coords_np.shape[0] > 0:
                        left_lane_plot.set_data(s_left_coords_np[:,0], s_left_coords_np[:,1])
                    else:
                        left_lane_plot.set_data([],[])
                    
                    if r_found and s_right_coords_np.shape[0] > 0:
                        right_lane_plot.set_data(s_right_coords_np[:,0], s_right_coords_np[:,1])
                    else:
                        right_lane_plot.set_data([],[])

                    # 원본 콘 체인 시각화
                    if l_found and raw_left_cones_np.shape[0] > 0:
                        raw_left_lane_points_plot.set_data(raw_left_cones_np[:,0], raw_left_cones_np[:,1])
                    else:
                        raw_left_lane_points_plot.set_data([],[])

                    if r_found and raw_right_cones_np.shape[0] > 0:
                        raw_right_lane_points_plot.set_data(raw_right_cones_np[:,0], raw_right_cones_np[:,1])
                    else:
                        raw_right_lane_points_plot.set_data([],[])
                    
                    if center_coords_np.shape[0] > 0:
                        center_path_plot.set_data(center_coords_np[:,0], center_coords_np[:,1])
                    else:
                        center_path_plot.set_data([],[])
                    
                    try:
                        fig.canvas.draw_idle() # 이전 frame 업데이트
                        plt.pause(0.001)       # GUI 이벤트 처리 및 잠시 대기
                    except Exception: # 창이 닫혔을 경우 등 예외 처리
                        pass 
            
            self.node_rate.sleep()

if __name__ == '__main__':
    try:
        detector = ConeLaneDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Cone Lane Detector node terminated.")
    finally:
        if ENABLE_MATPLOTLIB_VIZ:
            plt.ioff() # 대화형 모드 끄기
            plt.close('all') # 모든 창 닫기