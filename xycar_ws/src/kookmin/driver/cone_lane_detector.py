#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from xycar_msgs.msg import ConeLanes
import std_msgs.msg
import matplotlib.pyplot as plt

# =============================================
# 파라미터 
# =============================================
LIDAR_ANGLE_RANGE_HALF = 95 
LIDAR_MAX_DIST = 12.0 
CONE_CLUSTER_RADIUS = 0.3 

CHAIN_INITIAL_SEED_Y_MAX = 4.0   
CHAIN_MAX_NEXT_CONE_DIST = 5.0
CHAIN_MAX_HEADING_CHANGE_RAD = np.radians(80)
CHAIN_MIN_POINTS_FOR_LANE = 2

EXPECTED_LANE_WIDTH_APPROX = 3.5 
X_SEED_CENTER_THRESHOLD = 0.05

ADD_COST_WEIGHT_DISTANCE = 2.0
ADD_COST_WEIGHT_ANGLE = 10.0
ADD_COST_WEIGHT_LATERAL = 2.0
MAX_ACCEPTABLE_CONNECTION_COST = 20.0 

# 제어 목표점 계산을 위한 Y좌표
TARGET_Y_FOR_CONTROL = 3.5

# ==========================================
ENABLE_MATPLOTLIB_VIZ = True 

if ENABLE_MATPLOTLIB_VIZ:
    fig, ax = plt.subplots(figsize=(6,3)) 
    if LIDAR_ANGLE_RANGE_HALF <= 90:
        plot_x_max_abs = LIDAR_MAX_DIST * math.sin(math.radians(LIDAR_ANGLE_RANGE_HALF))
    else:
        plot_x_max_abs = LIDAR_MAX_DIST
    ax.set_xlim(-(plot_x_max_abs + 0.5), plot_x_max_abs + 0.5)
    ax.set_ylim(-0.5, LIDAR_MAX_DIST + 0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Cone Lane Detector Viz")
    ax.set_xlabel("X (m) - Lateral")
    ax.set_ylabel("Y (m) - Forward")
    ax.grid(True)
    raw_lidar_points_plot, = ax.plot([], [], 'ko', markersize=3, label='Clustered Pts')
    left_lane_plot, = ax.plot([], [], 'r.-', markersize=5, linewidth=1.5, label='Left Lane Fit')
    right_lane_plot, = ax.plot([], [], 'b.-', markersize=5, linewidth=1.5, label='Right Lane Fit')
    center_path_plot, = ax.plot([], [], 'g--', linewidth=1.5, label='Center Path')
    raw_left_lane_points_plot, = ax.plot([], [], 'rx', markersize=8, markeredgewidth=2, label='Raw Left Cones')  
    raw_right_lane_points_plot, = ax.plot([], [], 'bx', markersize=8, markeredgewidth=2, label='Raw Right Cones') 
    target_point_plot, = ax.plot([], [], 'm*', markersize=5, markeredgewidth=1.5, label='Control Target')
    ax.legend(fontsize='small', loc='upper right')

# =============================================
# 유틸리티 함수
# =============================================
def convert_lidar_to_xy(current_ranges_data):
    if current_ranges_data is None or len(current_ranges_data) != 360: return np.array([]) 
    indices_left = np.arange(0, LIDAR_ANGLE_RANGE_HALF + 1); indices_right = np.arange(360 - LIDAR_ANGLE_RANGE_HALF, 360) 
    valid_angular_indices = np.unique(np.concatenate((indices_left, indices_right)))
    selected_ranges = current_ranges_data[valid_angular_indices]; selected_angles_rad = np.radians(valid_angular_indices.astype(float)) 
    valid_value_indices = np.where(np.isfinite(selected_ranges) & (selected_ranges < LIDAR_MAX_DIST))[0]
    if len(valid_value_indices) == 0: return np.array([])
    final_ranges = selected_ranges[valid_value_indices]; final_angles_rad = selected_angles_rad[valid_value_indices]
    x_coords = final_ranges * (-1) * np.sin(final_angles_rad) 
    y_coords = final_ranges * np.cos(final_angles_rad)
    return np.column_stack((x_coords, y_coords))

def cluster_lidar_points(points, cluster_radius_sq):
    if points.shape[0] == 0: return np.array([])
    n_points = points.shape[0]; processed_indices = np.zeros(n_points, dtype=bool); cluster_centers = []
    for i in range(n_points):
        if processed_indices[i]: continue
        current_cluster_members_indices = [i]; processed_indices[i] = True
        queue = [i]; head = 0
        while head < len(queue):
            seed_idx = queue[head]; head += 1
            for j in range(n_points):
                if processed_indices[j]: continue
                if np.sum((points[seed_idx] - points[j])**2) < cluster_radius_sq:
                    current_cluster_members_indices.append(j); processed_indices[j] = True; queue.append(j)
        if current_cluster_members_indices: cluster_centers.append(np.mean(points[current_cluster_members_indices], axis=0))
    return np.array(cluster_centers)

def fit_polynomial_to_lane(lane_points_xy_data, degree=1):
    min_pts_for_degree = degree + 1 
    if lane_points_xy_data is None or lane_points_xy_data.shape[0] < min_pts_for_degree: return None
    x_values = lane_points_xy_data[:, 0]; y_values = lane_points_xy_data[:, 1]
    try: poly_coeffs = np.polyfit(y_values, x_values, degree) 
    except Exception: poly_coeffs = None
    return poly_coeffs

def _calculate_angle_diff(vec1, vec2):
    angle1 = math.atan2(vec1[1], vec1[0]); angle2 = math.atan2(vec2[1], vec2[0])
    diff = angle2 - angle1
    while diff > math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    return diff

# =============================================
# 핵심 차선 감지 로직
# =============================================
def detect_lanes_from_cones(xy_coords_data_raw):
    clustered_cones = cluster_lidar_points(xy_coords_data_raw, CONE_CLUSTER_RADIUS**2)
    num_total_cones = clustered_cones.shape[0]
    sampled_left_lane_coords = []; sampled_right_lane_coords = []; center_path_coords = []
    left_lane_detected_flag = False; right_lane_detected_flag = False
    left_poly_deg = 0; right_poly_deg = 0
    raw_left_cones_for_plot = np.array([])
    raw_right_cones_for_plot = np.array([])
    target_point_detected = False
    target_point = None
    target_heading = 0.0
    lateral_error = 0.0 # =================> lateral_error 변수 추가 및 초기화

    if num_total_cones < CHAIN_MIN_POINTS_FOR_LANE:
        return np.array(sampled_left_lane_coords), np.array(sampled_right_lane_coords), \
               np.array(center_path_coords), left_lane_detected_flag, \
               right_lane_detected_flag, left_poly_deg, right_poly_deg, \
               raw_left_cones_for_plot, raw_right_cones_for_plot, \
               target_point_detected, target_point, target_heading, \
               lateral_error # =================> 반환 값에 lateral_error 추가

    is_cone_used = np.zeros(num_total_cones, dtype=bool)
    
    def build_one_lane_chain(start_cone_idx, all_cones, used_mask):
        if start_cone_idx is None or start_cone_idx < 0 or start_cone_idx >= len(all_cones) or used_mask[start_cone_idx]: return []
        current_chain = [all_cones[start_cone_idx]]; used_mask[start_cone_idx] = True
        last_added_cone = all_cones[start_cone_idx]; previous_segment_vector = None 
        while True:
            best_next_cone_idx = -1
            min_cost = float('inf') 

            for i in range(num_total_cones):
                if used_mask[i]: continue
                candidate_cone = all_cones[i]; vector_to_candidate = candidate_cone - last_added_cone
                dist_sq_to_candidate = np.sum(vector_to_candidate**2)
                lateral_distance_to_candidate = abs(vector_to_candidate[0]) 

                if dist_sq_to_candidate >= CHAIN_MAX_NEXT_CONE_DIST**2: continue
                
                angle_change = 0.0
                if previous_segment_vector is not None:
                    angle_change = _calculate_angle_diff(previous_segment_vector, vector_to_candidate)
                    if abs(angle_change) > CHAIN_MAX_HEADING_CHANGE_RAD:
                        continue 
                
                current_candidate_cost = 0.0
                current_candidate_cost += ADD_COST_WEIGHT_DISTANCE * dist_sq_to_candidate
                if previous_segment_vector is not None: 
                    current_candidate_cost += ADD_COST_WEIGHT_ANGLE * abs(angle_change)
                current_candidate_cost += ADD_COST_WEIGHT_LATERAL * lateral_distance_to_candidate
                
                if current_candidate_cost < min_cost:
                    min_cost = current_candidate_cost
                    best_next_cone_idx = i
            
            if best_next_cone_idx != -1:
                if min_cost > MAX_ACCEPTABLE_CONNECTION_COST:
                    best_next_cone_idx = -1
            
            if best_next_cone_idx != -1:
                new_cone_to_add = all_cones[best_next_cone_idx]; current_chain.append(new_cone_to_add)
                used_mask[best_next_cone_idx] = True
                if len(current_chain) > 1: 
                    previous_segment_vector = new_cone_to_add - last_added_cone 
                last_added_cone = new_cone_to_add 
            else: 
                break 
        return current_chain

    seed_candidates_in_y_zone = clustered_cones[clustered_cones[:, 1] <= CHAIN_INITIAL_SEED_Y_MAX]
    if seed_candidates_in_y_zone.shape[0] == 0: seed_candidates_in_y_zone = clustered_cones 
    left_seed_idx_in_clustered_cones = None; right_seed_idx_in_clustered_cones = None
    if seed_candidates_in_y_zone.shape[0] > 0:
        left_seed_pool = seed_candidates_in_y_zone[seed_candidates_in_y_zone[:, 0] < -X_SEED_CENTER_THRESHOLD]
        right_seed_pool = seed_candidates_in_y_zone[seed_candidates_in_y_zone[:, 0] > X_SEED_CENTER_THRESHOLD]
        if left_seed_pool.shape[0] > 0:
            sorted_left_seeds = left_seed_pool[np.lexsort((left_seed_pool[:, 0], left_seed_pool[:, 1]))]
            temp_idx = np.where((clustered_cones == sorted_left_seeds[0]).all(axis=1))[0]
            if temp_idx.size > 0: left_seed_idx_in_clustered_cones = temp_idx[0]
        if right_seed_pool.shape[0] > 0:
            sorted_right_seeds = right_seed_pool[np.lexsort((-right_seed_pool[:, 0], right_seed_pool[:, 1]))] 
            temp_idx = np.where((clustered_cones == sorted_right_seeds[0]).all(axis=1))[0]
            if temp_idx.size > 0: right_seed_idx_in_clustered_cones = temp_idx[0]

    final_left_lane_pts = build_one_lane_chain(left_seed_idx_in_clustered_cones, clustered_cones, is_cone_used)
    final_right_lane_pts = build_one_lane_chain(right_seed_idx_in_clustered_cones, clustered_cones, is_cone_used)
    
    raw_left_cones_for_plot = np.array(final_left_lane_pts)
    raw_right_cones_for_plot = np.array(final_right_lane_pts)

    left_poly_coeffs = None; right_poly_coeffs = None

    if raw_left_cones_for_plot.shape[0] >= CHAIN_MIN_POINTS_FOR_LANE:
        degree_left = min(3, raw_left_cones_for_plot.shape[0] - 1) 
        if degree_left > 0 : 
            left_poly_coeffs = fit_polynomial_to_lane(raw_left_cones_for_plot, degree=degree_left)
            if left_poly_coeffs is not None: left_lane_detected_flag = True; left_poly_deg = degree_left
    if raw_right_cones_for_plot.shape[0] >= CHAIN_MIN_POINTS_FOR_LANE:
        degree_right = min(3, raw_right_cones_for_plot.shape[0] - 1)
        if degree_right > 0: 
            right_poly_coeffs = fit_polynomial_to_lane(raw_right_cones_for_plot, degree=degree_right)
            if right_poly_coeffs is not None: right_lane_detected_flag = True; right_poly_deg = degree_right
    
    path_sample_y_values = np.linspace(0.1, LIDAR_MAX_DIST * 0.9, 15) 
    for y_val_sample in path_sample_y_values:
        x_left_on_poly = np.polyval(left_poly_coeffs, y_val_sample) if left_poly_coeffs is not None else None
        x_right_on_poly = np.polyval(right_poly_coeffs, y_val_sample) if right_poly_coeffs is not None else None
        if x_left_on_poly is not None: sampled_left_lane_coords.append([x_left_on_poly, y_val_sample])
        if x_right_on_poly is not None: sampled_right_lane_coords.append([x_right_on_poly, y_val_sample])
        if x_left_on_poly is not None and x_right_on_poly is not None:
            if EXPECTED_LANE_WIDTH_APPROX*0.2 < (x_right_on_poly-x_left_on_poly) < EXPECTED_LANE_WIDTH_APPROX*2.5 : 
                 center_path_coords.append([(x_left_on_poly+x_right_on_poly)/2.0, y_val_sample])
        elif left_poly_coeffs is not None and x_left_on_poly is not None: 
            center_path_coords.append([x_left_on_poly + EXPECTED_LANE_WIDTH_APPROX/2.0, y_val_sample])
        elif right_poly_coeffs is not None and x_right_on_poly is not None: 
            center_path_coords.append([x_right_on_poly - EXPECTED_LANE_WIDTH_APPROX/2.0, y_val_sample])
    
    center_path_np = np.array(center_path_coords)
    if center_path_np.shape[0] > 1:
        # =================> lateral_error 계산 로직 추가
        # 중앙 경로의 첫번째 포인트(y가 0에 가장 가까움)의 x값을 횡방향 오차로 사용
        lateral_error = center_path_np[0, 0]
        # =================>

        y_diffs = np.abs(center_path_np[:, 1] - TARGET_Y_FOR_CONTROL)
        closest_idx = np.argmin(y_diffs)
        target_point_candidate = center_path_np[closest_idx]
        
        target_point_candidate[0] = max(-5.0, min(target_point_candidate[0], 5.0))

        if closest_idx > 0:
            dx = target_point_candidate[0]
            dy = target_point_candidate[1]
            final_heading = math.atan2(dx, dy)
            
            target_point_detected = True
            target_point = target_point_candidate
            target_heading = final_heading
    
    return np.array(sampled_left_lane_coords), np.array(sampled_right_lane_coords), \
           np.array(center_path_coords), left_lane_detected_flag, \
           right_lane_detected_flag, left_poly_deg, right_poly_deg, \
           raw_left_cones_for_plot, raw_right_cones_for_plot, \
           target_point_detected, target_point, target_heading, \
           lateral_error # =================> 반환 값에 lateral_error 추가

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
            plt.ion()
            plt.show()
        rospy.loginfo("Cone Lane Detector Node Initialized.")

    def lidar_callback(self, msg):
        self.current_lidar_ranges = np.array(msg.ranges[0:360])

    def run(self):
        rospy.loginfo("Cone Lane Detector Node Running...")
        while not rospy.is_shutdown():
            if self.current_lidar_ranges is not None:
                xy_raw_points = convert_lidar_to_xy(self.current_lidar_ranges)
                
                # =================> lateral_err 변수 추가
                s_left_coords_np, s_right_coords_np, center_coords_np, \
                l_found, r_found, l_deg, r_deg, \
                raw_left_cones_np, raw_right_cones_np, \
                tgt_detected, tgt_point, tgt_heading, lateral_err = detect_lanes_from_cones(xy_raw_points)
                
                cone_lanes_msg = ConeLanes()
                cone_lanes_msg.header = std_msgs.msg.Header()
                cone_lanes_msg.header.stamp = rospy.Time.now()
                cone_lanes_msg.header.frame_id = "base_link" 

                cone_lanes_msg.left_lane_detected = l_found
                if s_left_coords_np.size > 0:
                    cone_lanes_msg.left_lane_points = [Point(x=p[0], y=p[1], z=0.0) for p in s_left_coords_np]
                cone_lanes_msg.left_lane_degree = l_deg

                cone_lanes_msg.right_lane_detected = r_found
                if s_right_coords_np.size > 0:
                    cone_lanes_msg.right_lane_points = [Point(x=p[0], y=p[1], z=0.0) for p in s_right_coords_np]
                cone_lanes_msg.right_lane_degree = r_deg
                
                if center_coords_np.size > 0:
                    cone_lanes_msg.center_path = [Point(x=p[0], y=p[1], z=0.0) for p in center_coords_np]
                
                # =================> 메시지에 lateral_error 값 채우기
                cone_lanes_msg.lateral_error = lateral_err
                # =================>

                cone_lanes_msg.target_point_detected = tgt_detected
                if tgt_detected:
                    cone_lanes_msg.target_point = Point(x=tgt_point[0], y=tgt_point[1], z=0.0)
                    cone_lanes_msg.target_heading = tgt_heading
                    rospy.loginfo(f"[Target Info] Point(x,y): ({tgt_point[0]:.2f}, {tgt_point[1]:.2f}), Heading: {tgt_heading:.3f} rad, Lateral Err: {lateral_err:.3f} m")
                
                self.cone_lanes_publisher.publish(cone_lanes_msg)

                if ENABLE_MATPLOTLIB_VIZ:
                    clustered_viz = cluster_lidar_points(xy_raw_points, CONE_CLUSTER_RADIUS**2)
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

                    if tgt_detected:
                        target_point_plot.set_data([tgt_point[0]], [tgt_point[1]])
                    else:
                        target_point_plot.set_data([], [])
                    
                    try:
                        fig.canvas.draw_idle()
                        plt.pause(0.001)
                    except Exception: 
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
            plt.ioff()
            plt.close('all')