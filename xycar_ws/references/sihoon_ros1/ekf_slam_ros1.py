#!/usr/bin/env pythonimu
# -*- coding: utf-8 -*-

from __future__ import division
import rospy
import numpy as np
import math
import pyqtgraph as pg # pyqtgraph는 ROS1에서도 사용 가능합니다.

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry # Odometry 메시지 타입은 IMU로부터 추정된 값을 담기 위해 유지할 수 있으나, 실제 구독은 Imu로 합니다.
                                # 또는 사용자 정의 메시지를 사용할 수도 있습니다. 여기서는 Odometry 구조를 활용해 유사하게 만듭니다.

import slam_utils_ros1 as slam_utils # ROS1 버전으로 수정된 파일 임포트
import tree_extraction_ros1 as tree_extraction # ROS1 버전으로 수정된 파일 임포트
from scipy.stats.distributions import chi2
import message_filters # ROS1의 message_filters 사용

##############################################
# 1. SLAM 관련 함수들 (ROS2 코드와 거의 동일)
##############################################

def motion_model_nonlinear(u, dt, ekf_state, vehicle_params):
    """
    비선형 모션 모델.
    v,w = 제어입력, (x,y,phi) = 현재 로봇 상태
    """
    v = u[0]
    w = u[1]

    x_state = ekf_state['x'].copy()
    x0, y0, phi0 = x_state[0], x_state[1], x_state[2]

    if abs(w) < 1e-9:
        el1 = v * dt * math.cos(phi0)
        el2 = v * dt * math.sin(phi0)
        el3 = w * dt
    else:
        el1 = -(v/w) * math.sin(phi0) + (v/w) * math.sin(phi0 + w*dt)
        el2 =  (v/w) * math.cos(phi0) - (v/w) * math.cos(phi0 + w*dt)
        el3 =  w * dt

    motion = np.array([[el1],
                       [el2],
                       [slam_utils.clamp_angle(el3)]])

    if abs(w) < 1e-9:
        G_3x3 = np.array([
            [1.0, 0.0, -dt*v*math.sin(phi0)],
            [0.0, 1.0,  dt*v*math.cos(phi0)],
            [0.0, 0.0, 1.0]
        ])
    else:
        dtheta = w*dt
        G_3x3 = np.array([
            [1, 0,  (v/w)*(math.cos(phi0) - math.cos(phi0+dtheta))],
            [0, 1,  (v/w)*(math.sin(phi0+dtheta) - math.sin(phi0))],
            [0, 0,  1]
        ])
    return motion, G_3x3


def odom_predict_from_imu(u, dt, ekf_state, vehicle_params, alpha_params):
    """
    예측단계 (IMU 기반). '동적' 프로세스 잡음(Q_k)을 문헌 방식으로 구성.
    ROS2의 odom_predict와 유사하나, sigmas 파라미터는 laser_update에서 사용되므로 제거.
    """
    t_st = ekf_state['x'].copy().reshape(-1, 1)
    t_cov = ekf_state['P'].copy()
    
    # 차원 불일치 수정: 상태 벡터가 아닌 공분산 행렬 크기에 맞춰 dim_features 계산
    # dim_features = t_st.shape[0] - 3  # 이전 코드: 랜드마크 피처의 차원 (각 랜드마크는 x,y 2차원)
    dim_features = t_cov.shape[0] - 3  # 수정: 공분산 행렬 크기 기반으로 계산

    # 로봇 상태+랜드마크 중, 로봇 부분만 뽑아내는 블록 행렬
    F_x = np.hstack((np.eye(3), np.zeros((3, dim_features))))

    mot, G_3x3 = motion_model_nonlinear(u, dt, ekf_state, vehicle_params)

    Gt_1 = np.hstack((G_3x3, np.zeros((3, dim_features))))
    Gt_2 = np.hstack((np.zeros((dim_features, 3)), np.eye(dim_features)))
    Gt   = np.vstack((Gt_1, Gt_2))

    v, w = u[0], u[1]
    alpha1, alpha2, alpha3, alpha4 = alpha_params['alpha1'], alpha_params['alpha2'], alpha_params['alpha3'], alpha_params['alpha4']

    # 프로세스 노이즈 공분산 Q_k (모션에 따라 변함)
    # 이 부분은 오도메트리 입력 자체의 노이즈를 모델링
    # M_k in Probabilistic Robotics Table 5.5
    # 여기서는 Thrun의 책 Table 5.6의 단순화된 형태와 유사하게 v, w에 대한 노이즈로 구성
    # (alpha1*v^2 + alpha2*w^2) 등이 실제로는 회전, 이동, 최종 회전에 대한 노이즈 분산으로 표현됨
    # V_3x2는 제어 공간에서 상태 공간으로의 매핑 자코비안과 유사한 역할
    # R_big는 Q_k를 전체 상태 공분산에 더하기 위한 확장 행렬
    
    # 제어 입력 u=(v,w)에 대한 노이즈 모델링 (M_k in Thrun)
    # 실제로는 더 복잡한 모델이 있지만, 여기서는 각 제어 입력에 대한 분산으로 가정
    # Q_control_space = diag(alpha1*v^2 + alpha2*w^2, alpha3*v^2 + alpha4*w^2)
    # V_k는 u에 대한 f의 편미분 (로봇 모션 모델에 따라 다름)
    # 여기서는 단순화된 V_k 사용, 실제로는 motion_model_nonlinear의 입력 u에 대한 자코비안
    
    # 수정: Thrun 책의 식 (5.26)에 따른 정확한 제어 자코비안 V_3x2 구현
    phi0 = ekf_state['x'][2]
    if abs(w) < 1e-9:  # 직선 운동 (w ≈ 0)
        V_3x2 = np.array([
            [dt*math.cos(phi0),  0],
            [dt*math.sin(phi0),  0],
            [0,                  dt]
        ])
    else:  # 곡선 운동 (w ≠ 0)
        # Thrun 책 식 (5.26)에 따른 정확한 자코비안
        r = v/w  # 회전 반경
        dtheta = w*dt  # 각도 변화
        
        V_3x2 = np.array([
            # df1/dv (x 좌표에 대한 v의 영향)
            [(-math.sin(phi0) + math.sin(phi0 + dtheta))/w,
            # df1/dw (x 좌표에 대한 w의 영향)
             (v*(math.sin(phi0) - math.sin(phi0 + dtheta))/w**2) + 
             (v*math.cos(phi0 + dtheta)*dt/w)],
            
            # df2/dv (y 좌표에 대한 v의 영향)
            [(math.cos(phi0) - math.cos(phi0 + dtheta))/w,
            # df2/dw (y 좌표에 대한 w의 영향)
             (v*(math.cos(phi0 + dtheta) - math.cos(phi0))/w**2) + 
             (v*math.sin(phi0 + dtheta)*dt/w)],
            
            # df3/dv, df3/dw (방향에 대한 영향)
            [0, dt]
        ])


    # 제어 입력의 노이즈 (제어 공간에서)
    Q_control_space = np.diag([alpha1*(v**2) + alpha2*(w**2),  # v에 대한 분산
                               alpha3*(v**2) + alpha4*(w**2)]) # w에 대한 분산 (Thrun 책에서는 보통 alpha3*trans + alpha4*rot)
                                                              # 여기서는 v,w로 표현된 것을 사용

    # 상태 공간에서의 프로세스 노이즈 (로봇의 포즈에만 해당)
    process_noise_robot_pose = V_3x2 @ Q_control_space @ V_3x2.T

    # 전체 상태 벡터에 대한 프로세스 노이즈 (랜드마크 부분은 0)
    R_big = np.zeros((3+dim_features, 3+dim_features))
    R_big[0:3, 0:3] = process_noise_robot_pose

    new_cov = Gt @ t_cov @ Gt.T + R_big
    new_cov = slam_utils.make_symmetric(new_cov)

    new_x = t_st + F_x.T @ mot
    new_x[2] = slam_utils.clamp_angle(new_x[2]) # 각도 정규화

    ekf_state['x'] = new_x.flatten()
    ekf_state['P'] = new_cov

    return ekf_state


def laser_measurement_model(ekf_state, landmark_id):
    """ 랜드마크 관측 모델 및 자코비안 H 계산 """
    t_st = ekf_state['x'].copy()
    t_st[2] = slam_utils.clamp_angle(t_st[2]) # 로봇 방향 정규화
    dim_total = t_st.shape[0] # 전체 상태 벡터 크기 (x, y, phi + 2*num_landmarks)

    # 로봇의 상태
    r_x, r_y, phi = t_st[0], t_st[1], t_st[2]
    # 특정 랜드마크의 상태 (landmark_id는 0부터 시작)
    m_x = t_st[3 + 2*landmark_id]
    m_y = t_st[4 + 2*landmark_id]

    # 예상 관측 계산 (z_hat = [range, bearing])
    del_x = m_x - r_x
    del_y = m_y - r_y
    q = del_x**2 + del_y**2
    sqrt_q = np.sqrt(q)
    
    # 예상 거리
    zhat_range = sqrt_q
    # 예상 방위각 (랜드마크 방향 - 로봇 방향)
    zhat_bearing = slam_utils.clamp_angle(np.arctan2(del_y, del_x) - phi)
    
    zhat = np.array([[zhat_range], [zhat_bearing]])

    # 관측 모델의 자코비안 H 계산
    # H는 전체 상태 벡터 t_st에 대한 편미분 [dh/dx, dh/dy, dh/dphi, ..., dh/dmx_j, dh/dmy_j, ...]
    # 여기서는 로봇 상태 (x,y,phi)와 해당 랜드마크 (mx_j, my_j)에 대한 부분만 계산하고,
    # F_x_j를 사용해 전체 H로 확장
    
    # h_small은 [r_x, r_y, phi, m_x, m_y] 5개 변수에 대한 편미분
    # (dh_range/drx, dh_range/dry, dh_range/dphi, dh_range/dmx, dh_range/dmy)
    # (dh_bearing/drx, dh_bearing/dry, dh_bearing/dphi, dh_bearing/dmx, dh_bearing/dmy)
    h_small = np.zeros((2,5))
    if q == 0: # 매우 드문 경우 (로봇과 랜드마크가 정확히 같은 위치)
        # 자코비안 정의가 어려우므로, 작은 값으로 대체하거나 에러 처리
        # 여기서는 0으로 두면 이후 연산에서 문제가 될 수 있으므로, 이전 코드의 형태를 따름
        # (이전 코드에서는 q로 나누므로, q=0이면 에러 발생. 실제로는 이런 상황 방지 필요)
        # 여기서는 q가 0이 아니라고 가정하고 진행 (실제로는 매우 작은 값 epsilon으로 대체)
        # 이전 코드에서는 q로 나누므로, 이 부분은 문제가 될 수 있음.
        # sqrt_q * del_x / q => del_x / sqrt_q
        # 안전하게 하려면 q가 매우 작을 때 (e.g., < 1e-9) 처리를 추가해야 함.
        # 여기서는 원본 코드의 구조를 따름 (q가 0이 아니라고 가정)
        pass # 아래 계산을 그대로 진행

    h_small[0,0] = -del_x / sqrt_q
    h_small[0,1] = -del_y / sqrt_q
    h_small[0,2] = 0
    h_small[0,3] = del_x / sqrt_q
    h_small[0,4] = del_y / sqrt_q

    h_small[1,0] = del_y / q
    h_small[1,1] = -del_x / q
    h_small[1,2] = -1
    h_small[1,3] = -del_y / q
    h_small[1,4] = del_x / q
    
    # F_x_j: 로봇 상태와 j번째 랜드마크 상태를 전체 상태 벡터에서 선택하는 행렬
    F_x_j = np.zeros((5, dim_total))
    F_x_j[0:3, 0:3] = np.eye(3) # 로봇 상태 (x,y,phi) 부분
    F_x_j[3, 3 + 2*landmark_id] = 1 # j번째 랜드마크의 x 좌표
    F_x_j[4, 4 + 2*landmark_id] = 1 # j번째 랜드마크의 y 좌표

    H = h_small @ F_x_j
    return zhat, H


def initialize_landmark(ekf_state, tree_measurement, t_now, sigmas_init):
    """ 새로운 랜드마크를 상태 벡터와 공분산 행렬에 추가 """
    # tree_measurement: [range, bearing, diameter]
    r_x, r_y, phi = ekf_state['x'][0], ekf_state['x'][1], ekf_state['x'][2]
    m_r, m_th = tree_measurement[0], tree_measurement[1] # 거리와 방위각 사용

    # 랜드마크의 전역 좌표 계산
    m_x = r_x + m_r * np.cos(m_th + phi)
    m_y = r_y + m_r * np.sin(m_th + phi)
    
    # 상태 벡터 확장
    ekf_state['x'] = np.hstack((ekf_state['x'], m_x, m_y))

    # 공분산 행렬 확장
    # 새로운 랜드마크의 초기 불확실성은 sigmas_init 값을 사용
    # 기존 공분산 행렬의 크기
    dim_old = ekf_state['P'].shape[0]
    # 새로운 랜드마크에 대한 공분산 (2x2)
    # 여기서는 간단히 대각 행렬로 초기화, sigmas_init는 [sigma_x^2, sigma_y^2] 또는 단일 값 sigma^2
    # 원본 ROS2 코드는 0.3으로 고정값을 사용했음. 이를 따르거나 파라미터화 가능.
    # 여기서는 sigmas_init 파라미터를 추가하여 유연성 확보
    # 만약 sigmas_init이 단일 값이면 제곱해서 사용, 두개면 각각 사용
    if isinstance(sigmas_init, (list, tuple)) and len(sigmas_init) == 2:
        P_new_landmark = np.diag(sigmas_init)
    else: # 단일 값으로 가정 (분산 값)
        P_new_landmark = np.diag([sigmas_init, sigmas_init])


    # 기존 P 행렬에 새로운 랜드마크 부분 추가 (0으로 채움)
    P_temp = np.zeros((dim_old + 2, dim_old + 2))
    P_temp[:dim_old, :dim_old] = ekf_state['P']
    # 새로운 랜드마크의 초기 공분산 설정
    P_temp[dim_old:dim_old+2, dim_old:dim_old+2] = P_new_landmark
    
    # (선택적) 로봇 포즈와 새 랜드마크 간의 상관관계 초기화
    # G_r (로봇 포즈에 대한 g의 자코비안), G_z (관측에 대한 g의 자코비안)
    # P_xr,m = G_r @ P_robot @ G_z.T (Thrun 책 참고)
    # 여기서는 단순화를 위해 0으로 초기화된 것을 사용 (ROS2 코드 방식)
    # 보다 정확하게 하려면, 초기화 시점의 로봇-랜드마크 간 상관관계 계산 필요
    # J_xr = [[1, 0, -m_r*sin(phi+m_th)], [0, 1, m_r*cos(phi+m_th)]] (dxm/dxr, dym/dyr)
    # J_z = [[cos(phi+m_th), -m_r*sin(phi+m_th)], [sin(phi+m_th), m_r*cos(phi+m_th)]] (dxm/dr, dym/d_beta)
    # P_robot = ekf_state['P'][:3,:3]
    # Sigma_landmark_obs = np.diag([sigmas['range']**2, sigmas['bearing']**2])
    # P_mm = J_xr @ P_robot @ J_xr.T + J_z @ Sigma_landmark_obs @ J_z.T
    # P_temp[dim_old:dim_old+2, dim_old:dim_old+2] = P_mm
    # P_rm = J_xr @ P_robot
    # P_temp[0:3, dim_old:dim_old+2] = P_rm.T
    # P_temp[dim_old:dim_old+2, 0:3] = P_rm
    # 위 방식이 더 정확하나, ROS2 코드는 단순 초기화였으므로 우선 해당 방식 유지

    ekf_state['P'] = P_temp
    ekf_state['num_landmarks'] += 1
    ekf_state['landmark_last_seen'].append(t_now)
    ekf_state['landmark_inflated'].append(False)

    ekf_state['x'] = ekf_state['x'].flatten()
    return ekf_state


def compute_data_association(ekf_state, measurements, sigmas, params):
    """ 데이터 연관: 측정된 랜드마크와 기존 맵의 랜드마크를 매칭 """
    if ekf_state["num_landmarks"] == 0: # 맵에 랜드마크가 없으면 모든 측정은 새 랜드마크 후보
        return [-1 for _ in measurements] # -1: 새 랜드마크로 초기화

    n_lmark = ekf_state['num_landmarks']
    n_scans = len(measurements) # measurements는 tree_extraction 결과 [[r1,b1,d1], [r2,b2,d2]...]

    # 마할라노비스 거리 행렬 M (n_scans x n_lmark)
    M = np.zeros((n_scans, n_lmark))
    # 관측 노이즈 공분산 R (또는 Q_t in ROS2 code)
    Q_t = np.array([
        [sigmas['range']**2, 0],
        [0,                  sigmas['bearing']**2]
    ])

    # 게이팅을 위한 카이제곱 임계값
    # 수정: 통계적으로 합리적인 신뢰구간 사용 (95%와 99%)
    # alpha: 기존 랜드마크와의 연관을 위한 임계값 (95% -> chi2.ppf(0.95, 2) ≈ 5.991)
    # beta: 새 랜드마크로 초기화할지, 버릴지 결정하는 임계값 (더 엄격, 99% -> chi2.ppf(0.99, 2) ≈ 9.21)
    # 이전: 0.99999995와 0.99999999 (거의 무제한)
    gating_threshold_association = chi2.ppf(params.get("gating_alpha", 0.95), df=2)  # df=2 (거리, 방위각)
    gating_threshold_new_landmark = chi2.ppf(params.get("gating_beta", 0.99), df=2)  # 더 큰 값 (더 엄격)

    for j in range(n_lmark): # 각 기존 랜드마크에 대해
        zhat, H = laser_measurement_model(ekf_state, j)
        # 혁신 공분산 S = H P H^T + Q_t
        S = H @ ekf_state['P'] @ H.T + Q_t
        try:
            Sinv = np.linalg.inv(S) # slam_utils.invert_2x2_matrix(S) 사용 가능
        except np.linalg.LinAlgError: # 특이 행렬인 경우
            rospy.logwarn("S matrix is singular in data association for landmark %d. Skipping.", j)
            M[:,j] = np.inf # 이 랜드마크와는 연관시키지 않음
            continue


        for i in range(n_scans): # 각 측정에 대해
            # 측정값 z = [range, bearing]
            z_measured = np.array([[measurements[i][0]], [measurements[i][1]]])
            # 혁신(잔차) nu = z - z_hat
            innovation = z_measured - zhat
            innovation[1] = slam_utils.clamp_angle(innovation[1]) # 방위각 차이 정규화
            
            # 마할라노비스 거리 d^2 = nu^T S^-1 nu
            M[i, j] = innovation.T @ Sinv @ innovation

    # 최근접 이웃 (Nearest Neighbor) 또는 최적 할당 (e.g., Hungarian algorithm)
    # ROS2 코드는 solve_cost_matrix_heuristic 사용 (일종의 탐욕적 최근접 이웃)
    # M_new는 새로운 랜드마크를 위한 열을 추가 (비용은 alpha)
    # A = gating_threshold_association * np.ones((n_scans, n_scans)) # ROS2 코드 방식
    # M_new = np.hstack((M, A))
    # pairs = slam_utils.solve_cost_matrix_heuristic(M_new)
    # pairs.sort()
    # assoc = [-1] * n_scans
    # for p in pairs:
    #     scan_idx, landmark_idx = p[0], p[1]
    #     if landmark_idx < n_lmark: # 기존 랜드마크와 매칭
    #         if M[scan_idx, landmark_idx] < gating_threshold_association:
    #             assoc[scan_idx] = landmark_idx
    #         # else: 매칭 실패 (새 랜드마크 후보로 남거나 버려짐)
    #     # else: landmark_idx >= n_lmark 이면 새 랜드마크로 할당된 것 (heuristic 방식)
    #     # 이 부분은 heuristic solver의 동작에 따라 해석이 달라짐.
    #     # 여기서는 간단한 Global Nearest Neighbor with Gating
    
    assoc = [-1] * n_scans # 초기에는 모두 새 랜드마크(-1) 또는 미연관(-2)
    # 각 측정에 대해 가장 가능성 높은 기존 랜드마크 찾기
    for i in range(n_scans):
        min_dist = gating_threshold_association
        best_lmk_idx = -1
        for j in range(n_lmark):
            if M[i,j] < min_dist:
                min_dist = M[i,j]
                best_lmk_idx = j
        
        if best_lmk_idx != -1:
            assoc[i] = best_lmk_idx # 연관 성공
        else: # 가장 가까운 기존 랜드마크도 게이팅 통과 못함
              # 이 측정값이 새 랜드마크가 될 가능성이 있는지 확인
              # (이 부분은 ROS2 코드의 heuristic solver와 다름, 여기서는 명시적 beta 체크 없음)
              # ROS2 코드의 -2 의미: "어떤 기존 랜드마크와도 연관되지 않았지만, 새 랜드마크로 초기화하기에는 beta 임계값을 넘는 경우"
              # 여기서는 일단 -1 (새 랜드마크 후보)로 두고, laser_update에서 처리하거나,
              # 또는 여기서 beta 임계값을 사용해 -2 (버림)으로 결정할 수 있음.
              # ROS2 코드의 M[i,j] < beta 체크를 모방하려면:
            is_potential_new = True # 기본적으로는 새 랜드마크 후보
            # 만약 어떤 기존 랜드마크 j에 대해 M[i,j] < gating_threshold_new_landmark 라면,
            # 혼동될 수 있으므로 새 랜드마크로 만들지 않거나 (-2), 또는 가장 가까운 것과 연관 시도할 수 있음.
            # ROS2 코드의 로직:
            # 1. alpha로 heuristic solve (M_new 사용)
            # 2. assoc[i] == -1 (새 랜드마크로 할당된 경우)는 그대로 -1
            # 3. assoc[i]가 기존 랜드마크인데 alpha 넘으면 다시 -1로 (이건 위의 GNN에서 이미 처리됨)
            # 4. 최종적으로 assoc[i] == -1 인 것들에 대해, 만약 어떤 기존 j에 대해 M[i,j] < beta 이면 assoc[i] = -2
            # 여기서는 GNN을 사용했으므로, assoc[i] == -1 이면 어떤 기존 랜드마크와도 alpha 이내로 연관 안됨.
            # 이 때, 이 측정값이 어떤 기존 랜드마크와 beta 이내로 가까운지 확인.
            is_too_close_to_existing_for_new = False
            if assoc[i] == -1: # 새 랜드마크 후보인 경우
                for j_lmk in range(n_lmark):
                    if M[i, j_lmk] < gating_threshold_new_landmark:
                        is_too_close_to_existing_for_new = True
                        break
                if is_too_close_to_existing_for_new:
                    assoc[i] = -2 # 새 랜드마크로 부적합 (기존 것과 너무 가까움)
                # else: assoc[i]는 여전히 -1 (새 랜드마크로 적합)

    return assoc


def laser_update(trees, assoc, ekf_state, sigmas, params, t_now):
    """ EKF 업데이트 단계: 연관된 랜드마크 관측을 사용해 상태와 공분산 업데이트 """
    # 관측 노이즈 공분산
    Q_t = np.array([
        [sigmas['range']**2,   0],
        [0,                    sigmas['bearing']**2]
    ])

    # Iterated EKF 파라미터 (선택적)
    max_iter = params.get("iekf_max_iter", 1) # 기본값 1 (일반 EKF)
    eps_converge = params.get("iekf_eps_converge", 1e-7)
    
    # 초기 랜드마크 분산 (initialize_landmark에서 사용)
    # ROS2에서는 0.3 고정값. 여기서는 파라미터화.
    init_landmark_cov_val = params.get("init_landmark_cov_val", 0.3)


    for i, tree_meas in enumerate(trees): # 각 측정된 트리에 대해
        landmark_assoc_idx = assoc[i] # 데이터 연관 결과

        if landmark_assoc_idx == -1: # 새 랜드마크로 초기화
            ekf_state = initialize_landmark(ekf_state, tree_meas, t_now, init_landmark_cov_val)
            # 방금 추가된 랜드마크의 인덱스
            # 수정: 초기화 후 바로 이 관측으로 업데이트 수행
            landmark_assoc_idx = ekf_state['num_landmarks'] - 1
            # continue 제거: 초기화 후 바로 업데이트 진행

        elif landmark_assoc_idx == -2: # 유효하지 않은 측정 (버림)
            continue
        
        # landmark_assoc_idx는 기존 랜드마크의 인덱스 (0부터 시작)
        # Iterated EKF 루프 (max_iter = 1 이면 일반 EKF와 동일)
        x_iter = ekf_state['x'].copy() # 반복 계산용 상태
        P_iter = ekf_state['P'].copy() # 반복 계산용 공분산
        
        # 측정값 z
        z_measured = np.array([[tree_meas[0]], [tree_meas[1]]]) # [range, bearing]

        for iter_count in range(max_iter):
            # 현재 반복 상태(x_iter)를 기준으로 zhat, H 계산
            # 이 부분이 중요: IEKF는 업데이트된 x_iter를 사용해 H를 다시 계산
            temp_ekf_state_for_model = {'x': x_iter, 'P': P_iter} # laser_measurement_model에 전달할 임시 상태
            z_hat, H = laser_measurement_model(temp_ekf_state_for_model, landmark_assoc_idx)

            # 칼만 게인 K 계산
            S = H @ P_iter @ H.T + Q_t # P_iter 사용
            try:
                K = P_iter @ H.T @ np.linalg.inv(S)
            except np.linalg.LinAlgError:
                rospy.logwarn("S matrix is singular during laser_update for landmark %d. Skipping update.", landmark_assoc_idx)
                break # 이 랜드마크 업데이트 중단

            # 상태 업데이트
            innovation = z_measured - z_hat
            innovation[1] = slam_utils.clamp_angle(innovation[1]) # 방위각 차이 정규화
            
            x_new_iter = x_iter + K @ innovation
            x_new_iter[2] = slam_utils.clamp_angle(x_new_iter[2]) # 로봇 방향 정규화

            # 공분산 업데이트 (Joseph form for stability, or standard (I-KH)P)
            I_KH = np.eye(x_iter.shape[0]) - K @ H
            P_new_iter = I_KH @ P_iter @ I_KH.T + K @ Q_t @ K.T # Joseph form
            # P_new_iter = (np.eye(x_iter.shape[0]) - K @ H) @ P_iter # Standard form
            P_new_iter = slam_utils.make_symmetric(P_new_iter)

            # IEKF 수렴 조건 체크
            if max_iter > 1:
                # x_new_iter와 x_iter의 차이가 작으면 수렴
                if np.linalg.norm(x_new_iter - x_iter) < eps_converge:
                    x_iter = x_new_iter
                    P_iter = P_new_iter
                    break
            
            x_iter = x_new_iter
            P_iter = P_new_iter
            # 루프 마지막이면 최종값으로 확정됨

        # 최종 업데이트된 상태와 공분산을 ekf_state에 반영
        ekf_state['x'] = x_iter.flatten()
        ekf_state['P'] = P_iter
        
        # 랜드마크 관리 정보 업데이트
        ekf_state['landmark_last_seen'][landmark_assoc_idx] = t_now
        ekf_state['landmark_inflated'][landmark_assoc_idx] = False # 관측되었으므로 inflation 리셋

    return ekf_state

##############################################
# 2. ROS1 노드: EKFSLAMNodeROS1
##############################################
class EKFSLAMNodeROS1:
    def __init__(self):
        rospy.init_node('ekf_slam_node_ros1', anonymous=True)

        # 파라미터 로드 (rospy.get_param 사용)
        self.vehicle_params = {
            "a": rospy.get_param("~vehicle_params/a", 0.13),
            "b": rospy.get_param("~vehicle_params/b", 0.0),
            "L": rospy.get_param("~vehicle_params/L", 0.09),
            "H": rospy.get_param("~vehicle_params/H", 0.07)
        }

        self.filter_params = {
            "max_laser_range": rospy.get_param("~filter_params/max_laser_range", 10.0), # tree_extraction의 M11과 유사 역할
            "do_plot": rospy.get_param("~filter_params/do_plot", True),
            "plot_raw_laser": rospy.get_param("~filter_params/plot_raw_laser", True),
            "plot_map_covariances": rospy.get_param("~filter_params/plot_map_covariances", True),
            "gating_alpha": rospy.get_param("~filter_params/gating_alpha", 0.95), # 데이터 연관용
            "gating_beta": rospy.get_param("~filter_params/gating_beta", 0.99),   # 새 랜드마크 판별용
            "iekf_max_iter": rospy.get_param("~filter_params/iekf_max_iter", 1),
            "init_landmark_cov_val": rospy.get_param("~filter_params/init_landmark_cov_val", 0.3) # 새 랜드마크 초기 분산
        }

        self.sigmas = { # 관측 노이즈 표준편차
            "range": rospy.get_param("~sigmas/range", 0.1), # ROS2 코드 0.01에서 현실적인 값으로 변경 가능
            "bearing": rospy.get_param("~sigmas/bearing", np.deg2rad(2.0)) # 2도
        }
        # IMU 기반 예측을 위한 모션 노이즈 파라미터 (alpha 값들)
        self.alpha_params = {
            "alpha1": rospy.get_param("~alpha_params/alpha1", 0.01), # v^2에 의한 영향 (주로 전진 방향)
            "alpha2": rospy.get_param("~alpha_params/alpha2", 0.01), # w^2에 의한 영향 (주로 전진 방향)
            "alpha3": rospy.get_param("~alpha_params/alpha3", 0.001),# v^2에 의한 영향 (주로 회전 방향)
            "alpha4": rospy.get_param("~alpha_params/alpha4", 0.01) # w^2에 의한 영향 (주로 회전 방향)
        }

        self.reinit_params = { # 오랫동안 안보인 랜드마크 불확실성 증가
            "lost_threshold": rospy.get_param("~reinit_params/lost_threshold", 15.0), # 초
            "scale_factor": rospy.get_param("~reinit_params/scale_factor", 5.0)      # 분산 증가 배율
        }

        # EKF 상태 초기화
        initial_x = rospy.get_param("~initial_pose/x", 0.0)
        initial_y = rospy.get_param("~initial_pose/y", 0.0)
        initial_phi_deg = rospy.get_param("~initial_pose/phi_deg", 0.0) # ROS2는 36도였음
        
        self.ekf_state = {
            "x": np.array([initial_x, initial_y, np.deg2rad(initial_phi_deg)]),
            "P": np.diag([rospy.get_param("~initial_cov/x", 0.0025), 
                          rospy.get_param("~initial_cov/y", 0.0025), 
                          np.deg2rad(rospy.get_param("~initial_cov/phi_deg", 1.0))**2]), # 분산이므로 제곱
            "num_landmarks": 0,
            "landmark_last_seen": [],
            "landmark_inflated": []
        }

        # IMU 기반 오도메트리 추정을 위한 변수
        self.last_imu_time = None
        self.current_velocity_y = 0.0 # 현재 추정된 로봇의 x축 선속도 (로봇 좌표계 기준)
        self.last_odom_time_for_ekf = None # EKF 예측 간의 시간 간격 dt 계산용

        self.state_history = { # 플로팅을 위한 상태 저장
            't': [rospy.Time.now().to_sec() if rospy.Time.now().to_sec() > 0 else 0.0], # 초기 시간 핸들링
            'x': self.ekf_state['x'][:3].copy().reshape(1,3), # (N,3) 형태로 유지
            'P_diag': np.diag(self.ekf_state['P'][:3,:3]).copy().reshape(1,3) # (N,3) 형태로 유지
        }


        if self.filter_params["do_plot"]:
            # PyQtGraph QApplication 문제 해결을 위해 mkQApp 호출
            # QApplication 생성은 slam_utils.init_plot()에서 처리하도록 수정
            # import pyqtgraph as pg
            # try:
            #     pg.mkQApp("EKF SLAM Viewer")
            #     rospy.loginfo("PyQtGraph QApplication created successfully")
            # except Exception as e:
            #     rospy.logwarn("Error creating PyQtGraph QApplication: %s", str(e))
            self.plot = slam_utils.init_plot()
            if self.plot is not None:
                rospy.loginfo("Plot window initialized successfully")
            else:
                rospy.logwarn("Failed to initialize plot window")
        else:
            self.plot = None

        # ROS1 Subscribers (LaserScan, Imu)
        self.scan_sub = message_filters.Subscriber('/scan', LaserScan)
        self.imu_sub = message_filters.Subscriber('/imu', Imu) # Odometry 대신 Imu 구독

        # ApproximateTimeSynchronizer 설정
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.imu_sub],
            queue_size=rospy.get_param("~sync/queue_size", 20), # 큐 사이즈 증가
            slop=rospy.get_param("~sync/slop", 0.1), # 슬롭 값 증가 (IMU, Lidar 주기가 다를 수 있음)
            allow_headerless=False # ROS 메시지는 헤더가 있음
        )
        self.ts.registerCallback(self.sync_callback)

        rospy.loginfo("EKF SLAM Sync Node (ROS1) initialized.")
        rospy.loginfo("Subscribing to /scan and /imu")
        rospy.loginfo("Initial EKF state: x=%.2f, y=%.2f, phi=%.2f deg", 
                      self.ekf_state['x'][0], self.ekf_state['x'][1], np.rad2deg(self.ekf_state['x'][2]))


    def sync_callback(self, scan_msg, imu_msg):
        current_time_ros = rospy.Time.now()
        t_scan_sec = scan_msg.header.stamp.to_sec()
        t_imu_sec = imu_msg.header.stamp.to_sec()

        if self.last_imu_time is None:
            self.last_imu_time = t_imu_sec
            self.last_odom_time_for_ekf = t_imu_sec # EKF dt 계산용 시간도 초기화
            rospy.loginfo("First IMU message received. Initializing time.")
            return

        # IMU로부터 dt 및 제어 입력 (v, w) 추정
        dt_imu = t_imu_sec - self.last_imu_time
        if dt_imu <= 1e-9: # dt가 너무 작거나 음수면 스킵 (시간 역행 방지)
            rospy.logwarn_throttle(1.0, "IMU dt is zero or negative (%.4f sec). Skipping prediction.", dt_imu)
            # self.last_imu_time = t_imu_sec # 시간을 업데이트해야 다음 dt가 정상적으로 계산됨
            return

        # 각속도 w (로봇 z축 기준)
        # IMU 좌표계의 각속도를 로봇 좌표계로 변환
        w = imu_msg.angular_velocity.z  # 일반적으로 z축 회전은 그대로 사용 가능
        
        # IMU orientation을 이용한 가속도 변환 및 중력 보상
        try:
            # IMU의 quaternion을 추출
            qx = imu_msg.orientation.x
            qy = imu_msg.orientation.y
            qz = imu_msg.orientation.z
            qw = imu_msg.orientation.w
            
            # quaternion이 유효한지 확인 (모두 0이면 초기화되지 않은 상태)
            if abs(qx) < 1e-10 and abs(qy) < 1e-10 and abs(qz) < 1e-10 and abs(qw) < 1e-10:
                # 유효하지 않은 경우 raw 가속도 사용
                rospy.logwarn_throttle(5.0, "Invalid IMU orientation quaternion. Using raw acceleration.")
                ax_robot = imu_msg.linear_acceleration.x
            else:
                # quaternion을 회전 행렬로 변환
                # 회전 행렬 계산 (quaternion -> rotation matrix)
                # R = [
                #     [1-2*(qy²+qz²), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
                #     [2*(qx*qy+qw*qz), 1-2*(qx²+qz²), 2*(qy*qz-qw*qx)],
                #     [2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx²+qy²)]
                # ]
                
                # 중력 벡터 (지구 좌표계에서는 [0, 0, 9.81])
                g = 9.81  # m/s²
                
                # IMU 좌표계에서 측정된 가속도
                ax_imu = imu_msg.linear_acceleration.x
                ay_imu = imu_msg.linear_acceleration.y
                az_imu = imu_msg.linear_acceleration.z
                
                # IMU 좌표계에서 로봇 좌표계로 가속도 변환
                # 간단한 방법: IMU가 로봇에 고정되어 있고 주로 x축 방향으로 움직인다고 가정
                # 중력 성분 보상 (IMU의 z축과 중력 방향 간의 각도 고려)
                # 회전 행렬의 세 번째 행은 IMU의 z축이 지구 좌표계에서 어느 방향을 가리키는지 나타냄
                
                # 중력 방향 계산 (IMU 좌표계에서)
                R_z_x = 2 * (qx * qz - qw * qy)
                R_z_y = 2 * (qy * qz + qw * qx)
                R_z_z = 1 - 2 * (qx * qx + qy * qy)
                
                # 중력 성분 제거
                ax_comp = ax_imu - g * R_z_x
                ay_comp = ay_imu - g * R_z_y
                az_comp = az_imu - g * R_z_z
                
                # IMU 좌표계에서 로봇 좌표계로 변환
                # 여기서는 간단히 IMU의 x축이 로봇의 전진 방향과 일치한다고 가정
                # 실제로는 TF 변환 행렬을 사용하는 것이 더 정확함
                ay_robot = ay_comp
        except Exception as e:
            rospy.logwarn("Error processing IMU orientation: %s. Using raw acceleration.", str(e))
            ay_robot = imu_msg.linear_acceleration.y
        
        # 가속도 적분하여 속도 추정 (드리프트 방지 로직 제거)
        self.current_velocity_y += ay_robot * dt_imu
        v = self.current_velocity_y
        u = np.array([v, w], dtype=float)
        self.last_imu_time = t_imu_sec # IMU 시간 업데이트

        # EKF 예측 단계
        # EKF 예측에 사용될 dt는 last_odom_time_for_ekf 기준
        dt_ekf = t_imu_sec - self.last_odom_time_for_ekf
        if dt_ekf <= 1e-9 : # 이전 IMU 처리에서 dt_imu가 0이라 return 되었을 수 있음.
            rospy.logwarn_throttle(1.0, "EKF dt is zero or negative (%.4f sec). Skipping prediction.", dt_ekf)
            # self.last_odom_time_for_ekf = t_imu_sec # 시간을 업데이트해야 다음 dt가 정상적으로 계산됨
            return

        self.ekf_state = odom_predict_from_imu(u, dt_ekf, self.ekf_state,
                                               self.vehicle_params,
                                               self.alpha_params)
        self.last_odom_time_for_ekf = t_imu_sec # EKF 예측 시간 업데이트

        # 상태 기록 (예측 후)
        self.state_history['t'].append(t_imu_sec) # IMU 시간을 기준으로 기록
        
        # 로봇 자세만 1x3 행벡터로 강제 변환
        row = self.ekf_state['x'][0:3].reshape(1, 3)
        
        # self.state_history['x']가 혹시 1-D로 바뀌어 있어도 2-D로 복구
        if self.state_history['x'].ndim == 1:
            self.state_history['x'] = self.state_history['x'].reshape(1, -1)
            
        # 열 개수가 다르면(이전에 잘못 저장된 경우) 앞의 3열만 살리기
        if self.state_history['x'].shape[1] != 3:
            rospy.logwarn("state_history['x'] 열 개수(%d) 교정 → 3", self.state_history['x'].shape[1])
            self.state_history['x'] = self.state_history['x'][:, :3]
            
        # 안전하게 붙이기
        self.state_history['x'] = np.vstack((self.state_history['x'], row))
        
        # P_diag도 같은 방식으로 처리
        p_diag_row = np.diag(self.ekf_state['P'][:3, :3]).reshape(1, 3)
        
        if self.state_history['P_diag'].ndim == 1:
            self.state_history['P_diag'] = self.state_history['P_diag'].reshape(1, -1)
            
        if self.state_history['P_diag'].shape[1] != 3:
            rospy.logwarn("state_history['P_diag'] 열 개수(%d) 교정 → 3", self.state_history['P_diag'].shape[1])
            self.state_history['P_diag'] = self.state_history['P_diag'][:, :3]
            
        self.state_history['P_diag'] = np.vstack((self.state_history['P_diag'], p_diag_row))

        # Lidar 데이터 처리
        # 1. 사용자 요청: 1 미만 값(사각지대)을 100(range_max)으로 변경
        ranges_np = np.array(scan_msg.ranges, dtype=np.float32)
        
        # 사각지대 처리: 0이 아니고 1 미만인 값을 range_max로 설정
        # scan_example.txt에서 range_min = 0.1 이므로, 0.1 ~ 1.0 사이의 값을 의미함.
        # 0.0은 보통 'no return'을 의미할 수 있으므로 건드리지 않거나, range_max로 동일하게 처리 가능.
        # 여기서는 명시적으로 "1 밑의 값들"을 처리.
        blind_spot_threshold = 1.0
        # ranges_np[(ranges_np < blind_spot_threshold) & (ranges_np > 1e-3)] = scan_msg.range_max # 0이 아닌 값만
        for i in range(len(ranges_np)):
            if ranges_np[i] > 1e-6 and ranges_np[i] < blind_spot_threshold : # 0이 아니고 1미만
                 ranges_np[i] = scan_msg.range_max

        # 2. NaN 및 Inf 값, range_max 초과 값 처리 (기존 ROS2 함수 사용)
        # 이 함수는 NaN을 0 또는 이전 값으로, Inf/range_max 초과 값을 range_max로 만듦
        ranges_cleaned = self.replace_nan_inf(ranges_np, scan_msg.range_max)

        # 3. 각도 배열 생성 (360도 스캔용)
        if len(ranges_cleaned) > 0:
            angles_from_scan = scan_msg.angle_min + np.arange(len(ranges_cleaned)) * scan_msg.angle_increment
            # scan_msg.angle_max까지 정확히 가는지 확인 (보통 len(ranges)와 angle_increment로 계산하면 맞음)
        else:
            angles_from_scan = np.array([])
            rospy.logwarn_throttle(1.0, "Received empty or invalid laser scan ranges.")
            return # 빈 스캔이면 더 이상 처리 불가

        # 랜드마크 불확실성 증가 (오랫동안 안보인 경우)
        self.inflate_lost_landmarks(t_scan_sec) # 스캔 시간을 기준으로

        # 랜드마크 추출
        # tree_extraction 함수는 이제 ranges와 angles를 모두 받음
        trees = tree_extraction.extract_trees(ranges_cleaned, angles_from_scan, self.filter_params)
        
        if len(trees) > 0:
            # 데이터 연관
            assoc = compute_data_association(self.ekf_state, trees,
                                             self.sigmas, self.filter_params)
            # EKF 업데이트
            self.ekf_state = laser_update(trees, assoc, self.ekf_state,
                                          self.sigmas, self.filter_params, t_scan_sec)

            # 상태 기록 (업데이트 후)
            self.state_history['t'].append(t_scan_sec) # 스캔 시간을 기준으로 기록
            
            # 로봇 자세만 1x3 행벡터로 강제 변환
            row = self.ekf_state['x'][0:3].reshape(1, 3)
            
            # self.state_history['x']가 혹시 1-D로 바뀌어 있어도 2-D로 복구
            if self.state_history['x'].ndim == 1:
                self.state_history['x'] = self.state_history['x'].reshape(1, -1)
                
            # 열 개수가 다르면(이전에 잘못 저장된 경우) 앞의 3열만 살리기
            if self.state_history['x'].shape[1] != 3:
                rospy.logwarn("state_history['x'] 열 개수(%d) 교정 → 3", self.state_history['x'].shape[1])
                self.state_history['x'] = self.state_history['x'][:, :3]
                
            # 안전하게 붙이기
            self.state_history['x'] = np.vstack((self.state_history['x'], row))
            
            # P_diag도 같은 방식으로 처리
            p_diag_row = np.diag(self.ekf_state['P'][:3, :3]).reshape(1, 3)
            
            if self.state_history['P_diag'].ndim == 1:
                self.state_history['P_diag'] = self.state_history['P_diag'].reshape(1, -1)
                
            if self.state_history['P_diag'].shape[1] != 3:
                rospy.logwarn("state_history['P_diag'] 열 개수(%d) 교정 → 3", self.state_history['P_diag'].shape[1])
                self.state_history['P_diag'] = self.state_history['P_diag'][:, :3]
                
            self.state_history['P_diag'] = np.vstack((self.state_history['P_diag'], p_diag_row))
        else:
            assoc = [] # 트리가 없으면 연관 결과도 없음

        # 플로팅
        if self.filter_params["do_plot"] and self.plot is not None:
            # plot_scan을 위해 angles_from_scan 전달
            slam_utils.do_plot(
                self.state_history['x'],
                self.ekf_state,
                trees,
                ranges_cleaned, # 클리닝된 range 사용
                angles_from_scan, # 해당 각도 사용
                assoc,
                self.plot,
                self.filter_params
            )

        if self.ekf_state['x'].ndim > 1:
            self.ekf_state['x'] = self.ekf_state['x'].flatten()

    def inflate_lost_landmarks(self, t_sec):
        """ 오랫동안 관측되지 않은 랜드마크의 불확실성을 증가시킴 """
        T_thr = self.reinit_params["lost_threshold"]
        scale = self.reinit_params["scale_factor"]

        nL = self.ekf_state["num_landmarks"]
        for j in range(nL): # 각 랜드마크에 대해
            # landmark_last_seen은 리스트이므로 인덱스 j로 접근
            if j < len(self.ekf_state["landmark_last_seen"]):
                 elapsed = t_sec - self.ekf_state["landmark_last_seen"][j]
                 if (elapsed > T_thr) and (not self.ekf_state['landmark_inflated'][j]):
                    idx_in_P = 3 + 2*j # P 행렬에서 해당 랜드마크의 시작 인덱스
                    # 랜드마크의 공분산 부분 (2x2)에 scale 적용
                    self.ekf_state["P"][idx_in_P : idx_in_P+2, idx_in_P : idx_in_P+2] *= scale
                    self.ekf_state['landmark_inflated'][j] = True
                    rospy.loginfo_throttle(5.0, "Inflated covariance for landmark %d (elapsed: %.2f s)", j, elapsed)
            else:
                rospy.logwarn_throttle(1.0, "Landmark index %d out of bounds for landmark_last_seen (len: %d)", 
                                       j, len(self.ekf_state["landmark_last_seen"]))


    @staticmethod
    def replace_nan_inf(ranges_in, range_max_val):
        """ LiDAR 데이터에서 NaN, Inf 값을 처리하고, range_max_val을 초과하는 값을 제한. """
        cleaned = ranges_in.copy()
        # NaN 처리: 0.0 또는 이전 유효 값으로 대체 (ROS2 코드 방식)
        # 여기서는 0.0을 range_max로 처리하는 것이 더 안전할 수 있음 (센서 에러로 간주)
        # 또는 바로 앞/뒤 유효값으로. ROS2 코드는 0 또는 이전 값.
        # scan_example에서 0은 안나오고, 작은 값들이 사각지대. NaN은 보통 에러.
        # 여기서는 NaN을 range_max로 설정하여 필터링되도록 유도
        has_nan = False
        for i in range(len(cleaned)):
            if np.isnan(cleaned[i]):
                cleaned[i] = range_max_val # NaN은 최대 거리로 처리 (필터링 유도)
                has_nan = True
            elif np.isinf(cleaned[i]):
                cleaned[i] = range_max_val # Inf도 최대 거리로
            elif cleaned[i] > range_max_val:
                cleaned[i] = range_max_val # 최대 거리 초과 값 제한
            elif cleaned[i] < 1e-6: # 0에 매우 가까운 값 (잠재적 에러 또는 실제 0)
                                   # scan_min이 0.1이므로, 0은 보통 에러.
                cleaned[i] = range_max_val # 0 값도 최대 거리로 (필터링 유도)

        # if has_nan:
        #     rospy.logdebug_throttle(1.0, "NaN values found in laser scan, replaced with range_max.")
        return cleaned

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz로 UI 갱신
        try:
            while not rospy.is_shutdown():
                # 콜백 한 번 실행 (rospy.spin()과 달리 블로킹하지 않음)
                rospy.rostime.wallsleep(0.0001)  # rospy.spin_once() 대신 최소 sleep으로 콜백 처리 기회 제공
                
                # Qt 이벤트 처리 (센서 데이터와 무관하게 주기적으로 수행)
                if self.filter_params["do_plot"] and self.plot is not None:
                    try:
                        if self.plot.get("app") is not None:
                            self.plot["app"].processEvents()
                        else:
                            # 앱 인스턴스가 없는 경우 (드문 케이스)
                            import pyqtgraph as pg
                            app = pg.mkQApp()  # 이미 있으면 기존 인스턴스 반환
                            app.processEvents()
                    except Exception as e:
                        rospy.logwarn("Qt processEvents failed: %s", str(e))
                
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received, shutting down...")
        finally:
            if self.filter_params["do_plot"] and self.plot is not None:
                try:
                    rospy.loginfo("EKF SLAM node shutting down. Plot window may remain open for inspection.")
                    if self.plot.get("app") is not None:
                        # 여기서 exec_()를 호출하면 창이 닫힐 때까지 블록됨
                        # 대신 processEvents()를 호출하여 창이 즉시 닫히지 않도록 함
                        self.plot["app"].processEvents()
                    else:
                        # pg.QtGui.QApplication.processEvents() 대신 pg.mkQApp().processEvents() 사용
                        import pyqtgraph as pg
                        app = pg.mkQApp()  # 이미 있으면 기존 인스턴스 반환
                        app.processEvents()
                except Exception as e:
                    rospy.logwarn("Error handling plot window on shutdown: %s", str(e))


if __name__ == '__main__':
    try:
        node = EKFSLAMNodeROS1()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("EKF SLAM node (ROS1) shut down.")
    except Exception as e:
        rospy.logerr("Unhandled exception in EKF SLAM node (ROS1): %s", str(e))
        import traceback
        traceback.print_exc()
