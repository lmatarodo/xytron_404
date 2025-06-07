from __future__ import division
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import pyqtgraph as pg

import slam_utils
import tree_extraction
from scipy.stats.distributions import chi2
from message_filters import Subscriber, ApproximateTimeSynchronizer

##############################################
# 1. SLAM 관련 함수들
##############################################

def motion_model_nonlinear(u, dt, ekf_state, vehicle_params):
    """
    문헌 (2.4)에 가까운 비선형 모션 모델.
    v,w = 제어입력, (x,y,phi) = 현재 로봇 상태
    """
    v = u[0]
    w = u[1]

    # 현재 로봇 상태
    x_state = ekf_state['x'].copy()
    x0, y0, phi0 = x_state[0], x_state[1], x_state[2]

    # w=0 인 경우 수치오차 방지
    if abs(w) < 1e-9:
        # w가 거의 0이면, 직선 근사
        el1 = v * dt * math.cos(phi0)
        el2 = v * dt * math.sin(phi0)
        el3 = w * dt
    else:
        # 원호 이동
        el1 = -(v/w) * math.sin(phi0) + (v/w) * math.sin(phi0 + w*dt)
        el2 =  (v/w) * math.cos(phi0) - (v/w) * math.cos(phi0 + w*dt)
        el3 =  w * dt

    # 이동량 (delta_x, delta_y, delta_phi)
    motion = np.array([[el1],
                       [el2],
                       [slam_utils.clamp_angle(el3)]])

    # -------------------------------------------------------
    # G_3x3: 상태 x=(x,y,phi)에 대한 자코비안
    # 문헌 (2.5) 참고. 실제론 v,w,theta 모두 고려해야 함.
    # 여기서는 w != 0 가정시 편미분을 예시적으로 적음.
    # (실제로는 if w=0 근처 처리 등 별도 분기 필요)
    # -------------------------------------------------------
    # 편의상 간단히 수치 근사하거나, 직접 미분식을 적을 수 있습니다.
    # 여기서는 문헌식(2.5)에 대응하도록 간단히 구현 예시:
    if abs(w) < 1e-9:
        # w=0이면, 아래 near-zero 근사
        G_3x3 = np.array([
            [1.0, 0.0, -dt*v*math.sin(phi0)],
            [0.0, 1.0,  dt*v*math.cos(phi0)],
            [0.0, 0.0, 1.0]
        ])
    else:
        # w != 0
        # 실제 문헌(2.5)와 똑같이 구현하려면
        # G_3x3 = ...
        # 다만 예시로 간단히 해석가능한 형태만 적어둠
        dtheta = w*dt
        G_3x3 = np.array([
            [1, 0,  (v/w)*(math.cos(phi0) - math.cos(phi0+dtheta))],
            [0, 1,  (v/w)*(math.sin(phi0+dtheta) - math.sin(phi0))],
            [0, 0,  1]
        ])

    return motion, G_3x3


def odom_predict(u, dt, ekf_state, vehicle_params, sigmas, alpha_params):
    """
    예측단계(오도메트리). '동적' 프로세스 잡음(Q_k)을 문헌 방식으로 구성.
    """
    t_st = ekf_state['x'].copy().reshape(-1, 1)
    t_cov = ekf_state['P'].copy()
    dim = t_st.shape[0] - 3

    # 로봇 상태+랜드마크 중, 로봇 부분만 뽑아내는 블록 행렬
    F_x = np.hstack((np.eye(3), np.zeros((3, dim))))

    # -- 수정된 부분 1: 비선형 모션 모델 사용 --
    mot, G_3x3 = motion_model_nonlinear(u, dt, ekf_state, vehicle_params)

    # 로봇 상태 3D + 랜드마크 dimD 전체에 해당하는 Gt(= block matrix)
    Gt_1 = np.hstack((G_3x3,               np.zeros((3, dim))))
    Gt_2 = np.hstack((np.zeros((dim, 3)),  np.eye(dim)))
    Gt   = np.vstack((Gt_1, Gt_2))

    # --- 수정된 부분 2: 동적 공분산 Q_k = alpha1*(v^2)+alpha2*(w^2) ...
    v, w = u[0], u[1]
    alpha1 = alpha_params['alpha1']
    alpha2 = alpha_params['alpha2']
    alpha3 = alpha_params['alpha3']
    alpha4 = alpha_params['alpha4']

    q_v = alpha1*(v**2) + alpha2*(w**2)
    q_w = alpha3*(v**2) + alpha4*(w**2)
    Q_2x2 = np.diag([q_v, q_w])

    # V_k(= partial f wrt u), 여기서는 간단히:
    phi0 = ekf_state['x'][2]
    if abs(w) < 1e-9:
        # near-zero w → 단순화
        V_3x2 = np.array([
            [dt*math.cos(phi0),  0],
            [dt*math.sin(phi0),  0],
            [0,                  dt]
        ])
    else:
        # w != 0 → 실제 문헌에선 더 복잡한 식 가능
        # 여긴 간단히 동일 형태
        V_3x2 = np.array([
            [dt*math.cos(phi0),  0],
            [dt*math.sin(phi0),  0],
            [0,                  dt]
        ])

    # (3x2)*(2x2)*(3x2)^T = 3x3
    process_noise_3x3 = V_3x2 @ Q_2x2 @ V_3x2.T

    # block 확장해서 (3+dim)x(3+dim)짜리
    R_big = np.zeros((3+dim, 3+dim))
    R_big[0:3, 0:3] = process_noise_3x3

    # 최종 공분산 예측
    new_cov = Gt @ t_cov @ Gt.T + R_big
    new_cov = slam_utils.make_symmetric(new_cov)

    # 상태도 업데이트
    new_x = t_st + F_x.T @ mot

    ekf_state['x'] = new_x.flatten()
    ekf_state['P'] = new_cov

    return ekf_state


def laser_measurement_model(ekf_state, landmark_id):
    t_st = ekf_state['x'].copy()
    t_st[2] = slam_utils.clamp_angle(t_st[2])
    dim = t_st.shape[0]

    r_x, r_y, phi = t_st[0], t_st[1], t_st[2]
    m_x = t_st[3 + 2*landmark_id]
    m_y = t_st[4 + 2*landmark_id]

    del_x = m_x - r_x
    del_y = m_y - r_y
    q = del_x**2 + del_y**2
    sqrt_q = np.sqrt(q)

    zhat = [
        [sqrt_q],
        [slam_utils.clamp_angle(np.arctan2(del_y, del_x) - phi)]
    ]

    h = np.array([
        [-sqrt_q*del_x, -sqrt_q*del_y, 0, sqrt_q*del_x, sqrt_q*del_y],
        [del_y,         -del_x,       -q, -del_y,      del_x]
    ]) / q

    F_x = np.zeros((5, dim))
    F_x[:3, :3] = np.eye(3)
    F_x[3, 3+2*landmark_id] = 1
    F_x[4, 4+2*landmark_id] = 1

    H = h @ F_x
    return zhat, H


def initialize_landmark(ekf_state, tree, t_now):
    t_x, t_y, phi = ekf_state['x'][0], ekf_state['x'][1], ekf_state['x'][2]
    m_r, m_th = tree[0], tree[1]

    m_x = t_x + m_r * np.cos(m_th + phi)
    m_y = t_y + m_r * np.sin(m_th + phi)
    ekf_state['x'] = np.hstack((ekf_state['x'], m_x, m_y))

    temp_p = ekf_state['P']
    dim_old = temp_p.shape[0]
    temp_p = np.vstack((temp_p, np.zeros((2, dim_old))))
    temp_p = np.hstack((temp_p, np.zeros((dim_old+2, 2))))
    temp_p[-2, -2] = 0.3
    temp_p[-1, -1] = 0.3

    ekf_state['P'] = temp_p
    ekf_state['num_landmarks'] += 1

    ekf_state['landmark_last_seen'].append(t_now)
    ekf_state['landmark_inflated'].append(False)

    return ekf_state


def compute_data_association(ekf_state, measurements, sigmas, params):
    if ekf_state["num_landmarks"] == 0:
        return [-1 for _ in measurements]

    n_lmark = ekf_state['num_landmarks']
    n_scans = len(measurements)

    M = np.zeros((n_scans, n_lmark))
    Q_t = np.array([
        [sigmas['range']**2, 0],
        [0,                  sigmas['bearing']**2]
    ])

    alpha = chi2.ppf(0.99999995, 2)
    beta = chi2.ppf(0.99999999, 2)

    for i in range(n_lmark):
        zhat, H = laser_measurement_model(ekf_state, i)
        S = H @ ekf_state['P'] @ H.T + Q_t
        Sinv = slam_utils.invert_2x2_matrix(S)

        for j in range(n_scans):
            temp_z = measurements[j][:2]
            res = temp_z - np.squeeze(zhat)
            M[j, i] = res.T @ (Sinv @ res)

    A = alpha * np.ones((n_scans, n_scans))
    M_new = np.hstack((M, A))
    pairs = slam_utils.solve_cost_matrix_heuristic(M_new)
    pairs.sort()

    pairs = [(x[0], -1) if x[1] >= n_lmark else (x[0], x[1]) for x in pairs]
    assoc = [p[1] for p in pairs]

    for i in range(len(assoc)):
        if assoc[i] == -1:
            for j in range(M.shape[1]):
                if M[i, j] < beta:
                    assoc[i] = -2
                    break

    return assoc


def laser_update(trees, assoc, ekf_state, sigmas, params, t_now):
    Q_t = np.array([
        [sigmas['range']**2,   0],
        [0,                    sigmas['bearing']**2]
    ])

    max_iter = 10
    eps_converge = 1e-7

    for i, tree in enumerate(trees):
        j = assoc[i]

        if j == -1:
            ekf_state = initialize_landmark(ekf_state, tree, t_now)
            j = ekf_state['num_landmarks'] - 1
        elif j == -2:
            continue

        x_pred = ekf_state['x'].copy()
        P_pred = ekf_state['P'].copy()
        z = np.array([[tree[0]], [tree[1]]])
        dim = x_pred.shape[0]

        x_i = x_pred
        P_i = P_pred
        old_cost_value = 1.0e12

        for _ in range(max_iter):
            ekf_state['x'] = x_i
            z_hat, H = laser_measurement_model(ekf_state, j)

            S = H @ P_i @ H.T + Q_t
            K = P_i @ H.T @ np.linalg.inv(S)

            inno = z - z_hat
            x_new = x_i + np.squeeze(K @ inno)
            x_new[2] = slam_utils.clamp_angle(x_new[2])

            P_new = (np.eye(dim) - K @ H) @ P_i
            P_new = slam_utils.make_symmetric(P_new)

            cost_value = np.linalg.norm(inno)
            if cost_value > old_cost_value:
                break
            old_cost_value = cost_value

            diff_norm = np.linalg.norm(x_new - x_i)
            x_i, P_i = x_new, P_new
            if diff_norm < eps_converge:
                break

        ekf_state['x'] = x_i
        ekf_state['P'] = P_i

        ekf_state['landmark_last_seen'][j] = t_now
        ekf_state['landmark_inflated'][j] = False

    return ekf_state


##############################################
# 2. ROS2 노드: EKFSLAMNode
##############################################
class EKFSLAMNode(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')

        self.vehicle_params = {
            "a": 0.13,
            "b": 0.0,
            "L": 0.09,
            "H": 0.07
        }

        self.filter_params = {
            "max_laser_range": 10,
            "do_plot": True,
            "plot_raw_laser": True,
            "plot_map_covariances": True
        }

        # -- 수정된 부분: 오도메트리 sigma(상수) 대신 알파계수 사용
        # 관측(레이저) 노이즈는 그대로
        self.sigmas = {
            # range/bearing는 레이저 센서 잡음
            "range": 0.01,
            "bearing": 2.0 * np.pi/180
        }
        self.alpha_params = {
            "alpha1": 0.01,
            "alpha2": 0.01,
            "alpha3": 0.001,
            "alpha4": 0.01
        }

        self.reinit_params = {
            "lost_threshold": 15.0,
            "scale_factor": 5
        }

        self.ekf_state = {
            "x": np.array([0.0, 0.0, 36*np.pi/180]),
            "P": np.diag([0.0025, 0.0025, 1]),
            "num_landmarks": 0,
            "landmark_last_seen": [],
            "landmark_inflated": []
        }

        self.last_odom_t = -1.0

        self.state_history = {
            't': [0.0],
            'x': self.ekf_state['x'],
            'P': np.diag(self.ekf_state['P'])
        }

        if self.filter_params["do_plot"]:
            self.plot = slam_utils.init_plot()
        else:
            self.plot = None

        self.scan_sub = Subscriber(self, LaserScan, '/scan')
        self.odom_sub = Subscriber(self, Odometry, '/odom')

        self.ts = ApproximateTimeSynchronizer(
            [self.scan_sub, self.odom_sub],
            queue_size=10,
            slop=0.085,
            allow_headerless=False
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("EKF SLAM Sync Node initialized. Using ApproxTimeSync.")


    def sync_callback(self, scan_msg, odom_msg):
        t_odom_sec = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec*1e-9
        t_scan_sec = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec*1e-9

        if self.last_odom_t < 0:
            self.last_odom_t = t_odom_sec
            return

        dt = t_odom_sec - self.last_odom_t
        if dt <= 0:
            return

        v = odom_msg.twist.twist.linear.x
        w = odom_msg.twist.twist.angular.z
        u = np.array([v, w], dtype=float)

        # -- 수정된 예측단계 호출 --
        self.ekf_state = odom_predict(u, dt, self.ekf_state,
                                      self.vehicle_params,
                                      self.sigmas,       # 레이저 노이즈 등 유지
                                      self.alpha_params) # 새로 추가된 alpha

        self.last_odom_t = t_odom_sec

        self.state_history['x'] = np.vstack((self.state_history['x'], self.ekf_state['x'][0:3]))
        self.state_history['P'] = np.vstack((self.state_history['P'],
                                   np.diag(self.ekf_state['P'][:3, :3])))
        self.state_history['t'].append(t_odom_sec)

        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        ranges = self.replace_nan_inf(ranges, scan_msg.range_max)

        self.inflate_lost_landmarks(t_scan_sec)
        trees = tree_extraction.extract_trees(ranges, self.filter_params)
        if len(trees) > 0:
            assoc = compute_data_association(self.ekf_state, trees,
                                             self.sigmas, self.filter_params)
            self.ekf_state = laser_update(trees, assoc, self.ekf_state,
                                          self.sigmas, self.filter_params, t_scan_sec)

            self.state_history['x'] = np.vstack((self.state_history['x'], self.ekf_state['x'][0:3]))
            self.state_history['P'] = np.vstack((self.state_history['P'],
                                       np.diag(self.ekf_state['P'][:3, :3])))
            self.state_history['t'].append(t_scan_sec)

        if self.filter_params["do_plot"]:
            slam_utils.do_plot(
                self.state_history['x'],
                self.ekf_state,
                trees,
                ranges,
                assoc if len(trees) > 0 else [],
                self.plot,
                self.filter_params
            )

    def inflate_lost_landmarks(self, t_sec):
        T_thr = self.reinit_params["lost_threshold"]
        scale = self.reinit_params["scale_factor"]

        nL = self.ekf_state["num_landmarks"]
        for j in range(nL):
            elapsed = t_sec - self.ekf_state["landmark_last_seen"][j]
            if (elapsed > T_thr) and (self.ekf_state['landmark_inflated'][j] == False):
                idx = 3 + 2*j
                self.ekf_state["P"][idx:idx+2, idx:idx+2] *= scale
                self.ekf_state['landmark_inflated'][j] = True

    @staticmethod
    def replace_nan_inf(ranges, range_max):
        cleaned = ranges.copy()
        for i in range(len(cleaned)):
            if math.isnan(cleaned[i]):
                cleaned[i] = 0.0 if i == 0 else cleaned[i-1]
        for i in range(len(cleaned)):
            if math.isinf(cleaned[i]) or cleaned[i] > range_max:
                cleaned[i] = range_max
        return cleaned


def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()