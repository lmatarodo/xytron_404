#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import time

try:
    # ROS1 환경에서는 PyQt5를 직접 사용하거나, rqt 등이 내부적으로 사용.
    # pyqtgraph는 독립적으로 설치되어야 함.
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtGui, QtCore
    # QtGui → QtWidgets 모듈 변경 대응
    from pyqtgraph.Qt import QtWidgets  # QApplication이 이동한 모듈
    can_plot = True
except ImportError:
    # rospy를 임포트해서 로그를 남길 수 있도록 함 (이 파일이 단독 실행되지 않는다고 가정)
    try:
        import rospy
        rospy.logwarn("Cannot import pyqtgraph or PyQt5. Plotting will be disabled.")
    except ImportError:
        print("Warning: Cannot import pyqtgraph or PyQt5. Plotting will be disabled.")
    can_plot = False

# 파일 읽기 함수 (ROS2와 동일)
def read_data_file(file_name):
    with open(file_name, "r") as f:
        raw_data = f.readlines()
    data = [ [float(x) for x in line.strip().split(',')] for line in raw_data ]
    return np.array(data)

# 랜드마크(tree)를 전역 좌표로 변환 (ROS2와 거의 동일)
def tree_to_global_xy(trees, ekf_state):
    if not isinstance(trees, np.ndarray):
        trees_np = np.array(trees)
    else:
        trees_np = trees

    if trees_np.ndim == 1 and trees_np.shape[0] > 0: # 단일 트리인 경우 2D로 만듦
        trees_np = trees_np.reshape(1, -1)
    elif trees_np.shape[0] == 0:
        return np.array([])


    # trees_np는 각 행이 [range, bearing, diameter] 형태여야 함
    if trees_np.shape[1] < 2:
        # print("Warning: tree_to_global_xy expects at least 2 columns (range, bearing) in trees.")
        return np.array([])

    phi = ekf_state["x"][2]
    mu_robot = ekf_state["x"][0:2].reshape(2,1) # 로봇의 x, y 위치

    ranges = trees_np[:,0]
    bearings = trees_np[:,1]
    
    # 랜드마크의 전역 x, y 좌표 계산
    global_x = mu_robot[0] + ranges * np.cos(phi + bearings)
    global_y = mu_robot[1] + ranges * np.sin(phi + bearings)
    
    return np.vstack((global_x, global_y)).T # (N, 2) 형태의 배열 반환

# 측정된 랜드마크(tree) 플로팅 (ROS2와 거의 동일)
def plot_tree_measurements(trees, assoc, ekf_state, plot_handle):
    if not can_plot or plot_handle is None or len(trees) == 0:
        return

    # 프레임 카운터 증가 및 업데이트 주기 제어 (CPU 부하 감소)
    plot_handle["plot_counter"] = (plot_handle["plot_counter"] + 1) % 10
    if plot_handle["plot_counter"] != 0:  # 10프레임마다 한 번만 업데이트
        return

    G_trees = tree_to_global_xy(trees, ekf_state) # (N, 2)
    if G_trees.shape[0] == 0:
        return

    mu_robot = ekf_state["x"][0:2] # 로봇 위치 [x, y]

    if "lasers" not in plot_handle: # plot_handle은 딕셔너리여야 함
        plot_handle["lasers"] = []
        plot_handle["laser_in_axis"] = []

    num_measurements = G_trees.shape[0]
    for i in range(num_measurements):
        # 로봇 위치에서 랜드마크 위치까지 선을 그림
        line_data = np.array([mu_robot, G_trees[i,:]]) # [[rx,ry], [lx,ly]]
        
        color = 'b' # 기본색 (새 랜드마크 후보)
        if i < len(assoc):
            if assoc[i] >= 0: # 기존 랜드마크와 연관됨
                color = 'g'
            elif assoc[i] == -2: # 유효하지 않은 측정 (버림)
                color = 'r'
        
        # pyqtgraph 아이템 관리
        if i >= len(plot_handle["lasers"]): # 새 plot item 생성
            new_item = plot_handle["axis"].plot(pen=pg.mkPen(color, width=1))  # 선 두께 감소 (2→1)
            plot_handle["lasers"].append(new_item)
            plot_handle["laser_in_axis"].append(True) # 이미 axis에 추가됨
        else: # 기존 plot item 업데이트
            plot_handle["lasers"][i].setData(line_data[:,0], line_data[:,1], pen=pg.mkPen(color, width=1))  # 선 두께 감소 (2→1)
            if not plot_handle["laser_in_axis"][i]:
                plot_handle["axis"].addItem(plot_handle["lasers"][i])
                plot_handle["laser_in_axis"][i] = True
    
    # 사용되지 않는 이전 laser plot item들 제거
    for i in range(num_measurements, len(plot_handle["lasers"])):
        if plot_handle["laser_in_axis"][i]:
            plot_handle["axis"].removeItem(plot_handle["lasers"][i])
            plot_handle["laser_in_axis"][i] = False


# 로봇 궤적 플로팅 (ROS2와 동일)
def plot_trajectory(traj_history, plot_handle): # traj_history는 (N,3) 또는 (N,2)
    if not can_plot or plot_handle is None or traj_history.shape[0] <= 1:
        return
    
    if "trajectory" not in plot_handle or plot_handle["trajectory"] is None:
        plot_handle["trajectory"] = plot_handle["axis"].plot(pen='k') # 검은색으로 궤적
    
    plot_handle["trajectory"].setData(traj_history[:,0], traj_history[:,1]) # x, y 좌표만 사용


# 맵(랜드마크) 플로팅 (ROS2와 동일)
def plot_map(ekf_state, plot_handle, params):
    if not can_plot or plot_handle is None or ekf_state["num_landmarks"] == 0:
        # 만약 랜드마크가 없어졌다면 기존 심볼들 제거
        if "map_symbols" in plot_handle and plot_handle["map_symbols"] is not None:
            plot_handle["axis"].removeItem(plot_handle["map_symbols"])
            plot_handle["map_symbols"] = None
        if "map_covariances" in plot_handle:
            for item in plot_handle["map_covariances"]:
                plot_handle["axis"].removeItem(item)
            plot_handle["map_covariances"] = []
        return

    # 랜드마크 위치 (x,y)들을 (num_landmarks, 2) 형태로 추출
    landmarks_xy = ekf_state["x"][3:].reshape(-1, 2)
    
    if "map_symbols" not in plot_handle or plot_handle["map_symbols"] is None:
        # ScatterPlotItem 사용이 더 효율적일 수 있음
        plot_handle["map_symbols"] = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 120)) # 초록색 '+'
        plot_handle["axis"].addItem(plot_handle["map_symbols"])
    
    plot_handle["map_symbols"].setData(pos=landmarks_xy, symbol='+')

    # 랜드마크 공분산 타원 플로팅
    if params.get("plot_map_covariances", False):
        if "map_covariances" not in plot_handle:
            plot_handle["map_covariances"] = []

        num_existing_ellipses = len(plot_handle["map_covariances"])
        for i in range(ekf_state["num_landmarks"]):
            idx_in_P = 3 + 2*i
            # 랜드마크 i의 공분산 P_landmark (2x2)
            P_landmark = ekf_state["P"][idx_in_P : idx_in_P+2, idx_in_P : idx_in_P+2]
            # 랜드마크 i의 평균 위치 mu_landmark (x,y)
            mu_landmark = ekf_state["x"][idx_in_P : idx_in_P+2]
            
            ellipse_points = get_covariance_ellipse_points(mu_landmark, P_landmark)
            
            if i >= num_existing_ellipses: # 새 타원 아이템 생성
                ellipse_item = plot_handle["axis"].plot(pen='b') # 파란색 타원
                plot_handle["map_covariances"].append(ellipse_item)
            else: # 기존 타원 아이템 업데이트
                ellipse_item = plot_handle["map_covariances"][i]
            
            ellipse_item.setData(ellipse_points[:,0], ellipse_points[:,1])
        
        # 남는 타원 아이템들 제거
        for i in range(ekf_state["num_landmarks"], num_existing_ellipses):
            plot_handle["axis"].removeItem(plot_handle["map_covariances"][i])
        if ekf_state["num_landmarks"] < num_existing_ellipses:
            plot_handle["map_covariances"] = plot_handle["map_covariances"][:ekf_state["num_landmarks"]]


# 공분산 타원 계산 (ROS2와 동일)
def get_covariance_ellipse_points(mu_xy, P_xy, n_points=20, chi2_val=None): # chi2_val for scaling (e.g. 5.991 for 95%)
    # P_xy는 2x2 공분산 행렬
    # mu_xy는 [x,y] 평균 위치
    if P_xy.shape != (2,2):
        raise ValueError("P_xy must be a 2x2 matrix.")

    # 고유값 분해 (P = U S U^T)
    eigenvalues, eigenvectors = np.linalg.eigh(P_xy)
    
    # 고유값이 음수이면 (수치적 오류 가능성), 매우 작은 양수로 대체
    if eigenvalues[0] < 0: eigenvalues[0] = 1e-9
    if eigenvalues[1] < 0: eigenvalues[1] = 1e-9

    # 타원의 축 길이 (표준편차의 배수)
    # chi2_val이 주어지면 해당 신뢰구간에 맞춤. 아니면 보통 1-sigma 또는 3-sigma.
    # 여기서는 3-sigma 사용 (ROS2 코드 방식)
    scale_factor = np.sqrt(chi2_val) if chi2_val is not None else 3.0 # 3-sigma
    
    # 타원의 반장축, 반단축 길이
    a = scale_factor * np.sqrt(eigenvalues[0]) # 첫번째 고유값에 대응하는 축
    b = scale_factor * np.sqrt(eigenvalues[1]) # 두번째 고유값에 대응하는 축

    # 기본 원 생성 (반지름 1)
    theta = np.linspace(0, 2 * np.pi, n_points)
    circle_points = np.array([np.cos(theta), np.sin(theta)]) # (2, n_points)

    # 타원 변환: S^(1/2) @ R @ circle_points
    # R은 eigenvectors, S^(1/2)는 sqrt(eigenvalues) 대각행렬
    # transformed_points = eigenvectors @ np.diag([a,b]) @ circle_points # 순서 주의
    # P = V D V^T, D = diag(lambda1, lambda2), V = eigenvectors
    # x_ellipse = V @ diag(sqrt(lambda1), sqrt(lambda2)) @ unit_circle
    
    # 타원의 각도 (첫번째 고유벡터의 각도)
    angle = np.arctan2(eigenvectors[1,0], eigenvectors[0,0])
    
    # 회전 행렬
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])

    # 크기 조절된 타원 포인트 (회전 전)
    scaled_ellipse_points = np.array([a * np.cos(theta), b * np.sin(theta)])
    
    # 회전 및 이동
    # ellipse_transformed = R @ scaled_ellipse_points # eigenvectors가 이미 회전을 포함
    # Cholesky 분해 L L^T = P. G = L. circ = 3 * base_circ @ G.T + mu
    # ROS2 코드 방식: G = np.linalg.cholesky(P + offset * np.eye)
    # 여기서는 고유값 분해를 사용한 방식
    
    # 타원 포인트 계산 (ROS2 코드의 get_covariance_ellipse_points와 유사하게)
    # base_circ는 단위 원 위의 점들 [(cos,sin), (cos,sin)...] (N,2)
    base_circ_xy = np.array([np.cos(theta), np.sin(theta)]).T # (n_points, 2)
    
    # P_xy가 양의 정부호가 아닐 수 있으므로 작은 offset 추가
    offset = 1e-6 - min(0, eigenvalues.min()) # ROS2 코드 방식
    # G = np.linalg.cholesky(P_xy + offset * np.eye(2)) # Cholesky 분해
    # ellipse_final_points = scale_factor * (base_circ_xy @ G.T) + mu_xy # ROS2 코드 방식
    
    # 고유값 분해를 사용한 방법 (더 직관적일 수 있음)
    # 타원 파라미터: 중심 mu_xy, 반축 길이 sqrt(eigval1), sqrt(eigval2), 회전각 angle
    # x = xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi)
    # y = yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi)
    # 여기서 a,b는 sqrt(eigenvalues), phi는 eigenvectors[0]의 각도
    # t는 0 to 2pi
    
    # eigenvectors의 첫번째 열이 첫번째 고유값에 대한 방향, 두번째 열이 두번째 고유값에 대한 방향
    # 타원의 점 x = mu + eigenvector_matrix * [a*cos(t); b*sin(t)]
    ellipse_points_relative = eigenvectors @ np.diag([np.sqrt(eigenvalues[0]), np.sqrt(eigenvalues[1])]) @ circle_points
    ellipse_final_points = mu_xy.reshape(2,1) + scale_factor * ellipse_points_relative
    
    return ellipse_final_points.T # (n_points, 2) 형태


# 원본 LiDAR 스캔 데이터 전역 좌표 변환 및 플로팅
# 이 함수는 이제 scan_ranges와 scan_angles를 모두 받음
def convert_scan_to_global_xy(ekf_state, scan_ranges, scan_angles, params):
    if len(scan_ranges) == 0 or len(scan_angles) == 0 or len(scan_ranges) != len(scan_angles):
        return np.array([])

    # 로봇의 현재 포즈
    phi_robot = ekf_state["x"][2]
    pos_robot = ekf_state["x"][0:2].reshape(2,1) # 로봇 위치 [x; y]

    # 유효한 거리 값만 필터링 (params의 max_laser_range 사용)
    # tree_extraction에서 이미 M11로 필터링하지만, 플로팅용으로도 필요
    max_range = params.get("max_laser_range", 10.0) # 기본값 10m
    
    valid_indices = (scan_ranges < max_range) & (scan_ranges > 0.05) # 매우 작은 값도 제외 (센서 최소 범위 가정)
    
    filtered_ranges = scan_ranges[valid_indices]
    filtered_angles = scan_angles[valid_indices] # 로컬 좌표계에서의 각도 (센서 기준)

    if len(filtered_ranges) == 0:
        return np.array([])

    # 로컬 극좌표 -> 로컬 직교좌표
    local_x = filtered_ranges * np.cos(filtered_angles)
    local_y = filtered_ranges * np.sin(filtered_angles)
    
    # 로컬 직교좌표 -> 전역 직교좌표
    # R = [[cos(phi_robot), -sin(phi_robot)], [sin(phi_robot), cos(phi_robot)]]
    # global_xy = R @ local_xy_vector + pos_robot
    global_x = pos_robot[0] + local_x * np.cos(phi_robot) - local_y * np.sin(phi_robot)
    global_y = pos_robot[1] + local_x * np.sin(phi_robot) + local_y * np.cos(phi_robot)
    
    return np.vstack((global_x, global_y)).T # (N, 2)


# LiDAR 스캔 플로팅 (ROS2와 거의 동일, scan_angles 추가)
def plot_scan(ekf_state, scan_ranges, scan_angles, plot_handle, params):
    if not can_plot or plot_handle is None or len(scan_ranges) == 0:
        if "scan_plot" in plot_handle and plot_handle["scan_plot"] is not None: # scan_plot으로 이름 변경
             plot_handle["axis"].removeItem(plot_handle["scan_plot"])
             plot_handle["scan_plot"] = None
        return

    if "scan_plot" not in plot_handle or plot_handle["scan_plot"] is None:
        plot_handle["scan_plot"] = pg.ScatterPlotItem(pen=None, symbol="o", size=3, brush=pg.mkBrush(100, 100, 100, 150)) # 회색 점
        plot_handle["axis"].addItem(plot_handle["scan_plot"])

    # 전역 좌표로 변환된 스캔 포인트
    global_scan_points = convert_scan_to_global_xy(ekf_state, scan_ranges, scan_angles, params)
    
    if global_scan_points.shape[0] > 0:
        plot_handle["scan_plot"].setData(pos=global_scan_points)
    else: # 유효한 포인트가 없으면 기존 데이터 지움
        plot_handle["scan_plot"].clear()


# 로봇 플로팅 (ROS2와 동일)
def plot_robot(ekf_state, plot_handle):
    if not can_plot or plot_handle is None:
        return

    # 로봇 모양 (삼각형) 정의 - 크기 조절 가능
    robot_size = 0.15 # m
    triangle = robot_size * np.array([[1, 0], [-0.5, -0.4], [-0.5, 0.4], [1, 0]]) # 앞쪽이 뾰족한 삼각형

    # 로봇 방향으로 회전
    phi = ekf_state["x"][2]
    R_mat = np.array([[np.cos(phi), -np.sin(phi)], 
                      [np.sin(phi),  np.cos(phi)]])
    rotated_triangle = triangle @ R_mat.T # 각 행 벡터를 회전

    # 로봇 위치로 이동
    translated_triangle = rotated_triangle + ekf_state["x"][:2] # [x,y] 위치 더함

    if "robot_shape" not in plot_handle or plot_handle["robot_shape"] is None: # plot_handle은 딕셔너리
        plot_handle["robot_shape"] = plot_handle["axis"].plot(pen=pg.mkPen('k', width=2))
    
    plot_handle["robot_shape"].setData(translated_triangle[:,0], translated_triangle[:,1])


# 로봇 공분산 플로팅 (ROS2와 동일)
def plot_robot_covariance(ekf_state, plot_handle):
    if not can_plot or plot_handle is None:
        return

    # 로봇 포즈의 공분산 (x,y 부분만)
    P_robot_xy = ekf_state["P"][:2,:2]
    mu_robot_xy = ekf_state["x"][:2]
    
    # 카이제곱 값 (예: 2자유도 95% 신뢰구간 -> chi2.ppf(0.95, 2) approx 5.991)
    # 여기서는 ROS2 코드처럼 3-sigma 사용 (chi2_val=9에 해당)
    ellipse_points = get_covariance_ellipse_points(mu_robot_xy, P_robot_xy, chi2_val=9.0) # 3-sigma
    
    if "robot_cov_ellipse" not in plot_handle or plot_handle["robot_cov_ellipse"] is None:
        plot_handle["robot_cov_ellipse"] = plot_handle["axis"].plot(pen='b') # 파란색 타원
    
    plot_handle["robot_cov_ellipse"].setData(ellipse_points[:,0], ellipse_points[:,1])


# 전체 상태 플로팅 (맵, 로봇, 로봇 공분산)
def plot_state(ekf_state, plot_handle, params):
    if not can_plot or plot_handle is None:
        return
    plot_map(ekf_state, plot_handle, params)
    plot_robot(ekf_state, plot_handle)
    plot_robot_covariance(ekf_state, plot_handle)


# 메인 플로팅 함수 (ROS2와 유사, scan_angles 추가)
def do_plot(xhist, ekf_state, trees, scan_ranges, scan_angles, assoc, plot_handle, params):
    if not can_plot or plot_handle is None:
        return
        
    # 현재 시간 기록 및 업데이트 주기 제어 (CPU 부하 감소)
    current_time = time.time()
    min_update_interval = 0.033  # 약 30 FPS (초당 30회 업데이트)
    
    if current_time - plot_handle.get("last_plot_time", 0) < min_update_interval:
        # 마지막 업데이트 후 충분한 시간이 지나지 않았으면 스킵
        return
        
    # 시간 업데이트
    plot_handle["last_plot_time"] = current_time

    plot_trajectory(xhist, plot_handle)
    plot_state(ekf_state, plot_handle, params) # 맵, 로봇, 로봇 공분산
    plot_tree_measurements(trees, assoc, ekf_state, plot_handle) # 측정된 트리(랜드마크)
    
    if len(scan_ranges) > 0 and params.get("plot_raw_laser", False):
        plot_scan(ekf_state, scan_ranges, scan_angles, plot_handle, params) # 원본 LiDAR 스캔

    # pyqtgraph 이벤트 처리 (main_final1_ros1.py 에서 호출)
    # pg.QtGui.QApplication.processEvents()


# 플롯 초기화 (ROS2와 거의 동일)
def init_plot():
    if not can_plot:
        # rospy 사용 가능하면 rospy.logwarn, 아니면 print
        try:
            import rospy
            rospy.logwarn("Plotting is disabled as pyqtgraph or PyQt5 is not available.")
        except ImportError:
            print("Warning: Plotting is disabled as pyqtgraph or PyQt5 is not available.")
        return None

    pg.setConfigOption("background", "w")
    pg.setConfigOption("foreground", "k")
    
    # Qt 애플리케이션 인스턴스 확인 (이미 생성되었을 수 있음)
    try:
        # PyQt5(Old) - QtGui, PyQt6/PySide6(New) - QtWidgets
        app = QtWidgets.QApplication.instance()
        if app is None:
            # 인스턴스가 없을 때만 생성
            app = pg.mkQApp()
    except Exception as e:
        try:
            import rospy
            rospy.logwarn("Error getting/creating QApplication: %s", str(e))
            # 대체 방법 시도: 직접 pg.mkQApp() 사용
            try:
                app = pg.mkQApp()
                rospy.loginfo("Successfully created QApplication using pg.mkQApp()")
            except Exception as e2:
                rospy.logerr("Failed to create QApplication: %s", str(e2))
                return None
        except ImportError:
            print("Warning: Error getting/creating QApplication:", str(e))
            return None
    
    # GraphicsLayoutWidget 사용
    win = pg.GraphicsLayoutWidget(show=True)
    win.setWindowTitle("EKF SLAM (ROS1)")
    
    axis = win.addPlot()
    axis.setAspectLocked(True) # 가로세로 비율 고정
    axis.setLabel('left', 'Y (m)')
    axis.setLabel('bottom', 'X (m)')
    axis.showGrid(x=True, y=True)
    
    # plot_handle 딕셔너리 초기화
    plot_handle = {
        "win": win,
        "axis": axis,
        "app": app,  # Qt 애플리케이션 인스턴스 저장
        "trajectory": None,
        "map_symbols": None,
        "map_covariances": [],
        "robot_shape": None,
        "robot_cov_ellipse": None,
        "lasers": [], # plot_tree_measurements 용
        "laser_in_axis": [], # plot_tree_measurements 용
        "scan_plot": None, # plot_scan 용
        "last_plot_time": 0, # 마지막 그래픽 업데이트 시간 (CPU 부하 감소용)
        "plot_counter": 0  # 프레임 카운터 (일부 무거운 그래픽은 n프레임마다 갱신)
    }
    return plot_handle


# 각도 정규화 (-pi ~ pi) (ROS2와 동일)
def clamp_angle(theta_rad):
    return (theta_rad + np.pi) % (2 * np.pi) - np.pi

# 대칭 행렬 만들기 (ROS2와 동일)
def make_symmetric(P):
    return 0.5 * (P + P.T)

# 2x2 행렬의 역행렬 (ROS2와 동일, 수치 안정성 고려 가능)
def invert_2x2_matrix(M):
    # M = [[a,b], [c,d]]
    # M_inv = 1/(ad-bc) * [[d, -b], [-c, a]]
    a, b = M[0,0], M[0,1]
    c, d = M[1,0], M[1,1]
    
    determinant = a*d - b*c
    if abs(determinant) < 1e-9: # 매우 작은 행렬식 (특이 행렬에 가까움)
        # rospy 사용 가능하면 rospy.logwarn
        try:
            import rospy
            rospy.logwarn_throttle(1.0, "Determinant is close to zero during 2x2 matrix inversion. Result may be unstable.")
        except ImportError:
            print("Warning: Determinant is close to zero during 2x2 matrix inversion.")
        # 특이 행렬이면 np.linalg.pinv 사용하거나, 매우 큰 값으로 채워진 행렬 반환 가능
        # 여기서는 일단 계산 진행
        if abs(determinant) < 1e-12: # 더 엄격하게 체크해서 문제 방지
            return np.array([[np.inf, np.inf],[np.inf, np.inf]])


    inv_det = 1.0 / determinant
    M_inv = np.array([[d * inv_det, -b * inv_det],
                      [-c * inv_det, a * inv_det]])
    return M_inv

# 비용 행렬 휴리스틱 풀이 (ROS2와 동일)
# 데이터 연관에서 사용될 수 있음 (Global Nearest Neighbor의 변형)
def solve_cost_matrix_heuristic(M_cost):
    # M_cost: (n_measurements, n_landmarks_or_new) 형태의 비용 행렬
    # 각 측정에 대해 가장 비용이 낮은 랜드마크(또는 새 랜드마크 옵션)를 할당하는 탐욕적 방식
    if M_cost.shape[0] == 0:
        return []
        
    n_msmts = M_cost.shape[0]
    assigned_landmarks = [False] * M_cost.shape[1] # 각 랜드마크가 이미 할당되었는지 추적
    
    result_pairs = [] # (measurement_idx, landmark_idx) 저장

    # 각 측정에 대해 가장 비용이 낮은 랜드마크 찾기
    # 측정 순서는 M_cost의 행 순서대로 또는 비용이 낮은 측정부터 등 다양하게 가능
    # ROS2 코드는 M.min(axis=1)의 argsort 사용: 가장 작은 최소 비용을 갖는 측정부터 처리
    
    if n_msmts > 0 and M_cost.shape[1] > 0 : # 측정과 랜드마크/옵션이 모두 있을 때
        # 각 측정(행)에 대한 최소 비용 값들의 순서
        min_costs_for_each_measurement = np.min(M_cost, axis=1)
        measurement_processing_order = np.argsort(min_costs_for_each_measurement)
    else: # 처리할 것이 없음
        return []

    for msmt_idx in measurement_processing_order:
        # 현재 측정에 대한 비용들
        costs_for_this_msmt = M_cost[msmt_idx, :]
        
        # 아직 할당되지 않은 랜드마크/옵션 중에서 가장 비용이 낮은 것 찾기
        best_match_idx = -1
        min_cost_val = np.inf
        
        # 비용 순으로 랜드마크/옵션 정렬
        sorted_potential_matches = np.argsort(costs_for_this_msmt)

        for potential_lmk_idx in sorted_potential_matches:
            if not assigned_landmarks[potential_lmk_idx]: # 아직 할당 안된 경우
                if costs_for_this_msmt[potential_lmk_idx] < min_cost_val : # 더 좋은 매치 (필요 없을수도, argsort 했으므로)
                    min_cost_val = costs_for_this_msmt[potential_lmk_idx]
                    best_match_idx = potential_lmk_idx
                    break # 찾았으므로 종료 (가장 좋은 것 하나만)
        
        if best_match_idx != -1:
            result_pairs.append((msmt_idx, best_match_idx))
            assigned_landmarks[best_match_idx] = True # 이 랜드마크/옵션은 할당됨
            # 만약 한 랜드마크가 여러 측정에 할당될 수 있다면 assigned_landmarks 체크 불필요
            # 하지만 보통은 1:1 또는 N:1(측정:랜드마크) 매칭

    return result_pairs