#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

# 이 함수는 이제 scan_ranges (거리 배열)와 scan_angles (각도 배열)을 직접 받습니다.
def extract_trees(scan_ranges, scan_angles, filter_params):
    """
    LiDAR 스캔 데이터로부터 트리(랜드마크)를 추출합니다.
    scan_ranges: LiDAR 거리 값 배열 (numpy array)
    scan_angles: 각 거리 값에 해당하는 각도 배열 (numpy array, 라디안 단위, 센서 좌표계)
    filter_params: 필터링 및 추출에 사용될 파라미터 딕셔너리
    반환값: 추출된 트리 정보 리스트. 각 요소는 [range, bearing, diameter] 형태.
           실제로는 [중심까지의 range, 중심까지의 bearing, 직경]을 반환해야 함.
           현재 코드는 dL5를 직경으로, Rs를 중심으로의 거리로, (A5+A5u)/2를 중심으로의 각도로 반환.
    """

    # 파라미터 로드 (filter_params 딕셔너리 또는 기본값 사용)
    # 이 파라미터들은 ROS 파라미터 서버를 통해 main_final1_ros1.py에서 설정될 수 있음
    # 여기서는 filter_params에서 직접 가져오거나, 없으면 기본값 사용하도록 수정 가능
    # 예: M11 = filter_params.get("tree_M11", 1.2)
    # 현재는 하드코딩된 값을 사용 (ROS2 코드와 동일하게 유지)

    # M11: 고려할 최대 거리. 이보다 먼 포인트는 무시. (max_laser_range와 유사)
    M11 = filter_params.get("max_laser_range", 10.0) # main_final1의 filter_params와 연동
    # M10: 고려할 최소 거리. 이보다 가까운 포인트는 무시.
    M10 = filter_params.get("tree_M10", 0.1)
    # daa: 유효 세그먼트의 각도 제한 관련 (원래 180도 스캔 기준이었을 수 있음)
    #      360도 스캔에서는 이 파라미터의 의미나 적용 방식 재검토 필요할 수 있음.
    #      현재는 원본 로직 유지.
    daa = filter_params.get("tree_daa_deg", 10.0) * np.pi / 180.0 # 각도로 받고 라디안으로 변환
    # M2: 연속된 포인트 간 최대 허용 거리 점프. 이보다 크면 다른 세그먼트.
    M2 = filter_params.get("tree_M2", 0.01)
    # M2a: 연속된 포인트 간 최대 허용 각도 점프.
    M2a = filter_params.get("tree_M2a_deg", 2.5) * np.pi / 180.0
    # M3: 세그먼트 병합/정리 시 사용되는 거리 임계값.
    M3 = filter_params.get("tree_M3", 0.07)
    # M5: 유효한 트리로 간주될 최소 직경(또는 길이).
    M5 = filter_params.get("tree_M5", 0.05)
    # daMin2: 시점에서 볼 때 두 객체(세그먼트)가 얼마나 가까이 붙어있을 수 있는지에 대한 최소 각도.
    #         이보다 가까우면 하나(더 멀리 있는 것)를 제거.
    daMin2 = filter_params.get("tree_daMin2_deg", 2.0) * np.pi / 180.0


    RR_input = np.asarray(scan_ranges)
    AA_input = np.asarray(scan_angles)

    if len(RR_input) == 0 or len(AA_input) == 0 or len(RR_input) != len(AA_input):
        return []

    # 1단계: 유효한 거리의 포인트들만 선택 (M11 이내)
    # 추가로 M10 (최소 거리)도 고려 가능. ROS2 코드는 M11만 사용.
    # 여기서는 M11 (최대) 와 M10 (최소) 모두 적용
    valid_indices_initial = np.where((RR_input < M11) & (RR_input > M10))[0]
    
    if len(valid_indices_initial) < 1:
        return []
        
    R1 = RR_input[valid_indices_initial] # 유효 거리값들
    A1 = AA_input[valid_indices_initial] # 해당 각도들
    
    # 2단계: 거리 또는 각도 점프가 큰 곳을 찾아 세그먼트 분리 지점(ii2) 식별
    # np.diff(R1)은 R1[i+1]-R1[i] 배열. np.diff(A1)도 마찬가지.
    # 점프 지점은 diff 결과의 인덱스 i에 해당. 실제로는 R1, A1의 i와 i+1 사이.
    # np.flatnonzero는 True인 위치의 인덱스를 반환.
    jump_indices = np.flatnonzero((np.abs(np.diff(R1)) > M2) | (np.diff(A1) > M2a))
    
    # 세그먼트 시작/끝 인덱스 생성
    # 각 점프 지점의 다음 포인트가 새 세그먼트의 시작.
    # 첫번째 세그먼트 시작은 0. 마지막 세그먼트 끝은 L1-1.
    segment_start_indices = np.insert(jump_indices + 1, 0, 0)
    segment_end_indices = np.append(jump_indices, len(R1) - 1)
    
    num_segments_L2 = len(segment_start_indices)
    if num_segments_L2 == 0 : # jump_indices가 비어있고, R1도 비어있으면 발생 가능 (위에서 처리됨)
        return []

    # 각 세그먼트의 시작점(A2,R2)과 끝점(A2u,R2u) 정보 추출
    A2  = A1[segment_start_indices] # 각 세그먼트 시작 각도
    R2  = R1[segment_start_indices] # 각 세그먼트 시작 거리
    A2u = A1[segment_end_indices]   # 각 세그먼트 끝 각도
    R2u = R1[segment_end_indices]   # 각 세그먼트 끝 거리

    # 세그먼트 시작/끝점의 직교 좌표 (로컬 센서 좌표계)
    x2  = R2  * np.cos(A2)
    y2  = R2  * np.sin(A2)
    x2u = R2u * np.cos(A2u)
    y2u = R2u * np.sin(A2u)

    # 3단계: 너무 짧거나 중복되는 세그먼트 제거 (flag 사용)
    # flag 배열: 각 세그먼트가 유효한지(0) 아닌지(1) 표시
    flag = np.zeros(num_segments_L2, dtype=int)
    M3_squared = M3*M3

    # 인접 세그먼트 간의 끝점-시작점 거리 체크 (최대 3개 건너뜀)
    if num_segments_L2 > 1:
        for k_skip in range(1, min(4, num_segments_L2)): # 1, 2, 3칸 건너뜀
            # dx2 = x2[k:] - x2u[:-k] => (i+k)번째 시작점과 i번째 끝점 비교
            dx_seg_ends = x2[k_skip:] - x2u[:-k_skip]
            dy_seg_ends = y2[k_skip:] - y2u[:-k_skip]
            dist_sq_seg_ends = dx_seg_ends*dx_seg_ends + dy_seg_ends*dy_seg_ends
            
            # 거리가 M3 이내면 관련된 두 세그먼트 모두 flag=1 (제거 대상)
            close_indices = np.flatnonzero(dist_sq_seg_ends < M3_squared)
            if len(close_indices) > 0:
                flag[close_indices] = 1       # i번째 세그먼트
                flag[close_indices + k_skip] = 1 # (i+k)번째 세그먼트
    
    # 각도상으로 매우 가까운 세그먼트 처리 (daMin2 사용)
    if num_segments_L2 > 1:
        # (i+1)번째 세그먼트 시작 각도와 i번째 세그먼트 끝 각도 차이
        angular_diff_between_segments = A2[1:] - A2u[:-1]
        # 이 차이가 daMin2보다 작으면 두 세그먼트가 매우 가까움
        very_close_angularly_indices = np.flatnonzero(angular_diff_between_segments < daMin2)
        
        if len(very_close_angularly_indices) > 0:
            # 둘 중 더 멀리 있는 세그먼트를 제거 대상으로 flag=1
            # R2[idx+1] (다음 세그먼트 시작 거리) vs R2u[idx] (현재 세그먼트 끝 거리)
            # ff = (R2[very_close_angularly_indices+1] > R2u[very_close_angularly_indices])
            # target_indices_to_flag = very_close_angularly_indices + ff # True(1)이면 다음것, False(0)이면 현재것
            
            # 원본 MATLAB 코드: ff = (R2(ii3+1)>R2u(ii3)); ii3=ii3+ff; flag(ii3)=1;
            # R2(i+1) > R2u(i) 이면, (i+1)번째 세그먼트가 더 뒤에 있음. 얘를 flag.
            # 아니면, i번째 세그먼트가 더 뒤에 있거나 비슷. 이때는 i번째를 flag. (MATLAB은 +ff로 인덱스 조정)
            # 여기서는 더 간단히: 둘 다 플래그하거나, 둘 중 하나를 선택.
            # 원본은 더 뒤에 있는 것을 선택해서 플래그함.
            for idx in very_close_angularly_indices:
                if R2[idx+1] > R2u[idx]: # 다음 것이 더 멀면 다음 것 플래그
                    flag[idx+1] = 1
                else: # 현재 것이 더 멀거나 비슷하면 현재 것 플래그
                    flag[idx] = 1


    # flag=0 인 유효한 세그먼트들만 선택 (R4, A4 등)
    valid_segment_indices_after_flag = np.flatnonzero(flag == 0)
    if len(valid_segment_indices_after_flag) == 0:
        return []

    # 유효한 세그먼트들의 정보 R4, A4, x4, y4 (시작점), R4u, A4u, x4u, y4u (끝점)
    # 그리고 이 세그먼트들이 원래 R1, A1에서 몇번째 포인트부터 몇번째까지였는지 (ii4, ii4u)
    # ii4, ii4u는 R1, A1에서의 인덱스.
    # segment_start_indices[valid_segment_indices_after_flag]
    # segment_end_indices[valid_segment_indices_after_flag]
    
    ii4  = segment_start_indices[valid_segment_indices_after_flag]
    ii4u = segment_end_indices[valid_segment_indices_after_flag]
    
    R4  = R1[ii4]  # R2[valid_segment_indices_after_flag] 와 동일
    A4  = A1[ii4]
    x4  = x2[valid_segment_indices_after_flag]
    y4  = y2[valid_segment_indices_after_flag]
    R4u = R1[ii4u] # R2u[valid_segment_indices_after_flag] 와 동일
    A4u = A1[ii4u]
    x4u = x2u[valid_segment_indices_after_flag]
    y4u = y2u[valid_segment_indices_after_flag]

    # 4단계: 세그먼트 길이(직경) 필터링 (M5 사용)
    # 세그먼트 시작점과 끝점 사이의 직선 거리 (dl2)
    dx_seg_len = x4u - x4 # 끝x - 시작x
    dy_seg_len = y4u - y4 # 끝y - 시작y
    dist_sq_seg_len = dx_seg_len*dx_seg_len + dy_seg_len*dy_seg_len
    
    # M5보다 짧은 세그먼트는 제거
    # 원본: dl2 < (M5*M5) 인 것을 선택. 즉, 길이가 M5 이내인 것.
    # 여기서는 길이가 M5 이상인 것을 선택해야 함. dl2 >= (M5*M5)
    # ROS2 코드에서는 dl2 < (M5*M5) 인것을 ii5로 선택했음. (이것은 직경이 M5보다 작은것)
    # 그리고 L5 = len(ii5) < 1 이면 리턴. 즉, M5보다 작은 세그먼트가 없으면 리턴? -> 아님.
    # M5는 최소 직경이므로, dl2 (세그먼트 길이 제곱)가 (M5*M5) 보다 커야 함.
    # ROS2 코드의 ii5 = np.flatnonzero(dl2<(M5*M5))는 "길이가 M5보다 작은 세그먼트"를 찾음.
    # 그리고 if L5 < 1 (M5보다 작은 세그먼트가 없으면) return []. -> 이건 반대.
    # 아마도 M5는 최대 직경이었거나, 코드가 약간 혼동됨.
    # 일반적인 트리/기둥 추출에서는 최소 직경을 사용. 즉, dl2 > M5*M5.
    # 원본 MATLAB 코드: ii5 = find(dl2 < M5c); ... L5=length(ii5); if L5<1, Trees=[]; return; end;
    # 이 부분은 "세그먼트의 직선 길이가 M5보다 작은 것들만 고려한다"는 의미.
    # 즉, M5는 최대 허용 길이/직경. 나무 같은 작은 물체를 찾기 위함.
    # 따라서, dl2 < (M5*M5)가 맞음.
    
    valid_length_indices = np.flatnonzero(dist_sq_seg_len < (M5*M5))
    if len(valid_length_indices) == 0:
        return []

    # M5 길이 조건을 만족하는 세그먼트들 (R5, A5 등)
    R5  = R4[valid_length_indices]
    A5  = A4[valid_length_indices]
    R5u = R4u[valid_length_indices]
    A5u = A4u[valid_length_indices]
    ii4_filtered  = ii4[valid_length_indices]  # R1, A1에서의 시작 인덱스
    ii4u_filtered = ii4u[valid_length_indices] # R1, A1에서의 끝 인덱스
    
    # 세그먼트의 실제 길이 (호 길이 추정) dL5
    # (세그먼트 끝각도 - 시작각도) * (시작거리+끝거리)/2
    # A5, A5u는 로컬 센서 각도. (A5u - A5)가 각도 폭.
    # 여기서는 (A5u + np.pi/360 - A5) 사용. 미세 조정값?
    # ROS2 코드: dL5 = (A5u + np.pi/360 - A5) * (R5+R5u)/2
    # 이 dL5가 직경으로 사용됨.
    # A5u와 A5는 같은 세그먼트의 끝각도, 시작각도.
    # 각도 차이는 항상 양수여야 함. (A5u > A5 가정)
    # 만약 스캔 방향에 따라 A5u < A5일 수 있다면 abs() 필요.
    # 또는 scan_angles가 단조증가한다고 가정.
    angular_span = A5u - A5
    # 각도차가 음수면 2pi 더해서 양수로 만들거나, clamp_angle 처리 필요.
    # 여기서는 scan_angles가 정렬되어 있다고 가정하고 진행.
    # 작은 각도 보정값 (pi/360 = 0.5도)은 왜 있는지 불명확. 노이즈 처리일수도.
    # 여기서는 원본대로.
    dL5 = (angular_span + (np.pi/360.0)) * (R5 + R5u) / 2.0


    # 5단계: 추가 필터링 (daa, 거리 비교 등)
    # daa: 유효 각도 범위 필터링 (원래 전방 180도 스캔 기준일 수 있음)
    # (A5 + np.pi/2 > daa) & (A5u < (np.pi - daa))
    # A5, A5u는 센서 로컬 각도. 만약 0도가 전방, 시계반대방향으로 증가한다면,
    # A5는 -pi/2 ~ pi/2 (ROS2의 AA = np.linspace(-np.pi/2, np.pi/2, len(RR)) 가정시)
    # scan_angles가 0 ~ 2pi 범위라면 이 조건은 재해석 필요.
    # 현재 scan_angles는 0 ~ 2pi로 가정하고 들어옴.
    # 이 필터는 아마도 센서의 특정 FOV 내의 객체만 고려하려는 의도.
    # 예를 들어 로봇 전방의 객체.
    # 여기서는 이 필터를 일단 그대로 적용. (사용자가 알고리즘 유지 원함)
    # R5 > M10 (최소 거리 이상) 조건도 여기서 다시 체크됨.
    
    # ROS2 코드의 조건: ( (R5>M10) & (A5+np.pi/2>daa) & (A5u<(np.pi-daa)) )
    # A5, A5u가 0~2pi 범위라면, A5+pi/2는 pi/2 ~ 2.5pi.
    # daa가 작은 각도(예: 10도)라면, A5+pi/2 > daa는 거의 항상 참.
    # A5u < (pi-daa)는 A5u가 pi보다 약간 작은 값까지 허용. (즉, 로봇의 왼쪽 반원)
    # 이 필터는 360도 스캔에서는 특정 방향의 물체만 선택하게 될 수 있음.
    # 주석: 이 필터는 센서의 전방(-90도 ~ +90도)을 가정하고 만들어진 것일 수 있습니다.
    #       A5, A5u가 0~2pi 범위의 전역 각도라면, 이 조건은 센서 기준 특정 방향을 선택합니다.
    #       예: daa=10도. A5u < 170도. 즉, 센서의 0도(x축 양의방향)부터 반시계로 170도 이내.
    #       그리고 A5+90도 > 10도. A5 > -80도. (0~360 범위에서는 항상 참)
    #       결국 A5u < pi-daa 정도의 조건이 주로 작용.
    
    # 여기서는 필터링 조건을 단순화하거나, 파라미터로 FOV를 명시적으로 받는 것이 좋을 수 있음.
    # 일단은 원본 로직을 최대한 따름.
    final_filter_indices = np.flatnonzero(
        (R5 > M10) & \
        # 아래 두 조건은 원래 A가 -pi/2 ~ pi/2 범위일 때 의미가 있었을 것.
        # 0 ~ 2pi 범위에서는 재해석 필요. 여기서는 일단 적용.
        ( (A5 + np.pi/2.0) > daa ) & \
        ( A5u < (np.pi - daa) )
    )

    if len(final_filter_indices) == 0:
        return []

    R5_final  = R5[final_filter_indices]
    A5_final  = A5[final_filter_indices]
    R5u_final = R5u[final_filter_indices]
    A5u_final = A5u[final_filter_indices]
    ii4_final  = ii4_filtered[final_filter_indices]
    ii4u_final = ii4u_filtered[final_filter_indices]
    dL5_final = dL5[final_filter_indices] # 계산된 직경/길이

    # 추가 조건: 세그먼트 시작/끝 거리 차이와 직경 비교
    # compa = ( np.abs(R5-R5u) < (dL5/3) )
    # 이 조건은 세그먼_트가 너무 가늘거나 하지 않은지, 즉 시작과 끝의 깊이 차이가
    # 계산된 폭(dL5)에 비해 너무 크지 않은지를 보는 것.
    # 나무 기둥 같은 경우 R5와 R5u가 비슷해야 함.
    is_compact_segment = (np.abs(R5_final - R5u_final) < (dL5_final / 3.0))
    
    compact_indices = np.flatnonzero(is_compact_segment)
    if len(compact_indices) == 0:
        return []

    # 최종 선택된 세그먼트들의 정보
    # Rs: 세그먼트 중심까지의 평균 거리 (R1의 중간값 사용)
    # angles: 세그먼트 중심까지의 평균 각도 ((A5+A5u)/2)
    # diameters: 세그먼트 직경 (dL5)

    # R1에서 해당 세그먼트의 중간 인덱스 계산
    # auxi = (ii4+ii4u)/2; iia=floor(auxi); iib=ceil(auxi); Rs=(R1(iia)+R1(iib))/2
    # ii4_final, ii4u_final은 R1 배열에서의 시작/끝 인덱스
    mid_indices_float = (ii4_final[compact_indices].astype(float) + ii4u_final[compact_indices].astype(float)) / 2.0
    mid_indices_floor = np.floor(mid_indices_float).astype(int)
    mid_indices_ceil = np.ceil(mid_indices_float).astype(int)
    
    # R1의 인덱스가 범위를 벗어나지 않도록 클리핑
    max_R1_idx = len(R1) - 1
    mid_indices_floor = np.clip(mid_indices_floor, 0, max_R1_idx)
    mid_indices_ceil = np.clip(mid_indices_ceil, 0, max_R1_idx)

    # 세그먼트 중심까지의 거리 Rs (R1의 중간 두 포인트 평균)
    # Rs = (R1[mid_indices_floor] + R1[mid_indices_ceil]) / 2.0
    # 또는 더 간단히 (R5_final + R5u_final)/2 로 근사 가능.
    # ROS2 코드는 R1의 중간값을 썼으므로 이를 따름.
    Rs_at_center = (R1[mid_indices_floor] + R1[mid_indices_ceil]) / 2.0

    # 최종 반환될 랜드마크 정보
    # 거리: Rs + dL5/2 (중심점까지의 거리 Rs에 반지름 dL5/2를 더함? -> Rs가 중심으로의 거리여야 함)
    #       ROS2 코드: ranges = Rs + dL5 / 2.0. 이것은 물체 표면까지의 거리로 보임.
    #       EKF에서는 보통 랜드마크 '중심'까지의 거리와 방위각을 사용.
    #       여기서는 Rs_at_center를 중심으로의 거리로 사용.
    final_ranges = Rs_at_center
    final_angles = (A5_final[compact_indices] + A5u_final[compact_indices]) / 2.0
    final_diameters = dL5_final[compact_indices]
    
    # 각도 정규화 (-pi ~ pi)
    final_angles = (final_angles + np.pi) % (2 * np.pi) - np.pi


    # 유효한 직경을 가진 것만 최종 반환 (dL5_final > 0)
    # dL5_final은 이미 양수라고 가정 (angular_span이 양수이므로)
    # 만약 angular_span이 0에 가까우면 dL5도 0에 가까울 수 있음.
    # 매우 작은 직경은 의미 없으므로, 최소 직경 M5와 비교해서도 필터링 가능.
    # (이미 M5는 세그먼트 길이 필터링에 사용됨)
    # 여기서는 dL5가 유효하다고 가정.
    
    # zip으로 묶어서 리스트로 반환
    extracted_landmarks = []
    for r, ang, dia in zip(final_ranges, final_angles, final_diameters):
        if dia > 1e-3 : # 매우 작은 직경은 제외 (수치 오류 방지)
             extracted_landmarks.append([r, ang, dia])
             
    return extracted_landmarks