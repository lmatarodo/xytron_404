def convert_lidar_to_xy(current_ranges_data):
    # 입력 데이터 유효성 검사: None이거나 길이가 360이 아니면 빈 배열 반환
    if current_ranges_data is None or len(current_ranges_data) != 360:
        return np.array([])

    # 1. 사용할 각도 인덱스 계산
    # 차량 정면(0도) 기준 좌측으로 LIDAR_ANGLE_RANGE_HALF도까지의 인덱스
    indices_left = np.arange(0, LIDAR_ANGLE_RANGE_HALF + 1)  # 예: 0, 1, ..., 95
    # 차량 정면(0도) 기준 우측으로 LIDAR_ANGLE_RANGE_HALF도까지의 인덱스
    # (360도 체계에서 우측 각도는 큰 값으로 표현됨. 예: -95도는 360-95 = 265도)
    indices_right = np.arange(360 - LIDAR_ANGLE_RANGE_HALF, 360)  # 예: 265, 266, ..., 359

    # 좌측과 우측 인덱스 배열을 합치고, 중복 제거 (0도 등이 중복될 수 있으나, arange 특성상 여기선 중복 없음)
    valid_angular_indices = np.unique(np.concatenate((indices_left, indices_right)))

    # 2. 선택된 각도에 해당하는 거리 데이터 및 각도(라디안) 추출
    # 정의된 유효 각도 인덱스에 해당하는 거리 값들만 선택
    selected_ranges = current_ranges_data[valid_angular_indices]
    # 선택된 각도 인덱스(그 자체로 각도 값을 의미)를 라디안으로 변환
    selected_angles_rad = np.radians(valid_angular_indices.astype(float))

    # 3. 유효한 거리 값 필터링
    # 선택된 거리 값들 중에서 유한한 값(무한대, NaN 제외)이면서,
    # LIDAR_MIN_DIST와 LIDAR_MAX_DIST 범위 내에 있는 값들의 인덱스만 추출
    valid_value_indices = np.where(
        np.isfinite(selected_ranges) &
        (selected_ranges > LIDAR_MIN_DIST) &
        (selected_ranges < LIDAR_MAX_DIST)
    )[0] # where는 튜플을 반환하므로 [0]으로 실제 인덱스 배열 가져옴

    # 유효한 거리 값이 하나도 없으면 빈 배열 반환
    if len(valid_value_indices) == 0:
        return np.array([])

    # 최종적으로 사용할 거리 값들과 해당 각도(라디안)만 선택
    final_ranges = selected_ranges[valid_value_indices]
    final_angles_rad = selected_angles_rad[valid_value_indices]

    # 4. XY 좌표 변환
    # X = R * sin(theta): 차량 좌측이 양수 (+)
    # Y = R * cos(theta): 차량 전방이 양수 (+)
    # (라이다 센서가 차량 정면을 0도로 하고, 반시계 방향으로 각도가 증가하는 것을 가정)
    x_coords = final_ranges * np.sin(final_angles_rad)
    y_coords = final_ranges * np.cos(final_angles_rad)

    # 변환된 X, Y 좌표들을 (N, 2) 형태의 배열로 묶어서 반환
    return np.column_stack((x_coords, y_coords))