DETECTION_COUNT    = 6      # 연속으로 이만큼 포인트가 잡히면 차량으로 판정 

SECTOR_WIDTH       = 7      # 앞 0번 인덱스 기준으로 ±2 인덱스 조사 

DETECTION_STREAK_THRESHOLD = 2     # 스캔이 연속으로 성공해야 할 횟수 

 

# 차선 번호 관련 변수 

current_lane = 0  # 현재 차선 번호 (0: 불확실, 1: 1차선, 2: 2차선) 

is_lane_changing = False  # 차선 변경 중인지 여부 

lane_change_path = []  # 차선 변경 경로 

lane_change_direction = 0  # 차선 변경 방향 (1: 우회전, -1: 좌회전) 

lane_change_start_time = 0  # 차선 변경 시작 시간 

last_lane_change_time = 0  # 마지막 차선 변경 시간 

 

# 속도 증가 관련 전역 변수 

is_speed_boosted = False 

speed_boost_start_time = 0 

speed_boost_duration = 10.0  # 5초 동안 속도 증가 유지 

has_speed_boosted = False  # 전체 구간에서 속도 증가를 한 번만 하기 위한 플래그 

 

# 차선 감지 관련 변수 추가 

last_left_lane_detected = True 

last_right_lane_detected = True 

left_lane_missing_time = 0 

right_lane_missing_time = 0 

LANE_MISSING_THRESHOLD = 2.0  # 차선 미감지 시 차선 변경 시도 시간 (초) 

 

# 이전 조향값 저장을 위한 변수들 

steering_history = []  # 이전 조향값들을 저장할 리스트 

MAX_HISTORY_SIZE = 10  # 저장할 최대 조향값 개수 

 

#LANE_CHANGE_INTERVAL = 6.0  #폐기 # 차선 변경 간격 (초) 

LANE_CHANGE_DURATION = 2.0  #3.0  차선 변경 소요 시간 (초) 

PATH_POINTS = 20  #30 # 경로 생성 시 샘플링할 포인트 수 

LANE_CHANGE_DISTANCE = 300 #200  # 차선 변경 시 전방 주시 거리 

 

 

ef lane_callback(data): 

    global lane_data, current_lane, is_lane_changing, lane_change_path 

    global lane_change_direction, lane_change_start_time, last_lane_change_time 

    global has_speed_boosted, is_speed_boosted 

    lane_data = data 

     

    # 현재 시간 확인 

    current_time = time.time() 

 

    # 실측 lane 번호 업데이트 

    if data.lane_number in [1, 2]: 

        current_lane = data.lane_number 

 

    # 차선 변경 조건 

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

 

 

def kanayama_control(): 

    global lane_data, current_lane, is_lane_changing 

    global lane_change_path, lane_change_direction, lane_change_start_time 

    global L, last_left_lane_detected, last_right_lane_detected 

    global left_lane_missing_time, right_lane_missing_time 

    global is_speed_boosted, speed_boost_start_time, has_speed_boosted 

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

            # 차선 변경 완료 후 속도 증가 시작 (한 번만) 

            if not has_speed_boosted: 

                is_speed_boosted = True 

                speed_boost_start_time = current_time 

                has_speed_boosted = True 

                rospy.loginfo("속도 증가 시작: 1.8배 속도로 가속") 

                return 0.0, Fix_Speed * 1.2  # 1.5배 속도로 시작 

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

            increased_speed = Fix_Speed 

            steering_angle = steering_angle 

            return steering_angle, increased_speed 

 

    # 속도 증가 중인 경우 

    if is_speed_boosted: 

        current_time = time.time() 

        if current_time - speed_boost_start_time > speed_boost_duration: 

            is_speed_boosted = False 

            rospy.loginfo("속도 증가 종료: 정상 속도로 복귀") 

        else: 

            rospy.loginfo("속도 증가 중: %.1f초 남음", speed_boost_duration - (current_time - speed_boost_start_time)) 

            # 속도만 증가시키고 조향은 정상적으로 계산 

            steering_angle, _ = calculate_normal_steering() 

            return steering_angle, Fix_Speed * 1.2  # 1.5배 속도 유지 

 

    # 기존의 kanayama_control 로직 

    return calculate_normal_steering() 

 

def calculate_normal_steering(): 

    """정상적인 조향각 계산 로직""" 

    global last_left_lane_detected, last_right_lane_detected 

    global left_lane_missing_time, right_lane_missing_time 

    global lane_data, L, is_speed_boosted, steering_history 

 

    left_x = lane_data.left_x 

    right_x = lane_data.right_x 

    left_slope = lane_data.left_slope 

    right_slope = lane_data.right_slope 

    lane_width = 3.5 

     

    # 파라미터 

    K_y = 0.85 

    K_phi = 3.0 

    v_r = Fix_Speed 

     

    # 속도 증가 시 조향 제어 파라미터 조정 

    if is_speed_boosted: 

        K_y = 0.75  # 횡방향 오차에 대한 이득 감소 

        K_phi = 2.5  # 방향 오차에 대한 이득 감소 

     

    current_time = time.time() 

     

    if left_x == 130 and right_x == -130: 

        rospy.logwarn("Both lanes lost, using average steering angle.") 

        if steering_history: 

            avg_steering = sum(steering_history) / len(steering_history) 

            rospy.loginfo("평균 조향각 사용: %.1f도", avg_steering) 

            return avg_steering, Fix_Speed 

        else: 

            return 0.0, Fix_Speed 

    elif left_x == 130: 

        # 왼쪽 차선이 감지되지 않을 때 

        if not last_left_lane_detected: 

            # 이전에도 왼쪽 차선이 없었다면 시간 체크 

            if left_lane_missing_time == 0: 

                left_lane_missing_time = current_time 

            elif current_time - left_lane_missing_time >= LANE_MISSING_THRESHOLD: 

                # 2초 이상 왼쪽 차선이 없으면 오른쪽으로 차선 변경 

                lateral_err = -(0.5 - (right_x / 150.0)) * lane_width 

                heading_err = right_slope 

                steering_angle = 15.0 if is_speed_boosted else 20.0  # 속도 증가 시 조향각 감소 

                rospy.loginfo("왼쪽 차선 2초 이상 미감지: 오른쪽으로 차선 변경 시도 (조향각: %.1f도)", steering_angle) 

                return steering_angle, v_r 

        else: 

            # 이전에는 왼쪽 차선이 있었던 경우 

            left_lane_missing_time = current_time 

            last_left_lane_detected = False 

             

        lateral_err = -(0.5 - (right_x / 150.0)) * lane_width 

        heading_err = right_slope 

    elif right_x == -130: 

        # 오른쪽 차선이 감지되지 않을 때 

        if not last_right_lane_detected: 

            # 이전에도 오른쪽 차선이 없었다면 시간 체크 

            if right_lane_missing_time == 0: 

                right_lane_missing_time = current_time 

            elif current_time - right_lane_missing_time >= LANE_MISSING_THRESHOLD: 

                # 2초 이상 오른쪽 차선이 없으면 왼쪽으로 차선 변경 

                lateral_err = (0.5 - (left_x / 150.0)) * lane_width 

                heading_err = left_slope 

                steering_angle = -15.0 if is_speed_boosted else -20.0  # 속도 증가 시 조향각 감소 

                rospy.loginfo("오른쪽 차선 2초 이상 미감지: 왼쪽으로 차선 변경 시도 (조향각: %.1f도)", steering_angle) 

                return steering_angle, v_r 

        else: 

            # 이전에는 오른쪽 차선이 있었던 경우 

            right_lane_missing_time = current_time 

            last_right_lane_detected = False 

             

        lateral_err = (0.5 - (left_x / 150.0)) * lane_width 

        heading_err = left_slope 

    else: 

        # 양쪽 차선 모두 감지된 경우 

        last_left_lane_detected = True 

        last_right_lane_detected = True 

        left_lane_missing_time = 0 

        right_lane_missing_time = 0 

         

        # 0으로 나누는 상황 방지 

        if abs(left_x + right_x) < 0.1:  # 매우 작은 값으로 설정 

            rospy.logwarn("차선 위치가 중앙에 너무 가까움, 이전 조향각 유지") 

            if steering_history: 

                avg_steering = sum(steering_history) / len(steering_history) 

                return avg_steering, v_r 

            return 0.0, v_r  # 현재 속도 유지하면서 조향각 0 반환 

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

 

    # 속도 증가 시 조향각 제한을 더 엄격하게 설정 

    if is_speed_boosted: 

        steering_angle = max(min(steering_angle, 20.0), -20.0)  # 속도 증가 시 조향각 제한 

    else: 

        steering_angle = max(min(steering_angle, 50.0), -50.0)  # 일반적인 조향각 제한 

 

    # 현재 조향각을 히스토리에 추가 

    steering_history.append(steering_angle) 

    if len(steering_history) > MAX_HISTORY_SIZE: 

        steering_history.pop(0)  # 가장 오래된 값 제거 

 

    return steering_angle, v 