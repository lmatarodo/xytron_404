# 콘 레인 정보를 담는 메시지 타입

# 메시지 헤더
std_msgs/Header header

# 중앙 경로 포인트들
geometry_msgs/Point[] center_path

# 횡방향 오차 (차량 중심과 경로 중심 사이의 거리)
float32 lateral_error

# 목표 지점 감지 여부
bool target_point_detected

# 목표 지점 좌표
geometry_msgs/Point target_point

# 목표 지점에서의 경로 방향 (radian)
float32 target_heading

# 왼쪽 차선 감지 여부
bool left_lane_detected

# 오른쪽 차선 감지 여부
bool right_lane_detected

# 왼쪽 차선 각도 (degree)
float32 left_lane_degree

# 오른쪽 차선 각도 (degree)
float32 right_lane_degree

# 왼쪽 차선 포인트들
geometry_msgs/Point[] left_lane_points

# 오른쪽 차선 포인트들
geometry_msgs/Point[] right_lane_points 