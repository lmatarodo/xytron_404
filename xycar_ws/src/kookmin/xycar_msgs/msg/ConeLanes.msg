# ConeLanes.msg

# 메시지 헤더 (타임스탬프, 프레임 ID 포함)
std_msgs/Header header

# 왼쪽 차선 감지 정보
bool left_lane_detected             # 왼쪽 차선 감지 성공 여부
geometry_msgs/Point[] left_lane_points  # 피팅된 왼쪽 차선의 샘플링 포인트 (x, y, z=0) 리스트
int8 left_lane_degree                   # 왼쪽 차선 피팅에 사용된 다항식 차수 (0이면 피팅 안됨 또는 실패)

# 오른쪽 차선 감지 정보
bool right_lane_detected            # 오른쪽 차선 감지 성공 여부
geometry_msgs/Point[] right_lane_points # 피팅된 오른쪽 차선의 샘플링 포인트 (x, y, z=0) 리스트
int8 right_lane_degree                  # 오른쪽 차선 피팅에 사용된 다항식 차수 (0이면 피팅 안됨 또는 실패)

# 중앙 주행 경로
geometry_msgs/Point[] center_path       # 계산된 주행 중앙 경로 포인트 (x, y, z=0) 리스트