#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import laneinfo # 사용자 정의 메시지

class LaneDetect:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('lane_detection_node', anonymous=False)

        # ROS Subscriber & Publisher
        rospy.Subscriber('/usb_cam/image_raw/', Image, self.camera_callback, queue_size=1)
        self.pub = rospy.Publisher("lane_info", laneinfo, queue_size=1)
        rospy.loginfo("Lane Detection Node Initialized.")

    def camera_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            lane_info_msg = self.process_image(img)
            if lane_info_msg:
                self.pub.publish(lane_info_msg)
        except Exception as e:
            rospy.logerr(f"Error in camera_callback: {e}")


    def warpping(self, image):
        # 원본 이미지 크기: 640x480
        # ROI 꼭짓점 (원본 이미지 기준) - 값들은 차량 및 카메라 설치에 따라 조정 필요
        # 좌상, 우상, 좌하, 우하
        source = np.float32([[161,300], [471,300], [0, 390], [630, 390]]) 
        
        # Bird's Eye View 이미지 크기
        bev_width = 260
        bev_height = 260
        destination = np.float32([[0, 0], [bev_width, 0], [0, bev_height], [bev_width, bev_height]])
        
        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        bird_image = cv2.warpPerspective(image, transform_matrix, (bev_width, bev_height))
        return bird_image

    def color_filter(self, image):
        # 흰색 차선 필터링 (HSV)
        # 이 부분은 흰색 차선만 검출하도록 되어있음. 실제 환경에 따라 노란색도 추가 필요할 수 있음.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 180]) # HSV 값 범위 조정 필요
        upper_white = np.array([180, 30, 255]) # 
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        masked = cv2.bitwise_and(image, image, mask=white_mask)
        
        # 차량 앞부분 영역(예: 후드) 마스킹 (BEV 이미지 기준)
        # 이 좌표는 warpping 후의 bev_width, bev_height (260x260) 기준
        # (80, 80)에서 (180, 260) 영역을 검은색으로 칠함 -> (x1,y1), (x2,y2)
        # y좌표가 클수록 이미지 하단(차량에 가까운 쪽)
        # 이 영역은 차선이 아닌 차량 일부가 나올 수 있는 영역을 제거하기 위함.
        # (80, 180) 부터 (180, 260)으로 수정 (y 시작점을 좀 더 아래로)
        cv2.rectangle(masked, (80, 180), (180, 260), (0, 0, 0), -1) 
        return masked

    def plothistogram(self, image):
        # 이미지 하단 절반 영역에 대한 히스토그램
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        midpoint = np.int_(histogram.shape[0]/2)
        leftbase = np.argmax(histogram[:midpoint])
        rightbase = np.argmax(histogram[midpoint:]) + midpoint
        return leftbase, rightbase, histogram

    def slide_window_search(self, binary_warped, left_current, right_current):
        nwindows = 15 # 윈도우 개수
        window_height = np.int_(binary_warped.shape[0] / nwindows) 
        nonzero = binary_warped.nonzero() # 0이 아닌 픽셀의 인덱스
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        
        margin = 30 # 윈도우 좌우 마진
        minpix = 10 # 윈도우 내 최소 픽셀 수 (차선으로 인정)
        
        left_lane_inds = []
        right_lane_inds = []

        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255 # 시각화용

        for window in range(nwindows):
            # 윈도우 y 범위
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            
            # 왼쪽 윈도우 x 범위
            win_xleft_low = max(0, left_current - margin) # 이미지 경계 체크
            win_xleft_high = min(binary_warped.shape[1], left_current + margin) # 이미지 경계 체크
            # 오른쪽 윈도우 x 범위
            win_xright_low = max(0, right_current - margin) # 이미지 경계 체크
            win_xright_high = min(binary_warped.shape[1], right_current + margin) # 이미지 경계 체크

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 1)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 1)

            # 현재 윈도우 내에 있는 0이 아닌 픽셀들의 인덱스
            good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                              (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                               (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # 다음 윈도우의 중심 업데이트 (minpix 이상 감지 시)
            if len(good_left_inds) > minpix:
                left_current = np.int_(np.mean(nonzero_x[good_left_inds]))
            if len(good_right_inds) > minpix:        
                right_current = np.int_(np.mean(nonzero_x[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds) if len(left_lane_inds) > 0 else np.array([])
        right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) > 0 else np.array([])

        leftx = nonzero_x[left_lane_inds] if len(left_lane_inds) > 0 else np.array([])
        lefty = nonzero_y[left_lane_inds] if len(left_lane_inds) > 0 else np.array([]) 
        rightx = nonzero_x[right_lane_inds] if len(right_lane_inds) > 0 else np.array([])
        righty = nonzero_y[right_lane_inds] if len(right_lane_inds) > 0 else np.array([])

        # 차선 포인트가 충분히 있을 때만 1차 다항식 피팅 (직선)
        left_fit = [0, binary_warped.shape[1]//4] # 기본값: 왼쪽 1/4 지점 수직선
        if len(leftx) > 2 and len(lefty) > 2: # 최소 3점 이상 필요
            try:
                left_fit = np.polyfit(lefty, leftx, 1)
            except np.linalg.LinAlgError: # 피팅 실패 시
                rospy.logwarn("Left lane fitting failed.")
                pass # 기본값 유지

        right_fit = [0, binary_warped.shape[1]*3//4] # 기본값: 오른쪽 3/4 지점 수직선
        if len(rightx) > 2 and len(righty) > 2: # 최소 3점 이상 필요
            try:
                right_fit = np.polyfit(righty, rightx, 1)
            except np.linalg.LinAlgError: # 피팅 실패 시
                rospy.logwarn("Right lane fitting failed.")
                pass # 기본값 유지
        
        # 피팅된 차선 시각화용 y좌표 생성
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty + left_fit[1]
        right_fitx = right_fit[0] * ploty + right_fit[1]

        # 피팅된 차선 시각화 (노란색 점)
        for i in range(len(ploty)):
            if 0 <= int(left_fitx[i]) < binary_warped.shape[1] and 0 <= int(ploty[i]) < binary_warped.shape[0]:
                 cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)
            if 0 <= int(right_fitx[i]) < binary_warped.shape[1] and 0 <= int(ploty[i]) < binary_warped.shape[0]:
                 cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)

        return {'left_fitx': left_fitx, 'left_slope': left_fit[0], 
                'right_fitx': right_fitx, 'right_slope': right_fit[0], 
                'ploty': ploty}, out_img

    def detect_lane_color(self, image, left_x_bev, right_x_bev):
        """BEV 이미지상의 차선 위치를 기반으로 원본 이미지에서 차선 색상 감지 (1차선/2차선 구분용)"""
        try:
            # BEV 좌표 (left_x_bev, right_x_bev)는 이미지 하단 기준.
            # 이 함수는 BEV 이미지(warpped_img)를 입력으로 받음
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # 흰색 범위 정의 (HSV) - 값은 환경에 따라 튜닝 필요
            white_lower = np.array([0, 0, 200]) 
            white_upper = np.array([180, 50, 255]) # Saturation 상한 약간 높임
            
            height, width = image.shape[:2] # BEV 이미지 크기 (260x260)
            
            # ROI 설정: BEV 이미지의 y 중간 ~ 하단, x는 감지된 차선 주변
            roi_y_start = int(height * 0.7) # 이미지 하단 30% 영역
            roi_y_end = height
            roi_width_half = 10 # 차선 주변 x축 ROI 폭 (픽셀)

            left_white_sum = 0
            # 왼쪽 차선 영역 (BEV 이미지 기준)
            # left_x_bev는 BEV 이미지의 x좌표
            lx = int(left_x_bev)
            if roi_width_half <= lx < width - roi_width_half : # ROI가 이미지 범위 내에 있는지 확인
                left_roi_bev = hsv[roi_y_start:roi_y_end, lx - roi_width_half : lx + roi_width_half]
                left_white_mask_bev = cv2.inRange(left_roi_bev, white_lower, white_upper)
                left_white_sum = np.sum(left_white_mask_bev)
            else:
                rospy.logdebug("Left lane ROI for color check is out of BEV image bounds.")


            right_white_sum = 0
            # 오른쪽 차선 영역 (BEV 이미지 기준)
            rx = int(right_x_bev)
            if roi_width_half <= rx < width - roi_width_half:
                right_roi_bev = hsv[roi_y_start:roi_y_end, rx - roi_width_half : rx + roi_width_half]
                right_white_mask_bev = cv2.inRange(right_roi_bev, white_lower, white_upper)
                right_white_sum = np.sum(right_white_mask_bev)
            else:
                rospy.logdebug("Right lane ROI for color check is out of BEV image bounds.")

            # 차선 번호 결정 (흰색 픽셀 수 비교)
            # 흰색은 주로 실선 또는 점선. 노란색은 중앙선.
            # 여기서는 흰색만 감지하므로, 흰색이 더 많은 쪽을 기준으로 판단.
            # (가정: 1차선은 왼쪽이 황색, 오른쪽이 백색. 2차선은 왼쪽이 백색, 오른쪽이 황색 또는 백색)
            # 이 로직은 단순화된 버전이며, 실제 도로 환경에 맞게 수정 필요.
            # 여기서는 단순히 흰색 픽셀이 많은 쪽을 기준으로 1, 2차선을 임의로 할당.
            # 더 정확하려면 노란색도 감지해야 함.
            
            threshold_white_pixels = 50 # 흰색으로 판단하기 위한 최소 픽셀 수 (조정 필요)

            is_left_white = left_white_sum > threshold_white_pixels
            is_right_white = right_white_sum > threshold_white_pixels

            if is_left_white and not is_right_white: # 왼쪽만 흰색 -> 2차선으로 가정 (왼쪽이 차선, 오른쪽이 도로 경계 또는 황색선)
                return 2 
            elif not is_left_white and is_right_white: # 오른쪽만 흰색 -> 1차선으로 가정 (오른쪽이 차선, 왼쪽이 도로 경계 또는 황색선)
                return 1
            elif is_left_white and is_right_white: # 양쪽 다 흰색 -> 주행 중인 차로의 중앙으로 가정, 임의로 1차선 또는 이전 차선 유지
                return 1 # 또는 이전 차선 정보 사용
            else: # 둘 다 흰색 아님 (또는 매우 적음)
                return 0 # 불확실
                
        except Exception as e:
            rospy.logwarn(f"Error in detect_lane_color: {str(e)}")
            return 0 # 오류 발생 시 0 반환

    # === [수정됨] 라바콘 감지 관련 함수 ===
    def detect_orange_cone(self, image_bev):
        """BEV 이미지에서 주황색 라바콘 감지 (모드 전환용)"""
        try:
            hsv = cv2.cvtColor(image_bev, cv2.COLOR_BGR2HSV)
            # 주황색 HSV 범위 (값은 환경에 따라 튜닝)
            # 예시: Hue(0-179), Sat(0-255), Val(0-255)
            lower_orange1 = np.array([0, 100, 100]) 
            upper_orange1 = np.array([10, 255, 255]) # 낮은 Hue 범위 (빨간색에 가까운 주황)
            lower_orange2 = np.array([170, 100, 100]) 
            upper_orange2 = np.array([179, 255, 255]) # 높은 Hue 범위 (빨간색에 가까운 주황)
            
            mask1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
            mask2 = cv2.inRange(hsv, lower_orange2, upper_orange2)
            orange_mask = cv2.bitwise_or(mask1, mask2)

            # 주황색 픽셀이 하나라도 있으면 라바콘이 있다고 판단
            return np.any(orange_mask > 0)
        except Exception as e:
            rospy.logwarn(f"Error in detect_orange_cone: {str(e)}")
            return False
    # ====================================

    def process_image(self, img):
        # Step 1: BEV 변환
        warpped_img = self.warpping(img)

        # Step 2: Blurring (노이즈 제거)
        # blurred_img = cv2.GaussianBlur(warpped_img, (5, 5), 0) # 가우시안 블러 추가
        blurred_img = warpped_img # 일단 원본 사용, 필요시 활성화

        # Step 3: 차선용 색상 필터링 및 이진화 (흰색 차선)
        filtered_lane_img = self.color_filter(blurred_img) # 흰색 필터링
        gray_lane_img = cv2.cvtColor(filtered_lane_img, cv2.COLOR_BGR2GRAY)
        _, binary_lane_img = cv2.threshold(gray_lane_img, 100, 255, cv2.THRESH_BINARY) # 임계값 조정

        # === [수정됨] Step 4: 주황색 라바콘 감지 (카메라 기반, 모드 전환용) ===
        cone_detected_by_camera = self.detect_orange_cone(warpped_img) # 원본 warpped_img 사용
        if cone_detected_by_camera:
             rospy.loginfo("Camera: Orange Cone Detected!")
        # ===============================================================

        # Step 5: 차선 히스토그램 및 슬라이딩 윈도우 (차선 주행용)
        # 이진화된 차선 이미지를 사용
        left_base, right_base, _ = self.plothistogram(binary_lane_img)
        draw_info, out_img_viz = self.slide_window_search(binary_lane_img, left_base, right_base)

        # Step 6: 차선 번호 결정 (BEV 이미지와 감지된 차선 x좌표 사용)
        # draw_info['left_fitx'][-1]는 BEV 이미지 하단에서의 왼쪽 차선 x좌표
        lane_number = self.detect_lane_color(warpped_img, 
                                             draw_info['left_fitx'][-1], 
                                             draw_info['right_fitx'][-1])

        # ROS 메시지 생성
        pub_msg = laneinfo()
        
        # 왼쪽 차선 정보 (BEV 이미지 하단 기준)
        # left_x: 이미지 중앙(130)에서 왼쪽 차선까지의 거리 (픽셀), 오른쪽이 +
        # draw_info['left_fitx'][-1]는 BEV 이미지 왼쪽 상단 (0,0) 기준 x좌표
        # 차량 중심이 BEV 이미지 x축 130이라고 가정
        img_center_x_bev = warpped_img.shape[1] / 2.0 
        
        # left_x: 차량 중심 기준 왼쪽 차선의 x좌표. 차량 중심보다 왼쪽에 있으면 음수, 오른쪽에 있으면 양수.
        # XYCAR 기준: left_x는 차량 중심(0)으로부터 왼쪽 차선까지의 x 거리. (오른쪽이 +)
        # 즉, (차량중심 x좌표) - (왼쪽차선 x좌표)
        # 여기서는 BEV 이미지 x=130이 차량 중심.
        # draw_info['left_fitx'][-1]는 이미지 왼쪽부터의 x좌표.
        # 값이 130보다 작으면 차량 중심보다 왼쪽.
        pub_msg.left_x = img_center_x_bev - draw_info['left_fitx'][-1]
        pub_msg.left_y = draw_info['ploty'][-1] # BEV 이미지 하단 y좌표 (거의 259)
        slope_left_rad = np.arctan(draw_info['left_slope']) if abs(draw_info['left_slope']) < 100 else 0 # 기울기 매우 크면 0
        pub_msg.left_slope = slope_left_rad

        # 오른쪽 차선 정보
        # right_x: 차량 중심 기준 오른쪽 차선의 x좌표. 차량 중심보다 왼쪽에 있으면 음수, 오른쪽에 있으면 양수.
        # XYCAR 기준: right_x는 차량 중심(0)으로부터 오른쪽 차선까지의 x 거리. (오른쪽이 +)
        # 즉, (오른쪽차선 x좌표) - (차량중심 x좌표)
        pub_msg.right_x = draw_info['right_fitx'][-1] - img_center_x_bev
        pub_msg.right_y = draw_info['ploty'][-1]
        slope_right_rad = np.arctan(draw_info['right_slope']) if abs(draw_info['right_slope']) < 100 else 0
        pub_msg.right_slope = slope_right_rad
        
        # 차선 번호 설정
        pub_msg.lane_number = lane_number
        
        # === [수정됨] 라바콘 감지 플래그 설정 ===
        pub_msg.cone_detected_flag = cone_detected_by_camera
        # =====================================

        # 디버깅용 이미지 표시
        try:
            cv2.imshow("raw_img (640x480)", cv2.resize(img, (640,480)))
            cv2.imshow("bird_img (260x260)", warpped_img)
            # cv2.imshow("lane_binary (260x260)", binary_lane_img) # 차선 이진화 결과
            cv2.imshow("lane_detection_result (260x260)", out_img_viz) # 슬라이딩 윈도우 결과
            cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn(f"CV2 imshow error: {e}")
            
        return pub_msg

if __name__ == "__main__":
    try:
        ld_node = LaneDetect()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lane Detection node terminated.")
    finally:
        cv2.destroyAllWindows()