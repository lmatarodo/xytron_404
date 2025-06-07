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
        # 차선 감지용 BEV 설정 (기존 유지)
        # 원본 이미지 크기: 640x480
        # ROI 꼭짓점 (원본 이미지 기준) - 좌상, 우상, 좌하, 우하
        source = np.float32([[161,300], [471,300], [0, 390], [630, 390]]) 
        
        bev_width = 260
        bev_height = 260
        destination = np.float32([[0, 0], [bev_width, 0], [0, bev_height], [bev_width, bev_height]])
        
        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        bird_image = cv2.warpPerspective(image, transform_matrix, (bev_width, bev_height))
        return bird_image

    # === [새로 추가됨] 라바콘 감지 전용 BEV 변환 함수 ===
    def warpping_for_cone_detection(self, image):
        # 라바콘 감지용 BEV 설정 (더 넓은 전방 시야)
        # 이 값들은 예시이며, 반드시 실제 환경에 맞게 튜닝해야 합니다.
        # 아이디어: 상단 y를 줄여서 더 멀리, 좌우 x 범위를 넓혀서 더 넓게
        # (주의: source 좌표가 이미지 경계를 너무 많이 벗어나면 왜곡이 심해질 수 있음)
        img_h, img_w = image.shape[:2] # 원본 이미지 높이, 너비 (480, 640)

        # 예시 source 좌표 (튜닝 필요)
        # 좌상, 우상, 좌하, 우하
        # 상단 y를 300 -> 260으로 변경 (더 멀리)
        # 하단 y를 390 -> 380으로 약간 변경
        # 좌우 폭을 더 넓게 (예: x좌표를 바깥쪽으로 이동)
        source_cone = np.float32([
            [100, 260],  # 좌상 (기존 161, 300)
            [img_w - 100, 260],  # 우상 (기존 471, 300)
            [0, img_h - 100],      # 좌하 (기존 0, 390) -> y를 380으로
            [img_w - 0, img_h - 100]       # 우하 (기존 630, 390) -> y를 380으로
        ])
        # 위 좌표는 (x,y) 순서. 원본 이미지의 좌상단이 (0,0)
        # 더 공격적인 전방 시야 확보 예시:
        # source_cone = np.float32([
        #     [50, 240], 
        #     [img_w - 50, 240], 
        #     [-100, img_h - 80], # 좌우로 더 넓게 (이미지 밖으로 나가도 괜찮음)
        #     [img_w + 100, img_h - 80]
        # ])


        bev_width = 260  # BEV 이미지 크기는 동일하게 유지하거나 변경 가능
        bev_height = 260
        destination_cone = np.float32([[0, 0], [bev_width, 0], [0, bev_height], [bev_width, bev_height]])
        
        transform_matrix_cone = cv2.getPerspectiveTransform(source_cone, destination_cone)
        bird_image_cone = cv2.warpPerspective(image, transform_matrix_cone, (bev_width, bev_height))
        return bird_image_cone
    # =====================================================

    def color_filter(self, image):
        # 흰색 차선 필터링 (HSV)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 180]) 
        upper_white = np.array([180, 30, 255]) 
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        masked = cv2.bitwise_and(image, image, mask=white_mask)
        
        cv2.rectangle(masked, (80, 180), (180, 260), (0, 0, 0), -1) 
        return masked

    def plothistogram(self, image):
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        midpoint = np.int_(histogram.shape[0]/2)
        leftbase = np.argmax(histogram[:midpoint])
        rightbase = np.argmax(histogram[midpoint:]) + midpoint
        return leftbase, rightbase, histogram

    def slide_window_search(self, binary_warped, left_current, right_current):
        nwindows = 15 
        window_height = np.int_(binary_warped.shape[0] / nwindows) 
        nonzero = binary_warped.nonzero() 
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        
        margin = 30 
        minpix = 10 
        
        left_lane_inds = []
        right_lane_inds = []

        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255 

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            
            win_xleft_low = max(0, left_current - margin) 
            win_xleft_high = min(binary_warped.shape[1], left_current + margin) 
            win_xright_low = max(0, right_current - margin) 
            win_xright_high = min(binary_warped.shape[1], right_current + margin) 

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 1)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 1)

            good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                              (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                               (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

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

        left_fit = [0, binary_warped.shape[1]//4] 
        if len(leftx) > 2 and len(lefty) > 2: 
            try:
                left_fit = np.polyfit(lefty, leftx, 1)
            except np.linalg.LinAlgError: 
                rospy.logwarn("Left lane fitting failed.")
                pass 

        right_fit = [0, binary_warped.shape[1]*3//4] 
        if len(rightx) > 2 and len(righty) > 2: 
            try:
                right_fit = np.polyfit(righty, rightx, 1)
            except np.linalg.LinAlgError: 
                rospy.logwarn("Right lane fitting failed.")
                pass 
        
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty + left_fit[1]
        right_fitx = right_fit[0] * ploty + right_fit[1]

        for i in range(len(ploty)):
            if 0 <= int(left_fitx[i]) < binary_warped.shape[1] and 0 <= int(ploty[i]) < binary_warped.shape[0]:
                 cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)
            if 0 <= int(right_fitx[i]) < binary_warped.shape[1] and 0 <= int(ploty[i]) < binary_warped.shape[0]:
                 cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)

        return {'left_fitx': left_fitx, 'left_slope': left_fit[0], 
                'right_fitx': right_fitx, 'right_slope': right_fit[0], 
                'ploty': ploty}, out_img

    def detect_lane_color(self, image_bev, left_x_bev, right_x_bev):
        try:
            hsv = cv2.cvtColor(image_bev, cv2.COLOR_BGR2HSV)
            white_lower = np.array([0, 0, 200]) 
            white_upper = np.array([180, 50, 255]) 
            
            height, width = image_bev.shape[:2] 
            roi_y_start = int(height * 0.7) 
            roi_y_end = height
            roi_width_half = 10 

            left_white_sum = 0
            lx = int(left_x_bev)
            if roi_width_half <= lx < width - roi_width_half : 
                left_roi_bev = hsv[roi_y_start:roi_y_end, lx - roi_width_half : lx + roi_width_half]
                left_white_mask_bev = cv2.inRange(left_roi_bev, white_lower, white_upper)
                left_white_sum = np.sum(left_white_mask_bev)
            else:
                rospy.logdebug("Left lane ROI for color check is out of BEV image bounds.")

            right_white_sum = 0
            rx = int(right_x_bev)
            if roi_width_half <= rx < width - roi_width_half:
                right_roi_bev = hsv[roi_y_start:roi_y_end, rx - roi_width_half : rx + roi_width_half]
                right_white_mask_bev = cv2.inRange(right_roi_bev, white_lower, white_upper)
                right_white_sum = np.sum(right_white_mask_bev)
            else:
                rospy.logdebug("Right lane ROI for color check is out of BEV image bounds.")
            
            threshold_white_pixels = 50 
            is_left_white = left_white_sum > threshold_white_pixels
            is_right_white = right_white_sum > threshold_white_pixels

            if is_left_white and not is_right_white: 
                return 2 
            elif not is_left_white and is_right_white: 
                return 1
            elif is_left_white and is_right_white: 
                return 1 
            else: 
                return 0 
                
        except Exception as e:
            rospy.logwarn(f"Error in detect_lane_color: {str(e)}")
            return 0 

    def detect_orange_cone(self, image_bev_for_cone): # 입력 이미지 변수명 변경
        """BEV 이미지에서 주황색 라바콘 감지 (모드 전환용)"""
        try:
            hsv = cv2.cvtColor(image_bev_for_cone, cv2.COLOR_BGR2HSV)
            lower_orange1 = np.array([0, 100, 100]) 
            upper_orange1 = np.array([10, 255, 255]) 
            lower_orange2 = np.array([170, 100, 100]) 
            upper_orange2 = np.array([179, 255, 255]) 
            
            mask1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
            mask2 = cv2.inRange(hsv, lower_orange2, upper_orange2)
            orange_mask = cv2.bitwise_or(mask1, mask2)

            return np.any(orange_mask > 0)
        except Exception as e:
            rospy.logwarn(f"Error in detect_orange_cone: {str(e)}")
            return False

    def process_image(self, img):
        # Step 1a: 차선 감지용 BEV 변환
        warpped_img_lane = self.warpping(img)

        # === [수정됨] Step 1b: 라바콘 감지용 BEV 변환 ===
        warpped_img_cone = self.warpping_for_cone_detection(img)
        # ===============================================

        # Step 2: 차선용 이미지 Blurring (노이즈 제거)
        blurred_img_lane = cv2.GaussianBlur(warpped_img_lane, (5, 5), 0) # 가우시안 블러 추가

        # Step 3: 차선용 색상 필터링 및 이진화 (흰색 차선)
        filtered_lane_img = self.color_filter(blurred_img_lane) 
        gray_lane_img = cv2.cvtColor(filtered_lane_img, cv2.COLOR_BGR2GRAY)
        _, binary_lane_img = cv2.threshold(gray_lane_img, 100, 255, cv2.THRESH_BINARY) 

        # === [수정됨] Step 4: 주황색 라바콘 감지 (라바콘 감지용 BEV 사용) ===
        cone_detected_by_camera = self.detect_orange_cone(warpped_img_cone) # 입력 변경
        if cone_detected_by_camera:
             rospy.loginfo("Camera: Orange Cone Detected!")
        # ===============================================================

        # Step 5: 차선 히스토그램 및 슬라이딩 윈도우 (차선 감지용 BEV 사용)
        left_base, right_base, _ = self.plothistogram(binary_lane_img)
        draw_info, out_img_viz_lane = self.slide_window_search(binary_lane_img, left_base, right_base)

        # Step 6: 차선 번호 결정 (차선 감지용 BEV와 감지된 차선 x좌표 사용)
        lane_number = self.detect_lane_color(warpped_img_lane, 
                                             draw_info['left_fitx'][-1], 
                                             draw_info['right_fitx'][-1])

        pub_msg = laneinfo()
        img_center_x_bev = warpped_img_lane.shape[1] / 2.0 
        
        pub_msg.left_x = img_center_x_bev - draw_info['left_fitx'][-1]
        pub_msg.left_y = draw_info['ploty'][-1] 
        slope_left_rad = np.arctan(draw_info['left_slope']) if abs(draw_info['left_slope']) < 100 else 0 
        pub_msg.left_slope = slope_left_rad

        pub_msg.right_x = draw_info['right_fitx'][-1] - img_center_x_bev
        pub_msg.right_y = draw_info['ploty'][-1]
        slope_right_rad = np.arctan(draw_info['right_slope']) if abs(draw_info['right_slope']) < 100 else 0
        pub_msg.right_slope = slope_right_rad
        
        pub_msg.lane_number = lane_number
        pub_msg.cone_detected_flag = cone_detected_by_camera

        # 디버깅용 이미지 표시
        try:
            cv2.imshow("raw_img (640x480)", cv2.resize(img, (640,480)))
            cv2.imshow("bev_for_lane (260x260)", warpped_img_lane)
            # === [수정됨] 라바콘 감지용 BEV 시각화 추가 ===
            cv2.imshow("bev_for_cone_detection (260x260)", warpped_img_cone) 
            # ============================================
            cv2.imshow("lane_detection_result (260x260)", out_img_viz_lane) 
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
