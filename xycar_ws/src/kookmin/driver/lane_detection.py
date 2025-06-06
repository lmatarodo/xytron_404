#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import laneinfo


class LaneDetect:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('lane_detection_node', anonymous=False)

        # ROS Subscriber & Publisher
        rospy.Subscriber('/usb_cam/image_raw/', Image, self.camera_callback, queue_size=1)
        self.pub = rospy.Publisher("lane_info", laneinfo, queue_size=1)

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #img_bottom =     이렇게 y절반으로 위아래 나눠서 스티어(아래) + alpha*스티어(위)    where alpha < 1 하는 방법으로 제어는 어떨련지..?
        #img_top =        그러면 안에 내부 함수에 하드코딩 되어있는 숫자값들을 변수로들로 바꿔야할 것 같습니다요
        lane_info = self.process_image(img)
        self.pub.publish(lane_info)

    def warpping(self, image): #y=−0.55794x+390.00000 #y=0.56522x+33.91304
        source = np.float32([[161,300], [471,300], [0, 390], [630, 390]]) #순서대로 좌상/우상/좌하/우하
        #source = np.float32([[180,300], [450,300], [0, 420], [639, 420]]) #순서대로 좌상/우상/좌하/우하
        #source = np.float32([[181,280], [431,280], [20, 390], [630, 390]])
        #source = np.float32([[234, 260], [400, 260], [0, 390], [630, 390]])
        #source = np.float32([[237, 260], [400, 260], [0, 390], [630, 390]]) #처음뽑은 값
        #source = np.float32([[280, 280], [520, 280], [0, 430], [800, 430]]) #original
        destination = np.float32([[0, 0], [260, 0], [0, 260], [260, 260]])
        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        bird_image = cv2.warpPerspective(image, transform_matrix, (260, 260))
        return bird_image

    def color_filter(self, image):
        lower = np.array([0, 255, 255]) #수정
        upper = np.array([255, 255, 255])
        white_mask = cv2.inRange(image, lower, upper)
        masked = cv2.bitwise_and(image, image, mask=white_mask)
        cv2.rectangle(masked, (80, 80), (180, 260), (0, 0, 0), -1)
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
        left_lane = []
        right_lane = []

        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height
            win_y_high = binary_warped.shape[0] - w * window_height
            
            # 윈도우 좌표를 이미지 범위 내로 제한
            win_xleft_low = max(0, left_current - margin)
            win_xleft_high = min(binary_warped.shape[1] - 1, left_current + margin)
            win_xright_low = max(0, right_current - margin)
            win_xright_high = min(binary_warped.shape[1] - 1, right_current + margin)

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                        (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                        (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]

            if len(good_left) > minpix:
                left_lane.append(good_left)
                left_current = np.int_(np.mean(nonzero_x[good_left]))  

            if len(good_right) > minpix:
                right_lane.append(good_right)
                right_current = np.int_(np.mean(nonzero_x[good_right]))

        left_lane = np.concatenate(left_lane) if len(left_lane) > 0 else np.array([])
        right_lane = np.concatenate(right_lane) if len(right_lane) > 0 else np.array([])
        leftx = nonzero_x[left_lane] if len(left_lane) > 0 else np.array([])
        lefty = nonzero_y[left_lane] if len(left_lane) > 0 else np.array([])
        rightx = nonzero_x[right_lane] if len(right_lane) > 0 else np.array([])
        righty = nonzero_y[right_lane] if len(right_lane) > 0 else np.array([])

        if len(leftx) > 0 and len(lefty) > 0:
            left_fit = np.polyfit(lefty, leftx, 1)
        else:
            left_fit = [0, 0]

        if len(rightx) > 0 and len(righty) > 0:
            right_fit = np.polyfit(righty, rightx, 1)
        else:
            right_fit = [0, 0]

        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty + left_fit[1]
        right_fitx = right_fit[0] * ploty + right_fit[1]

        for i in range(len(ploty)):
            cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)
            cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)

        return {'left_fitx': left_fitx, 'left_slope': left_fit[0], 'right_fitx': right_fitx, 'right_slope': right_fit[0], 'ploty': ploty}, out_img

    def detect_lane_color(self, image, left_x, right_x):
        try:
            # HSV 색상 공간으로 변환
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # 흰색 범위 정의
            white_lower = np.array([0, 0, 200])
            white_upper = np.array([180, 30, 255])
            
            # 이미지 크기 확인
            height, width = image.shape[:2]
            
            # 왼쪽 차선 영역이 이미지 내에 있는지 확인
            left_x = int(left_x)
            if 10 <= left_x < width - 10:
                left_roi = hsv[int(height*0.8):, left_x-10:left_x+10]
                left_white = cv2.inRange(left_roi, white_lower, white_upper)
            else:
                return 0  # 왼쪽 차선이 이미지 범위를 벗어남
            
            # 오른쪽 차선 영역이 이미지 내에 있는지 확인
            right_x = int(right_x)
            if 10 <= right_x < width - 10:
                right_roi = hsv[int(height*0.8):, right_x-10:right_x+10]
                right_white = cv2.inRange(right_roi, white_lower, white_upper)
            else:
                return 0  # 오른쪽 차선이 이미지 범위를 벗어남
            
            # 차선 번호 결정 (흰색 차선의 위치만으로 판단)
            left_white_sum = np.sum(left_white)
            right_white_sum = np.sum(right_white)
            
            # 흰색 차선이 더 많이 감지된 쪽을 기준으로 판단
            if left_white_sum > right_white_sum:
                return 1  # 흰색 차선이 왼쪽에 있으면 1차선
            elif right_white_sum > left_white_sum:
                return 2  # 흰색 차선이 오른쪽에 있으면 2차선
            else:
                return 0  # 불확실한 경우
                
        except Exception as e:
            rospy.logwarn(f"차선 색상 감지 중 오류 발생: {str(e)}")
            return 0  # 오류 발생 시 0 반환

    def detect_orange_cone(self, image):
        """버드아이뷰 이미지에서 주황색 라바콘을 감지"""
        try:
            # HSV 색상 공간으로 변환
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 100])
            upper_orange = np.array([20, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            
            # 오렌지 픽셀이 하나라도 있으면 라바콘이 있다고 판단
            return np.any(mask > 0)
        except Exception as e:
            rospy.logwarn(f"라바콘 감지 중 오류 발생: {str(e)}")
            return False

    def warpping_for_cone_detection(self, image):
        """라바콘 감지를 위한 더 넓은 전방 시야의 BEV 변환 함수"""
        img_h, img_w = image.shape[:2] # 원본 이미지 높이, 너비 (480, 640)

        # v0에서 사용된 더 넓은 ROI 좌표 (필요시 튜닝)
        # 아이디어: 차선 감지보다 y값을 낮춰(위로) 더 먼 곳을, x값 범위를 넓혀 더 넓은 영역을 봄
        source_cone = np.float32([
            [100, 260],                  # 좌상 (더 멀리, 더 넓게)
            [img_w - 100, 260],          # 우상 (더 멀리, 더 넓게)
            [0, img_h - 100],            # 좌하
            [img_w, img_h - 100]         # 우하
        ])
        
        destination_cone = np.float32([[0, 0], [260, 0], [0, 260], [260, 260]])
        transform_matrix_cone = cv2.getPerspectiveTransform(source_cone, destination_cone)
        bird_image_cone = cv2.warpPerspective(image, transform_matrix_cone, (260, 260))
        return bird_image_cone

    def process_image(self, img):
        # Step 1: BEV 변환
        warpped_img = self.warpping(img)
        #warpped_img_cone = self.warpping_for_cone_detection(img)

        # Step 2: Blurring을 통해 노이즈를 제거
        blurred_img = cv2.GaussianBlur(warpped_img, (0, 0), 1)

        # Step 3: 색상 필터링 및 이진화 (차선용)
        filtered_img = self.color_filter(blurred_img)
        gray_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(gray_img, 170, 255, cv2.THRESH_BINARY)

        # Step 4: 주황색 라바콘 감지
        cone_detected = self.detect_orange_cone(warpped_img)
        #cone_detected = self.detect_orange_cone(warpped_img_cone)

        if cone_detected:
            rospy.loginfo("라바콘 감지됨 (오렌지 픽셀 존재)")

        left_base, right_base, hist = self.plothistogram(binary_img)
        draw_info, out_img = self.slide_window_search(binary_img, left_base, right_base)
        
        # ROS 메시지 생성
        pub_msg = laneinfo()
        
        # 왼쪽 차선 정보
        pub_msg.left_x = 130.0 - np.float32(draw_info['left_fitx'][-1])
        pub_msg.left_y = np.float32(draw_info['ploty'][-1])
        slope_left = draw_info['left_slope']
        pub_msg.left_slope = np.float32(np.arctan(slope_left))

        # 오른쪽 차선 정보 
        pub_msg.right_x = np.float32(draw_info['right_fitx'][-1]) - 130.0
        pub_msg.right_y = np.float32(draw_info['ploty'][-1])
        slope_right = draw_info['right_slope']
        pub_msg.right_slope = np.float32(np.arctan(slope_right))

        # 차선 번호 설정
        pub_msg.lane_number = self.detect_lane_color(warpped_img, draw_info['left_fitx'][-1], draw_info['right_fitx'][-1])
        pub_msg.cone_detected_flag = cone_detected

        # 이미지 크기 조절
        display_size = (640, 480)
        img_resized = cv2.resize(img, display_size)
        warpped_resized = cv2.resize(warpped_img, (260, 260))
        blurred_resized = cv2.resize(blurred_img, (260, 260))
        filtered_resized = cv2.resize(filtered_img, (260, 260))
        gray_resized = cv2.resize(gray_img, (260, 260))
        binary_resized = cv2.resize(binary_img, (260, 260))
        out_resized = cv2.resize(out_img, (260, 260))

        # 디버깅용 이미지 표시
        #cv2.imshow("raw_img", img_resized)
        #cv2.imshow("bird_img", warpped_resized)
        cv2.imshow("result_img", out_resized)
        cv2.waitKey(1)
        return pub_msg

if __name__ == "__main__":
    LaneDetect()
    rospy.spin()