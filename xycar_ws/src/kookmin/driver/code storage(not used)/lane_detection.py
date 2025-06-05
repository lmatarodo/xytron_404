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
        self.pub = rospy.Publisher("lane_info", laneinfo, queue_size=1)#! /usr/bin/env python3
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

    def get_orange_cone_centers(self, image):
        """주황색 라바콘의 중심점을 추출"""
        try:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 100])
            upper_orange = np.array([20, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            
            # 팽창 적용
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # 전체 이미지에서 라바콘 검출
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            centers = []

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 5:  # 최소 면적 조건
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])  # 오프셋 제거
                        centers.append((cx, cy))
            
            # 중심점을 x좌표 기준으로 정렬
            centers.sort(key=lambda x: x[0])
            return centers
        except Exception as e:
            rospy.logwarn(f"라바콘 중심점 추출 중 오류 발생: {str(e)}")
            return []

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

    def orange_mask(self, image):
        """주황색 영역을 추출한 이진화 이미지 생성"""
        try:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 100])
            upper_orange = np.array([20, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            
            # 팽창 적용 (라바콘 픽셀 확장)
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=2)
            return mask
        except Exception as e:
            rospy.logwarn(f"주황색 마스크 생성 중 오류 발생: {str(e)}")
            return None

    def fit_cones(self, cones):
        """라바콘 중심점들로부터 2차 곡선 계수를 계산"""
        if len(cones) >= 3:
            x = np.array([pt[0] for pt in cones])
            y = np.array([pt[1] for pt in cones])
            return np.polyfit(y, x, 2)  # y 기준
        return None

    def process_image(self, img):
        # Step 1: BEV 변환
        warpped_img = self.warpping(img)

        # Step 2: Blurring을 통해 노이즈를 제거
        blurred_img = cv2.GaussianBlur(warpped_img, (0, 0), 1)

        # Step 3: 색상 필터링 및 이진화 (차선용)
        filtered_img = self.color_filter(blurred_img)
        gray_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(gray_img, 170, 255, cv2.THRESH_BINARY)

        # Step 4: 주황색 라바콘 마스크 생성
        binary_orange_img = self.orange_mask(warpped_img)
        cone_detected = self.detect_orange_cone(warpped_img)  # 라바콘 감지 여부
        cone_centers = self.get_orange_cone_centers(warpped_img) if cone_detected else []

        if cone_detected:
            rospy.loginfo("라바콘 감지됨 (오렌지 픽셀 존재)")
            if len(cone_centers) >= 2:
                rospy.loginfo(f"라바콘 중심점 {len(cone_centers)}개 검출 - 경로 생성 가능")
            else:
                rospy.loginfo(f"라바콘 중심점 {len(cone_centers)}개 검출 - 경로 생성 불가")

        # 결과 이미지 초기화
        out_img = np.dstack((binary_orange_img, binary_orange_img, binary_orange_img)) * 255 if binary_orange_img is not None else np.dstack((binary_img, binary_img, binary_img)) * 255

        # Step 5: 라바콘 또는 차선 기반으로 경로 생성
        if cone_detected and len(cone_centers) >= 2:  # 최소 2개의 라바콘이 필요
            # 라바콘 중심점을 좌우로 분리
            left_cones = []
            right_cones = []
            center_x = warpped_img.shape[1] // 2  # 이미지 중앙 x좌표
            
            for pt in cone_centers:
                if pt[0] < center_x:
                    left_cones.append(pt)
                else:
                    right_cones.append(pt)

            # 좌우 라바콘에 대해 각각 곡선 fitting
            left_fit = self.fit_cones(left_cones)
            right_fit = self.fit_cones(right_cones)

            if left_fit is not None and right_fit is not None:
                # 곡선 포인트 생성 (이미지 전체에 대해)
                h = warpped_img.shape[0]
                ploty = np.linspace(0, h - 1, 15)  # 전체 높이에 15개의 윈도우
                left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
                right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

                # 곡선을 따라 슬라이딩 윈도우 시각화
                for y, lx, rx in zip(ploty, left_fitx, right_fitx):
                    y_low = int(y - 10)
                    y_high = int(y + 10)
                    lx, rx = int(lx), int(rx)
                    # 왼쪽 윈도우
                    cv2.rectangle(out_img, (lx-15, y_low), (lx+15, y_high), (0, 255, 0), 1)
                    # 오른쪽 윈도우
                    cv2.rectangle(out_img, (rx-15, y_low), (rx+15, y_high), (0, 255, 0), 1)

                # 라바콘 중심점 시각화
                for pt in left_cones:
                    cv2.circle(out_img, pt, 5, (0, 0, 255), -1)  # 왼쪽 라바콘: 빨간색
                for pt in right_cones:
                    cv2.circle(out_img, pt, 5, (255, 0, 0), -1)  # 오른쪽 라바콘: 파란색

                rospy.loginfo("✅ 라바콘 기반 곡선 추적 완료 (전체 이미지)")

                # ROS 메시지 생성
                pub_msg = laneinfo()
                
                # 왼쪽 차선 정보 (마지막 윈도우 기준)
                pub_msg.left_x = 130.0 - np.float32(left_fitx[-1])
                pub_msg.left_y = np.float32(ploty[-1])
                pub_msg.left_slope = np.float32(np.arctan(2*left_fit[0]*ploty[-1] + left_fit[1]))

                # 오른쪽 차선 정보 (마지막 윈도우 기준)
                pub_msg.right_x = np.float32(right_fitx[-1]) - 130.0
                pub_msg.right_y = np.float32(ploty[-1])
                pub_msg.right_slope = np.float32(np.arctan(2*right_fit[0]*ploty[-1] + right_fit[1]))

                # 차선 번호 설정 (라바콘 감지 시 1차선으로 가정)
                pub_msg.lane_number = 1
                
                # 라바콘 감지 상태 설정
                pub_msg.cone_detected = True
            else:
                # 곡선 fitting 실패 시 기존 차선 방식으로 전환
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
                
                # 라바콘 감지 상태 설정
                pub_msg.cone_detected = False
        else:
            # 기존 차선 히스토그램 사용
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
            
            # 라바콘 감지 상태 설정
            pub_msg.cone_detected = False

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
        cv2.imshow("raw_img", img_resized)
        cv2.imshow("bird_img", warpped_resized)
        if binary_orange_img is not None:
            orange_resized = cv2.resize(binary_orange_img, (260, 260))
            cv2.imshow("orange_mask", orange_resized)
        cv2.imshow("result_img", out_resized)
        cv2.waitKey(1)
        return pub_msg

if __name__ == "__main__":
    LaneDetect()
    rospy.spin()

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
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin

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

    def process_image(self, img):
        # Step 1: BEV 변환
        warpped_img = self.warpping(img)

        # Step 2: Blurring을 통해 노이즈를 제거
        blurred_img = cv2.GaussianBlur(warpped_img, (0, 0), 1)

        # Step 3: 색상 필터링 및 이진화
        filtered_img = self.color_filter(blurred_img)
        gray_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(gray_img, 170, 255, cv2.THRESH_BINARY)

        # Step 4: 히스토그램
        left_base, right_base, hist = self.plothistogram(binary_img)

        # Step 5: 슬라이딩 윈도우
        draw_info, out_img = self.slide_window_search(binary_img, left_base, right_base)

        # Step 6: 차선 색상 감지
        lane_number = self.detect_lane_color(warpped_img, draw_info['left_fitx'][-1], draw_info['right_fitx'][-1])
        
        # 차선 번호 출력
        if lane_number == 1:
            rospy.loginfo("현재 1차선 주행 중")
        elif lane_number == 2:
            rospy.loginfo("현재 2차선 주행 중")
        else:
            rospy.loginfo("차선 감지 불확실")

        # Step 7: ROS 메시지 생성 및 발행
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
        pub_msg.lane_number = lane_number

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
        cv2.imshow("raw_img", img_resized)
        cv2.imshow("bird_img", warpped_resized)
        #cv2.imshow('blur_img', blurred_resized)
        #cv2.imshow("filter_img", filtered_resized)
        #cv2.imshow("gray_img", gray_resized)
        #cv2.imshow("binary_img", binary_resized)
        cv2.imshow("result_img", out_resized)
        cv2.waitKey(1)
        return pub_msg

if __name__ == "__main__":
    LaneDetect()
    rospy.spin() 