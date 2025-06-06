;; Auto-generated. Do not edit!


(when (boundp 'xycar_msgs::ConeLanes)
  (if (not (find-package "XYCAR_MSGS"))
    (make-package "XYCAR_MSGS"))
  (shadow 'ConeLanes (find-package "XYCAR_MSGS")))
(unless (find-package "XYCAR_MSGS::CONELANES")
  (make-package "XYCAR_MSGS::CONELANES"))

(in-package "ROS")
;;//! \htmlinclude ConeLanes.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass xycar_msgs::ConeLanes
  :super ros::object
  :slots (_header _left_lane_detected _left_lane_points _left_lane_degree _right_lane_detected _right_lane_points _right_lane_degree _center_path _lateral_error _target_point_detected _target_point _target_heading ))

(defmethod xycar_msgs::ConeLanes
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_lane_detected __left_lane_detected) nil)
    ((:left_lane_points __left_lane_points) ())
    ((:left_lane_degree __left_lane_degree) 0)
    ((:right_lane_detected __right_lane_detected) nil)
    ((:right_lane_points __right_lane_points) ())
    ((:right_lane_degree __right_lane_degree) 0)
    ((:center_path __center_path) ())
    ((:lateral_error __lateral_error) 0.0)
    ((:target_point_detected __target_point_detected) nil)
    ((:target_point __target_point) (instance geometry_msgs::Point :init))
    ((:target_heading __target_heading) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_lane_detected __left_lane_detected)
   (setq _left_lane_points __left_lane_points)
   (setq _left_lane_degree (round __left_lane_degree))
   (setq _right_lane_detected __right_lane_detected)
   (setq _right_lane_points __right_lane_points)
   (setq _right_lane_degree (round __right_lane_degree))
   (setq _center_path __center_path)
   (setq _lateral_error (float __lateral_error))
   (setq _target_point_detected __target_point_detected)
   (setq _target_point __target_point)
   (setq _target_heading (float __target_heading))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_lane_detected
   (&optional (__left_lane_detected :null))
   (if (not (eq __left_lane_detected :null)) (setq _left_lane_detected __left_lane_detected)) _left_lane_detected)
  (:left_lane_points
   (&rest __left_lane_points)
   (if (keywordp (car __left_lane_points))
       (send* _left_lane_points __left_lane_points)
     (progn
       (if __left_lane_points (setq _left_lane_points (car __left_lane_points)))
       _left_lane_points)))
  (:left_lane_degree
   (&optional __left_lane_degree)
   (if __left_lane_degree (setq _left_lane_degree __left_lane_degree)) _left_lane_degree)
  (:right_lane_detected
   (&optional (__right_lane_detected :null))
   (if (not (eq __right_lane_detected :null)) (setq _right_lane_detected __right_lane_detected)) _right_lane_detected)
  (:right_lane_points
   (&rest __right_lane_points)
   (if (keywordp (car __right_lane_points))
       (send* _right_lane_points __right_lane_points)
     (progn
       (if __right_lane_points (setq _right_lane_points (car __right_lane_points)))
       _right_lane_points)))
  (:right_lane_degree
   (&optional __right_lane_degree)
   (if __right_lane_degree (setq _right_lane_degree __right_lane_degree)) _right_lane_degree)
  (:center_path
   (&rest __center_path)
   (if (keywordp (car __center_path))
       (send* _center_path __center_path)
     (progn
       (if __center_path (setq _center_path (car __center_path)))
       _center_path)))
  (:lateral_error
   (&optional __lateral_error)
   (if __lateral_error (setq _lateral_error __lateral_error)) _lateral_error)
  (:target_point_detected
   (&optional (__target_point_detected :null))
   (if (not (eq __target_point_detected :null)) (setq _target_point_detected __target_point_detected)) _target_point_detected)
  (:target_point
   (&rest __target_point)
   (if (keywordp (car __target_point))
       (send* _target_point __target_point)
     (progn
       (if __target_point (setq _target_point (car __target_point)))
       _target_point)))
  (:target_heading
   (&optional __target_heading)
   (if __target_heading (setq _target_heading __target_heading)) _target_heading)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _left_lane_detected
    1
    ;; geometry_msgs/Point[] _left_lane_points
    (apply #'+ (send-all _left_lane_points :serialization-length)) 4
    ;; int8 _left_lane_degree
    1
    ;; bool _right_lane_detected
    1
    ;; geometry_msgs/Point[] _right_lane_points
    (apply #'+ (send-all _right_lane_points :serialization-length)) 4
    ;; int8 _right_lane_degree
    1
    ;; geometry_msgs/Point[] _center_path
    (apply #'+ (send-all _center_path :serialization-length)) 4
    ;; float32 _lateral_error
    4
    ;; bool _target_point_detected
    1
    ;; geometry_msgs/Point _target_point
    (send _target_point :serialization-length)
    ;; float32 _target_heading
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _left_lane_detected
       (if _left_lane_detected (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point[] _left_lane_points
     (write-long (length _left_lane_points) s)
     (dolist (elem _left_lane_points)
       (send elem :serialize s)
       )
     ;; int8 _left_lane_degree
       (write-byte _left_lane_degree s)
     ;; bool _right_lane_detected
       (if _right_lane_detected (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point[] _right_lane_points
     (write-long (length _right_lane_points) s)
     (dolist (elem _right_lane_points)
       (send elem :serialize s)
       )
     ;; int8 _right_lane_degree
       (write-byte _right_lane_degree s)
     ;; geometry_msgs/Point[] _center_path
     (write-long (length _center_path) s)
     (dolist (elem _center_path)
       (send elem :serialize s)
       )
     ;; float32 _lateral_error
       (sys::poke _lateral_error (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _target_point_detected
       (if _target_point_detected (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _target_point
       (send _target_point :serialize s)
     ;; float32 _target_heading
       (sys::poke _target_heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _left_lane_detected
     (setq _left_lane_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point[] _left_lane_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left_lane_points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _left_lane_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int8 _left_lane_degree
     (setq _left_lane_degree (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _left_lane_degree 127) (setq _left_lane_degree (- _left_lane_degree 256)))
   ;; bool _right_lane_detected
     (setq _right_lane_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point[] _right_lane_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right_lane_points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _right_lane_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int8 _right_lane_degree
     (setq _right_lane_degree (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _right_lane_degree 127) (setq _right_lane_degree (- _right_lane_degree 256)))
   ;; geometry_msgs/Point[] _center_path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _center_path (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _center_path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _lateral_error
     (setq _lateral_error (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _target_point_detected
     (setq _target_point_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _target_point
     (send _target_point :deserialize buf ptr-) (incf ptr- (send _target_point :serialization-length))
   ;; float32 _target_heading
     (setq _target_heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get xycar_msgs::ConeLanes :md5sum-) "17f723016cb6dd916e265822742d21c5")
(setf (get xycar_msgs::ConeLanes :datatype-) "xycar_msgs/ConeLanes")
(setf (get xycar_msgs::ConeLanes :definition-)
      "# ConeLanes.msg

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

# 중앙 주행 경로 및 제어 정보
geometry_msgs/Point[] center_path       # 계산된 주행 중앙 경로 포인트 (x, y, z=0) 리스트
float32 lateral_error                 # 차량의 횡방향 오차 (중앙 경로의 y=0에 가장 가까운 지점의 x값)
bool target_point_detected            # 제어 목표점 탐지 성공 여부
geometry_msgs/Point target_point      # 제어 목표점 좌표 (x, y, z=0)
float32 target_heading                # 제어 목표점에서의 경로 각도 (radian)
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :xycar_msgs/ConeLanes "17f723016cb6dd916e265822742d21c5")


