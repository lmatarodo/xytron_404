; Auto-generated. Do not edit!


(cl:in-package xycar_msgs-msg)


;//! \htmlinclude ConeLanes.msg.html

(cl:defclass <ConeLanes> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_lane_detected
    :reader left_lane_detected
    :initarg :left_lane_detected
    :type cl:boolean
    :initform cl:nil)
   (left_lane_points
    :reader left_lane_points
    :initarg :left_lane_points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (left_lane_degree
    :reader left_lane_degree
    :initarg :left_lane_degree
    :type cl:fixnum
    :initform 0)
   (right_lane_detected
    :reader right_lane_detected
    :initarg :right_lane_detected
    :type cl:boolean
    :initform cl:nil)
   (right_lane_points
    :reader right_lane_points
    :initarg :right_lane_points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (right_lane_degree
    :reader right_lane_degree
    :initarg :right_lane_degree
    :type cl:fixnum
    :initform 0)
   (center_path
    :reader center_path
    :initarg :center_path
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass ConeLanes (<ConeLanes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConeLanes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConeLanes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xycar_msgs-msg:<ConeLanes> is deprecated: use xycar_msgs-msg:ConeLanes instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConeLanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xycar_msgs-msg:header-val is deprecated.  Use xycar_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_lane_detected-val :lambda-list '(m))
(cl:defmethod left_lane_detected-val ((m <ConeLanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xycar_msgs-msg:left_lane_detected-val is deprecated.  Use xycar_msgs-msg:left_lane_detected instead.")
  (left_lane_detected m))

(cl:ensure-generic-function 'left_lane_points-val :lambda-list '(m))
(cl:defmethod left_lane_points-val ((m <ConeLanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xycar_msgs-msg:left_lane_points-val is deprecated.  Use xycar_msgs-msg:left_lane_points instead.")
  (left_lane_points m))

(cl:ensure-generic-function 'left_lane_degree-val :lambda-list '(m))
(cl:defmethod left_lane_degree-val ((m <ConeLanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xycar_msgs-msg:left_lane_degree-val is deprecated.  Use xycar_msgs-msg:left_lane_degree instead.")
  (left_lane_degree m))

(cl:ensure-generic-function 'right_lane_detected-val :lambda-list '(m))
(cl:defmethod right_lane_detected-val ((m <ConeLanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xycar_msgs-msg:right_lane_detected-val is deprecated.  Use xycar_msgs-msg:right_lane_detected instead.")
  (right_lane_detected m))

(cl:ensure-generic-function 'right_lane_points-val :lambda-list '(m))
(cl:defmethod right_lane_points-val ((m <ConeLanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xycar_msgs-msg:right_lane_points-val is deprecated.  Use xycar_msgs-msg:right_lane_points instead.")
  (right_lane_points m))

(cl:ensure-generic-function 'right_lane_degree-val :lambda-list '(m))
(cl:defmethod right_lane_degree-val ((m <ConeLanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xycar_msgs-msg:right_lane_degree-val is deprecated.  Use xycar_msgs-msg:right_lane_degree instead.")
  (right_lane_degree m))

(cl:ensure-generic-function 'center_path-val :lambda-list '(m))
(cl:defmethod center_path-val ((m <ConeLanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xycar_msgs-msg:center_path-val is deprecated.  Use xycar_msgs-msg:center_path instead.")
  (center_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConeLanes>) ostream)
  "Serializes a message object of type '<ConeLanes>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_lane_detected) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'left_lane_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'left_lane_points))
  (cl:let* ((signed (cl:slot-value msg 'left_lane_degree)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_lane_detected) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'right_lane_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'right_lane_points))
  (cl:let* ((signed (cl:slot-value msg 'right_lane_degree)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'center_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'center_path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConeLanes>) istream)
  "Deserializes a message object of type '<ConeLanes>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'left_lane_detected) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'left_lane_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'left_lane_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_lane_degree) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'right_lane_detected) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'right_lane_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'right_lane_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_lane_degree) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'center_path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'center_path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConeLanes>)))
  "Returns string type for a message object of type '<ConeLanes>"
  "xycar_msgs/ConeLanes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConeLanes)))
  "Returns string type for a message object of type 'ConeLanes"
  "xycar_msgs/ConeLanes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConeLanes>)))
  "Returns md5sum for a message object of type '<ConeLanes>"
  "af8510db732fe4223bfc5f0cf49c6646")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConeLanes)))
  "Returns md5sum for a message object of type 'ConeLanes"
  "af8510db732fe4223bfc5f0cf49c6646")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConeLanes>)))
  "Returns full string definition for message of type '<ConeLanes>"
  (cl:format cl:nil "# ConeLanes.msg~%~%# 메시지 헤더 (타임스탬프, 프레임 ID 포함)~%std_msgs/Header header~%~%# 왼쪽 차선 감지 정보~%bool left_lane_detected             # 왼쪽 차선 감지 성공 여부~%geometry_msgs/Point[] left_lane_points  # 피팅된 왼쪽 차선의 샘플링 포인트 (x, y, z=0) 리스트~%int8 left_lane_degree                   # 왼쪽 차선 피팅에 사용된 다항식 차수 (0이면 피팅 안됨 또는 실패)~%~%# 오른쪽 차선 감지 정보~%bool right_lane_detected            # 오른쪽 차선 감지 성공 여부~%geometry_msgs/Point[] right_lane_points # 피팅된 오른쪽 차선의 샘플링 포인트 (x, y, z=0) 리스트~%int8 right_lane_degree                  # 오른쪽 차선 피팅에 사용된 다항식 차수 (0이면 피팅 안됨 또는 실패)~%~%# 중앙 주행 경로~%geometry_msgs/Point[] center_path       # 계산된 주행 중앙 경로 포인트 (x, y, z=0) 리스트~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConeLanes)))
  "Returns full string definition for message of type 'ConeLanes"
  (cl:format cl:nil "# ConeLanes.msg~%~%# 메시지 헤더 (타임스탬프, 프레임 ID 포함)~%std_msgs/Header header~%~%# 왼쪽 차선 감지 정보~%bool left_lane_detected             # 왼쪽 차선 감지 성공 여부~%geometry_msgs/Point[] left_lane_points  # 피팅된 왼쪽 차선의 샘플링 포인트 (x, y, z=0) 리스트~%int8 left_lane_degree                   # 왼쪽 차선 피팅에 사용된 다항식 차수 (0이면 피팅 안됨 또는 실패)~%~%# 오른쪽 차선 감지 정보~%bool right_lane_detected            # 오른쪽 차선 감지 성공 여부~%geometry_msgs/Point[] right_lane_points # 피팅된 오른쪽 차선의 샘플링 포인트 (x, y, z=0) 리스트~%int8 right_lane_degree                  # 오른쪽 차선 피팅에 사용된 다항식 차수 (0이면 피팅 안됨 또는 실패)~%~%# 중앙 주행 경로~%geometry_msgs/Point[] center_path       # 계산된 주행 중앙 경로 포인트 (x, y, z=0) 리스트~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConeLanes>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'left_lane_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'right_lane_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'center_path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConeLanes>))
  "Converts a ROS message object to a list"
  (cl:list 'ConeLanes
    (cl:cons ':header (header msg))
    (cl:cons ':left_lane_detected (left_lane_detected msg))
    (cl:cons ':left_lane_points (left_lane_points msg))
    (cl:cons ':left_lane_degree (left_lane_degree msg))
    (cl:cons ':right_lane_detected (right_lane_detected msg))
    (cl:cons ':right_lane_points (right_lane_points msg))
    (cl:cons ':right_lane_degree (right_lane_degree msg))
    (cl:cons ':center_path (center_path msg))
))
