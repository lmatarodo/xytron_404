;; Auto-generated. Do not edit!


(when (boundp 'xycar_msgs::laneinfo)
  (if (not (find-package "XYCAR_MSGS"))
    (make-package "XYCAR_MSGS"))
  (shadow 'laneinfo (find-package "XYCAR_MSGS")))
(unless (find-package "XYCAR_MSGS::LANEINFO")
  (make-package "XYCAR_MSGS::LANEINFO"))

(in-package "ROS")
;;//! \htmlinclude laneinfo.msg.html


(defclass xycar_msgs::laneinfo
  :super ros::object
  :slots (_left_x _left_y _left_slope _right_x _right_y _right_slope _lane_number _cone_detected_flag ))

(defmethod xycar_msgs::laneinfo
  (:init
   (&key
    ((:left_x __left_x) 0.0)
    ((:left_y __left_y) 0.0)
    ((:left_slope __left_slope) 0.0)
    ((:right_x __right_x) 0.0)
    ((:right_y __right_y) 0.0)
    ((:right_slope __right_slope) 0.0)
    ((:lane_number __lane_number) 0)
    ((:cone_detected_flag __cone_detected_flag) nil)
    )
   (send-super :init)
   (setq _left_x (float __left_x))
   (setq _left_y (float __left_y))
   (setq _left_slope (float __left_slope))
   (setq _right_x (float __right_x))
   (setq _right_y (float __right_y))
   (setq _right_slope (float __right_slope))
   (setq _lane_number (round __lane_number))
   (setq _cone_detected_flag __cone_detected_flag)
   self)
  (:left_x
   (&optional __left_x)
   (if __left_x (setq _left_x __left_x)) _left_x)
  (:left_y
   (&optional __left_y)
   (if __left_y (setq _left_y __left_y)) _left_y)
  (:left_slope
   (&optional __left_slope)
   (if __left_slope (setq _left_slope __left_slope)) _left_slope)
  (:right_x
   (&optional __right_x)
   (if __right_x (setq _right_x __right_x)) _right_x)
  (:right_y
   (&optional __right_y)
   (if __right_y (setq _right_y __right_y)) _right_y)
  (:right_slope
   (&optional __right_slope)
   (if __right_slope (setq _right_slope __right_slope)) _right_slope)
  (:lane_number
   (&optional __lane_number)
   (if __lane_number (setq _lane_number __lane_number)) _lane_number)
  (:cone_detected_flag
   (&optional (__cone_detected_flag :null))
   (if (not (eq __cone_detected_flag :null)) (setq _cone_detected_flag __cone_detected_flag)) _cone_detected_flag)
  (:serialization-length
   ()
   (+
    ;; float32 _left_x
    4
    ;; float32 _left_y
    4
    ;; float32 _left_slope
    4
    ;; float32 _right_x
    4
    ;; float32 _right_y
    4
    ;; float32 _right_slope
    4
    ;; int32 _lane_number
    4
    ;; bool _cone_detected_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _left_x
       (sys::poke _left_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_y
       (sys::poke _left_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_slope
       (sys::poke _left_slope (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_x
       (sys::poke _right_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_y
       (sys::poke _right_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_slope
       (sys::poke _right_slope (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _lane_number
       (write-long _lane_number s)
     ;; bool _cone_detected_flag
       (if _cone_detected_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _left_x
     (setq _left_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_y
     (setq _left_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_slope
     (setq _left_slope (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_x
     (setq _right_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_y
     (setq _right_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_slope
     (setq _right_slope (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _lane_number
     (setq _lane_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _cone_detected_flag
     (setq _cone_detected_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get xycar_msgs::laneinfo :md5sum-) "80e7564081aa5c4a700b9b80391fcc78")
(setf (get xycar_msgs::laneinfo :datatype-) "xycar_msgs/laneinfo")
(setf (get xycar_msgs::laneinfo :definition-)
      "float32 left_x
float32 left_y
float32 left_slope
float32 right_x
float32 right_y
float32 right_slope
int32 lane_number  # 1: 1차선, 2: 2차선 
bool cone_detected_flag 
")



(provide :xycar_msgs/laneinfo "80e7564081aa5c4a700b9b80391fcc78")


