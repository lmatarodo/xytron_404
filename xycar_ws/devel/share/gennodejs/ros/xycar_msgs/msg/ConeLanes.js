// Auto-generated. Do not edit!

// (in-package xycar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ConeLanes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.center_path = null;
      this.lateral_error = null;
      this.target_point_detected = null;
      this.target_point = null;
      this.target_heading = null;
      this.left_lane_detected = null;
      this.right_lane_detected = null;
      this.left_lane_degree = null;
      this.right_lane_degree = null;
      this.left_lane_points = null;
      this.right_lane_points = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('center_path')) {
        this.center_path = initObj.center_path
      }
      else {
        this.center_path = [];
      }
      if (initObj.hasOwnProperty('lateral_error')) {
        this.lateral_error = initObj.lateral_error
      }
      else {
        this.lateral_error = 0.0;
      }
      if (initObj.hasOwnProperty('target_point_detected')) {
        this.target_point_detected = initObj.target_point_detected
      }
      else {
        this.target_point_detected = false;
      }
      if (initObj.hasOwnProperty('target_point')) {
        this.target_point = initObj.target_point
      }
      else {
        this.target_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('target_heading')) {
        this.target_heading = initObj.target_heading
      }
      else {
        this.target_heading = 0.0;
      }
      if (initObj.hasOwnProperty('left_lane_detected')) {
        this.left_lane_detected = initObj.left_lane_detected
      }
      else {
        this.left_lane_detected = false;
      }
      if (initObj.hasOwnProperty('right_lane_detected')) {
        this.right_lane_detected = initObj.right_lane_detected
      }
      else {
        this.right_lane_detected = false;
      }
      if (initObj.hasOwnProperty('left_lane_degree')) {
        this.left_lane_degree = initObj.left_lane_degree
      }
      else {
        this.left_lane_degree = 0.0;
      }
      if (initObj.hasOwnProperty('right_lane_degree')) {
        this.right_lane_degree = initObj.right_lane_degree
      }
      else {
        this.right_lane_degree = 0.0;
      }
      if (initObj.hasOwnProperty('left_lane_points')) {
        this.left_lane_points = initObj.left_lane_points
      }
      else {
        this.left_lane_points = [];
      }
      if (initObj.hasOwnProperty('right_lane_points')) {
        this.right_lane_points = initObj.right_lane_points
      }
      else {
        this.right_lane_points = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConeLanes
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [center_path]
    // Serialize the length for message field [center_path]
    bufferOffset = _serializer.uint32(obj.center_path.length, buffer, bufferOffset);
    obj.center_path.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [lateral_error]
    bufferOffset = _serializer.float32(obj.lateral_error, buffer, bufferOffset);
    // Serialize message field [target_point_detected]
    bufferOffset = _serializer.bool(obj.target_point_detected, buffer, bufferOffset);
    // Serialize message field [target_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.target_point, buffer, bufferOffset);
    // Serialize message field [target_heading]
    bufferOffset = _serializer.float32(obj.target_heading, buffer, bufferOffset);
    // Serialize message field [left_lane_detected]
    bufferOffset = _serializer.bool(obj.left_lane_detected, buffer, bufferOffset);
    // Serialize message field [right_lane_detected]
    bufferOffset = _serializer.bool(obj.right_lane_detected, buffer, bufferOffset);
    // Serialize message field [left_lane_degree]
    bufferOffset = _serializer.float32(obj.left_lane_degree, buffer, bufferOffset);
    // Serialize message field [right_lane_degree]
    bufferOffset = _serializer.float32(obj.right_lane_degree, buffer, bufferOffset);
    // Serialize message field [left_lane_points]
    // Serialize the length for message field [left_lane_points]
    bufferOffset = _serializer.uint32(obj.left_lane_points.length, buffer, bufferOffset);
    obj.left_lane_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [right_lane_points]
    // Serialize the length for message field [right_lane_points]
    bufferOffset = _serializer.uint32(obj.right_lane_points.length, buffer, bufferOffset);
    obj.right_lane_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConeLanes
    let len;
    let data = new ConeLanes(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [center_path]
    // Deserialize array length for message field [center_path]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.center_path = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.center_path[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [lateral_error]
    data.lateral_error = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_point_detected]
    data.target_point_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [target_point]
    data.target_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_heading]
    data.target_heading = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_lane_detected]
    data.left_lane_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_lane_detected]
    data.right_lane_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_lane_degree]
    data.left_lane_degree = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_lane_degree]
    data.right_lane_degree = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_lane_points]
    // Deserialize array length for message field [left_lane_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.left_lane_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.left_lane_points[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [right_lane_points]
    // Deserialize array length for message field [right_lane_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.right_lane_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.right_lane_points[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.center_path.length;
    length += 24 * object.left_lane_points.length;
    length += 24 * object.right_lane_points.length;
    return length + 55;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xycar_msgs/ConeLanes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d95ea1cfb987f44e10d98bcddc53d76';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConeLanes(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.center_path !== undefined) {
      resolved.center_path = new Array(msg.center_path.length);
      for (let i = 0; i < resolved.center_path.length; ++i) {
        resolved.center_path[i] = geometry_msgs.msg.Point.Resolve(msg.center_path[i]);
      }
    }
    else {
      resolved.center_path = []
    }

    if (msg.lateral_error !== undefined) {
      resolved.lateral_error = msg.lateral_error;
    }
    else {
      resolved.lateral_error = 0.0
    }

    if (msg.target_point_detected !== undefined) {
      resolved.target_point_detected = msg.target_point_detected;
    }
    else {
      resolved.target_point_detected = false
    }

    if (msg.target_point !== undefined) {
      resolved.target_point = geometry_msgs.msg.Point.Resolve(msg.target_point)
    }
    else {
      resolved.target_point = new geometry_msgs.msg.Point()
    }

    if (msg.target_heading !== undefined) {
      resolved.target_heading = msg.target_heading;
    }
    else {
      resolved.target_heading = 0.0
    }

    if (msg.left_lane_detected !== undefined) {
      resolved.left_lane_detected = msg.left_lane_detected;
    }
    else {
      resolved.left_lane_detected = false
    }

    if (msg.right_lane_detected !== undefined) {
      resolved.right_lane_detected = msg.right_lane_detected;
    }
    else {
      resolved.right_lane_detected = false
    }

    if (msg.left_lane_degree !== undefined) {
      resolved.left_lane_degree = msg.left_lane_degree;
    }
    else {
      resolved.left_lane_degree = 0.0
    }

    if (msg.right_lane_degree !== undefined) {
      resolved.right_lane_degree = msg.right_lane_degree;
    }
    else {
      resolved.right_lane_degree = 0.0
    }

    if (msg.left_lane_points !== undefined) {
      resolved.left_lane_points = new Array(msg.left_lane_points.length);
      for (let i = 0; i < resolved.left_lane_points.length; ++i) {
        resolved.left_lane_points[i] = geometry_msgs.msg.Point.Resolve(msg.left_lane_points[i]);
      }
    }
    else {
      resolved.left_lane_points = []
    }

    if (msg.right_lane_points !== undefined) {
      resolved.right_lane_points = new Array(msg.right_lane_points.length);
      for (let i = 0; i < resolved.right_lane_points.length; ++i) {
        resolved.right_lane_points[i] = geometry_msgs.msg.Point.Resolve(msg.right_lane_points[i]);
      }
    }
    else {
      resolved.right_lane_points = []
    }

    return resolved;
    }
};

module.exports = ConeLanes;
