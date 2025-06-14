# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from xycar_msgs/ConeLanes.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class ConeLanes(genpy.Message):
  _md5sum = "9d95ea1cfb987f44e10d98bcddc53d76"
  _type = "xycar_msgs/ConeLanes"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# 콘 레인 정보를 담는 메시지 타입

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
"""
  __slots__ = ['header','center_path','lateral_error','target_point_detected','target_point','target_heading','left_lane_detected','right_lane_detected','left_lane_degree','right_lane_degree','left_lane_points','right_lane_points']
  _slot_types = ['std_msgs/Header','geometry_msgs/Point[]','float32','bool','geometry_msgs/Point','float32','bool','bool','float32','float32','geometry_msgs/Point[]','geometry_msgs/Point[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,center_path,lateral_error,target_point_detected,target_point,target_heading,left_lane_detected,right_lane_detected,left_lane_degree,right_lane_degree,left_lane_points,right_lane_points

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ConeLanes, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.center_path is None:
        self.center_path = []
      if self.lateral_error is None:
        self.lateral_error = 0.
      if self.target_point_detected is None:
        self.target_point_detected = False
      if self.target_point is None:
        self.target_point = geometry_msgs.msg.Point()
      if self.target_heading is None:
        self.target_heading = 0.
      if self.left_lane_detected is None:
        self.left_lane_detected = False
      if self.right_lane_detected is None:
        self.right_lane_detected = False
      if self.left_lane_degree is None:
        self.left_lane_degree = 0.
      if self.right_lane_degree is None:
        self.right_lane_degree = 0.
      if self.left_lane_points is None:
        self.left_lane_points = []
      if self.right_lane_points is None:
        self.right_lane_points = []
    else:
      self.header = std_msgs.msg.Header()
      self.center_path = []
      self.lateral_error = 0.
      self.target_point_detected = False
      self.target_point = geometry_msgs.msg.Point()
      self.target_heading = 0.
      self.left_lane_detected = False
      self.right_lane_detected = False
      self.left_lane_degree = 0.
      self.right_lane_degree = 0.
      self.left_lane_points = []
      self.right_lane_points = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.center_path)
      buff.write(_struct_I.pack(length))
      for val1 in self.center_path:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_fB3df2B2f().pack(_x.lateral_error, _x.target_point_detected, _x.target_point.x, _x.target_point.y, _x.target_point.z, _x.target_heading, _x.left_lane_detected, _x.right_lane_detected, _x.left_lane_degree, _x.right_lane_degree))
      length = len(self.left_lane_points)
      buff.write(_struct_I.pack(length))
      for val1 in self.left_lane_points:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      length = len(self.right_lane_points)
      buff.write(_struct_I.pack(length))
      for val1 in self.right_lane_points:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.center_path is None:
        self.center_path = None
      if self.target_point is None:
        self.target_point = geometry_msgs.msg.Point()
      if self.left_lane_points is None:
        self.left_lane_points = None
      if self.right_lane_points is None:
        self.right_lane_points = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.center_path = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.center_path.append(val1)
      _x = self
      start = end
      end += 43
      (_x.lateral_error, _x.target_point_detected, _x.target_point.x, _x.target_point.y, _x.target_point.z, _x.target_heading, _x.left_lane_detected, _x.right_lane_detected, _x.left_lane_degree, _x.right_lane_degree,) = _get_struct_fB3df2B2f().unpack(str[start:end])
      self.target_point_detected = bool(self.target_point_detected)
      self.left_lane_detected = bool(self.left_lane_detected)
      self.right_lane_detected = bool(self.right_lane_detected)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.left_lane_points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.left_lane_points.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.right_lane_points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.right_lane_points.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.center_path)
      buff.write(_struct_I.pack(length))
      for val1 in self.center_path:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_fB3df2B2f().pack(_x.lateral_error, _x.target_point_detected, _x.target_point.x, _x.target_point.y, _x.target_point.z, _x.target_heading, _x.left_lane_detected, _x.right_lane_detected, _x.left_lane_degree, _x.right_lane_degree))
      length = len(self.left_lane_points)
      buff.write(_struct_I.pack(length))
      for val1 in self.left_lane_points:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      length = len(self.right_lane_points)
      buff.write(_struct_I.pack(length))
      for val1 in self.right_lane_points:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.center_path is None:
        self.center_path = None
      if self.target_point is None:
        self.target_point = geometry_msgs.msg.Point()
      if self.left_lane_points is None:
        self.left_lane_points = None
      if self.right_lane_points is None:
        self.right_lane_points = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.center_path = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.center_path.append(val1)
      _x = self
      start = end
      end += 43
      (_x.lateral_error, _x.target_point_detected, _x.target_point.x, _x.target_point.y, _x.target_point.z, _x.target_heading, _x.left_lane_detected, _x.right_lane_detected, _x.left_lane_degree, _x.right_lane_degree,) = _get_struct_fB3df2B2f().unpack(str[start:end])
      self.target_point_detected = bool(self.target_point_detected)
      self.left_lane_detected = bool(self.left_lane_detected)
      self.right_lane_detected = bool(self.right_lane_detected)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.left_lane_points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.left_lane_points.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.right_lane_points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.right_lane_points.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_fB3df2B2f = None
def _get_struct_fB3df2B2f():
    global _struct_fB3df2B2f
    if _struct_fB3df2B2f is None:
        _struct_fB3df2B2f = struct.Struct("<fB3df2B2f")
    return _struct_fB3df2B2f
