// Generated by gencpp from file xycar_msgs/XycarUltrasonic.msg
// DO NOT EDIT!


#ifndef XYCAR_MSGS_MESSAGE_XYCARULTRASONIC_H
#define XYCAR_MSGS_MESSAGE_XYCARULTRASONIC_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>

namespace xycar_msgs
{
template <class ContainerAllocator>
struct XycarUltrasonic_
{
  typedef XycarUltrasonic_<ContainerAllocator> Type;

  XycarUltrasonic_()
    : header()
    , ranges()  {
    }
  XycarUltrasonic_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , ranges(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::sensor_msgs::Range_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::sensor_msgs::Range_<ContainerAllocator> >> _ranges_type;
  _ranges_type ranges;





  typedef boost::shared_ptr< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> const> ConstPtr;

}; // struct XycarUltrasonic_

typedef ::xycar_msgs::XycarUltrasonic_<std::allocator<void> > XycarUltrasonic;

typedef boost::shared_ptr< ::xycar_msgs::XycarUltrasonic > XycarUltrasonicPtr;
typedef boost::shared_ptr< ::xycar_msgs::XycarUltrasonic const> XycarUltrasonicConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator1> & lhs, const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.ranges == rhs.ranges;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator1> & lhs, const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace xycar_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4c1d9b3ff03219d31e41c86817a72ba8";
  }

  static const char* value(const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4c1d9b3ff03219d3ULL;
  static const uint64_t static_value2 = 0x1e41c86817a72ba8ULL;
};

template<class ContainerAllocator>
struct DataType< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xycar_msgs/XycarUltrasonic";
  }

  static const char* value(const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"sensor_msgs/Range[] ranges\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/Range\n"
"# Single range reading from an active ranger that emits energy and reports\n"
"# one range reading that is valid along an arc at the distance measured. \n"
"# This message is  not appropriate for laser scanners. See the LaserScan\n"
"# message if you are working with a laser scanner.\n"
"\n"
"# This message also can represent a fixed-distance (binary) ranger.  This\n"
"# sensor will have min_range===max_range===distance of detection.\n"
"# These sensors follow REP 117 and will output -Inf if the object is detected\n"
"# and +Inf if the object is outside of the detection range.\n"
"\n"
"Header header           # timestamp in the header is the time the ranger\n"
"                        # returned the distance reading\n"
"\n"
"# Radiation type enums\n"
"# If you want a value added to this list, send an email to the ros-users list\n"
"uint8 ULTRASOUND=0\n"
"uint8 INFRARED=1\n"
"\n"
"uint8 radiation_type    # the type of radiation used by the sensor\n"
"                        # (sound, IR, etc) [enum]\n"
"\n"
"float32 field_of_view   # the size of the arc that the distance reading is\n"
"                        # valid for [rad]\n"
"                        # the object causing the range reading may have\n"
"                        # been anywhere within -field_of_view/2 and\n"
"                        # field_of_view/2 at the measured range. \n"
"                        # 0 angle corresponds to the x-axis of the sensor.\n"
"\n"
"float32 min_range       # minimum range value [m]\n"
"float32 max_range       # maximum range value [m]\n"
"                        # Fixed distance rangers require min_range==max_range\n"
"\n"
"float32 range           # range data [m]\n"
"                        # (Note: values < range_min or > range_max\n"
"                        # should be discarded)\n"
"                        # Fixed distance rangers only output -Inf or +Inf.\n"
"                        # -Inf represents a detection within fixed distance.\n"
"                        # (Detection too close to the sensor to quantify)\n"
"                        # +Inf represents no detection within the fixed distance.\n"
"                        # (Object out of range)\n"
;
  }

  static const char* value(const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.ranges);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct XycarUltrasonic_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xycar_msgs::XycarUltrasonic_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xycar_msgs::XycarUltrasonic_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "ranges[]" << std::endl;
    for (size_t i = 0; i < v.ranges.size(); ++i)
    {
      s << indent << "  ranges[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sensor_msgs::Range_<ContainerAllocator> >::stream(s, indent + "    ", v.ranges[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // XYCAR_MSGS_MESSAGE_XYCARULTRASONIC_H
