// Generated by gencpp from file foscam_cam_ros/tripletsResponse.msg
// DO NOT EDIT!


#ifndef FOSCAM_CAM_ROS_MESSAGE_TRIPLETSRESPONSE_H
#define FOSCAM_CAM_ROS_MESSAGE_TRIPLETSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>

namespace foscam_cam_ros
{
template <class ContainerAllocator>
struct tripletsResponse_
{
  typedef tripletsResponse_<ContainerAllocator> Type;

  tripletsResponse_()
    : left_rgb_img()
    , middle_rgb_img()
    , right_rgb_img()  {
    }
  tripletsResponse_(const ContainerAllocator& _alloc)
    : left_rgb_img(_alloc)
    , middle_rgb_img(_alloc)
    , right_rgb_img(_alloc)  {
  (void)_alloc;
    }



   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _left_rgb_img_type;
  _left_rgb_img_type left_rgb_img;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _middle_rgb_img_type;
  _middle_rgb_img_type middle_rgb_img;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _right_rgb_img_type;
  _right_rgb_img_type right_rgb_img;




  typedef boost::shared_ptr< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct tripletsResponse_

typedef ::foscam_cam_ros::tripletsResponse_<std::allocator<void> > tripletsResponse;

typedef boost::shared_ptr< ::foscam_cam_ros::tripletsResponse > tripletsResponsePtr;
typedef boost::shared_ptr< ::foscam_cam_ros::tripletsResponse const> tripletsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace foscam_cam_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f3400cd4e3093d296da02c00cafb7d3";
  }

  static const char* value(const ::foscam_cam_ros::tripletsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f3400cd4e3093d2ULL;
  static const uint64_t static_value2 = 0x96da02c00cafb7d3ULL;
};

template<class ContainerAllocator>
struct DataType< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "foscam_cam_ros/tripletsResponse";
  }

  static const char* value(const ::foscam_cam_ros::tripletsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/Image left_rgb_img\n\
sensor_msgs/Image middle_rgb_img\n\
sensor_msgs/Image right_rgb_img\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::foscam_cam_ros::tripletsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left_rgb_img);
      stream.next(m.middle_rgb_img);
      stream.next(m.right_rgb_img);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct tripletsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::foscam_cam_ros::tripletsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::foscam_cam_ros::tripletsResponse_<ContainerAllocator>& v)
  {
    s << indent << "left_rgb_img: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.left_rgb_img);
    s << indent << "middle_rgb_img: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.middle_rgb_img);
    s << indent << "right_rgb_img: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.right_rgb_img);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FOSCAM_CAM_ROS_MESSAGE_TRIPLETSRESPONSE_H
