// Generated by gencpp from file foscam_cam_ros/tripletsRequest.msg
// DO NOT EDIT!


#ifndef FOSCAM_CAM_ROS_MESSAGE_TRIPLETSREQUEST_H
#define FOSCAM_CAM_ROS_MESSAGE_TRIPLETSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>

namespace foscam_cam_ros
{
template <class ContainerAllocator>
struct tripletsRequest_
{
  typedef tripletsRequest_<ContainerAllocator> Type;

  tripletsRequest_()
    : key()  {
    }
  tripletsRequest_(const ContainerAllocator& _alloc)
    : key(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::String_<ContainerAllocator>  _key_type;
  _key_type key;




  typedef boost::shared_ptr< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct tripletsRequest_

typedef ::foscam_cam_ros::tripletsRequest_<std::allocator<void> > tripletsRequest;

typedef boost::shared_ptr< ::foscam_cam_ros::tripletsRequest > tripletsRequestPtr;
typedef boost::shared_ptr< ::foscam_cam_ros::tripletsRequest const> tripletsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5059070306634068d6fd4acc32329af8";
  }

  static const char* value(const ::foscam_cam_ros::tripletsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5059070306634068ULL;
  static const uint64_t static_value2 = 0xd6fd4acc32329af8ULL;
};

template<class ContainerAllocator>
struct DataType< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "foscam_cam_ros/tripletsRequest";
  }

  static const char* value(const ::foscam_cam_ros::tripletsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/String key\n\
\n\
================================================================================\n\
MSG: std_msgs/String\n\
string data\n\
";
  }

  static const char* value(const ::foscam_cam_ros::tripletsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.key);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct tripletsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::foscam_cam_ros::tripletsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::foscam_cam_ros::tripletsRequest_<ContainerAllocator>& v)
  {
    s << indent << "key: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.key);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FOSCAM_CAM_ROS_MESSAGE_TRIPLETSREQUEST_H
