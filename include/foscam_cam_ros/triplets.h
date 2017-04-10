// Generated by gencpp from file foscam_cam_ros/triplets.msg
// DO NOT EDIT!


#ifndef FOSCAM_CAM_ROS_MESSAGE_TRIPLETS_H
#define FOSCAM_CAM_ROS_MESSAGE_TRIPLETS_H

#include <ros/service_traits.h>


#include <foscam_cam_ros/tripletsRequest.h>
#include <foscam_cam_ros/tripletsResponse.h>


namespace foscam_cam_ros
{

struct triplets
{

typedef tripletsRequest Request;
typedef tripletsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct triplets
} // namespace foscam_cam_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::foscam_cam_ros::triplets > {
  static const char* value()
  {
    return "bb92b2f43626096a42ec604d83e404af";
  }

  static const char* value(const ::foscam_cam_ros::triplets&) { return value(); }
};

template<>
struct DataType< ::foscam_cam_ros::triplets > {
  static const char* value()
  {
    return "foscam_cam_ros/triplets";
  }

  static const char* value(const ::foscam_cam_ros::triplets&) { return value(); }
};


// service_traits::MD5Sum< ::foscam_cam_ros::tripletsRequest> should match 
// service_traits::MD5Sum< ::foscam_cam_ros::triplets > 
template<>
struct MD5Sum< ::foscam_cam_ros::tripletsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::foscam_cam_ros::triplets >::value();
  }
  static const char* value(const ::foscam_cam_ros::tripletsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::foscam_cam_ros::tripletsRequest> should match 
// service_traits::DataType< ::foscam_cam_ros::triplets > 
template<>
struct DataType< ::foscam_cam_ros::tripletsRequest>
{
  static const char* value()
  {
    return DataType< ::foscam_cam_ros::triplets >::value();
  }
  static const char* value(const ::foscam_cam_ros::tripletsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::foscam_cam_ros::tripletsResponse> should match 
// service_traits::MD5Sum< ::foscam_cam_ros::triplets > 
template<>
struct MD5Sum< ::foscam_cam_ros::tripletsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::foscam_cam_ros::triplets >::value();
  }
  static const char* value(const ::foscam_cam_ros::tripletsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::foscam_cam_ros::tripletsResponse> should match 
// service_traits::DataType< ::foscam_cam_ros::triplets > 
template<>
struct DataType< ::foscam_cam_ros::tripletsResponse>
{
  static const char* value()
  {
    return DataType< ::foscam_cam_ros::triplets >::value();
  }
  static const char* value(const ::foscam_cam_ros::tripletsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FOSCAM_CAM_ROS_MESSAGE_TRIPLETS_H
