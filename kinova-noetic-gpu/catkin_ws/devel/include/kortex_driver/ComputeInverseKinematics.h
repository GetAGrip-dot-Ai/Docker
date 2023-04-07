// Generated by gencpp from file kortex_driver/ComputeInverseKinematics.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_COMPUTEINVERSEKINEMATICS_H
#define KORTEX_DRIVER_MESSAGE_COMPUTEINVERSEKINEMATICS_H

#include <ros/service_traits.h>


#include <kortex_driver/ComputeInverseKinematicsRequest.h>
#include <kortex_driver/ComputeInverseKinematicsResponse.h>


namespace kortex_driver
{

struct ComputeInverseKinematics
{

typedef ComputeInverseKinematicsRequest Request;
typedef ComputeInverseKinematicsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ComputeInverseKinematics
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::ComputeInverseKinematics > {
  static const char* value()
  {
    return "290825a1e5de199dc18075d261a5fee3";
  }

  static const char* value(const ::kortex_driver::ComputeInverseKinematics&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::ComputeInverseKinematics > {
  static const char* value()
  {
    return "kortex_driver/ComputeInverseKinematics";
  }

  static const char* value(const ::kortex_driver::ComputeInverseKinematics&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::ComputeInverseKinematicsRequest> should match
// service_traits::MD5Sum< ::kortex_driver::ComputeInverseKinematics >
template<>
struct MD5Sum< ::kortex_driver::ComputeInverseKinematicsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ComputeInverseKinematics >::value();
  }
  static const char* value(const ::kortex_driver::ComputeInverseKinematicsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ComputeInverseKinematicsRequest> should match
// service_traits::DataType< ::kortex_driver::ComputeInverseKinematics >
template<>
struct DataType< ::kortex_driver::ComputeInverseKinematicsRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ComputeInverseKinematics >::value();
  }
  static const char* value(const ::kortex_driver::ComputeInverseKinematicsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::ComputeInverseKinematicsResponse> should match
// service_traits::MD5Sum< ::kortex_driver::ComputeInverseKinematics >
template<>
struct MD5Sum< ::kortex_driver::ComputeInverseKinematicsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ComputeInverseKinematics >::value();
  }
  static const char* value(const ::kortex_driver::ComputeInverseKinematicsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ComputeInverseKinematicsResponse> should match
// service_traits::DataType< ::kortex_driver::ComputeInverseKinematics >
template<>
struct DataType< ::kortex_driver::ComputeInverseKinematicsResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ComputeInverseKinematics >::value();
  }
  static const char* value(const ::kortex_driver::ComputeInverseKinematicsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_COMPUTEINVERSEKINEMATICS_H