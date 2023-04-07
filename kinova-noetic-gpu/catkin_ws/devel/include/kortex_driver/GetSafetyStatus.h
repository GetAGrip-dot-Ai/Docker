// Generated by gencpp from file kortex_driver/GetSafetyStatus.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETSAFETYSTATUS_H
#define KORTEX_DRIVER_MESSAGE_GETSAFETYSTATUS_H

#include <ros/service_traits.h>


#include <kortex_driver/GetSafetyStatusRequest.h>
#include <kortex_driver/GetSafetyStatusResponse.h>


namespace kortex_driver
{

struct GetSafetyStatus
{

typedef GetSafetyStatusRequest Request;
typedef GetSafetyStatusResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetSafetyStatus
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetSafetyStatus > {
  static const char* value()
  {
    return "0b50aa56e0379c3a0c319bd4962efe2e";
  }

  static const char* value(const ::kortex_driver::GetSafetyStatus&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetSafetyStatus > {
  static const char* value()
  {
    return "kortex_driver/GetSafetyStatus";
  }

  static const char* value(const ::kortex_driver::GetSafetyStatus&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetSafetyStatusRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetSafetyStatus >
template<>
struct MD5Sum< ::kortex_driver::GetSafetyStatusRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetSafetyStatus >::value();
  }
  static const char* value(const ::kortex_driver::GetSafetyStatusRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetSafetyStatusRequest> should match
// service_traits::DataType< ::kortex_driver::GetSafetyStatus >
template<>
struct DataType< ::kortex_driver::GetSafetyStatusRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetSafetyStatus >::value();
  }
  static const char* value(const ::kortex_driver::GetSafetyStatusRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetSafetyStatusResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetSafetyStatus >
template<>
struct MD5Sum< ::kortex_driver::GetSafetyStatusResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetSafetyStatus >::value();
  }
  static const char* value(const ::kortex_driver::GetSafetyStatusResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetSafetyStatusResponse> should match
// service_traits::DataType< ::kortex_driver::GetSafetyStatus >
template<>
struct DataType< ::kortex_driver::GetSafetyStatusResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetSafetyStatus >::value();
  }
  static const char* value(const ::kortex_driver::GetSafetyStatusResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETSAFETYSTATUS_H