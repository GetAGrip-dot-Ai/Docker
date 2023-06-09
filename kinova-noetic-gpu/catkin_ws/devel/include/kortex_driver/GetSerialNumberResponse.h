// Generated by gencpp from file kortex_driver/GetSerialNumberResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETSERIALNUMBERRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETSERIALNUMBERRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/SerialNumber.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetSerialNumberResponse_
{
  typedef GetSerialNumberResponse_<ContainerAllocator> Type;

  GetSerialNumberResponse_()
    : output()  {
    }
  GetSerialNumberResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::SerialNumber_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetSerialNumberResponse_

typedef ::kortex_driver::GetSerialNumberResponse_<std::allocator<void> > GetSerialNumberResponse;

typedef boost::shared_ptr< ::kortex_driver::GetSerialNumberResponse > GetSerialNumberResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetSerialNumberResponse const> GetSerialNumberResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "072179b84c634619b5ec44c365c9a936";
  }

  static const char* value(const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x072179b84c634619ULL;
  static const uint64_t static_value2 = 0xb5ec44c365c9a936ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetSerialNumberResponse";
  }

  static const char* value(const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "SerialNumber output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/SerialNumber\n"
"\n"
"string serial_number\n"
;
  }

  static const char* value(const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetSerialNumberResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetSerialNumberResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::SerialNumber_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETSERIALNUMBERRESPONSE_H
