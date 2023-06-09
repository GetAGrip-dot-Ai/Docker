// Generated by gencpp from file kortex_driver/GetActuatorCountResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETACTUATORCOUNTRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETACTUATORCOUNTRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ActuatorInformation.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetActuatorCountResponse_
{
  typedef GetActuatorCountResponse_<ContainerAllocator> Type;

  GetActuatorCountResponse_()
    : output()  {
    }
  GetActuatorCountResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ActuatorInformation_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetActuatorCountResponse_

typedef ::kortex_driver::GetActuatorCountResponse_<std::allocator<void> > GetActuatorCountResponse;

typedef boost::shared_ptr< ::kortex_driver::GetActuatorCountResponse > GetActuatorCountResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetActuatorCountResponse const> GetActuatorCountResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "583bc5edda5ad44b6a08d185499c41bd";
  }

  static const char* value(const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x583bc5edda5ad44bULL;
  static const uint64_t static_value2 = 0x6a08d185499c41bdULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetActuatorCountResponse";
  }

  static const char* value(const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ActuatorInformation output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/ActuatorInformation\n"
"\n"
"uint32 count\n"
;
  }

  static const char* value(const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetActuatorCountResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetActuatorCountResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::ActuatorInformation_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETACTUATORCOUNTRESPONSE_H
