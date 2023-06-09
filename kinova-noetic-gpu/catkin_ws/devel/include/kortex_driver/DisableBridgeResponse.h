// Generated by gencpp from file kortex_driver/DisableBridgeResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_DISABLEBRIDGERESPONSE_H
#define KORTEX_DRIVER_MESSAGE_DISABLEBRIDGERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/BridgeResult.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct DisableBridgeResponse_
{
  typedef DisableBridgeResponse_<ContainerAllocator> Type;

  DisableBridgeResponse_()
    : output()  {
    }
  DisableBridgeResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::BridgeResult_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DisableBridgeResponse_

typedef ::kortex_driver::DisableBridgeResponse_<std::allocator<void> > DisableBridgeResponse;

typedef boost::shared_ptr< ::kortex_driver::DisableBridgeResponse > DisableBridgeResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::DisableBridgeResponse const> DisableBridgeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "14fb5ca3149c75d17c29aa203a0186ee";
  }

  static const char* value(const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x14fb5ca3149c75d1ULL;
  static const uint64_t static_value2 = 0x7c29aa203a0186eeULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/DisableBridgeResponse";
  }

  static const char* value(const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "BridgeResult output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/BridgeResult\n"
"\n"
"BridgeIdentifier bridge_id\n"
"uint32 status\n"
"================================================================================\n"
"MSG: kortex_driver/BridgeIdentifier\n"
"\n"
"uint32 bridge_id\n"
;
  }

  static const char* value(const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DisableBridgeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::DisableBridgeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::DisableBridgeResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::BridgeResult_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_DISABLEBRIDGERESPONSE_H
