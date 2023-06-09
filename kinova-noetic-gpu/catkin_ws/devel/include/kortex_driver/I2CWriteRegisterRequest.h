// Generated by gencpp from file kortex_driver/I2CWriteRegisterRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_I2CWRITEREGISTERREQUEST_H
#define KORTEX_DRIVER_MESSAGE_I2CWRITEREGISTERREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/I2CWriteRegisterParameter.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct I2CWriteRegisterRequest_
{
  typedef I2CWriteRegisterRequest_<ContainerAllocator> Type;

  I2CWriteRegisterRequest_()
    : input()  {
    }
  I2CWriteRegisterRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::I2CWriteRegisterParameter_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> const> ConstPtr;

}; // struct I2CWriteRegisterRequest_

typedef ::kortex_driver::I2CWriteRegisterRequest_<std::allocator<void> > I2CWriteRegisterRequest;

typedef boost::shared_ptr< ::kortex_driver::I2CWriteRegisterRequest > I2CWriteRegisterRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::I2CWriteRegisterRequest const> I2CWriteRegisterRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3269839fc92a235a39af85ff65c4ad63";
  }

  static const char* value(const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3269839fc92a235aULL;
  static const uint64_t static_value2 = 0x39af85ff65c4ad63ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/I2CWriteRegisterRequest";
  }

  static const char* value(const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "I2CWriteRegisterParameter input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/I2CWriteRegisterParameter\n"
"\n"
"uint32 device\n"
"uint32 device_address\n"
"uint32 register_address\n"
"uint32 register_address_size\n"
"uint32 timeout\n"
"I2CData data\n"
"================================================================================\n"
"MSG: kortex_driver/I2CData\n"
"\n"
"uint8[] data\n"
"uint32 size\n"
;
  }

  static const char* value(const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct I2CWriteRegisterRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::I2CWriteRegisterRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::I2CWriteRegisterParameter_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_I2CWRITEREGISTERREQUEST_H
