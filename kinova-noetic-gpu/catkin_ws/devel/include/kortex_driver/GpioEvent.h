// Generated by gencpp from file kortex_driver/GpioEvent.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GPIOEVENT_H
#define KORTEX_DRIVER_MESSAGE_GPIOEVENT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kortex_driver
{
template <class ContainerAllocator>
struct GpioEvent_
{
  typedef GpioEvent_<ContainerAllocator> Type;

  GpioEvent_()
    : input_type(0)
    , behavior(0)
    , input_identifier(0)  {
    }
  GpioEvent_(const ContainerAllocator& _alloc)
    : input_type(0)
    , behavior(0)
    , input_identifier(0)  {
  (void)_alloc;
    }



   typedef uint32_t _input_type_type;
  _input_type_type input_type;

   typedef uint32_t _behavior_type;
  _behavior_type behavior;

   typedef uint32_t _input_identifier_type;
  _input_identifier_type input_identifier;





  typedef boost::shared_ptr< ::kortex_driver::GpioEvent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GpioEvent_<ContainerAllocator> const> ConstPtr;

}; // struct GpioEvent_

typedef ::kortex_driver::GpioEvent_<std::allocator<void> > GpioEvent;

typedef boost::shared_ptr< ::kortex_driver::GpioEvent > GpioEventPtr;
typedef boost::shared_ptr< ::kortex_driver::GpioEvent const> GpioEventConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GpioEvent_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GpioEvent_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GpioEvent_<ContainerAllocator1> & lhs, const ::kortex_driver::GpioEvent_<ContainerAllocator2> & rhs)
{
  return lhs.input_type == rhs.input_type &&
    lhs.behavior == rhs.behavior &&
    lhs.input_identifier == rhs.input_identifier;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GpioEvent_<ContainerAllocator1> & lhs, const ::kortex_driver::GpioEvent_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GpioEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GpioEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GpioEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GpioEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GpioEvent_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GpioEvent_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GpioEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8f7ada35fd120a9401b9d95b206763c";
  }

  static const char* value(const ::kortex_driver::GpioEvent_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8f7ada35fd120a9ULL;
  static const uint64_t static_value2 = 0x401b9d95b206763cULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GpioEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GpioEvent";
  }

  static const char* value(const ::kortex_driver::GpioEvent_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GpioEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 input_type\n"
"uint32 behavior\n"
"uint32 input_identifier\n"
;
  }

  static const char* value(const ::kortex_driver::GpioEvent_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GpioEvent_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input_type);
      stream.next(m.behavior);
      stream.next(m.input_identifier);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GpioEvent_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GpioEvent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GpioEvent_<ContainerAllocator>& v)
  {
    s << indent << "input_type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.input_type);
    s << indent << "behavior: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.behavior);
    s << indent << "input_identifier: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.input_identifier);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GPIOEVENT_H
