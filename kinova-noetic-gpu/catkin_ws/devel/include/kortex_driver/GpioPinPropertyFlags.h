// Generated by gencpp from file kortex_driver/GpioPinPropertyFlags.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GPIOPINPROPERTYFLAGS_H
#define KORTEX_DRIVER_MESSAGE_GPIOPINPROPERTYFLAGS_H


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
struct GpioPinPropertyFlags_
{
  typedef GpioPinPropertyFlags_<ContainerAllocator> Type;

  GpioPinPropertyFlags_()
    {
    }
  GpioPinPropertyFlags_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(GPIOPROPERTY_UNKNOWN)
  #undef GPIOPROPERTY_UNKNOWN
#endif
#if defined(_WIN32) && defined(GPIOPROPERTY_INPUT)
  #undef GPIOPROPERTY_INPUT
#endif
#if defined(_WIN32) && defined(GPIOPROPERTY_OUTPUT)
  #undef GPIOPROPERTY_OUTPUT
#endif
#if defined(_WIN32) && defined(GPIOPROPERTY_ANALOG)
  #undef GPIOPROPERTY_ANALOG
#endif

  enum {
    GPIOPROPERTY_UNKNOWN = 0u,
    GPIOPROPERTY_INPUT = 1u,
    GPIOPROPERTY_OUTPUT = 2u,
    GPIOPROPERTY_ANALOG = 4u,
  };


  typedef boost::shared_ptr< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> const> ConstPtr;

}; // struct GpioPinPropertyFlags_

typedef ::kortex_driver::GpioPinPropertyFlags_<std::allocator<void> > GpioPinPropertyFlags;

typedef boost::shared_ptr< ::kortex_driver::GpioPinPropertyFlags > GpioPinPropertyFlagsPtr;
typedef boost::shared_ptr< ::kortex_driver::GpioPinPropertyFlags const> GpioPinPropertyFlagsConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >
{
  static const char* value()
  {
    return "531958ae411036543a3b84e9b7f802d3";
  }

  static const char* value(const ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x531958ae41103654ULL;
  static const uint64_t static_value2 = 0x3a3b84e9b7f802d3ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GpioPinPropertyFlags";
  }

  static const char* value(const ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 GPIOPROPERTY_UNKNOWN = 0\n"
"\n"
"uint32 GPIOPROPERTY_INPUT = 1\n"
"\n"
"uint32 GPIOPROPERTY_OUTPUT = 2\n"
"\n"
"uint32 GPIOPROPERTY_ANALOG = 4\n"
;
  }

  static const char* value(const ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GpioPinPropertyFlags_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::GpioPinPropertyFlags_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GPIOPINPROPERTYFLAGS_H
