// Generated by gencpp from file kortex_driver/I2CMode.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_I2CMODE_H
#define KORTEX_DRIVER_MESSAGE_I2CMODE_H


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
struct I2CMode_
{
  typedef I2CMode_<ContainerAllocator> Type;

  I2CMode_()
    {
    }
  I2CMode_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(I2C_MODE_UNSPECIFIED)
  #undef I2C_MODE_UNSPECIFIED
#endif
#if defined(_WIN32) && defined(I2C_MODE_STANDARD)
  #undef I2C_MODE_STANDARD
#endif
#if defined(_WIN32) && defined(I2C_MODE_FAST)
  #undef I2C_MODE_FAST
#endif
#if defined(_WIN32) && defined(I2C_MODE_FAST_PLUS)
  #undef I2C_MODE_FAST_PLUS
#endif

  enum {
    I2C_MODE_UNSPECIFIED = 0u,
    I2C_MODE_STANDARD = 1u,
    I2C_MODE_FAST = 2u,
    I2C_MODE_FAST_PLUS = 3u,
  };


  typedef boost::shared_ptr< ::kortex_driver::I2CMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::I2CMode_<ContainerAllocator> const> ConstPtr;

}; // struct I2CMode_

typedef ::kortex_driver::I2CMode_<std::allocator<void> > I2CMode;

typedef boost::shared_ptr< ::kortex_driver::I2CMode > I2CModePtr;
typedef boost::shared_ptr< ::kortex_driver::I2CMode const> I2CModeConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::I2CMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::I2CMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::I2CMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::I2CMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::I2CMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::I2CMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::I2CMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::I2CMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::I2CMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "56d9e828030957dbebbc8d714433e077";
  }

  static const char* value(const ::kortex_driver::I2CMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x56d9e828030957dbULL;
  static const uint64_t static_value2 = 0xebbc8d714433e077ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::I2CMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/I2CMode";
  }

  static const char* value(const ::kortex_driver::I2CMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::I2CMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 I2C_MODE_UNSPECIFIED = 0\n"
"\n"
"uint32 I2C_MODE_STANDARD = 1\n"
"\n"
"uint32 I2C_MODE_FAST = 2\n"
"\n"
"uint32 I2C_MODE_FAST_PLUS = 3\n"
;
  }

  static const char* value(const ::kortex_driver::I2CMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::I2CMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct I2CMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::I2CMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::I2CMode_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_I2CMODE_H
