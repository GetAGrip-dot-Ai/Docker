// Generated by gencpp from file kortex_driver/ArmLaterality.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ARMLATERALITY_H
#define KORTEX_DRIVER_MESSAGE_ARMLATERALITY_H


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
struct ArmLaterality_
{
  typedef ArmLaterality_<ContainerAllocator> Type;

  ArmLaterality_()
    {
    }
  ArmLaterality_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(ARM_LATERALITY_UNSPECIFIED)
  #undef ARM_LATERALITY_UNSPECIFIED
#endif
#if defined(_WIN32) && defined(ARM_LATERALITY_NOT_APPLICABLE)
  #undef ARM_LATERALITY_NOT_APPLICABLE
#endif
#if defined(_WIN32) && defined(ARM_LATERALITY_LEFT)
  #undef ARM_LATERALITY_LEFT
#endif
#if defined(_WIN32) && defined(ARM_LATERALITY_RIGHT)
  #undef ARM_LATERALITY_RIGHT
#endif

  enum {
    ARM_LATERALITY_UNSPECIFIED = 0u,
    ARM_LATERALITY_NOT_APPLICABLE = 1u,
    ARM_LATERALITY_LEFT = 2u,
    ARM_LATERALITY_RIGHT = 3u,
  };


  typedef boost::shared_ptr< ::kortex_driver::ArmLaterality_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ArmLaterality_<ContainerAllocator> const> ConstPtr;

}; // struct ArmLaterality_

typedef ::kortex_driver::ArmLaterality_<std::allocator<void> > ArmLaterality;

typedef boost::shared_ptr< ::kortex_driver::ArmLaterality > ArmLateralityPtr;
typedef boost::shared_ptr< ::kortex_driver::ArmLaterality const> ArmLateralityConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ArmLaterality_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ArmLaterality_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ArmLaterality_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ArmLaterality_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ArmLaterality_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ArmLaterality_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ArmLaterality_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ArmLaterality_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ArmLaterality_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e87e70b626397c379fd01cdd30b3b822";
  }

  static const char* value(const ::kortex_driver::ArmLaterality_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe87e70b626397c37ULL;
  static const uint64_t static_value2 = 0x9fd01cdd30b3b822ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ArmLaterality_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ArmLaterality";
  }

  static const char* value(const ::kortex_driver::ArmLaterality_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ArmLaterality_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 ARM_LATERALITY_UNSPECIFIED = 0\n"
"\n"
"uint32 ARM_LATERALITY_NOT_APPLICABLE = 1\n"
"\n"
"uint32 ARM_LATERALITY_LEFT = 2\n"
"\n"
"uint32 ARM_LATERALITY_RIGHT = 3\n"
;
  }

  static const char* value(const ::kortex_driver::ArmLaterality_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ArmLaterality_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArmLaterality_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ArmLaterality_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::ArmLaterality_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ARMLATERALITY_H
