// Generated by gencpp from file kortex_driver/JointAccelerationSoftLimits.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_JOINTACCELERATIONSOFTLIMITS_H
#define KORTEX_DRIVER_MESSAGE_JOINTACCELERATIONSOFTLIMITS_H


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
struct JointAccelerationSoftLimits_
{
  typedef JointAccelerationSoftLimits_<ContainerAllocator> Type;

  JointAccelerationSoftLimits_()
    : control_mode(0)
    , joint_acceleration_soft_limits()  {
    }
  JointAccelerationSoftLimits_(const ContainerAllocator& _alloc)
    : control_mode(0)
    , joint_acceleration_soft_limits(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _control_mode_type;
  _control_mode_type control_mode;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _joint_acceleration_soft_limits_type;
  _joint_acceleration_soft_limits_type joint_acceleration_soft_limits;





  typedef boost::shared_ptr< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> const> ConstPtr;

}; // struct JointAccelerationSoftLimits_

typedef ::kortex_driver::JointAccelerationSoftLimits_<std::allocator<void> > JointAccelerationSoftLimits;

typedef boost::shared_ptr< ::kortex_driver::JointAccelerationSoftLimits > JointAccelerationSoftLimitsPtr;
typedef boost::shared_ptr< ::kortex_driver::JointAccelerationSoftLimits const> JointAccelerationSoftLimitsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator1> & lhs, const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator2> & rhs)
{
  return lhs.control_mode == rhs.control_mode &&
    lhs.joint_acceleration_soft_limits == rhs.joint_acceleration_soft_limits;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator1> & lhs, const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >
{
  static const char* value()
  {
    return "69f7da5340a05ca3142f1af9be835bb9";
  }

  static const char* value(const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x69f7da5340a05ca3ULL;
  static const uint64_t static_value2 = 0x142f1af9be835bb9ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/JointAccelerationSoftLimits";
  }

  static const char* value(const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 control_mode\n"
"float32[] joint_acceleration_soft_limits\n"
;
  }

  static const char* value(const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.control_mode);
      stream.next(m.joint_acceleration_soft_limits);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointAccelerationSoftLimits_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::JointAccelerationSoftLimits_<ContainerAllocator>& v)
  {
    s << indent << "control_mode: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.control_mode);
    s << indent << "joint_acceleration_soft_limits[]" << std::endl;
    for (size_t i = 0; i < v.joint_acceleration_soft_limits.size(); ++i)
    {
      s << indent << "  joint_acceleration_soft_limits[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.joint_acceleration_soft_limits[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_JOINTACCELERATIONSOFTLIMITS_H
