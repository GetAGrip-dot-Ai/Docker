// Generated by gencpp from file kortex_driver/Orientation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ORIENTATION_H
#define KORTEX_DRIVER_MESSAGE_ORIENTATION_H


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
struct Orientation_
{
  typedef Orientation_<ContainerAllocator> Type;

  Orientation_()
    : theta_x(0.0)
    , theta_y(0.0)
    , theta_z(0.0)  {
    }
  Orientation_(const ContainerAllocator& _alloc)
    : theta_x(0.0)
    , theta_y(0.0)
    , theta_z(0.0)  {
  (void)_alloc;
    }



   typedef float _theta_x_type;
  _theta_x_type theta_x;

   typedef float _theta_y_type;
  _theta_y_type theta_y;

   typedef float _theta_z_type;
  _theta_z_type theta_z;





  typedef boost::shared_ptr< ::kortex_driver::Orientation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::Orientation_<ContainerAllocator> const> ConstPtr;

}; // struct Orientation_

typedef ::kortex_driver::Orientation_<std::allocator<void> > Orientation;

typedef boost::shared_ptr< ::kortex_driver::Orientation > OrientationPtr;
typedef boost::shared_ptr< ::kortex_driver::Orientation const> OrientationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::Orientation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::Orientation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::Orientation_<ContainerAllocator1> & lhs, const ::kortex_driver::Orientation_<ContainerAllocator2> & rhs)
{
  return lhs.theta_x == rhs.theta_x &&
    lhs.theta_y == rhs.theta_y &&
    lhs.theta_z == rhs.theta_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::Orientation_<ContainerAllocator1> & lhs, const ::kortex_driver::Orientation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Orientation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Orientation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Orientation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Orientation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Orientation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Orientation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::Orientation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2fb1d2b19db735698f3a1ad8c8e9d45b";
  }

  static const char* value(const ::kortex_driver::Orientation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2fb1d2b19db73569ULL;
  static const uint64_t static_value2 = 0x8f3a1ad8c8e9d45bULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::Orientation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/Orientation";
  }

  static const char* value(const ::kortex_driver::Orientation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::Orientation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"float32 theta_x\n"
"float32 theta_y\n"
"float32 theta_z\n"
;
  }

  static const char* value(const ::kortex_driver::Orientation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::Orientation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.theta_x);
      stream.next(m.theta_y);
      stream.next(m.theta_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Orientation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::Orientation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::Orientation_<ContainerAllocator>& v)
  {
    s << indent << "theta_x: ";
    Printer<float>::stream(s, indent + "  ", v.theta_x);
    s << indent << "theta_y: ";
    Printer<float>::stream(s, indent + "  ", v.theta_y);
    s << indent << "theta_z: ";
    Printer<float>::stream(s, indent + "  ", v.theta_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ORIENTATION_H
