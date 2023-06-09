// Generated by gencpp from file kortex_driver/CalibrationParameter_value.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CALIBRATIONPARAMETER_VALUE_H
#define KORTEX_DRIVER_MESSAGE_CALIBRATIONPARAMETER_VALUE_H


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
struct CalibrationParameter_value_
{
  typedef CalibrationParameter_value_<ContainerAllocator> Type;

  CalibrationParameter_value_()
    : signedIntValue()
    , unsignedIntValue()
    , floatValue()  {
    }
  CalibrationParameter_value_(const ContainerAllocator& _alloc)
    : signedIntValue(_alloc)
    , unsignedIntValue(_alloc)
    , floatValue(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> _signedIntValue_type;
  _signedIntValue_type signedIntValue;

   typedef std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> _unsignedIntValue_type;
  _unsignedIntValue_type unsignedIntValue;

   typedef std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> _floatValue_type;
  _floatValue_type floatValue;





  typedef boost::shared_ptr< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> const> ConstPtr;

}; // struct CalibrationParameter_value_

typedef ::kortex_driver::CalibrationParameter_value_<std::allocator<void> > CalibrationParameter_value;

typedef boost::shared_ptr< ::kortex_driver::CalibrationParameter_value > CalibrationParameter_valuePtr;
typedef boost::shared_ptr< ::kortex_driver::CalibrationParameter_value const> CalibrationParameter_valueConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator1> & lhs, const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator2> & rhs)
{
  return lhs.signedIntValue == rhs.signedIntValue &&
    lhs.unsignedIntValue == rhs.unsignedIntValue &&
    lhs.floatValue == rhs.floatValue;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator1> & lhs, const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2b45c1a772c79ef326799a208a383734";
  }

  static const char* value(const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2b45c1a772c79ef3ULL;
  static const uint64_t static_value2 = 0x26799a208a383734ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/CalibrationParameter_value";
  }

  static const char* value(const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32[] signedIntValue\n"
"uint32[] unsignedIntValue\n"
"uint32[] floatValue\n"
;
  }

  static const char* value(const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.signedIntValue);
      stream.next(m.unsignedIntValue);
      stream.next(m.floatValue);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CalibrationParameter_value_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::CalibrationParameter_value_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::CalibrationParameter_value_<ContainerAllocator>& v)
  {
    s << indent << "signedIntValue[]" << std::endl;
    for (size_t i = 0; i < v.signedIntValue.size(); ++i)
    {
      s << indent << "  signedIntValue[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.signedIntValue[i]);
    }
    s << indent << "unsignedIntValue[]" << std::endl;
    for (size_t i = 0; i < v.unsignedIntValue.size(); ++i)
    {
      s << indent << "  unsignedIntValue[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.unsignedIntValue[i]);
    }
    s << indent << "floatValue[]" << std::endl;
    for (size_t i = 0; i < v.floatValue.size(); ++i)
    {
      s << indent << "  floatValue[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.floatValue[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CALIBRATIONPARAMETER_VALUE_H
