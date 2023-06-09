// Generated by gencpp from file kortex_driver/GetOptionValueRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETOPTIONVALUEREQUEST_H
#define KORTEX_DRIVER_MESSAGE_GETOPTIONVALUEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/OptionIdentifier.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetOptionValueRequest_
{
  typedef GetOptionValueRequest_<ContainerAllocator> Type;

  GetOptionValueRequest_()
    : input()  {
    }
  GetOptionValueRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::OptionIdentifier_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetOptionValueRequest_

typedef ::kortex_driver::GetOptionValueRequest_<std::allocator<void> > GetOptionValueRequest;

typedef boost::shared_ptr< ::kortex_driver::GetOptionValueRequest > GetOptionValueRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::GetOptionValueRequest const> GetOptionValueRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e66b1b0de1a710df3ff2cc0bc64b9ced";
  }

  static const char* value(const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe66b1b0de1a710dfULL;
  static const uint64_t static_value2 = 0x3ff2cc0bc64b9cedULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetOptionValueRequest";
  }

  static const char* value(const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "OptionIdentifier input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/OptionIdentifier\n"
"\n"
"uint32 sensor\n"
"uint32 option\n"
;
  }

  static const char* value(const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetOptionValueRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetOptionValueRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetOptionValueRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::OptionIdentifier_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETOPTIONVALUEREQUEST_H
