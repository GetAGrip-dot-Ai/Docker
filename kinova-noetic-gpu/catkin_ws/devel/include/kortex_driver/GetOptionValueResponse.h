// Generated by gencpp from file kortex_driver/GetOptionValueResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETOPTIONVALUERESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETOPTIONVALUERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/OptionValue.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetOptionValueResponse_
{
  typedef GetOptionValueResponse_<ContainerAllocator> Type;

  GetOptionValueResponse_()
    : output()  {
    }
  GetOptionValueResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::OptionValue_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetOptionValueResponse_

typedef ::kortex_driver::GetOptionValueResponse_<std::allocator<void> > GetOptionValueResponse;

typedef boost::shared_ptr< ::kortex_driver::GetOptionValueResponse > GetOptionValueResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetOptionValueResponse const> GetOptionValueResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "17f8b885036a7e161a39d06f11b7725b";
  }

  static const char* value(const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x17f8b885036a7e16ULL;
  static const uint64_t static_value2 = 0x1a39d06f11b7725bULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetOptionValueResponse";
  }

  static const char* value(const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "OptionValue output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/OptionValue\n"
"\n"
"uint32 sensor\n"
"uint32 option\n"
"float32 value\n"
;
  }

  static const char* value(const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetOptionValueResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetOptionValueResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetOptionValueResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::OptionValue_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETOPTIONVALUERESPONSE_H
