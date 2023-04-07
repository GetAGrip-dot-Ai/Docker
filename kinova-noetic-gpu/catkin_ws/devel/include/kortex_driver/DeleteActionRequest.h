// Generated by gencpp from file kortex_driver/DeleteActionRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_DELETEACTIONREQUEST_H
#define KORTEX_DRIVER_MESSAGE_DELETEACTIONREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ActionHandle.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct DeleteActionRequest_
{
  typedef DeleteActionRequest_<ContainerAllocator> Type;

  DeleteActionRequest_()
    : input()  {
    }
  DeleteActionRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ActionHandle_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct DeleteActionRequest_

typedef ::kortex_driver::DeleteActionRequest_<std::allocator<void> > DeleteActionRequest;

typedef boost::shared_ptr< ::kortex_driver::DeleteActionRequest > DeleteActionRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::DeleteActionRequest const> DeleteActionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::DeleteActionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::DeleteActionRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::DeleteActionRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::DeleteActionRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::DeleteActionRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7f4344a5ab21306fdb1649a360654634";
  }

  static const char* value(const ::kortex_driver::DeleteActionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7f4344a5ab21306fULL;
  static const uint64_t static_value2 = 0xdb1649a360654634ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/DeleteActionRequest";
  }

  static const char* value(const ::kortex_driver::DeleteActionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ActionHandle input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/ActionHandle\n"
"\n"
"uint32 identifier\n"
"uint32 action_type\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::DeleteActionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DeleteActionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::DeleteActionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::DeleteActionRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::ActionHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_DELETEACTIONREQUEST_H
