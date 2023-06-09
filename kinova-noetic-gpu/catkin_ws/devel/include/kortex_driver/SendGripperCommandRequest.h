// Generated by gencpp from file kortex_driver/SendGripperCommandRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SENDGRIPPERCOMMANDREQUEST_H
#define KORTEX_DRIVER_MESSAGE_SENDGRIPPERCOMMANDREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/GripperCommand.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct SendGripperCommandRequest_
{
  typedef SendGripperCommandRequest_<ContainerAllocator> Type;

  SendGripperCommandRequest_()
    : input()  {
    }
  SendGripperCommandRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::GripperCommand_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SendGripperCommandRequest_

typedef ::kortex_driver::SendGripperCommandRequest_<std::allocator<void> > SendGripperCommandRequest;

typedef boost::shared_ptr< ::kortex_driver::SendGripperCommandRequest > SendGripperCommandRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::SendGripperCommandRequest const> SendGripperCommandRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b2e28834f592100adb21267b0746aa2d";
  }

  static const char* value(const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb2e28834f592100aULL;
  static const uint64_t static_value2 = 0xdb21267b0746aa2dULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/SendGripperCommandRequest";
  }

  static const char* value(const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "GripperCommand input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/GripperCommand\n"
"\n"
"uint32 mode\n"
"Gripper gripper\n"
"uint32 duration\n"
"================================================================================\n"
"MSG: kortex_driver/Gripper\n"
"\n"
"Finger[] finger\n"
"================================================================================\n"
"MSG: kortex_driver/Finger\n"
"\n"
"uint32 finger_identifier\n"
"float32 value\n"
;
  }

  static const char* value(const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SendGripperCommandRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::SendGripperCommandRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::GripperCommand_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SENDGRIPPERCOMMANDREQUEST_H
