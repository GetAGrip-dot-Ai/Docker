// Generated by gencpp from file kortex_driver/OnNotificationSequenceInfoTopicResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONSEQUENCEINFOTOPICRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONSEQUENCEINFOTOPICRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/NotificationHandle.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct OnNotificationSequenceInfoTopicResponse_
{
  typedef OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> Type;

  OnNotificationSequenceInfoTopicResponse_()
    : output()  {
    }
  OnNotificationSequenceInfoTopicResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::NotificationHandle_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> const> ConstPtr;

}; // struct OnNotificationSequenceInfoTopicResponse_

typedef ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<std::allocator<void> > OnNotificationSequenceInfoTopicResponse;

typedef boost::shared_ptr< ::kortex_driver::OnNotificationSequenceInfoTopicResponse > OnNotificationSequenceInfoTopicResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::OnNotificationSequenceInfoTopicResponse const> OnNotificationSequenceInfoTopicResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "29ff9348c5c8343d487a90668267a29e";
  }

  static const char* value(const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x29ff9348c5c8343dULL;
  static const uint64_t static_value2 = 0x487a90668267a29eULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/OnNotificationSequenceInfoTopicResponse";
  }

  static const char* value(const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "NotificationHandle output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/NotificationHandle\n"
"\n"
"uint32 identifier\n"
;
  }

  static const char* value(const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OnNotificationSequenceInfoTopicResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::OnNotificationSequenceInfoTopicResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::NotificationHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ONNOTIFICATIONSEQUENCEINFOTOPICRESPONSE_H
