// Generated by gencpp from file kortex_driver/UserNotificationList.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_USERNOTIFICATIONLIST_H
#define KORTEX_DRIVER_MESSAGE_USERNOTIFICATIONLIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/UserNotification.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct UserNotificationList_
{
  typedef UserNotificationList_<ContainerAllocator> Type;

  UserNotificationList_()
    : notifications()  {
    }
  UserNotificationList_(const ContainerAllocator& _alloc)
    : notifications(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::kortex_driver::UserNotification_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::kortex_driver::UserNotification_<ContainerAllocator> >> _notifications_type;
  _notifications_type notifications;





  typedef boost::shared_ptr< ::kortex_driver::UserNotificationList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::UserNotificationList_<ContainerAllocator> const> ConstPtr;

}; // struct UserNotificationList_

typedef ::kortex_driver::UserNotificationList_<std::allocator<void> > UserNotificationList;

typedef boost::shared_ptr< ::kortex_driver::UserNotificationList > UserNotificationListPtr;
typedef boost::shared_ptr< ::kortex_driver::UserNotificationList const> UserNotificationListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::UserNotificationList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::UserNotificationList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::UserNotificationList_<ContainerAllocator1> & lhs, const ::kortex_driver::UserNotificationList_<ContainerAllocator2> & rhs)
{
  return lhs.notifications == rhs.notifications;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::UserNotificationList_<ContainerAllocator1> & lhs, const ::kortex_driver::UserNotificationList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UserNotificationList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UserNotificationList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UserNotificationList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UserNotificationList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UserNotificationList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UserNotificationList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::UserNotificationList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "758e6f505a108790f7c5f9f46bc75ba3";
  }

  static const char* value(const ::kortex_driver::UserNotificationList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x758e6f505a108790ULL;
  static const uint64_t static_value2 = 0xf7c5f9f46bc75ba3ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::UserNotificationList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/UserNotificationList";
  }

  static const char* value(const ::kortex_driver::UserNotificationList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::UserNotificationList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"UserNotification[] notifications\n"
"================================================================================\n"
"MSG: kortex_driver/UserNotification\n"
"\n"
"uint32 user_event\n"
"UserProfileHandle modified_user\n"
"Timestamp timestamp\n"
"UserProfileHandle user_handle\n"
"Connection connection\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/Timestamp\n"
"\n"
"uint32 sec\n"
"uint32 usec\n"
"================================================================================\n"
"MSG: kortex_driver/Connection\n"
"\n"
"UserProfileHandle user_handle\n"
"string connection_information\n"
"uint32 connection_identifier\n"
;
  }

  static const char* value(const ::kortex_driver::UserNotificationList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::UserNotificationList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.notifications);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UserNotificationList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::UserNotificationList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::UserNotificationList_<ContainerAllocator>& v)
  {
    s << indent << "notifications[]" << std::endl;
    for (size_t i = 0; i < v.notifications.size(); ++i)
    {
      s << indent << "  notifications[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kortex_driver::UserNotification_<ContainerAllocator> >::stream(s, indent + "    ", v.notifications[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_USERNOTIFICATIONLIST_H
