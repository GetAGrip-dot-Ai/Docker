// Generated by gencpp from file kortex_driver/UserProfileList.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_USERPROFILELIST_H
#define KORTEX_DRIVER_MESSAGE_USERPROFILELIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/UserProfile.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct UserProfileList_
{
  typedef UserProfileList_<ContainerAllocator> Type;

  UserProfileList_()
    : user_profiles()  {
    }
  UserProfileList_(const ContainerAllocator& _alloc)
    : user_profiles(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::kortex_driver::UserProfile_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::kortex_driver::UserProfile_<ContainerAllocator> >> _user_profiles_type;
  _user_profiles_type user_profiles;





  typedef boost::shared_ptr< ::kortex_driver::UserProfileList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::UserProfileList_<ContainerAllocator> const> ConstPtr;

}; // struct UserProfileList_

typedef ::kortex_driver::UserProfileList_<std::allocator<void> > UserProfileList;

typedef boost::shared_ptr< ::kortex_driver::UserProfileList > UserProfileListPtr;
typedef boost::shared_ptr< ::kortex_driver::UserProfileList const> UserProfileListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::UserProfileList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::UserProfileList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::UserProfileList_<ContainerAllocator1> & lhs, const ::kortex_driver::UserProfileList_<ContainerAllocator2> & rhs)
{
  return lhs.user_profiles == rhs.user_profiles;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::UserProfileList_<ContainerAllocator1> & lhs, const ::kortex_driver::UserProfileList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UserProfileList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::UserProfileList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UserProfileList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::UserProfileList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UserProfileList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::UserProfileList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::UserProfileList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2d71d5ab953fb8ddd6c9d9d4a1379bb4";
  }

  static const char* value(const ::kortex_driver::UserProfileList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2d71d5ab953fb8ddULL;
  static const uint64_t static_value2 = 0xd6c9d9d4a1379bb4ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::UserProfileList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/UserProfileList";
  }

  static const char* value(const ::kortex_driver::UserProfileList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::UserProfileList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"UserProfile[] user_profiles\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfile\n"
"\n"
"UserProfileHandle handle\n"
"string username\n"
"string firstname\n"
"string lastname\n"
"string application_data\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::UserProfileList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::UserProfileList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.user_profiles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UserProfileList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::UserProfileList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::UserProfileList_<ContainerAllocator>& v)
  {
    s << indent << "user_profiles[]" << std::endl;
    for (size_t i = 0; i < v.user_profiles.size(); ++i)
    {
      s << indent << "  user_profiles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kortex_driver::UserProfile_<ContainerAllocator> >::stream(s, indent + "    ", v.user_profiles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_USERPROFILELIST_H
