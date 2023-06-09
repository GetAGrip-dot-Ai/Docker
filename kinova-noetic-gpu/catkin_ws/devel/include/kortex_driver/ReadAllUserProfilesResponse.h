// Generated by gencpp from file kortex_driver/ReadAllUserProfilesResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_READALLUSERPROFILESRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_READALLUSERPROFILESRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/UserProfileList.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ReadAllUserProfilesResponse_
{
  typedef ReadAllUserProfilesResponse_<ContainerAllocator> Type;

  ReadAllUserProfilesResponse_()
    : output()  {
    }
  ReadAllUserProfilesResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::UserProfileList_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ReadAllUserProfilesResponse_

typedef ::kortex_driver::ReadAllUserProfilesResponse_<std::allocator<void> > ReadAllUserProfilesResponse;

typedef boost::shared_ptr< ::kortex_driver::ReadAllUserProfilesResponse > ReadAllUserProfilesResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::ReadAllUserProfilesResponse const> ReadAllUserProfilesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "95aa94f3de5b5bf3c232754099cd6ba7";
  }

  static const char* value(const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x95aa94f3de5b5bf3ULL;
  static const uint64_t static_value2 = 0xc232754099cd6ba7ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ReadAllUserProfilesResponse";
  }

  static const char* value(const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "UserProfileList output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileList\n"
"\n"
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

  static const char* value(const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReadAllUserProfilesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ReadAllUserProfilesResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::UserProfileList_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_READALLUSERPROFILESRESPONSE_H
