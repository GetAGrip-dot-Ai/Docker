// Generated by gencpp from file kortex_driver/GetAllKinematicSoftLimitsResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETALLKINEMATICSOFTLIMITSRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETALLKINEMATICSOFTLIMITSRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/KinematicLimitsList.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetAllKinematicSoftLimitsResponse_
{
  typedef GetAllKinematicSoftLimitsResponse_<ContainerAllocator> Type;

  GetAllKinematicSoftLimitsResponse_()
    : output()  {
    }
  GetAllKinematicSoftLimitsResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::KinematicLimitsList_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetAllKinematicSoftLimitsResponse_

typedef ::kortex_driver::GetAllKinematicSoftLimitsResponse_<std::allocator<void> > GetAllKinematicSoftLimitsResponse;

typedef boost::shared_ptr< ::kortex_driver::GetAllKinematicSoftLimitsResponse > GetAllKinematicSoftLimitsResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetAllKinematicSoftLimitsResponse const> GetAllKinematicSoftLimitsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f318d6cff46e5c002a37e34b98bbff7";
  }

  static const char* value(const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f318d6cff46e5c0ULL;
  static const uint64_t static_value2 = 0x02a37e34b98bbff7ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetAllKinematicSoftLimitsResponse";
  }

  static const char* value(const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "KinematicLimitsList output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/KinematicLimitsList\n"
"\n"
"KinematicLimits[] kinematic_limits_list\n"
"================================================================================\n"
"MSG: kortex_driver/KinematicLimits\n"
"\n"
"uint32 control_mode\n"
"float32 twist_linear\n"
"float32 twist_angular\n"
"float32[] joint_speed_limits\n"
"float32[] joint_acceleration_limits\n"
;
  }

  static const char* value(const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetAllKinematicSoftLimitsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetAllKinematicSoftLimitsResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::KinematicLimitsList_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETALLKINEMATICSOFTLIMITSRESPONSE_H
