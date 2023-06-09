// Generated by gencpp from file kortex_driver/ValidateWaypointListRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_VALIDATEWAYPOINTLISTREQUEST_H
#define KORTEX_DRIVER_MESSAGE_VALIDATEWAYPOINTLISTREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/WaypointList.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ValidateWaypointListRequest_
{
  typedef ValidateWaypointListRequest_<ContainerAllocator> Type;

  ValidateWaypointListRequest_()
    : input()  {
    }
  ValidateWaypointListRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::WaypointList_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ValidateWaypointListRequest_

typedef ::kortex_driver::ValidateWaypointListRequest_<std::allocator<void> > ValidateWaypointListRequest;

typedef boost::shared_ptr< ::kortex_driver::ValidateWaypointListRequest > ValidateWaypointListRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::ValidateWaypointListRequest const> ValidateWaypointListRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e9c355de078272f6acdb19f10ac9518d";
  }

  static const char* value(const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe9c355de078272f6ULL;
  static const uint64_t static_value2 = 0xacdb19f10ac9518dULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ValidateWaypointListRequest";
  }

  static const char* value(const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "WaypointList input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/WaypointList\n"
"\n"
"Waypoint[] waypoints\n"
"float32 duration\n"
"bool use_optimal_blending\n"
"================================================================================\n"
"MSG: kortex_driver/Waypoint\n"
"\n"
"string name\n"
"Waypoint_type_of_waypoint oneof_type_of_waypoint\n"
"================================================================================\n"
"MSG: kortex_driver/Waypoint_type_of_waypoint\n"
"\n"
"AngularWaypoint[] angular_waypoint\n"
"CartesianWaypoint[] cartesian_waypoint\n"
"================================================================================\n"
"MSG: kortex_driver/AngularWaypoint\n"
"\n"
"float32[] angles\n"
"float32[] maximum_velocities\n"
"float32 duration\n"
"================================================================================\n"
"MSG: kortex_driver/CartesianWaypoint\n"
"\n"
"Pose pose\n"
"uint32 reference_frame\n"
"float32 maximum_linear_velocity\n"
"float32 maximum_angular_velocity\n"
"float32 blending_radius\n"
"================================================================================\n"
"MSG: kortex_driver/Pose\n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 theta_x\n"
"float32 theta_y\n"
"float32 theta_z\n"
;
  }

  static const char* value(const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ValidateWaypointListRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ValidateWaypointListRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::WaypointList_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_VALIDATEWAYPOINTLISTREQUEST_H
