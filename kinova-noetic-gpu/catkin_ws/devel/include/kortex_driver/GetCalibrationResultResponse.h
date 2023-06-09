// Generated by gencpp from file kortex_driver/GetCalibrationResultResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETCALIBRATIONRESULTRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETCALIBRATIONRESULTRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/CalibrationResult.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetCalibrationResultResponse_
{
  typedef GetCalibrationResultResponse_<ContainerAllocator> Type;

  GetCalibrationResultResponse_()
    : output()  {
    }
  GetCalibrationResultResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::CalibrationResult_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetCalibrationResultResponse_

typedef ::kortex_driver::GetCalibrationResultResponse_<std::allocator<void> > GetCalibrationResultResponse;

typedef boost::shared_ptr< ::kortex_driver::GetCalibrationResultResponse > GetCalibrationResultResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetCalibrationResultResponse const> GetCalibrationResultResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c30da30421956cb0c18ba7bbbeb697ac";
  }

  static const char* value(const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc30da30421956cb0ULL;
  static const uint64_t static_value2 = 0xc18ba7bbbeb697acULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetCalibrationResultResponse";
  }

  static const char* value(const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "CalibrationResult output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/CalibrationResult\n"
"\n"
"uint32 calibration_status\n"
"uint32 calibration_details\n"
;
  }

  static const char* value(const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetCalibrationResultResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetCalibrationResultResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::CalibrationResult_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETCALIBRATIONRESULTRESPONSE_H
