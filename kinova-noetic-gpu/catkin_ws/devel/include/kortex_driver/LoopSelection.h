// Generated by gencpp from file kortex_driver/LoopSelection.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_LOOPSELECTION_H
#define KORTEX_DRIVER_MESSAGE_LOOPSELECTION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kortex_driver
{
template <class ContainerAllocator>
struct LoopSelection_
{
  typedef LoopSelection_<ContainerAllocator> Type;

  LoopSelection_()
    : loop_selection(0)  {
    }
  LoopSelection_(const ContainerAllocator& _alloc)
    : loop_selection(0)  {
  (void)_alloc;
    }



   typedef uint32_t _loop_selection_type;
  _loop_selection_type loop_selection;





  typedef boost::shared_ptr< ::kortex_driver::LoopSelection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::LoopSelection_<ContainerAllocator> const> ConstPtr;

}; // struct LoopSelection_

typedef ::kortex_driver::LoopSelection_<std::allocator<void> > LoopSelection;

typedef boost::shared_ptr< ::kortex_driver::LoopSelection > LoopSelectionPtr;
typedef boost::shared_ptr< ::kortex_driver::LoopSelection const> LoopSelectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::LoopSelection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::LoopSelection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::LoopSelection_<ContainerAllocator1> & lhs, const ::kortex_driver::LoopSelection_<ContainerAllocator2> & rhs)
{
  return lhs.loop_selection == rhs.loop_selection;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::LoopSelection_<ContainerAllocator1> & lhs, const ::kortex_driver::LoopSelection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::LoopSelection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::LoopSelection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::LoopSelection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::LoopSelection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::LoopSelection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::LoopSelection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::LoopSelection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "404b340f5699ef6e44d5690bdce228f3";
  }

  static const char* value(const ::kortex_driver::LoopSelection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x404b340f5699ef6eULL;
  static const uint64_t static_value2 = 0x44d5690bdce228f3ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::LoopSelection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/LoopSelection";
  }

  static const char* value(const ::kortex_driver::LoopSelection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::LoopSelection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 loop_selection\n"
;
  }

  static const char* value(const ::kortex_driver::LoopSelection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::LoopSelection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.loop_selection);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LoopSelection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::LoopSelection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::LoopSelection_<ContainerAllocator>& v)
  {
    s << indent << "loop_selection: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.loop_selection);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_LOOPSELECTION_H
