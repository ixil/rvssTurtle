// Generated by gencpp from file rvss_workshop/kalmanState.msg
// DO NOT EDIT!


#ifndef RVSS_WORKSHOP_MESSAGE_KALMANSTATE_H
#define RVSS_WORKSHOP_MESSAGE_KALMANSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace rvss_workshop
{
template <class ContainerAllocator>
struct kalmanState_
{
  typedef kalmanState_<ContainerAllocator> Type;

  kalmanState_()
    : header()
    , stateMean()
    , stateCovariance()
    , seenLandmarks()  {
    }
  kalmanState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , stateMean(_alloc)
    , stateCovariance(_alloc)
    , seenLandmarks(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _stateMean_type;
  _stateMean_type stateMean;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _stateCovariance_type;
  _stateCovariance_type stateCovariance;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _seenLandmarks_type;
  _seenLandmarks_type seenLandmarks;




  typedef boost::shared_ptr< ::rvss_workshop::kalmanState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rvss_workshop::kalmanState_<ContainerAllocator> const> ConstPtr;

}; // struct kalmanState_

typedef ::rvss_workshop::kalmanState_<std::allocator<void> > kalmanState;

typedef boost::shared_ptr< ::rvss_workshop::kalmanState > kalmanStatePtr;
typedef boost::shared_ptr< ::rvss_workshop::kalmanState const> kalmanStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rvss_workshop::kalmanState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rvss_workshop::kalmanState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rvss_workshop

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'rvss_workshop': ['/home/monty/RVSS_ws/src/rvss_workshop/msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rvss_workshop::kalmanState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rvss_workshop::kalmanState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rvss_workshop::kalmanState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rvss_workshop::kalmanState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rvss_workshop::kalmanState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rvss_workshop::kalmanState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rvss_workshop::kalmanState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0f14a0eeede5ba8bdddce3e0e54f3254";
  }

  static const char* value(const ::rvss_workshop::kalmanState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0f14a0eeede5ba8bULL;
  static const uint64_t static_value2 = 0xdddce3e0e54f3254ULL;
};

template<class ContainerAllocator>
struct DataType< ::rvss_workshop::kalmanState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rvss_workshop/kalmanState";
  }

  static const char* value(const ::rvss_workshop::kalmanState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rvss_workshop::kalmanState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32[] stateMean\n\
float32[] stateCovariance\n\
float32[] seenLandmarks\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::rvss_workshop::kalmanState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rvss_workshop::kalmanState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.stateMean);
      stream.next(m.stateCovariance);
      stream.next(m.seenLandmarks);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct kalmanState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rvss_workshop::kalmanState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rvss_workshop::kalmanState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "stateMean[]" << std::endl;
    for (size_t i = 0; i < v.stateMean.size(); ++i)
    {
      s << indent << "  stateMean[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.stateMean[i]);
    }
    s << indent << "stateCovariance[]" << std::endl;
    for (size_t i = 0; i < v.stateCovariance.size(); ++i)
    {
      s << indent << "  stateCovariance[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.stateCovariance[i]);
    }
    s << indent << "seenLandmarks[]" << std::endl;
    for (size_t i = 0; i < v.seenLandmarks.size(); ++i)
    {
      s << indent << "  seenLandmarks[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.seenLandmarks[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RVSS_WORKSHOP_MESSAGE_KALMANSTATE_H
