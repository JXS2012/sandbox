/* Auto-generated by genmsg_cpp for file /home/jianxin/my_ros_workspace/sandbox/vicon_bridge/srv/viconCalibrateSegment.srv */
#ifndef VICON_BRIDGE_SERVICE_VICONCALIBRATESEGMENT_H
#define VICON_BRIDGE_SERVICE_VICONCALIBRATESEGMENT_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"



#include "geometry_msgs/PoseStamped.h"

namespace vicon_bridge
{
template <class ContainerAllocator>
struct viconCalibrateSegmentRequest_ {
  typedef viconCalibrateSegmentRequest_<ContainerAllocator> Type;

  viconCalibrateSegmentRequest_()
  : subject_name()
  , segment_name()
  , z_offset(0.0)
  , n_measurements(0)
  {
  }

  viconCalibrateSegmentRequest_(const ContainerAllocator& _alloc)
  : subject_name(_alloc)
  , segment_name(_alloc)
  , z_offset(0.0)
  , n_measurements(0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _subject_name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  subject_name;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _segment_name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  segment_name;

  typedef double _z_offset_type;
  double z_offset;

  typedef int32_t _n_measurements_type;
  int32_t n_measurements;


private:
  static const char* __s_getDataType_() { return "vicon_bridge/viconCalibrateSegmentRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "f57831d02c84e74975c7663933fe42d8"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "ca1b34be858a36828126364b1a577794"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "string subject_name\n\
string segment_name\n\
float64 z_offset\n\
int32 n_measurements\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, subject_name);
    ros::serialization::serialize(stream, segment_name);
    ros::serialization::serialize(stream, z_offset);
    ros::serialization::serialize(stream, n_measurements);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, subject_name);
    ros::serialization::deserialize(stream, segment_name);
    ros::serialization::deserialize(stream, z_offset);
    ros::serialization::deserialize(stream, n_measurements);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(subject_name);
    size += ros::serialization::serializationLength(segment_name);
    size += ros::serialization::serializationLength(z_offset);
    size += ros::serialization::serializationLength(n_measurements);
    return size;
  }

  typedef boost::shared_ptr< ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct viconCalibrateSegmentRequest
typedef  ::vicon_bridge::viconCalibrateSegmentRequest_<std::allocator<void> > viconCalibrateSegmentRequest;

typedef boost::shared_ptr< ::vicon_bridge::viconCalibrateSegmentRequest> viconCalibrateSegmentRequestPtr;
typedef boost::shared_ptr< ::vicon_bridge::viconCalibrateSegmentRequest const> viconCalibrateSegmentRequestConstPtr;


template <class ContainerAllocator>
struct viconCalibrateSegmentResponse_ {
  typedef viconCalibrateSegmentResponse_<ContainerAllocator> Type;

  viconCalibrateSegmentResponse_()
  : success(false)
  , status()
  , pose()
  {
  }

  viconCalibrateSegmentResponse_(const ContainerAllocator& _alloc)
  : success(false)
  , status(_alloc)
  , pose(_alloc)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  status;

  typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _pose_type;
   ::geometry_msgs::PoseStamped_<ContainerAllocator>  pose;


private:
  static const char* __s_getDataType_() { return "vicon_bridge/viconCalibrateSegmentResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "fd8b451f9e0c65ec25938e0acbd102d7"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "ca1b34be858a36828126364b1a577794"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool success\n\
string status\n\
geometry_msgs/PoseStamped pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, success);
    ros::serialization::serialize(stream, status);
    ros::serialization::serialize(stream, pose);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, success);
    ros::serialization::deserialize(stream, status);
    ros::serialization::deserialize(stream, pose);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(success);
    size += ros::serialization::serializationLength(status);
    size += ros::serialization::serializationLength(pose);
    return size;
  }

  typedef boost::shared_ptr< ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct viconCalibrateSegmentResponse
typedef  ::vicon_bridge::viconCalibrateSegmentResponse_<std::allocator<void> > viconCalibrateSegmentResponse;

typedef boost::shared_ptr< ::vicon_bridge::viconCalibrateSegmentResponse> viconCalibrateSegmentResponsePtr;
typedef boost::shared_ptr< ::vicon_bridge::viconCalibrateSegmentResponse const> viconCalibrateSegmentResponseConstPtr;

struct viconCalibrateSegment
{

typedef viconCalibrateSegmentRequest Request;
typedef viconCalibrateSegmentResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct viconCalibrateSegment
} // namespace vicon_bridge

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f57831d02c84e74975c7663933fe42d8";
  }

  static const char* value(const  ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf57831d02c84e749ULL;
  static const uint64_t static_value2 = 0x75c7663933fe42d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vicon_bridge/viconCalibrateSegmentRequest";
  }

  static const char* value(const  ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string subject_name\n\
string segment_name\n\
float64 z_offset\n\
int32 n_measurements\n\
\n\
";
  }

  static const char* value(const  ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fd8b451f9e0c65ec25938e0acbd102d7";
  }

  static const char* value(const  ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfd8b451f9e0c65ecULL;
  static const uint64_t static_value2 = 0x25938e0acbd102d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vicon_bridge/viconCalibrateSegmentResponse";
  }

  static const char* value(const  ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
string status\n\
geometry_msgs/PoseStamped pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.subject_name);
    stream.next(m.segment_name);
    stream.next(m.z_offset);
    stream.next(m.n_measurements);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct viconCalibrateSegmentRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
    stream.next(m.status);
    stream.next(m.pose);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct viconCalibrateSegmentResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vicon_bridge::viconCalibrateSegment> {
  static const char* value() 
  {
    return "ca1b34be858a36828126364b1a577794";
  }

  static const char* value(const vicon_bridge::viconCalibrateSegment&) { return value(); } 
};

template<>
struct DataType<vicon_bridge::viconCalibrateSegment> {
  static const char* value() 
  {
    return "vicon_bridge/viconCalibrateSegment";
  }

  static const char* value(const vicon_bridge::viconCalibrateSegment&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ca1b34be858a36828126364b1a577794";
  }

  static const char* value(const vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vicon_bridge/viconCalibrateSegment";
  }

  static const char* value(const vicon_bridge::viconCalibrateSegmentRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ca1b34be858a36828126364b1a577794";
  }

  static const char* value(const vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vicon_bridge/viconCalibrateSegment";
  }

  static const char* value(const vicon_bridge::viconCalibrateSegmentResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VICON_BRIDGE_SERVICE_VICONCALIBRATESEGMENT_H

