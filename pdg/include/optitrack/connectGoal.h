/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/sdevin/openrobots/src/robotpkg/wip/optitrack-genom3/work.xtion/templates/ros/client/ros/optitrack/msg/connectGoal.msg
 *
 */


#ifndef OPTITRACK_MESSAGE_CONNECTGOAL_H
#define OPTITRACK_MESSAGE_CONNECTGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace optitrack
{
template <class ContainerAllocator>
struct connectGoal_
{
  typedef connectGoal_<ContainerAllocator> Type;

  connectGoal_()
    : host()
    , host_port()
    , mcast()
    , mcast_port()  {
    }
  connectGoal_(const ContainerAllocator& _alloc)
    : host(_alloc)
    , host_port(_alloc)
    , mcast(_alloc)
    , mcast_port(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _host_type;
  _host_type host;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _host_port_type;
  _host_port_type host_port;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mcast_type;
  _mcast_type mcast;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mcast_port_type;
  _mcast_port_type mcast_port;




  typedef boost::shared_ptr< ::optitrack::connectGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack::connectGoal_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct connectGoal_

typedef ::optitrack::connectGoal_<std::allocator<void> > connectGoal;

typedef boost::shared_ptr< ::optitrack::connectGoal > connectGoalPtr;
typedef boost::shared_ptr< ::optitrack::connectGoal const> connectGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack::connectGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack::connectGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace optitrack

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/msg'], 'actionlib_msgs': ['/opt/ros/groovy/share/actionlib_msgs/msg'], 'optitrack': ['optitrack/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::connectGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::connectGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::connectGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::connectGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::connectGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::connectGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack::connectGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "72e781da33ee2b450ac8874774da10e4";
  }

  static const char* value(const ::optitrack::connectGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x72e781da33ee2b45ULL;
  static const uint64_t static_value2 = 0x0ac8874774da10e4ULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack::connectGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack/connectGoal";
  }

  static const char* value(const ::optitrack::connectGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack::connectGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# service connect\n\
string host\n\
string host_port\n\
string mcast\n\
string mcast_port\n\
";
  }

  static const char* value(const ::optitrack::connectGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack::connectGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.host);
      stream.next(m.host_port);
      stream.next(m.mcast);
      stream.next(m.mcast_port);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct connectGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack::connectGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack::connectGoal_<ContainerAllocator>& v)
  {
    s << indent << "host: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.host);
    s << indent << "host_port: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.host_port);
    s << indent << "mcast: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.mcast);
    s << indent << "mcast_port: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.mcast_port);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_MESSAGE_CONNECTGOAL_H
