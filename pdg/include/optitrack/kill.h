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
 * Auto-generated by gensrv_cpp from file /home/sdevin/openrobots/src/robotpkg/wip/optitrack-genom3/work.xtion/templates/ros/client/ros/optitrack/srv/kill.srv
 *
 */


#ifndef OPTITRACK_MESSAGE_KILL_H
#define OPTITRACK_MESSAGE_KILL_H

#include <ros/service_traits.h>


#include <optitrack/killRequest.h>
#include <optitrack/killResponse.h>


namespace optitrack
{

struct kill
{

typedef killRequest Request;
typedef killResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct kill
} // namespace optitrack


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::optitrack::kill > {
  static const char* value()
  {
    return "0d79450287345aef3f3e331856b25242";
  }

  static const char* value(const ::optitrack::kill&) { return value(); }
};

template<>
struct DataType< ::optitrack::kill > {
  static const char* value()
  {
    return "optitrack/kill";
  }

  static const char* value(const ::optitrack::kill&) { return value(); }
};


// service_traits::MD5Sum< ::optitrack::killRequest> should match 
// service_traits::MD5Sum< ::optitrack::kill > 
template<>
struct MD5Sum< ::optitrack::killRequest>
{
  static const char* value()
  {
    return MD5Sum< ::optitrack::kill >::value();
  }
  static const char* value(const ::optitrack::killRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::optitrack::killRequest> should match 
// service_traits::DataType< ::optitrack::kill > 
template<>
struct DataType< ::optitrack::killRequest>
{
  static const char* value()
  {
    return DataType< ::optitrack::kill >::value();
  }
  static const char* value(const ::optitrack::killRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::optitrack::killResponse> should match 
// service_traits::MD5Sum< ::optitrack::kill > 
template<>
struct MD5Sum< ::optitrack::killResponse>
{
  static const char* value()
  {
    return MD5Sum< ::optitrack::kill >::value();
  }
  static const char* value(const ::optitrack::killResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::optitrack::killResponse> should match 
// service_traits::DataType< ::optitrack::kill > 
template<>
struct DataType< ::optitrack::killResponse>
{
  static const char* value()
  {
    return DataType< ::optitrack::kill >::value();
  }
  static const char* value(const ::optitrack::killResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OPTITRACK_MESSAGE_KILL_H
