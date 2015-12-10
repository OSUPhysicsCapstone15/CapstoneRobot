#ifndef _ROS_pid_controller_msg_h
#define _ROS_pid_controller_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pid
{

  class controller_msg : public ros::Msg
  {
    public:
      float u;

    controller_msg():
      u(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->u);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->u));
     return offset;
    }

    const char * getType(){ return "pid/controller_msg"; };
    const char * getMD5(){ return "988df341e727ad40b85d2b8acf9471e9"; };

  };

}
#endif