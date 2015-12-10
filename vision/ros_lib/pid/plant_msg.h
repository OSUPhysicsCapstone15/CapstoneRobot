#ifndef _ROS_pid_plant_msg_h
#define _ROS_pid_plant_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pid
{

  class plant_msg : public ros::Msg
  {
    public:
      float x;
      float t;
      float setpoint;

    plant_msg():
      x(0),
      t(0),
      setpoint(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->t);
      offset += serializeAvrFloat64(outbuffer + offset, this->setpoint);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->t));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->setpoint));
     return offset;
    }

    const char * getType(){ return "pid/plant_msg"; };
    const char * getMD5(){ return "292a905e671af67a6c65822b0e898330"; };

  };

}
#endif