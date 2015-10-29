#ifndef _ROS_SERVICE_EncoderRequest_h
#define _ROS_SERVICE_EncoderRequest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot
{

static const char ENCODERREQUEST[] = "robot/EncoderRequest";

  class EncoderRequestRequest : public ros::Msg
  {
    public:

    EncoderRequestRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return ENCODERREQUEST; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class EncoderRequestResponse : public ros::Msg
  {
    public:
      int64_t leftCount;
      int64_t rightCount;

    EncoderRequestResponse():
      leftCount(0),
      rightCount(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_leftCount;
      u_leftCount.real = this->leftCount;
      *(outbuffer + offset + 0) = (u_leftCount.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftCount.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftCount.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftCount.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_leftCount.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_leftCount.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_leftCount.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_leftCount.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->leftCount);
      union {
        int64_t real;
        uint64_t base;
      } u_rightCount;
      u_rightCount.real = this->rightCount;
      *(outbuffer + offset + 0) = (u_rightCount.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightCount.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightCount.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightCount.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rightCount.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rightCount.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rightCount.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rightCount.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rightCount);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_leftCount;
      u_leftCount.base = 0;
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->leftCount = u_leftCount.real;
      offset += sizeof(this->leftCount);
      union {
        int64_t real;
        uint64_t base;
      } u_rightCount;
      u_rightCount.base = 0;
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rightCount = u_rightCount.real;
      offset += sizeof(this->rightCount);
     return offset;
    }

    const char * getType(){ return ENCODERREQUEST; };
    const char * getMD5(){ return "da965117f83be82d823cfdc1c3990723"; };

  };

  class EncoderRequest {
    public:
    typedef EncoderRequestRequest Request;
    typedef EncoderRequestResponse Response;
  };

}
#endif
