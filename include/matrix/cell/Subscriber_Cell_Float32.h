#pragma once
#ifndef _SUBSCRIBER_CELL_FLOAT32_H_
#define _SUBSCRIBER_CELL_FLOAT32_H_

#include <string>
#include <cstring>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "../config/console_format.h"

using namespace ros;
using namespace std;

class Subscriber_Cell_Float32
{
  private:
    ros::NodeHandle n_;
    ros::Subscriber *sub_ = NULL;
    string topic_ = "";
    int queue_size_ = 4;
    boost::shared_ptr< std_msgs::Float32 > msg_;
    
    void callBack(const std_msgs::Float32::ConstPtr& msg)
    {
      this -> msg_ -> data = msg -> data;
      flag_update = true;
    }
    
  private:
    bool flag_update = false;
    
  public:
    string getTopic() { return topic_; };
    boost::shared_ptr< std_msgs::Float32 > getMsg() { return msg_; };
    float getData() { return msg_ -> data; };
    bool init(ros::NodeHandle& n, string topic, int queue_size, boost::shared_ptr< std_msgs::Float32 >& msg)
    {
      if(sub_ != NULL) this -> shutdown();
      n_ = n;
      sub_ = new ros::Subscriber;
      *sub_ = n_.subscribe(topic.c_str(), queue_size, &Subscriber_Cell_Float32::callBack, this);
      this -> topic_ = sub_ -> getTopic();
      this -> queue_size_ = queue_size;
      msg = msg_;
    }
    bool init(ros::NodeHandle& n, string topic, int queue_size)
    {
      if(sub_ != NULL) this -> shutdown();
      n_ = n;
      sub_ = new ros::Subscriber;
      *sub_ = n_.subscribe(topic.c_str(), queue_size, &Subscriber_Cell_Float32::callBack, this);
      this -> topic_ = sub_ -> getTopic();
      this -> queue_size_ = queue_size;
    }
    void shutdown()
    {
      if(sub_ != NULL)
      {
        sub_ -> shutdown();
        delete sub_;
      }
    }
    
  public:
    Subscriber_Cell_Float32();
    ~Subscriber_Cell_Float32();
    Subscriber_Cell_Float32 *prev = NULL;
    Subscriber_Cell_Float32 *next = NULL;
};

Subscriber_Cell_Float32::Subscriber_Cell_Float32() : msg_(new std_msgs::Float32)
{}

Subscriber_Cell_Float32::~Subscriber_Cell_Float32()
{
    shutdown();
    if(next != this || next != NULL)
        delete next;
}

#endif
