#pragma once
#ifndef _IR_Color_Convertor_H_
#define _IR_Color_Convertor_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for colorerting the command line parameter to integer
#include <string>
#include <cstring>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
//#include <std_msgs/MultiArrayDimension.h>
//#include <std_msgs/MultiArrayLayout.h>
//#include <std_msgs/UInt16MultiArray.h>

#include "../../matrix/MATRIX_LINK.h"

using namespace std;
using namespace ros;
using namespace cv;

class IR_Color_Convertor// : public nodelet::Nodelet
{    
  public:
    string nodeName = "IR_Color_Convertor";
    string topic_image_thermal_sub = "IR_Color_Convertor/thermal_image";
    string topic_image_color_pub = "IR_Color_Convertor/color_image";
    string topic_image_color_resolution_sub = "IR_Color_Convertor/image_color_resolution";
    
  private:
    string ver_ = "1.0";
    int queue_size = 4;
    #ifdef _ROS_LINK_H_
    ROS_Link *ros_link;
    #endif
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_color_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_thermal;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_color;
    cv_bridge::CvImageConstPtr/*Cvimage_thermalPtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_color;
    image_transport::SubscriberStatusCallback disconnect_cb_image_color;
      
    image_transport::Subscriber it_thermal_sub_;
    image_transport::Publisher it_color_pub_;
    ros::Subscriber color_resolution_sub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void connectCb_image_color(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_color_pub_.getNumSubscribers() > 1) return;
      string status = topic_image_color_pub + " connected !";
      OUT_INFO(nodeName, status);
      sub_init();
      sub_topic_get();
    }
    void disconnectCb_image_color(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_color_pub_.getNumSubscribers() > 0) return;
      string status = topic_image_color_pub + " disconnected !";
      OUT_WARN(nodeName, status);
      sub_shutdown();
    }
    
  private:
    bool camera_init();
    bool camera_shutdown();
    void image_thermal_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_color_publish();
    void color_resolution_callBack(const std_msgs::Float32::ConstPtr& msg);
    
    void msg_image_header_init(boost::shared_ptr<sensor_msgs::Image> msg, string& id, int& height, int& width, string& encoding);
    void image_color_convert(float min_temperature=0, float max_temperature=0, bool auto_flag=true);
    
  private:
    Mat image_thermal;
    Mat image_color;
    float image_color_resolution = 1.0;
    
    bool is_msg_color_image_header_init = false;
    string thermal_image_encoding = "mono16";
    string color_image_encoding = "bgr8";
    

  public:
    IR_Color_Convertor(ros::NodeHandle& nh);
    ~IR_Color_Convertor();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_color_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image_thermal = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_color   = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_color    = boost::bind(&IR_Color_Convertor::connectCb_image_color, this, _1);
      disconnect_cb_image_color = boost::bind(&IR_Color_Convertor::disconnectCb_image_color, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
      #ifdef _ROS_LINK_H_
      ros_link = new ROS_Link(n_, nodeName);
      ros_link -> pub_init();
      ros_link -> sub_init();
      ros_link -> add_cell(color_resolution_sub_, topic_image_color_resolution_sub);
      #endif
    }
    
};

IR_Color_Convertor::IR_Color_Convertor(ros::NodeHandle& nh) : n_(nh)
{    
}

IR_Color_Convertor::~IR_Color_Convertor()
{msg_image_header_init
    #ifdef _ROS_LINK_H_
    if(ros_link != NULL)
        delete ros_link;
    #endif
}

void IR_Color_Convertor::run()
{  
    
    image_color_publish();
}

void IR_Color_Convertor::image_color_convert(float min_temperature=0, float max_temperature=0, bool auto_flag=true)
{
  if(image_thermal.empty()) return;
  try
  {
    //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
    image_thermal.convertTo(image_color, -1, 1.0, -1*min_temperature);
    image_color.convertTo(image_color, -1, 128.0/double(max_temperature - min_temperature), 0);
    //(*image).convertTo(img, CV_8UC1, 0.1, -100);
    image_color.convertTo(image_color, CV_8UC1, 1, 0);
    //GaussianBlur(img, img, Size(3, 3), 1);
    applyColorMap(image_color, image_color, COLORMAP_JET);
    msg_image_color = cv_bridge::CvImage(std_msgs::Header(), color_image_encoding, image_color).toImageMsg();
    //  msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, *image).toImageMsg();
    //sensor_msgs::ImagePtr msg_image;
    //msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, *image).toImageMsg();
  }
  catch (cv_bridge::Exception& e)
  {
    stringstream token;
    string tmp;
    token << e.what();
    token >> tmp;
    string status = "cv_bridge exception: " + tmp;
    OUT_ERROR(nodeName, status);
    return;
  }
}

void IR_Color_Convertor::msg_image_header_init(boost::shared_ptr<sensor_msgs::Image> msg, string& id, int& height, int& width, string& encoding)
{    
  if(is_msg_color_image_header_init) return;
  msg -> header.frame_id = id;
  msg -> height = height;
  msg -> width  = width;
  msg -> encoding = encoding;
  msg -> header.stamp = ros::Time::now();
    
  msg_image_header_attach(msg);
  is_msg_color_image_header_init = true;
}

void IR_Color_Convertor::image_color_publish()
{
  //msg_image_color = cv_bridge::CvImage(std_msgs::Header(), color_image_encoding, image_color).toImageMsg();
  msg_image_header_init(this -> msg_image_color, this -> topic_image_color_pub, height, width, this -> color_image_encoding);
  it_color_pub_.publish(this -> msg_image_color);
}

void IR_Color_Convertor::image_thermal_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::Image_encodings::MONO8);
      //cv_ptr -> image_thermal.copyTo(*image_thermal);
      this -> image_thermal = cv_ptr -> image;
      if(this -> image_thermal.empty())
        return;
      run();
      return;
    }
    catch (cv_bridge::Exception& e)
    {
      stringstream token;
      string tmp;
      token << e.what();
      token >> tmp;
      string status = "cv_bridge exception: " + tmp;
      OUT_ERROR(nodeName, status);
      return;
    }
}

void IR_Color_Convertor::color_resolution_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    stringstream token;
    string tmp;
    token << msg -> data;
    token >> tmp;
    string status = "Resolution ratio is changed to " + tmp + " !";
    OUT_INFO(nodeName, status);
    image_color_resolution = (msg -> data > 1.0)? 1.0 : msg -> data;
    image_color_resolution = (image_color_resolution < 0.0)? 0.0 : image_color_resolution;
}

void IR_Color_Convertor::pub_topic_get()
{
    topic_image_color_pub = it_color_pub_.getTopic();
}

void IR_Color_Convertor::pub_init()
{
    string status = "Publisher " + topic_image_thermal_sub + " initiating !";
    OUT_INFO(nodeName, status);
    it_color_pub_ = it_color_ -> advertise(topic_image_color_pub, queue_size, connect_cb_image_color, disconnect_cb_image_color);
}

void IR_Color_Convertor::pub_shutdown()
{
    string status = "Publisher " + topic_image_color_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    it_color_pub_.shutdown();
}

void IR_Color_Convertor::sub_topic_get()
{   
    topic_image_thermal_sub = it_thermal_sub_.getTopic();
    topic_image_color_resolution_sub = color_resolution_sub_.getTopic();
}

void IR_Color_Convertor::sub_init()
{
    string status = "Subscriber " + topic_image_thermal_sub + " initiating !";
    OUT_INFO(nodeName, status);
    it_thermal_sub_ = it_ -> subscribe(topic_image_thermal_sub.c_str(), queue_size, &IR_Color_Convertor::image_thermal_callBack, this);
    status = "Subscriber " + topic_image_color_resolution_sub + " initiating !";
    OUT_INFO(nodeName, status);
    color_resolution_sub_ = n_.subscribe(topic_image_color_resolution_sub.c_str(), queue_size, &IR_Color_Convertor::color_resolution_callBack, this);
}

void IR_Color_Convertor::sub_shutdown()
{
    string status = "Subscriber " + topic_image_thermal_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    it_thermal_sub_.shutdown();
    status = "Subscriber " + topic_image_color_resolution_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    color_resolution_sub_.shutdown();
}
#endif
