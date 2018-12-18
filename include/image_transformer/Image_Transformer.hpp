#pragma once
#ifndef _IMAGE_TRANSFORMER_H_
#define _IMAGE_TRANSFORMER_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sstream>


#include "../matrix/MATRIX_LINK.h"

using namespace std;
using namespace ros;
using namespace cv;

class Image_Transformer// : public nodelet::Nodelet
{    
  public:
    string nodeName = "Image_Transformer";
    string topic_image_sub = "Image_Transformer/image";
    string topic_image_roi_pub = "Image_Transformer/image_roi";
    string topic_image_roi_ratio_sub = "Image_Transformer/image_roi_ratio";
    string topic_image_roi_width_offset_sub = "Image_Transformer/image_roi_width_offset";
    string topic_image_roi_height_offset_sub = "Image_Transformer/image_roi_height_offset";
    string topic_image_roi_resolution_sub = "Image_Transformer/image_roi_resolution";
    
  private:
    string ver_ = "1.0";
    int queue_size = 4;
	#ifdef _OUT_LINK_H_
	OUT_Link *ros_link;
	#endif
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_roi_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_roi;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_roi;
    image_transport::SubscriberStatusCallback disconnect_cb_image_roi;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_roi_pub_;
    ros::Subscriber roi_ratio_sub_;
    ros::Subscriber roi_width_offset_sub_;
    ros::Subscriber roi_height_offset_sub_;
    ros::Subscriber roi_resolution_sub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void connectCb_image_roi(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_roi_pub_.getNumSubscribers() > 1) return;
      string status = topic_image_roi_pub + " connected !";
      OUT_INFO(nodeName, status);
      sub_init();
	  sub_topic_get();
    }
    void disconnectCb_image_roi(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_roi_pub_.getNumSubscribers() > 0) return;
      string status = topic_image_roi_pub + " disconnected !";
      OUT_WARN(nodeName, status);
      sub_shutdown();
    }
    
  private:
    bool camera_init();
    bool camera_shutdown();
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_roi_publish();
    void roi_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    void roi_width_offset_callBack(const std_msgs::Int32::ConstPtr& msg);
    void roi_height_offset_callBack(const std_msgs::Int32::ConstPtr& msg);
    void roi_resolution_callBack(const std_msgs::Float32::ConstPtr& msg);
    
  private:
    Mat image;
    Mat image_roi;
    float image_roi_ratio = 1.0;
    int image_roi_width_offset = 0;
    int image_roi_height_offset = 0;
    float image_roi_resolution = 1.0;
    
    void image_roi_get();

  public:
    Image_Transformer(ros::NodeHandle& nh);
    ~Image_Transformer();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_roi_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_roi = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_roi    = boost::bind(&Image_Transformer::connectCb_image_roi, this, _1);
      disconnect_cb_image_roi = boost::bind(&Image_Transformer::disconnectCb_image_roi, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
	  #ifdef _OUT_LINK_H_
	  ros_link = new OUT_Link(n_, nodeName);
	  ros_link -> pub_init();
	  ros_link -> sub_init();
	  ros_link -> add_cell(roi_ratio_sub_, topic_image_roi_ratio_sub);
	  ros_link -> add_cell(roi_resolution_sub_, topic_image_roi_resolution_sub);
	  #endif
    }
    
};

Image_Transformer::Image_Transformer(ros::NodeHandle& nh) : n_(nh)
{    
}

Image_Transformer::~Image_Transformer()
{
	#ifdef _OUT_LINK_H_
	if(ros_link != NULL)
		delete ros_link;
	#endif
}

void Image_Transformer::run()
{  
    image_roi_get();
    image_roi_publish();
}

void Image_Transformer::image_roi_get()
{
    int center_col = (image).cols/2;
    int center_row = (image).rows/2;
    int roi_width =  (image).cols*image_roi_ratio;
    int roi_height = (image).cols*image_roi_ratio;
    int col_s = ((center_col - roi_width/2  + image_roi_width_offset) < 0)? 0 : center_col - roi_width/2  + image_roi_width_offset;
    int col_e = ((center_col + roi_width/2  + image_roi_width_offset) > (image).cols)? (image).cols : center_col + roi_width/2  + image_roi_width_offset;
    int row_s = ((center_row - roi_height/2 + image_roi_height_offset)< 0)? 0 : center_row - roi_height/2 + image_roi_height_offset;
    int row_e = ((center_row + roi_height/2 + image_roi_height_offset)> (image).rows)? (image).rows : center_row + roi_height/2 + image_roi_height_offset;
    image_roi = (image)(Range(row_s, row_e), Range(col_s, col_e));
    resize(image_roi, image_roi, Size(int(image.cols*image_roi_resolution), int(image.rows*image_roi_resolution)));
}

void Image_Transformer::image_roi_publish()
{
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_roi).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_roi_pub_.publish(msg_image);
}

void Image_Transformer::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      //cv_ptr -> image.copyTo(*image);
      this -> image = cv_ptr -> image;
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
    //sensor_msgs::Image::Ptr cv_ptr = cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg();
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //this -> image = cv_ptr -> image;
}

void Image_Transformer::roi_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    stringstream token;
    string tmp;
    token << msg -> data;
    token >> tmp;
    string status = "ROI ratio is changed to " + tmp + " !";
    OUT_INFO(nodeName, status);
    image_roi_ratio = (msg -> data > 1.0)? 1.0 : msg -> data;
}

void Image_Transformer::roi_width_offset_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string tmp;
    token << msg -> data;
    token >> tmp;
    string status = "ROI width offset is changed to " + tmp + " !";
    OUT_INFO(nodeName, status);
    image_roi_width_offset = msg -> data;
}

void Image_Transformer::roi_height_offset_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string tmp;
    token << msg -> data;
    token >> tmp;
    string status = "ROI height offset is changed to " + tmp + " !";
    OUT_INFO(nodeName, status);
    image_roi_height_offset = msg -> data;
}

void Image_Transformer::roi_resolution_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    stringstream token;
    string tmp;
    token << msg -> data;
    token >> tmp;
    string status = "Resolution ratio is changed to " + tmp + " !";
    OUT_INFO(nodeName, status);
    image_roi_resolution = (msg -> data > 1.0)? 1.0 : msg -> data;
    image_roi_resolution = (image_roi_resolution < 0.0)? 0.0 : image_roi_resolution;
}

void Image_Transformer::pub_topic_get()
{
    topic_image_roi_pub = it_roi_pub_.getTopic();
}

void Image_Transformer::pub_init()
{
    string status = "Publisher " + topic_image_sub + " initiating !";
    OUT_INFO(nodeName, status);
    it_roi_pub_ = it_roi_ -> advertise(topic_image_roi_pub, queue_size, connect_cb_image_roi, disconnect_cb_image_roi);
}

void Image_Transformer::pub_shutdown()
{
    string status = "Publisher " + topic_image_roi_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    it_roi_pub_.shutdown();
}

void Image_Transformer::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_image_roi_ratio_sub = roi_ratio_sub_.getTopic();
    topic_image_roi_width_offset_sub = roi_width_offset_sub_.getTopic();
    topic_image_roi_height_offset_sub = roi_height_offset_sub_.getTopic();
    topic_image_roi_resolution_sub = roi_resolution_sub_.getTopic();
}

void Image_Transformer::sub_init()
{
    string status = "Subscriber " + topic_image_sub + " initiating !";
    OUT_INFO(nodeName, status);
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &Image_Transformer::image_callBack, this);
    status = "Subscriber " + topic_image_roi_ratio_sub + " initiating !";
    OUT_INFO(nodeName, status);
    roi_ratio_sub_ = n_.subscribe(topic_image_roi_ratio_sub.c_str(), queue_size, &Image_Transformer::roi_ratio_callBack, this);
    status = "Subscriber " + topic_image_roi_width_offset_sub + " initiating !";
    OUT_INFO(nodeName, status);
    roi_width_offset_sub_ = n_.subscribe(topic_image_roi_width_offset_sub.c_str(), queue_size, &Image_Transformer::roi_width_offset_callBack, this);
    status = "Subscriber " + topic_image_roi_height_offset_sub + " initiating !";
    OUT_INFO(nodeName, status);
    roi_height_offset_sub_ = n_.subscribe(topic_image_roi_height_offset_sub.c_str(), queue_size, &Image_Transformer::roi_height_offset_callBack, this);
    status = "Subscriber " + topic_image_roi_resolution_sub + " initiating !";
    OUT_INFO(nodeName, status);
    roi_resolution_sub_ = n_.subscribe(topic_image_roi_resolution_sub.c_str(), queue_size, &Image_Transformer::roi_resolution_callBack, this);
}

void Image_Transformer::sub_shutdown()
{
    string status = "Subscriber " + topic_image_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    it_sub_.shutdown();
    status = "Subscriber " + topic_image_roi_ratio_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    roi_ratio_sub_.shutdown();
    status = "Subscriber " + topic_image_roi_width_offset_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    roi_width_offset_sub_.shutdown();
    status = "Subscriber " + topic_image_roi_height_offset_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    roi_height_offset_sub_.shutdown();
    status = "Subscriber " + topic_image_roi_resolution_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    roi_resolution_sub_.shutdown();
}
#endif
