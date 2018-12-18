#pragma once
#ifndef _FACE_DETECTOR_H_
#define _FACE_DETECTOR_H_

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
#include <sensor_msgs/RegionOfInterest.h>
/*
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>
*/

#include "Face_Detector_Cascade.h"
#include "../../matrix/MATRIX_LINK.h"

using namespace std;
using namespace ros;
using namespace cv;

#define IMAGE           	0x01
#define IMAGE_FACE      	0x02
#define IMAGE_FULLBODY  	0x04
#define IMAGE_UPPERBODY  	0x08

class Face_Detector
{    
  public:
    string nodeName = "Face_Detector";
    string topic_image_sub = "Face_Detector/image";
    string topic_image_detect_pub = "Face_Detector/image_detect";
    string topic_image_face_pub = "Face_Detector/image_face";
    string topic_image_fullbody_pub = "Face_Detector/image_fullbody";
    string topic_image_upperbody_pub = "Face_Detector/image_upperbody";
    //string topic_auto_adjust_sub = "Face_Detector/auto_adjust";
    
  private:
    string ver_ = "1.1";
    int queue_size = 4;
    #ifdef _ROS_LINK_H_
    ROS_Link *ros_link;
    #endif
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_detect_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_detect;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_detect;
    image_transport::SubscriberStatusCallback disconnect_cb_image_detect;
    ros::SubscriberStatusCallback connect_cb_image_face;
    ros::SubscriberStatusCallback disconnect_cb_image_face;
    ros::SubscriberStatusCallback connect_cb_image_fullbody;
    ros::SubscriberStatusCallback disconnect_cb_image_fullbody;
    ros::SubscriberStatusCallback connect_cb_image_upperbody;
    ros::SubscriberStatusCallback disconnect_cb_image_upperbody;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_detect_pub_;
    ros::Publisher image_face_pub_;
    ros::Publisher image_fullbody_pub_;
    ros::Publisher image_upperbody_pub_;
    //ros::Subscriber auto_adjust_sub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    /*void sub_manual_topic_get();
    void sub_manual_init();
    void sub_manual_shutdown();*/
    void connectCb_image_detect(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_detect_pub_.getNumSubscribers() > 1) return;
      string status = topic_image_detect_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_fd == 0)
      {
        sub_init();
      }
      status_fd = status_fd | IMAGE;
    }
    void disconnectCb_image_detect(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_detect_pub_.getNumSubscribers() > 0) return;
      string status = topic_image_detect_pub + " disconnected !";
      OUT_WARN(nodeName, status);      
      status_fd = status_fd & (0xFF - IMAGE);
      if(status_fd == 0)
      {
        sub_shutdown();
      }
    }
    void connectCb_image_face(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_face_pub_.getNumSubscribers() > 1) return;
      string status = topic_image_face_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_fd == 0)
      {
        sub_init();
      }
      status_fd = status_fd | IMAGE_FACE;
    }
    void disconnectCb_image_face(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_face_pub_.getNumSubscribers() > 0) return;
      string status = topic_image_face_pub + " disconnected !";
      OUT_WARN(nodeName, status);      
      status_fd = status_fd & (0xFF - IMAGE_FACE);
      if(status_fd == 0)
      {
        sub_shutdown();
      }
    }
    void connectCb_image_fullbody(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_fullbody_pub_.getNumSubscribers() > 1) return;
      string status = topic_image_fullbody_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_fd == 0)
      {
        sub_init();
      }
      status_fd = status_fd | IMAGE_FULLBODY;
    }
    void disconnectCb_image_fullbody(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_fullbody_pub_.getNumSubscribers() > 0) return;
      string status = topic_image_fullbody_pub + " disconnected !";
      OUT_WARN(nodeName, status);      
      status_fd = status_fd & (0xFF - IMAGE_FULLBODY);
      if(status_fd == 0)
      {
        sub_shutdown();
      }
    }
    void connectCb_image_upperbody(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_fullbody_pub_.getNumSubscribers() > 1) return;
      string status = topic_image_upperbody_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_fd == 0)
      {
        sub_init();
      }
      status_fd = status_fd | IMAGE_UPPERBODY;
    }
    void disconnectCb_image_upperbody(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_fullbody_pub_.getNumSubscribers() > 0) return;
      string status = topic_image_upperbody_pub + " disconnected !";
      OUT_WARN(nodeName, status);      
      status_fd = status_fd & (0xFF - IMAGE_UPPERBODY);
      if(status_fd == 0)
      {
        sub_shutdown();
      }
    }
    
  private:
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_detect_publish();
    void image_face_publish();
    void image_fullbody_publish();
    void image_upperbody_publish();
	
	void image_face_detect();
	void image_fullbody_detect();
	void image_upperbody_detect();
    
  private:
    Face_Detector_Cascade *fdc;
    Mat image_raw;
    Mat image_detect;
    //vector<Rect> image_face;
    //vector<Rect> image_fullbody;
    int status_fd = 0;

  public:
    Face_Detector(ros::NodeHandle& nh);
    ~Face_Detector();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_detect_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_detect = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_detect    = boost::bind(&Face_Detector::connectCb_image_detect, this, _1);
      disconnect_cb_image_detect = boost::bind(&Face_Detector::disconnectCb_image_detect, this, _1);
      connect_cb_image_face    = boost::bind(&Face_Detector::connectCb_image_face, this, _1);
      disconnect_cb_image_face = boost::bind(&Face_Detector::disconnectCb_image_face, this, _1);
      connect_cb_image_fullbody    = boost::bind(&Face_Detector::connectCb_image_fullbody, this, _1);
      disconnect_cb_image_fullbody = boost::bind(&Face_Detector::disconnectCb_image_fullbody, this, _1);
      connect_cb_image_upperbody    = boost::bind(&Face_Detector::connectCb_image_upperbody, this, _1);
      disconnect_cb_image_upperbody = boost::bind(&Face_Detector::disconnectCb_image_upperbody, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
      #ifdef _ROS_LINK_H_
      ros_link = new ROS_Link(n_, nodeName);
      ros_link -> pub_init();
      ros_link -> sub_init();
      //ros_link -> add_cell(mser_min_width_sub_, topic_image_sub);
      #endif
    }
    
};

Face_Detector::Face_Detector(ros::NodeHandle& nh) : n_(nh)
{    
    fdc = new Face_Detector_Cascade;
}

Face_Detector::~Face_Detector()
{
    delete fdc;
}

void Face_Detector::run()
{  
	image_face_detect();
	image_fullbody_detect();
	image_upperbody_detect();
	
    image_detect_publish();
    image_face_publish();
    image_fullbody_publish();
    image_upperbody_publish();
}

void Face_Detector::image_face_detect()
{
  if((status_fd & IMAGE_FACE) == 0) return;
  fdc -> face_detect(this -> image_raw);
  //this -> image_face = fdc -> face;

  for(int i = 0; i < fdc -> face.size(); i++)
  {
     rectangle( this -> image_raw, fdc -> face[i], cv::Scalar(0, 0, 255), 1, 8, 0 );
  }
  return;
}

void Face_Detector::image_fullbody_detect()
{
  if((status_fd & IMAGE_FULLBODY) == 0) return;
  fdc -> fullbody_detect(this -> image_raw);
  //this -> image_fullbody = fdc -> fullbody;
	
  for(int i = 0; i < fdc -> fullbody.size(); i++)
  {
     rectangle( this -> image_raw, fdc -> fullbody[i], cv::Scalar(0, 255, 0), 1, 8, 0 );
  }
  return;
}

void Face_Detector::image_upperbody_detect()
{
  if((status_fd & IMAGE_UPPERBODY) == 0) return;
  fdc -> upperbody_detect(this -> image_raw);
  //this -> image_fullbody = fdc -> fullbody;
	
  for(int i = 0; i < fdc -> upperbody.size(); i++)
  {
     rectangle( this -> image_raw, fdc -> upperbody[i], cv::Scalar(0, 255, 0), 1, 8, 0 );
  }
  return;
}

void Face_Detector::image_detect_publish()
{
  if((status_fd & IMAGE) == 0) return;
  this -> image_detect = this -> image_raw;
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this -> image_detect).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_detect_pub_.publish(msg_image);
}

void Face_Detector::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      //cv_ptr -> image.copyTo(image_thermal);
      this -> image_raw = cv_ptr -> image;
      run();
      return;
    }
    catch (cv_bridge::Exception& e)
    {
      stringstream token;
      string str;
      token << e.what();
      token >> str;
      string status = "cv_bridge exception: " + str;
      OUT_ERROR(nodeName, status);
      return;
    }
}

void Face_Detector::image_face_publish()
{
    if((status_fd & IMAGE_FACE) == 0) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < fdc -> face.size(); i++)
    {
      msg.x_offset = fdc -> face[i].x + fdc -> face[i].width/2;      
      msg.y_offset = fdc -> face[i].y + fdc -> face[i].height/2;      
      msg.width    = fdc -> face[i].width;      
      msg.height   = fdc -> face[i].height;     
      image_face_pub_.publish(msg);
    }
}

void Face_Detector::image_fullbody_publish()
{
    if((status_fd & IMAGE_FULLBODY) == 0) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < fdc -> fullbody.size(); i++)
    {
      msg.x_offset = fdc -> fullbody[i].x + fdc -> fullbody[i].width/2;      
      msg.y_offset = fdc -> fullbody[i].y + fdc -> fullbody[i].height/2;      
      msg.width    = fdc -> fullbody[i].width;      
      msg.height   = fdc -> fullbody[i].height;     
      image_fullbody_pub_.publish(msg);
    }
}

void Face_Detector::image_upperbody_publish()
{
    if((status_fd & IMAGE_UPPERBODY) == 0) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < fdc -> upperbody.size(); i++)
    {
      msg.x_offset = fdc -> upperbody[i].x + fdc -> upperbody[i].width/2;      
      msg.y_offset = fdc -> upperbody[i].y + fdc -> upperbody[i].height/2;      
      msg.width    = fdc -> upperbody[i].width;      
      msg.height   = fdc -> upperbody[i].height;     
      image_upperbody_pub_.publish(msg);
    }
}
    
void Face_Detector::pub_topic_get()
{
    topic_image_detect_pub = it_detect_pub_.getTopic();
    topic_image_face_pub = image_face_pub_.getTopic();
    topic_image_fullbody_pub = image_fullbody_pub_.getTopic();
    topic_image_upperbody_pub = image_upperbody_pub_.getTopic();
}

void Face_Detector::pub_init()
{
    string status = "Publisher " + topic_image_detect_pub + " initiating !";
    OUT_INFO(nodeName, status);
    it_detect_pub_ = it_detect_ -> advertise(topic_image_detect_pub, queue_size, connect_cb_image_detect, disconnect_cb_image_detect);

    status = "Publisher " + topic_image_face_pub + " initiating !";
    OUT_INFO(nodeName, status);
    image_face_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_image_face_pub.c_str(), queue_size, connect_cb_image_face, disconnect_cb_image_face);

    status = "Publisher " + topic_image_fullbody_pub + " initiating !";
    OUT_INFO(nodeName, status);
    image_fullbody_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_image_fullbody_pub.c_str(), queue_size, connect_cb_image_fullbody, disconnect_cb_image_fullbody);

    status = "Publisher " + topic_image_upperbody_pub + " initiating !";
    OUT_INFO(nodeName, status);
    image_upperbody_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_image_upperbody_pub.c_str(), queue_size, connect_cb_image_upperbody, disconnect_cb_image_upperbody);
}

void Face_Detector::pub_shutdown()
{
	string status = "Publisher " + topic_image_detect_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    it_detect_pub_.shutdown();
	
	status = "Publisher " + topic_image_face_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    image_face_pub_.shutdown();
	
	status = "Publisher " + topic_image_fullbody_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    image_fullbody_pub_.shutdown();
	
	status = "Publisher " + topic_image_upperbody_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    image_upperbody_pub_.shutdown();
}

void Face_Detector::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
}

void Face_Detector::sub_init()
{
    string status = "Subscriber " + topic_image_sub + " initiating !";
    OUT_INFO(nodeName, status);
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &Face_Detector::image_callBack, this);
}

void Face_Detector::sub_shutdown()
{
    string status = "Subscriber " + topic_image_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    it_sub_.shutdown();
}

#endif

