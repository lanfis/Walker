#pragma once
#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <sstream>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>

#include "Camera_Driver.h"
#include "../../matrix/MATRIX_LINK.h"

using namespace std;
using namespace ros;
using namespace cv;

class Camera// : public nodelet::Nodelet
{    
  public:
    string nodeName = "Camera";
    string topic_image_pub = "Camera/image";
    string topic_camera_info_pub = "Camera/camera_info";
    string topic_device_id_sub = "Camera/device_id";
    string topic_resolution_ratio_sub = "Camera/resolution_ratio";
    
  private:
    string ver_ = "2.2";//depend on matrix_link
    int queue_size = 4;
	ROS_Link *ros_link;
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    image_transport::SubscriberStatusCallback connect_cb_image;
    image_transport::SubscriberStatusCallback disconnect_cb_image;
    sensor_msgs::CameraInfo camera_info_;
      
    image_transport::Publisher it_pub_;
    ros::Publisher camera_info_pub_;
    ros::Subscriber device_id_sub_;
    ros::Subscriber resolution_ratio_sub_;

    string image_encoding = "bgr8";
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void connectCb_image(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_pub_.getNumSubscribers() > 1) return;
	  string status = topic_image_pub + " connected !";
  	  OUT_INFO(nodeName.c_str(), status);
	  status = "Camera initializing ...";
  	  OUT_INFO(nodeName.c_str(), status);
      flag_activation_camera = true;
      camera_init();
      //sub_init();
    }
    void disconnectCb_image(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_pub_.getNumSubscribers() > 0) return;
	  string status = topic_image_pub + " disconnected !";
  	  OUT_WARN(nodeName.c_str(), status);
	  status = "Camera closing ...";
  	  OUT_WARN(nodeName.c_str(), status);
      flag_activation_camera = false;
      ros::Duration(queue_size*0.5).sleep();
      if(!flag_activation_camera)
        camera_shutdown();
      //sub_shutdown();
    }
    
  private:
    void msg_image_header_init(boost::shared_ptr<sensor_msgs::Image> msg);
    void msg_image_header_attach(boost::shared_ptr<sensor_msgs::Image> msg);
    bool camera_init();
    bool camera_shutdown();
    void image_publish();
    void camera_info_publish();
    void device_id_callBack(const std_msgs::Int32::ConstPtr& msg);
    void resolution_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    
  private:
    int device_id;
    Mat *image;
    bool flag_activation_camera;
    bool is_msg_image_header_init = false;
    
    Camera_Driver *camera = NULL;
    void image_roi_get();
    void create_Camera_Info();
    void create_Camera_Info(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const;
      

  public:
    Camera(ros::NodeHandle& nh);
    ~Camera();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      
      connect_cb_image    = boost::bind(&Camera::connectCb_image, this, _1);
      disconnect_cb_image = boost::bind(&Camera::disconnectCb_image, this, _1);
      
      pub_init();
      pub_topic_get();
      sub_init();
      sub_topic_get();

	  ros_link = new ROS_Link(n_, nodeName);
	  ros_link -> pub_init();
	  ros_link -> sub_init();
	  ros_link -> add_cell(resolution_ratio_sub_, topic_resolution_ratio_sub);
    }
    
};

Camera::Camera(ros::NodeHandle& nh) : n_(nh)
{    
}

Camera::~Camera()
{
    camera_shutdown();
}

void Camera::run()
{    
    if(!flag_activation_camera) return;
    if(camera == NULL)  return;
    if(camera -> run()) 
    {
        msg_image_header_init(this -> msg_image);
        image_publish();
    }
    camera_info_publish();
}

void Camera::image_publish()
{
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, *image).toImageMsg();
  msg_image_header_attach(msg_image);
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, *image).toImageMsg();
  it_pub_.publish(msg_image);
}

void Camera::msg_image_header_init(boost::shared_ptr<sensor_msgs::Image> msg)
{    
  if(is_msg_image_header_init) return;
  msg -> header.frame_id = this -> topic_image_pub;
  msg -> height = this -> camera -> height;
  msg -> width  = this -> camera -> width;
  msg -> encoding = this -> image_encoding;
  //msg -> step = ;
  is_msg_image_header_init = true;
}

void Camera::msg_image_header_attach(boost::shared_ptr<sensor_msgs::Image> msg)
{
  msg -> header.stamp = ros::Time::now();
}

bool Camera::camera_init()
{
    if(camera == NULL)
      camera = new Camera_Driver;
    if(camera -> is_open())
    {
      this -> image = &camera -> image;
	  string status = "Camera initializing ...ok !";
  	  OUT_INFO(nodeName.c_str(), status);
      return true;
    }
    else
    {
	  string status = "Camera initializing ...fail !";
  	  OUT_WARN(nodeName.c_str(), status);
      return false;
    }
}

bool Camera::camera_shutdown()
{
    if(camera == NULL) return true;
    this -> image = NULL;
    delete camera;
    camera = NULL;
	string status = "Camera closing ...ok !";
  	OUT_WARN(nodeName.c_str(), status);
    return true;
}

void Camera::camera_info_publish()
{
  create_Camera_Info();
  camera_info_pub_.publish(camera_info_);
}

void Camera::create_Camera_Info()
{
  cv::Size size;
  size.width = camera -> width;
  size.height = camera -> height;

  create_Camera_Info(size, camera -> cameraMatrix, camera -> distortion, cv::Mat::eye(3, 3, CV_64F), camera -> projection, camera_info_);
}

void Camera::create_Camera_Info(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const
{
  cameraInfo.height = size.height;
  cameraInfo.width = size.width;

  const double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    cameraInfo.K[i] = *itC;
  }

  const double *itR = rotation.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itR)
  {
    cameraInfo.R[i] = *itR;
  }

  const double *itP = projection.ptr<double>(0, 0);
  for(size_t i = 0; i < 12; ++i, ++itP)
  {
    cameraInfo.P[i] = *itP;
  }

  cameraInfo.distortion_model = "plumb_bob";
  cameraInfo.D.resize(distortion.cols);
  const double *itD = distortion.ptr<double>(0, 0);
  for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
  {
    cameraInfo.D[i] = *itD;
  }
}


void Camera::device_id_callBack(const std_msgs::Int32::ConstPtr& msg)
{
	stringstream token;
	string tmp;
	token << msg -> data; token >> tmp;
	string status = "Camera ID is changed to " + tmp + " !";
  	OUT_WARN(nodeName.c_str(), status);
    camera -> device_id = msg -> data;
}

void Camera::resolution_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
	stringstream token;
	string tmp;
	token << msg -> data; token >> tmp;
	string status = "Resolution ratio is changed to " + tmp + " !";
  	OUT_WARN(nodeName.c_str(), status);
    camera -> resolution_ratio = msg -> data;
}

void Camera::pub_topic_get()
{
    topic_image_pub = it_pub_.getTopic();
    topic_camera_info_pub = camera_info_pub_.getTopic();
}

void Camera::pub_init()
{
	string status = "Publisher " + topic_image_pub + " initiating !";
  	OUT_INFO(nodeName.c_str(), status);
    it_pub_ = it_ -> advertise(topic_image_pub, queue_size, connect_cb_image, disconnect_cb_image);
	status = "Publisher " + topic_camera_info_pub + " initiating !";
  	OUT_INFO(nodeName.c_str(), status);
    camera_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>(topic_camera_info_pub.c_str(), queue_size);
}

void Camera::pub_shutdown()
{
	string status = "Publisher " + topic_image_pub + " shuting down !";
  	OUT_WARN(nodeName.c_str(), status);
    it_pub_.shutdown();
	status = "Publisher " + topic_camera_info_pub + " shuting down !";
  	OUT_WARN(nodeName.c_str(), status);
    camera_info_pub_.shutdown();
}

void Camera::sub_topic_get()
{   
    topic_device_id_sub = device_id_sub_.getTopic();
    topic_resolution_ratio_sub = resolution_ratio_sub_.getTopic();
}

void Camera::sub_init()
{
	string status = "Subscriber " + topic_device_id_sub + " initiating !";
  	OUT_INFO(nodeName.c_str(), status);
    device_id_sub_ = n_.subscribe(topic_device_id_sub.c_str(), queue_size, &Camera::device_id_callBack, this);
	status = "Subscriber " + topic_resolution_ratio_sub + " initiating !";
  	OUT_INFO(nodeName.c_str(), status);
    resolution_ratio_sub_ = n_.subscribe(topic_resolution_ratio_sub.c_str(), queue_size, &Camera::resolution_ratio_callBack, this);
}

void Camera::sub_shutdown()
{
	string status = "Subscriber " + topic_device_id_sub + " shuting down !";
  	OUT_WARN(nodeName.c_str(), status);
    device_id_sub_.shutdown();
	status = "Subscriber " + topic_resolution_ratio_sub + " shuting down !";
  	OUT_WARN(nodeName.c_str(), status);
    resolution_ratio_sub_.shutdown();
}
#endif
