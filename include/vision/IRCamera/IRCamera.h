#pragma once
#ifndef _IRCAMERA_H_
#define _IRCAMERA_H_

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
/*
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>
*/
#include "IRCamera_Driver.h"
#include "../../matrix/MATRIX_LINK.h"

using namespace std;
using namespace ros;
using namespace cv;

//namespace ircamera_nodelet {
	
class IRCamera// : public nodelet::Nodelet
{    
  public:
    string nodeName = "IRCamera";
    string topic_image_pub = "IRCamera/thermal_image";
    string topic_color_image_pub = "IRCamera/color_image";
    string topic_camera_info_pub = "IRCamera/camera_info";
    string topic_device_id_sub = "IRCamera/device_id";
    string topic_resolution_ratio_sub = "IRCamera/resolution_ratio";
    
    //string param_xml_file = "/home/adel/Dropbox/Github/Walker/include/vision/IRCamera/ir_param.xml";
    string param_xml_file = "/home/pi/Walker/include/vision/IRCamera/ir_param.xml";
    
  private:
    string ver_ = "1.1";//depend on matrix_link
    int queue_size = 4;
    ROS_Link *ros_link;
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_color_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_color_image;
    image_transport::SubscriberStatusCallback connect_cb_image;
    image_transport::SubscriberStatusCallback disconnect_cb_image;
    image_transport::SubscriberStatusCallback connect_cb_color_image;
    image_transport::SubscriberStatusCallback disconnect_cb_color_image;
    sensor_msgs::CameraInfo camera_info_;
      
    image_transport::Publisher it_pub_;
    image_transport::Publisher it_color_pub_;
    ros::Publisher camera_info_pub_;
    ros::Subscriber device_id_sub_;
    ros::Subscriber resolution_ratio_sub_;

    string image_encoding = "mono16";
    string color_image_encoding = "bgr8";
    
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
      status = "IRCamera initializing ...";
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
      status = "IRCamera closing ...";
      OUT_WARN(nodeName.c_str(), status);
      flag_activation_camera = false;
      ros::Duration(queue_size*0.5).sleep();
      if(!flag_activation_camera)
        camera_shutdown();
      //sub_shutdown();
    }
    /*
    void connectCb_color_image(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_color_pub_.getNumSubscribers() > 1) return;
      string status = topic_color_image_pub + " connected !";
      OUT_INFO(nodeName.c_str(), status);
      status = "IRCamera initializing ...";
      OUT_INFO(nodeName.c_str(), status);
      flag_activation_camera = true;
      camera_init();
      //sub_init();
    }
    void disconnectCb_color_image(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_color_pub_.getNumSubscribers() > 0) return;
      string status = topic_color_image_pub + " disconnected !";
      OUT_WARN(nodeName.c_str(), status);
      status = "IRCamera closing ...";
      OUT_WARN(nodeName.c_str(), status);
      flag_activation_camera = false;
      ros::Duration(queue_size*0.5).sleep();
      if(!flag_activation_camera)
        camera_shutdown();
      //sub_shutdown();
    }
    */
  private:
    void msg_image_header_init(boost::shared_ptr<sensor_msgs::Image> msg);
    void msg_image_header_attach(boost::shared_ptr<sensor_msgs::Image> msg);
    //void msg_color_image_header_init(boost::shared_ptr<sensor_msgs::Image> msg);
    //void msg_color_image_header_attach(boost::shared_ptr<sensor_msgs::Image> msg);
    bool camera_init();
    bool camera_shutdown();
    void image_publish();
    void color_image_publish();
    void camera_info_publish();
    void device_id_callBack(const std_msgs::Int32::ConstPtr& msg);
    void resolution_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    
  private:
    int device_id;
    //Mat *image;
    //Mat color_image;
    unsigned short* thermal_image = NULL;
    unsigned short* visible_image = NULL;
    bool flag_activation_camera;
    bool is_msg_image_header_init = false;
    //bool is_msg_color_image_header_init = false;
    
    IRCamera_Driver *camera = NULL;
    void create_IRCamera_Info();
    void create_IRCamera_Info(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const;
      

  public:
    IRCamera(ros::NodeHandle& nh);
    ~IRCamera();
    void run();
    void camera_run(const ros::TimerEvent& event);
    
  public:
    virtual void init()
    {
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      //it_color_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      //msg_color_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      
      connect_cb_image    = boost::bind(&IRCamera::connectCb_image, this, _1);
      disconnect_cb_image = boost::bind(&IRCamera::disconnectCb_image, this, _1);
      //connect_cb_color_image    = boost::bind(&IRCamera::connectCb_color_image, this, _1);
      //disconnect_cb_color_image = boost::bind(&IRCamera::disconnectCb_color_image, this, _1);
      
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

IRCamera::IRCamera(ros::NodeHandle& nh) : n_(nh)
{    
}

IRCamera::~IRCamera()
{
    camera_shutdown();
}

void IRCamera::run()
{    
    if(!flag_activation_camera) return;
    if(camera == NULL)  return;
    ros::Timer timer = n_.createTimer(ros::Duration(camera -> time_delay), &IRCamera::camera_run, this);
    camera_info_publish();
}
/*
void IRCamera::color_image_publish()
{
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  Mat img;
  (*image).convertTo(img, -1, 1.0, -1*(camera -> min_temperature_val));
  img.convertTo(img, -1, 128.0/double(camera -> max_temperature_val - camera -> min_temperature_val), 0);
  //(*image).convertTo(img, CV_8UC1, 0.1, -100);
  img.convertTo(img, CV_8UC1, 1, 0);
  //GaussianBlur(img, img, Size(3, 3), 1);
  applyColorMap(img, img, COLORMAP_JET);
  msg_color_image = cv_bridge::CvImage(std_msgs::Header(), color_image_encoding, img).toImageMsg();
//  msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, *image).toImageMsg();
  msg_color_image -> header.stamp = ros::Time::now();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, *image).toImageMsg();
  it_color_pub_.publish(msg_color_image);
}

void IRCamera::msg_color_image_header_init(boost::shared_ptr<sensor_msgs::Image> msg)
{    
  if(is_msg_color_image_header_init) return;
  msg -> header.frame_id = this -> topic_color_image_pub;
  msg -> height = this -> camera -> height;
  msg -> width  = this -> camera -> width;
  msg -> encoding = this -> color_image_encoding;
  //msg -> step = ;
  msg_color_image_header_attach(msg);
  is_msg_color_image_header_init = true;
}

void IRCamera::msg_color_image_header_attach(boost::shared_ptr<sensor_msgs::Image> msg)
{
  msg -> header.stamp = ros::Time::now();
}
*/
void IRCamera::image_publish()
{
  /*
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  Mat img;
  (*image).convertTo(img, CV_16UC1, 1, -100);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, img).toImageMsg();
//  msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, *image).toImageMsg();
  msg_image -> header.stamp = ros::Time::now();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), image_encoding, *image).toImageMsg();
  it_pub_.publish(msg_image);
  */
  
  memcpy(&msg_image -> data[0], camera -> thermal_image, camera -> width * camera -> height * sizeof(*camera -> thermal_image));
  
  it_pub_.publish(msg_image);
}

void IRCamera::msg_image_header_init(boost::shared_ptr<sensor_msgs::Image> msg)
{    
  if(is_msg_image_header_init) return;
  msg -> header.frame_id = this -> topic_image_pub;
  msg -> height = this -> camera -> height;
  msg -> width  = this -> camera -> width;
  msg -> encoding = this -> image_encoding;
  msg -> step = msg -> width * 2;
  msg -> data.resize(msg -> height * msg -> step);
  
  msg_image_header_attach(msg);
  is_msg_image_header_init = true;
}

void IRCamera::msg_image_header_attach(boost::shared_ptr<sensor_msgs::Image> msg)
{
  msg -> header.stamp = ros::Time::now();
}

void IRCamera::camera_run(const ros::TimerEvent& event)
{
  if(camera == NULL)  return;
  if(camera -> run() == NULL) return;
  msg_image_header_init(this -> msg_image);
  image_publish();
  //msg_color_image_header_init(this -> msg_color_image);
  //color_image_publish();
}

bool IRCamera::camera_init()
{
    if(camera == NULL)
    {
      camera = new IRCamera_Driver(param_xml_file);
      //camera -> init(param_xml_file);
    }
    if(camera -> is_init())
    {
      this -> thermal_image = camera -> thermal_image;
      //this -> visible_image = camera -> visible_image;
      string status = "IRCamera initializing ...ok !";
      OUT_INFO(nodeName.c_str(), status);
      return true;
    }
    else
    {
      string status;
      if(camera -> is_open())
        status = "IRCamera connecting ...ok !";
      else
        status = "IRCamera initializing ...fail !";
      OUT_WARN(nodeName.c_str(), status);
      return false;
    }
}

bool IRCamera::camera_shutdown()
{
    if(camera == NULL) return true;
    thermal_image = NULL;
    visible_image = NULL;
    delete camera;
    camera = NULL;
    string status = "IRCamera closing ...ok !";
    OUT_WARN(nodeName.c_str(), status);
    return true;
}

void IRCamera::camera_info_publish()
{
  create_IRCamera_Info();
  camera_info_pub_.publish(camera_info_);
}

void IRCamera::create_IRCamera_Info()
{
  cv::Size size;
  size.width = camera -> width;
  size.height = camera -> height;

  //create_IRCamera_Info(size, camera -> cameraMatrix, camera -> distortion, cv::Mat::eye(3, 3, CV_64F), camera -> projection, camera_info_);
}

void IRCamera::create_IRCamera_Info(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const
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


void IRCamera::device_id_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string tmp;
    token << msg -> data; token >> tmp;
    string status = "IRCamera ID is changed to " + tmp + " !";
    OUT_WARN(nodeName.c_str(), status);
    camera -> device_id = msg -> data;
}

void IRCamera::resolution_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    stringstream token;
    string tmp;
    token << msg -> data; token >> tmp;
    string status = "Resolution ratio is changed to " + tmp + " !";
    OUT_WARN(nodeName.c_str(), status);
    //camera -> resolution_ratio = msg -> data;
}

void IRCamera::pub_topic_get()
{
    topic_image_pub = it_pub_.getTopic();
    topic_color_image_pub = it_color_pub_.getTopic();
    topic_camera_info_pub = camera_info_pub_.getTopic();
}

void IRCamera::pub_init()
{
    string status = "Publisher " + topic_image_pub + " initiating !";
    OUT_INFO(nodeName.c_str(), status);
    it_pub_ = it_ -> advertise(topic_image_pub, queue_size, connect_cb_image, disconnect_cb_image);
    //status = "Publisher " + topic_color_image_pub + " initiating !";
    //OUT_INFO(nodeName.c_str(), status);
    //it_color_pub_ = it_color_ -> advertise(topic_color_image_pub, queue_size, connect_cb_color_image, disconnect_cb_color_image);
    status = "Publisher " + topic_camera_info_pub + " initiating !";
    OUT_INFO(nodeName.c_str(), status);
    camera_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>(topic_camera_info_pub.c_str(), queue_size);
}

void IRCamera::pub_shutdown()
{
    string status = "Publisher " + topic_image_pub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    it_pub_.shutdown();
    //status = "Publisher " + topic_color_image_pub + " shuting down !";
    //OUT_WARN(nodeName.c_str(), status);
    //it_color_pub_.shutdown();
    status = "Publisher " + topic_camera_info_pub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    camera_info_pub_.shutdown();
}

void IRCamera::sub_topic_get()
{   
    topic_device_id_sub = device_id_sub_.getTopic();
    topic_resolution_ratio_sub = resolution_ratio_sub_.getTopic();
}

void IRCamera::sub_init()
{
    string status = "Subscriber " + topic_device_id_sub + " initiating !";
    OUT_INFO(nodeName.c_str(), status);
    device_id_sub_ = n_.subscribe(topic_device_id_sub.c_str(), queue_size, &IRCamera::device_id_callBack, this);
    status = "Subscriber " + topic_resolution_ratio_sub + " initiating !";
    OUT_INFO(nodeName.c_str(), status);
    resolution_ratio_sub_ = n_.subscribe(topic_resolution_ratio_sub.c_str(), queue_size, &IRCamera::resolution_ratio_callBack, this);
}

void IRCamera::sub_shutdown()
{
    string status = "Subscriber " + topic_device_id_sub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    device_id_sub_.shutdown();
    status = "Subscriber " + topic_resolution_ratio_sub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    resolution_ratio_sub_.shutdown();
}
//#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS(ircamera_nodelet::ircamera, nodelet::Nodelet);

#endif
