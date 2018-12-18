#pragma once
#ifndef _MSER_DETECTOR_H_
#define _MSER_DETECTOR_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/Temperature.h>
/*
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>
*/

#include "MSER_DETECTOR/MSER.h"
#include "../../matrix/MATRIX_LINK.h"

using namespace std;
using namespace ros;
using namespace cv;

#define IMAGE         0x01
#define ROI_BBOX_HIGH 0x02
#define ROI_BBOX_LOW  0x04
#define TEMP_MAX      0x08
#define TEMP_MIN      0x10
#define TEMP_SUG      0x20

class MSER_Detector
{    
  public:
    string nodeName = "MSER_Detector";
    string topic_image_sub = "MSER_Detector/image_thermal";
    string topic_image_mser_pub = "MSER_Detector/image_mser";
    string topic_mser_roi_bbox_high_pub = "MSER_Detector/mser_roi_bbox_high";
    string topic_mser_roi_bbox_low_pub = "MSER_Detector/mser_roi_bbox_low";
    string topic_mser_temperature_max_pub = "MSER_Detector/mser_temperature_max";
    string topic_mser_temperature_min_pub = "MSER_Detector/mser_temperature_min";
  
    string topic_ir_temperature_suggest_pub = "MSER_Detector/ir_temperature_suggest";
    
    string topic_mser_min_width_sub = "MSER_Detector/detect_min_width";
    string topic_mser_min_height_sub = "MSER_Detector/detect_min_height";
    string topic_mser_max_width_sub = "MSER_Detector/detect_max_width";
    string topic_mser_max_height_sub = "MSER_Detector/detect_max_height";
    
    string topic_mser_temperature_threshold_high_sub = "MSER_Detector/detect_temperature_threshold_high";
    string topic_mser_temperature_threshold_low_sub = "MSER_Detector/detect_temperature_threshold_low";
    string topic_mser_temperature_image_min_sub = "MSER_Detector/detect_temperature_image_min";
    string topic_mser_temperature_image_max_sub = "MSER_Detector/detect_temperature_image_max";
    /*
    double max_variation = MSER_MAX_VARIATION;
    double max_diversity = MSER_MAX_DIVERSITY;
    int max_evolution = MSER_MAX_EVOLUTION;
    double area_threshold = MSER_AREA_THRESHOLD;
    double min_margin = MSER_MIN_MARGIN;
    int edge_blur_size = MSER_EDGE_BLUR_SIZE;
    double min_window_ratio = MSER_MIN_WINDOW_RATIO;
    double max_window_ratio = MSER_MIN_WINDOW_RATIO;
    */
  private:
    string ver_ = "1.0";
    int queue_size = 4;
    #ifdef _ROS_LINK_H_
    ROS_Link *ros_link;
    #endif
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_mser_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_mser;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_mser;
    image_transport::SubscriberStatusCallback disconnect_cb_image_mser;
    ros::SubscriberStatusCallback connect_cb_mser_roi_bbox_high;
    ros::SubscriberStatusCallback disconnect_cb_mser_roi_bbox_high;
    ros::SubscriberStatusCallback connect_cb_mser_roi_bbox_low;
    ros::SubscriberStatusCallback disconnect_cb_mser_roi_bbox_low;
    ros::SubscriberStatusCallback connect_cb_mser_temperature_max;
    ros::SubscriberStatusCallback disconnect_cb_mser_temperature_max;
    ros::SubscriberStatusCallback connect_cb_mser_temperature_min;
    ros::SubscriberStatusCallback disconnect_cb_mser_temperature_min;
      
    ros::SubscriberStatusCallback connect_cb_ir_temperature_suggest;
    ros::SubscriberStatusCallback disconnect_cb_ir_temperature_suggest;
  
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_mser_pub_;
    ros::Publisher  mser_roi_bbox_pub_high_;
    ros::Publisher  mser_roi_bbox_pub_low_;
    ros::Publisher  mser_temperature_max_;
    ros::Publisher  mser_temperature_min_;
  
    ros::Publisher  ir_temperature_suggest_;
  
    ros::Subscriber mser_min_width_sub_;
    ros::Subscriber mser_min_height_sub_;
    ros::Subscriber mser_max_width_sub_;
    ros::Subscriber mser_max_height_sub_;
    ros::Subscriber mser_temperature_threshold_high_sub_;
    ros::Subscriber mser_temperature_threshold_low_sub_;
    ros::Subscriber mser_temperature_image_min_sub_;
    ros::Subscriber mser_temperature_image_max_sub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    /*
    void sub_manual_topic_get();
    void sub_manual_init();
    void sub_manual_shutdown();
    */
    void connectCb_image_mser(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_mser_pub_.getNumSubscribers() > 1) return;
      string status = topic_image_mser_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_mser == 0)
      {
        sub_init();
      }
      status_mser = status_mser | IMAGE;
    }
    void disconnectCb_image_mser(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_mser_pub_.getNumSubscribers() > 0) return;
      string status = topic_image_mser_pub + " disconnected !";
      OUT_WARN(nodeName, status);      
      status_mser = status_mser & (0xFF - IMAGE);
      if(status_mser == 0)
      {
        sub_shutdown();
      }
    }
    void connectCb_mser_roi_bbox_high(const ros::SingleSubscriberPublisher& ssp)
    {
      if(mser_roi_bbox_pub_high_.getNumSubscribers() > 1) return;
      string status = topic_mser_roi_bbox_high_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_mser == 0)
      {
        sub_init();
      }
      status_mser = status_mser | ROI_BBOX_HIGH;
    }
    void disconnectCb_mser_roi_bbox_high(const ros::SingleSubscriberPublisher& ssp)
    {
      if(mser_roi_bbox_pub_high_.getNumSubscribers() > 0) return;
      string status = topic_mser_roi_bbox_high_pub + " disconnected !";
      OUT_WARN(nodeName, status);  
      status_mser = status_mser & (0xFF - ROI_BBOX_HIGH);
      if(status_mser == 0)
      {
        sub_shutdown();
      }
    }
    void connectCb_mser_roi_bbox_low(const ros::SingleSubscriberPublisher& ssp)
    {
      if(mser_roi_bbox_pub_low_.getNumSubscribers() > 1) return;
      string status = topic_mser_roi_bbox_low_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_mser == 0)
      {
        sub_init();
      }
      status_mser = status_mser | ROI_BBOX_LOW;
    }
    void disconnectCb_mser_roi_bbox_low(const ros::SingleSubscriberPublisher& ssp)
    {
      if(mser_roi_bbox_pub_low_.getNumSubscribers() > 0) return;
      string status = topic_mser_roi_bbox_low_pub + " disconnected !";
      OUT_WARN(nodeName, status);  
      status_mser = status_mser & (0xFF - ROI_BBOX_LOW);
      if(status_mser == 0)
      {
        sub_shutdown();
      }
    }
    void connectCb_mser_temperature_max(const ros::SingleSubscriberPublisher& ssp)
    {
      if(mser_temperature_max_.getNumSubscribers() > 1) return;
      string status = topic_mser_temperature_max_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_mser == 0)
      {
        sub_init();
      }
      status_mser = status_mser | TEMP_MAX;
    }
    void disconnectCb_mser_temperature_max(const ros::SingleSubscriberPublisher& ssp)
    {
      if(mser_temperature_max_.getNumSubscribers() > 0) return;
      string status = topic_mser_temperature_max_pub + " disconnected !";
      OUT_WARN(nodeName, status);  
      status_mser = status_mser & (0xFF - TEMP_MAX);
      if(status_mser == 0)
      {
        sub_shutdown();
      }
    }
    void connectCb_mser_temperature_min(const ros::SingleSubscriberPublisher& ssp)
    {
      if(mser_temperature_min_.getNumSubscribers() > 1) return;
      string status = topic_mser_temperature_min_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_mser == 0)
      {
        sub_init();
      }
      status_mser = status_mser | TEMP_MIN;
    }
    void disconnectCb_mser_temperature_min(const ros::SingleSubscriberPublisher& ssp)
    {
      if(mser_temperature_min_.getNumSubscribers() > 0) return;
      string status = topic_mser_temperature_min_pub + " disconnected !";
      OUT_WARN(nodeName, status);  
      status_mser = status_mser & (0xFF - TEMP_MIN);
      if(status_mser == 0)
      {
        sub_shutdown();
      }
    }
    void connectCb_ir_temperature_suggest(const ros::SingleSubscriberPublisher& ssp)
    {
      if(ir_temperature_suggest_.getNumSubscribers() > 1) return;
      string status = topic_ir_temperature_suggest_pub + " connected !";
      OUT_INFO(nodeName, status);
      if(status_mser == 0)
      {
        sub_init();
      }
      status_mser = status_mser | TEMP_SUG;
    }
    void disconnectCb_ir_temperature_suggest(const ros::SingleSubscriberPublisher& ssp)
    {
      if(ir_temperature_suggest_.getNumSubscribers() > 0) return;
      string status = topic_ir_temperature_suggest_pub + " disconnected !";
      OUT_WARN(nodeName, status);  
      status_mser = status_mser & (0xFF - TEMP_SUG);
      if(status_mser == 0)
      {
        sub_shutdown();
      }
    }
    
  private:
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_mser_publish();
    void mser_roi_bbox_high_publish();
    void mser_roi_bbox_low_publish();
    void mser_temperature_max_publish(double temp);
    void mser_temperature_min_publish(double temp);
  
    void ir_temperature_suggest_publish();
  
    bool mser_convert();
    void mser_roi_bbox_measure_high();
    void mser_roi_bbox_measure_low();
    
    void mser_min_width_callBack(const std_msgs::Int32::ConstPtr& msg);
    void mser_min_height_callBack(const std_msgs::Int32::ConstPtr& msg);
    void mser_max_width_callBack(const std_msgs::Int32::ConstPtr& msg);
    void mser_max_height_callBack(const std_msgs::Int32::ConstPtr& msg);
    
    void mser_temperature_threshold_high_callBack(const std_msgs::Int32::ConstPtr& msg);
    void mser_temperature_threshold_low_callBack(const std_msgs::Int32::ConstPtr& msg);
    void mser_temperature_image_min_callBack(const std_msgs::Int32::ConstPtr& msg);
    void mser_temperature_image_max_callBack(const std_msgs::Int32::ConstPtr& msg);
    
  private:
    Mat image_thermal;
    Mat image_temperature;
    Mat image_threshold_high;
    Mat image_threshold_low;
    Mat image_mser;
    OBJ_MSER *mser;
    vector<Rect> mser_roi_bbox_high;
    vector<Rect> mser_roi_bbox_low;
    vector<vector<Point>> mser_roi_contours;
    int status_mser = 0;
  
    bool is_blur = true;
    
    double temperature_max = 0;
    double temperature_min = 0;
    int temperature_threshold_high = 30;
    int temperature_threshold_low = 27;
    int temperature_image_min = 0;//15;
    int temperature_image_max = 0;//35;

  public:
    MSER_Detector(ros::NodeHandle& nh);
    ~MSER_Detector();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_mser_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_mser = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_mser    = boost::bind(&MSER_Detector::connectCb_image_mser, this, _1);
      disconnect_cb_image_mser = boost::bind(&MSER_Detector::disconnectCb_image_mser, this, _1);
      connect_cb_mser_roi_bbox_high    = boost::bind(&MSER_Detector::connectCb_mser_roi_bbox_high, this, _1);
      disconnect_cb_mser_roi_bbox_high = boost::bind(&MSER_Detector::disconnectCb_mser_roi_bbox_high, this, _1);
      connect_cb_mser_roi_bbox_low    = boost::bind(&MSER_Detector::connectCb_mser_roi_bbox_low, this, _1);
      disconnect_cb_mser_roi_bbox_low = boost::bind(&MSER_Detector::disconnectCb_mser_roi_bbox_low, this, _1);
      connect_cb_mser_temperature_max    = boost::bind(&MSER_Detector::connectCb_mser_temperature_max, this, _1);
      disconnect_cb_mser_temperature_max = boost::bind(&MSER_Detector::disconnectCb_mser_temperature_max, this, _1);
      connect_cb_mser_temperature_min    = boost::bind(&MSER_Detector::connectCb_mser_temperature_min, this, _1);
      disconnect_cb_mser_temperature_min = boost::bind(&MSER_Detector::disconnectCb_mser_temperature_min, this, _1);
      
      connect_cb_ir_temperature_suggest    = boost::bind(&MSER_Detector::connectCb_ir_temperature_suggest, this, _1);
      disconnect_cb_ir_temperature_suggest = boost::bind(&MSER_Detector::disconnectCb_ir_temperature_suggest, this, _1);
      
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
      #ifdef _ROS_LINK_H_
      ros_link = new ROS_Link(n_, nodeName);
      ros_link -> pub_init();
      ros_link -> sub_init();
      ros_link -> add_cell(mser_min_width_sub_, topic_mser_min_width_sub);
      ros_link -> add_cell(mser_min_height_sub_, topic_mser_min_height_sub);
      ros_link -> add_cell(mser_max_width_sub_, topic_mser_max_width_sub);
      ros_link -> add_cell(mser_max_height_sub_, topic_mser_max_height_sub);
      ros_link -> add_cell(mser_temperature_threshold_high_sub_, topic_mser_temperature_threshold_high_sub);
      ros_link -> add_cell(mser_temperature_threshold_low_sub_, topic_mser_temperature_threshold_low_sub);
      ros_link -> add_cell(mser_temperature_image_min_sub_, topic_mser_temperature_image_min_sub);
      ros_link -> add_cell(mser_temperature_image_max_sub_, topic_mser_temperature_image_max_sub);
      #endif
    }
    
};

MSER_Detector::MSER_Detector(ros::NodeHandle& nh) : n_(nh)
{    
    mser = new OBJ_MSER;
    mser -> min_width = 3;
    mser -> min_height = 3;
    mser -> max_width = 80;
    mser -> max_height = 80;
    mser -> min_window_ratio = 0.8;
    mser -> max_window_ratio = 0.9;
}

MSER_Detector::~MSER_Detector()
{
    delete mser;
    #ifdef _ROS_LINK_H_
    if(ros_link != NULL)
      delete ros_link;
    #endif
}

void MSER_Detector::run()
{  
    if(!mser_convert()) return;
  
    mser_roi_bbox_high_publish();
    mser_roi_bbox_low_publish();
    mser_temperature_max_publish(this -> temperature_max);
    mser_temperature_min_publish(this -> temperature_min);
  
    ir_temperature_suggest_publish();
  
    image_mser_publish();
    return;
}

void MSER_Detector::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
      //cv_ptr -> image.copyTo(image_thermal);
      this -> image_thermal = cv_ptr -> image;
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

bool MSER_Detector::mser_convert()
{
    if(this -> image_thermal.empty())
        return false;
    double temp_amp = 0;
    double temp_shift = 0;
    
    minMaxLoc(this -> image_thermal, &(this -> temperature_min), &(this -> temperature_max));
    this -> temperature_max = this -> temperature_max * 0.1 - 100.0;
    this -> temperature_min = this -> temperature_min * 0.1 - 100.0;
  
    this -> image_thermal.convertTo(this -> image_temperature, CV_8UC1, 0.1, -100);//float t = ((((float)data[i])-1000.f))/10.f;
    
    //minMaxLoc(this -> image_temperature, &temp_min, &temp_max);
    if(!(this -> temperature_image_max == 0 && this -> temperature_image_min == 0))
    {
        temp_amp   = (255.0 - 0.0) / (this -> temperature_image_max - this -> temperature_image_min);
        temp_shift = 0.0 - this -> temperature_image_min * temp_amp;
    }
    else
    {
        temp_amp   = (255.0 - 0.0) / (this -> temperature_max - this -> temperature_min);
        temp_shift = 0.0 - this -> temperature_min * temp_amp;
    }
    (this -> image_temperature).convertTo(this -> image_mser, CV_8UC1, temp_amp, temp_shift);
  
    if(is_blur)
    {
      Mat temp;
      //GaussianBlur(this -> image_mser, this -> image_mser, Size(3, 3), 1);
      bilateralFilter(this -> image_mser, temp, 5, 30, 30);
      this -> image_mser = temp;
      //erode(this -> image_mser, this -> image_mser, Mat());
      //dilate(this -> image_mser, this -> image_mser, Mat());
    }
    applyColorMap(this -> image_mser, this -> image_mser, COLORMAP_JET);
    
    mser_roi_bbox_measure_high();
    mser_roi_bbox_measure_low();
  
    return true;
}

void MSER_Detector::mser_roi_bbox_measure_high()
{
    if((status_mser & ROI_BBOX_HIGH) == 0) return;

    Scalar color(0, 0, 255);
    threshold(this -> image_temperature, this -> image_threshold_high, this -> temperature_threshold_high, 255, THRESH_BINARY);
  
    mser -> region_detect(this -> image_threshold_high);
    this -> mser_roi_bbox_high = mser -> bboxes;
    mser -> draw(this -> image_mser, color);
    return;
}

void MSER_Detector::mser_roi_bbox_measure_low()
{
    if((status_mser & ROI_BBOX_LOW) == 0) return;

    Scalar color(0, 255, 0);
    threshold(this -> image_temperature, this -> image_threshold_low, this -> temperature_threshold_low, 255, THRESH_BINARY_INV);
  
    mser -> region_detect(this -> image_threshold_low);
    this -> mser_roi_bbox_low = mser -> bboxes;
    mser -> draw(this -> image_mser, color);
    return;
}

void MSER_Detector::image_mser_publish()
{
    if(status_mser & IMAGE == 0) return;
    //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
    msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this -> image_mser).toImageMsg();
    //sensor_msgs::ImagePtr msg_image;
    //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
    it_mser_pub_.publish(msg_image);
}

void MSER_Detector::mser_roi_bbox_high_publish()
{
    if((status_mser & ROI_BBOX_HIGH) == 0) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < this -> mser_roi_bbox_high.size(); i++)
    {
      msg.x_offset = this -> mser_roi_bbox_high[i].x + this -> mser_roi_bbox_high[i].width/2;      
      msg.y_offset = this -> mser_roi_bbox_high[i].y + this -> mser_roi_bbox_high[i].height/2;      
      msg.width    = this -> mser_roi_bbox_high[i].width;      
      msg.height   = this -> mser_roi_bbox_high[i].height;     
      mser_roi_bbox_pub_high_.publish(msg);
    }
}

void MSER_Detector::mser_roi_bbox_low_publish()
{
    if((status_mser & ROI_BBOX_LOW) == 0) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < this -> mser_roi_bbox_low.size(); i++)
    {
      msg.x_offset = this -> mser_roi_bbox_low[i].x + this -> mser_roi_bbox_low[i].width/2;      
      msg.y_offset = this -> mser_roi_bbox_low[i].y + this -> mser_roi_bbox_low[i].height/2;      
      msg.width    = this -> mser_roi_bbox_low[i].width;      
      msg.height   = this -> mser_roi_bbox_low[i].height;     
      mser_roi_bbox_pub_low_.publish(msg);
    }
}

void MSER_Detector::mser_temperature_max_publish(double temp)
{
    if((status_mser & TEMP_MAX) == 0) return;
    sensor_msgs::Temperature msg;
    msg.temperature = temp;  
    mser_temperature_max_.publish(msg);
}

void MSER_Detector::mser_temperature_min_publish(double temp)
{
    if((status_mser & TEMP_MIN) == 0) return;
    sensor_msgs::Temperature msg;
    msg.temperature = temp;  
    mser_temperature_min_.publish(msg);
}

void MSER_Detector::ir_temperature_suggest_publish()
{
    if((status_mser & TEMP_SUG) == 0) return;
    //boost::shared_ptr< std_msgs::String > msg_data_;
    std_msgs::String msg;
    string suggest_obj;
    if((status_mser & TEMP_MAX) == 0)
    {
      status_mser = status_mser | TEMP_MAX;
      return;
    }
    if((status_mser & TEMP_MIN) == 0)
    {
      status_mser = status_mser | TEMP_MIN;
      return;
    }
    if(36.0 < this -> temperature_max && this -> temperature_max < 38.0)
    {
        suggest_obj = "HUMAN BODY TEMPERATURE";
    }
    else if(this -> temperature_max > 80.0)
    {
        suggest_obj = "HIGH TEMPERATURE";
    }
    else if(this -> temperature_min < 20.0)
    {
        suggest_obj = "LOW TEMPERATURE";
    }
    else
    {
        suggest_obj = "NORMAL TEMPERATURE";
    }
    
    msg.data = suggest_obj;  
    ir_temperature_suggest_.publish(msg);
    return;
}

void MSER_Detector::mser_min_width_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string str;
    token << msg -> data;
    token >> str;
    string status = "MSER minimum width is changed to " + str;
    OUT_INFO(nodeName, status);
    mser -> min_width = msg -> data;
}

void MSER_Detector::mser_min_height_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string str;
    token << msg -> data;
    token >> str;
    string status = "MSER minimum height is changed to " + str;
    OUT_INFO(nodeName, status);
    mser -> min_height = msg -> data;
}

void MSER_Detector::mser_max_width_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string str;
    token << msg -> data;
    token >> str;
    string status = "MSER maximum width is changed to " + str;
    OUT_INFO(nodeName, status);
    mser -> max_width = msg -> data;
}

void MSER_Detector::mser_max_height_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string str;
    token << msg -> data;
    token >> str;
    string status = "MSER maximum height is changed to " + str;
    OUT_INFO(nodeName, status);
    mser -> max_height = msg -> data;
}

void MSER_Detector::mser_temperature_threshold_high_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string str;
    token << msg -> data;
    token >> str;
    string status = "MSER threshold high temperature is changed to " + str;
    OUT_INFO(nodeName, status);
    this -> temperature_threshold_high = msg -> data;
}

void MSER_Detector::mser_temperature_threshold_low_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string str;
    token << msg -> data;
    token >> str;
    string status = "MSER threshold low temperature is changed to " + str;
    OUT_INFO(nodeName, status);
    this -> temperature_threshold_low = msg -> data;
}

void MSER_Detector::mser_temperature_image_min_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string str;
    token << msg -> data;
    token >> str;
    string status = "MSER minimum temperature is changed to " + str;
    OUT_INFO(nodeName, status);
    this -> temperature_image_min = msg -> data;
}

void MSER_Detector::mser_temperature_image_max_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    stringstream token;
    string str;
    token << msg -> data;
    token >> str;
    string status = "MSER maximum temperature is changed to " + str;
    OUT_INFO(nodeName, status);
    this -> temperature_image_max = msg -> data;
}
    
void MSER_Detector::pub_topic_get()
{
    topic_image_mser_pub = it_mser_pub_.getTopic();
    topic_mser_roi_bbox_high_pub = mser_roi_bbox_pub_high_.getTopic();
    topic_mser_roi_bbox_low_pub = mser_roi_bbox_pub_low_.getTopic();
    topic_mser_temperature_max_pub = mser_temperature_max_.getTopic();
    topic_mser_temperature_min_pub = mser_temperature_min_.getTopic();
  
    topic_ir_temperature_suggest_pub = ir_temperature_suggest_.getTopic();
}

void MSER_Detector::pub_init()
{
    string status = "Publisher " + topic_image_sub + " initiating !";
    OUT_INFO(nodeName, status);
    it_mser_pub_ = it_mser_ -> advertise(topic_image_mser_pub, queue_size, connect_cb_image_mser, disconnect_cb_image_mser);

    status = "Publisher " + topic_mser_roi_bbox_high_pub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_roi_bbox_pub_high_ = n_.advertise< sensor_msgs::RegionOfInterest >(topic_mser_roi_bbox_high_pub.c_str(), queue_size, connect_cb_mser_roi_bbox_high, disconnect_cb_mser_roi_bbox_high);

    status = "Publisher " + topic_mser_roi_bbox_low_pub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_roi_bbox_pub_low_ = n_.advertise< sensor_msgs::RegionOfInterest >(topic_mser_roi_bbox_low_pub.c_str(), queue_size, connect_cb_mser_roi_bbox_low, disconnect_cb_mser_roi_bbox_low);

    status = "Publisher " + topic_mser_temperature_max_pub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_temperature_max_ = n_.advertise< sensor_msgs::Temperature >(topic_mser_temperature_max_pub.c_str(), queue_size, connect_cb_mser_temperature_max, disconnect_cb_mser_temperature_max);

    status = "Publisher " + topic_mser_temperature_min_pub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_temperature_min_ = n_.advertise< sensor_msgs::Temperature >(topic_mser_temperature_min_pub.c_str(), queue_size, connect_cb_mser_temperature_min, disconnect_cb_mser_temperature_min);

    status = "Publisher " + topic_ir_temperature_suggest_pub + " initiating !";
    OUT_INFO(nodeName, status);
    ir_temperature_suggest_ = n_.advertise< std_msgs::String >(topic_ir_temperature_suggest_pub.c_str(), queue_size, connect_cb_ir_temperature_suggest, disconnect_cb_ir_temperature_suggest);}

void MSER_Detector::pub_shutdown()
{
    string status = "Publisher " + topic_image_mser_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    it_mser_pub_.shutdown();

    status = "Publisher " + topic_mser_roi_bbox_high_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_roi_bbox_pub_high_.shutdown();

    status = "Publisher " + topic_mser_roi_bbox_low_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_roi_bbox_pub_low_.shutdown();

    status = "Publisher " + topic_mser_temperature_max_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_temperature_max_.shutdown();

    status = "Publisher " + topic_mser_temperature_min_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_temperature_min_.shutdown();

    status = "Publisher " + topic_ir_temperature_suggest_pub + " shuting down !";
    OUT_WARN(nodeName, status);
    ir_temperature_suggest_.shutdown();
}

void MSER_Detector::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_mser_min_width_sub = mser_min_width_sub_.getTopic();
    topic_mser_min_height_sub = mser_min_height_sub_.getTopic();
    topic_mser_max_width_sub = mser_max_width_sub_.getTopic();
    topic_mser_max_height_sub = mser_max_height_sub_.getTopic();
    topic_mser_temperature_threshold_high_sub = mser_temperature_threshold_high_sub_.getTopic();
    topic_mser_temperature_threshold_low_sub = mser_temperature_threshold_low_sub_.getTopic();
    topic_mser_temperature_image_min_sub = mser_temperature_image_max_sub_.getTopic();
    topic_mser_temperature_image_max_sub = mser_temperature_image_max_sub_.getTopic();
}

void MSER_Detector::sub_init()
{
    string status = "Subscriber " + topic_image_sub + " initiating !";
    OUT_INFO(nodeName, status);
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &MSER_Detector::image_callBack, this);

    status = "Subscriber " + topic_mser_min_width_sub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_min_width_sub_ = n_.subscribe(topic_mser_min_width_sub.c_str(), queue_size, &MSER_Detector::mser_min_width_callBack, this);

    status = "Subscriber " + topic_mser_min_height_sub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_min_height_sub_ = n_.subscribe(topic_mser_min_height_sub.c_str(), queue_size, &MSER_Detector::mser_min_height_callBack, this);
 
    status = "Subscriber " + topic_mser_max_width_sub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_max_width_sub_ = n_.subscribe(topic_mser_max_width_sub.c_str(), queue_size, &MSER_Detector::mser_max_width_callBack, this);
    
    status = "Subscriber " + topic_mser_max_height_sub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_max_height_sub_ = n_.subscribe(topic_mser_max_height_sub.c_str(), queue_size, &MSER_Detector::mser_max_height_callBack, this);
    
    status = "Subscriber " + topic_mser_temperature_threshold_high_sub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_temperature_threshold_high_sub_ = n_.subscribe(topic_mser_temperature_threshold_high_sub.c_str(), queue_size, &MSER_Detector::mser_temperature_threshold_high_callBack, this);
    
    status = "Subscriber " + topic_mser_temperature_threshold_low_sub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_temperature_threshold_low_sub_ = n_.subscribe(topic_mser_temperature_threshold_low_sub.c_str(), queue_size, &MSER_Detector::mser_temperature_threshold_low_callBack, this);
    
    status = "Subscriber " + topic_mser_temperature_image_min_sub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_temperature_image_min_sub_ = n_.subscribe(topic_mser_temperature_image_min_sub.c_str(), queue_size, &MSER_Detector::mser_temperature_image_min_callBack, this);
    
    status = "Subscriber " + topic_mser_temperature_image_max_sub + " initiating !";
    OUT_INFO(nodeName, status);
    mser_temperature_image_max_sub_ = n_.subscribe(topic_mser_temperature_image_max_sub.c_str(), queue_size, &MSER_Detector::mser_temperature_image_max_callBack, this);
}

void MSER_Detector::sub_shutdown()
{
    string status = "Subscriber " + topic_image_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    it_sub_.shutdown();

    status = "Subscriber " + topic_mser_min_width_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_min_width_sub_.shutdown();

    status = "Subscriber " + topic_mser_min_height_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_min_height_sub_.shutdown();

    status = "Subscriber " + topic_mser_max_width_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_max_width_sub_.shutdown();

    status = "Subscriber " + topic_mser_max_height_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_max_height_sub_.shutdown();

    status = "Subscriber " + topic_mser_temperature_threshold_high_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_temperature_threshold_high_sub_.shutdown();

    status = "Subscriber " + topic_mser_temperature_threshold_low_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_temperature_threshold_low_sub_.shutdown();

    status = "Subscriber " + topic_mser_temperature_image_min_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_temperature_image_min_sub_.shutdown();

    status = "Subscriber " + topic_mser_temperature_image_max_sub + " shuting down !";
    OUT_WARN(nodeName, status);
    mser_temperature_image_max_sub_.shutdown();
}
#endif

