#pragma once
#ifndef _ROS_VISION_SYSTEM_H_
#define _ROS_VISION_SYSTEM_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "Camera/Camera.h"
#include "IRCamera/IRCamera.h"

using namespace std;
using namespace cv;


class Vision_System
{
    private:
      string ver_ = "1.2";//add ir camera
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
	  Camera *camera;
	  #ifdef _IRCAMERA_H_
	  IRCamera *ir_camera;
	  #endif
	  
    public:
      Vision_System(ros::NodeHandle& n, int thread);
      ~Vision_System();
      void run();
};

Vision_System::Vision_System(ros::NodeHandle& n, int thread) : n_(n), spinner(thread)
{
    spinner.start();
	camera = new Camera(n_);
	camera -> init();
	#ifdef _IRCAMERA_H_
	ir_camera = new IRCamera(n_);
	ir_camera -> init();
	#endif
    run();
}

Vision_System::~Vision_System()
{
	delete camera;
	#ifdef _IRCAMERA_H_
	delete ir_camera;
	#endif
}

void Vision_System::run()
{
	camera -> run();
	#ifdef _IRCAMERA_H_
	ir_camera -> run();
	#endif
}


#endif
