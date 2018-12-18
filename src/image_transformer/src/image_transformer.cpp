#include <ros/ros.h>
#include <ros/spinner.h>
#include <nodelet/loader.h>
#include <string>
#include "image_transformer/Image_Transformer.hpp"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "image_transformer";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle n;

  ROS_INFO("%s activating ok !", nodeName.c_str());
  Image_Transformer it(n);
  it.init();
  /*
  while (ros::ok()) 
  {
    //it.run();
  }
  */
  ros::spin();
  
  ros::shutdown();
  return 0;
}

