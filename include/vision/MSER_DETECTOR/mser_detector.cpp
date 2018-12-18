#include <ros/ros.h>
#include <nodelet/nodelet.h>
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
//#include <std_msgs/MultiArrayDimension.h>
//#include <std_msgs/MultiArrayLayout.h>
//#include <std_msgs/UInt16MultiArray.h>

#include "MSER_DETECTOR/MSER_Detector.h"

using namespace std;
using namespace ros;
using namespace cv;

namespace mser_detector_nodelet {
  
class mser_detector : public nodelet::Nodelet
{    
  private:
    string ver_ = "1.1";
    ros::NodeHandle n_;
    MSER_Detector *md;

  public:
    mser_detector();//ros::NodeHandle& nh);
    ~mser_detector();
    //void run();
  
  public:
    virtual void onInit()
    {
      n_ = getNodeHandle();
      md = new MSER_Detector(n_);
      md -> init();
    }
    
};

mser_detector::mser_detector()// : n_(nh)
{    
}

mser_detector::~mser_detector()
{
    delete md;
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mser_detector_nodelet::mser_detector, nodelet::Nodelet);
