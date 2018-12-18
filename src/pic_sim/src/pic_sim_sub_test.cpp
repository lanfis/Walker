#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat img_ori = cv_bridge::toCvShare(msg, "mono16")->image;
    Mat img;
    img.create(img_ori.rows, img_ori.cols, CV_8UC3);
    img_ori.convertTo(img_ori, CV_8UC1, 0.1, -100);
    img_ori.convertTo(img_ori, -1, 6, 0);
    applyColorMap(img_ori, img, COLORMAP_JET);
    cv::imshow("view", img);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ROS_INFO("Initializing pic_sim_sub_test ...");
  ros::init(argc, argv, "pic_sim_sub_test");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("thermal_image", 1, imageCallback);


  ros::spin();
  cv::destroyWindow("view");
}
