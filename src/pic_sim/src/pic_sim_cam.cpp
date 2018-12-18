#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <sstream> // for converting the command line parameter to integer

const int loopRate = 30;

int main(int argc, char** argv)
{
  ROS_INFO("Initializing pic_sim_cam ...");

  // Check if video source has been passed as a parameter
  if(argv[1] == NULL)
  {
    ROS_INFO("Reading video source %s ...", argv[1]);
    return 1;
  }

  ros::init(argc, argv, "pic_sim_cam");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("pic_sim/image_raw", 1);

  // Convert the passed as command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;
  // Check if it is indeed a number
  if(!(video_sourceCmd >> video_source)) 
  {
    ROS_INFO("Reading video source fail");
    return 1;
  }

  cv::VideoCapture cap(video_source);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(loopRate);
  ROS_INFO("pic_sim_cam activating ...");
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
