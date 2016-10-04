#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stair_detector_geo/stair_detector_geo.h>

StairDetectorGeo sdg;
std::vector<cv::Point> bounding_box;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO_ONCE("Recevied Image");
  try
  {
    cv::Mat image = cv_bridge::toCvCopy(msg, "8UC1")->image;
    if (sdg.getStairs(image, bounding_box)) {
      sdg.drawBox(image, bounding_box);
      std::cout << "Found  stiars" << std::endl;
    }else{
      std::cout << "Can't find stiars" << std::endl;
    }
    cv::imshow("Result", image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '8UC1'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "stair_detector_geo");
  ros::NodeHandle nh;
  cv::namedWindow("Result");
  cv::startWindowThread();
  StarDetectorGeoParams param;
  // param.ignore_invalid = false;
  param.fill_invalid = true;
  sdg.setParam(param);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
  ROS_INFO_ONCE("Stair detector waiting for images");
  ros::spin();
  cv::destroyWindow("Result");
}