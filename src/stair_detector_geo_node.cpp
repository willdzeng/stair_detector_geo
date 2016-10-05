#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stair_detector_geo/stair_detector_geo.h>

StairDetectorGeo sdg;
std::vector<cv::Point> bounding_box;
// cv::Mat rgb_image;
// cv::Mat depth_image;
StarDetectorGeoParams param;

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_ONCE("Recevied RGB Image");
    try
    {
        cv::Mat rgb_image = cv_bridge::toCvCopy(msg, "8UC3")->image;
        sdg.drawBox(rgb_image, bounding_box);
        cv::imshow("Result", rgb_image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to '8UC3'.", msg->encoding.c_str());
    }
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_ONCE("Recevied Depth Image");
    try
    {
        cv::Mat depth_image = cv_bridge::toCvCopy(msg, "8UC1")->image;
        if (sdg.getStairs(depth_image, bounding_box)) {
            std::cout << "Found  stiars" << std::endl;
        } else {
            std::cout << "Can't find stiars" << std::endl;
        }
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

    // param.debug = false;
    param.fill_invalid = true;
    // param.ignore_invalid = true;
    sdg.setParam(param);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_depth = it.subscribe("depth/image", 1, depthImageCallback);
    image_transport::Subscriber sub_rgb = it.subscribe("rgb/image", 1, rgbImageCallback);
    ROS_INFO_ONCE("Stair detector waiting for images");
    ros::spin();
    cv::destroyWindow("Result");
    return 0;
}