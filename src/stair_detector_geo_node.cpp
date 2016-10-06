#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stair_detector_geo/stair_detector_geo.h>
#include <dynamic_reconfigure/server.h>
#include "stair_detector_geo/StairDetectorGeoConfig.h"
#include <boost/bind.hpp>

StairDetectorGeo sdg;
std::vector<cv::Point> tmp_bounding_box;
std::vector<cv::Point> show_bounding_box;
// cv::Mat rgb_image;
// cv::Mat depth_image;
StairDetectorGeoParams param;
bool stair_detected = false;
int detected_count = 0;
int max_valid_detection = 10;

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_ONCE("Recevied RGB Image");
    try
    {
        cv::Mat rgb_image = cv_bridge::toCvCopy(msg, "8UC3")->image;
        if (stair_detected) {
            sdg.drawBox(rgb_image, show_bounding_box);
        }
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
        if (sdg.getStairs(depth_image, tmp_bounding_box)) {
            detected_count++;
            std::cout << "Found Potential Stiars" << std::endl;
            if (detected_count > max_valid_detection) {
                stair_detected = true;
                show_bounding_box = tmp_bounding_box;
                std::cout << "Found  stiars" << std::endl;
            }
        } else {
            stair_detected = false;
            detected_count = 0;
            show_bounding_box.clear();
            std::cout << "Can't find stiars" << std::endl;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to '8UC1'.", msg->encoding.c_str());
    }
}

void reconfigureCB(stair_detector_geo::StairDetectorGeoConfig &config, uint32_t level) {
        // high level bools
    param.show_result = config.show_result;
    param.debug = config.debug;
    param.ignore_invalid = config.ignore_invalid;
    param.fill_invalid = config.fill_invalid;
    param.use_laplacian = config.use_laplacian;
    // canny
    param.canny_low_threshold = config.canny_low_threshold;
    param.canny_ratio = config.canny_ratio;
    param.canny_kernel_size = config.canny_kernel_size;
    // hough transfrom
    param.hough_min_line_length = config.hough_min_line_length;
    param.hough_max_line_gap = config.hough_max_line_gap;
    param.hough_threshold = config.hough_threshold;
    param.hough_rho = config.hough_rho;
    param.hough_theta = config.hough_theta;
    // filter by slope histogram
    param.filter_slope_hist_bin_width = config.filter_slope_hist_bin_width;
    // filter by hard slope
    param.filter_slope_bandwidth = config.filter_slope_bandwidth;
    // merge parameters
    param.merge_max_dist_diff = config.merge_max_dist_diff;
    param.merge_max_angle_diff = config.merge_max_angle_diff;
    param.merge_close_count = config.merge_close_count;
    // bounding box
    param.minimum_line_num = config.minimum_line_num;
    max_valid_detection         = config.max_valid_detection;
    sdg.setParam(param);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stair_detector_geo");
    ros::NodeHandle nh;
    cv::namedWindow("Result");
    cv::startWindowThread();

    // param.debug = false;
    param.fill_invalid = false;
    param.ignore_invalid = false;
    param.use_laplacian = false;
    sdg.setParam(param);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_depth = it.subscribe("depth/image", 1, depthImageCallback);
    image_transport::Subscriber sub_rgb = it.subscribe("rgb/image", 1, rgbImageCallback);

    dynamic_reconfigure::Server<stair_detector_geo::StairDetectorGeoConfig> dsrv(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<stair_detector_geo::StairDetectorGeoConfig>::CallbackType cb = boost::bind(&reconfigureCB, _1, _2);
    dsrv.setCallback(cb);

    ROS_INFO_ONCE("Stair detector waiting for images");
    ros::spin();
    cv::destroyWindow("Result");
    return 0;
}