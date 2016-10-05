#include "stair_detector_geo/stair_detector_geo.h"

int main(int argc, char **argv) {
	/// Load an image
	cv::Mat src = cv::imread(argv[1], CV_8UC1);
	// src.convertTo(src, CV_32F);

	// contrast the depth image scale
	// double min, max;
	// cv::minMaxIdx(src, &min, &max);
	// cv::convertScaleAbs(src, src, 255 / max);
	
	cv::Mat src_rgb;
	cv::cvtColor(src, src_rgb, CV_GRAY2RGB);
	double min, max;
	cv::minMaxIdx(src_rgb, &min, &max);
	cv::convertScaleAbs(src_rgb, src_rgb, 255 / max);
	
	StarDetectorGeoParams param;
	param.debug = true;
	param.ignore_invalid = true;
	// param.fill_invalid = true;
	StairDetectorGeo sdg;
	sdg.setParam(param);
	std::vector<cv::Point> bounding_box;
	sdg.getStairs(src, bounding_box);
	sdg.drawBox(src_rgb, bounding_box);

	imshow("Result1", src_rgb);

	// change the color map of the image;
	// cv::Mat falseColorsMap;
	// applyColorMap(src_, falseColorsMap, cv::COLORMAP_AUTUMN);

	// // /// Create a window
	// cv::namedWindow(parameter_window_, CV_WINDOW_AUTOSIZE);

	/// Create a Trackbar for user to enter threshold
	// cv::createTrackbar("Canny Min Threshold:",   parameter_window_, &canny_low_threshold_, max_lowThreshold_, process);
	// cv::createTrackbar("Hough Min Line Length:", parameter_window_, &hough_min_line_length_, max_lowThreshold_, process);
	// cv::createTrackbar("Hough Max Line Gap:",    parameter_window_, &hough_max_line_gap_, max_lowThreshold_, process);
	// cv::createTrackbar("Hough Rho:", 			 parameter_window_, &hough_rho_, max_lowThreshold_, process);
	// cv::createTrackbar("Hough Theta:", 			 parameter_window_, &hough_theta_, max_lowThreshold_, process);
	// cv::createTrackbar("Hough Threshold:", 		 parameter_window_, &hough_threshold_, max_lowThreshold_, process);

	// cv::createButton("Ignore Invalid", process, &ignore_invalid_);
	// process(0, 0);

	// /// Wait until user exit program by pressing a key
	int k;
	while (k = cv::waitKey(100)) {
		// std::cout << k << std::endl;
		// ESC key is 27
		if (k == 27) {
			return 0;
		}
	}
}
