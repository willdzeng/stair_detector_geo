#include "stair_detector_geo/stair_detector_geo.h"

StairDetectorGeo::StairDetectorGeo() {
	// run a empty set param to convert all the angle into radius
	StairDetectorGeoParams param;
	setParam(param);
}

bool StairDetectorGeo::getStairs(const cv::Mat& input_image, std::vector<cv::Point>& bounding_box) {
	cv::Mat src_gray = input_image.clone();
	cv::Mat src_rgb;
	std::vector<cv::Point> tmp_bounding_box;
	time_t begin = time(NULL);

	if ( input_image.channels() == 1) {
		double min, max;
		cv::minMaxIdx(src_gray, &min, &max);
		cv::convertScaleAbs(src_gray, src_gray, 255 / max);

		// make sure that it actually has the formmat of 8UC1
		src_gray.convertTo(src_gray, CV_8UC1);

		if (param_.fill_invalid) {
			fillByNearestNeighbour(src_gray, src_gray);
		}
		cv::cvtColor(src_gray, src_rgb, CV_GRAY2RGB);
	} else {
		std::cout << "Must use GrayScale Depth image as input" << std::endl;
		return false;
	}
	// std::cout << param_.canny_low_threshold << " " << param_.canny_ratio << " " << param_.canny_kernel_size << std::endl;
	cv::Mat edge;

	if (param_.use_laplacian) {
		laplacianEdgeDetection(src_gray, edge);
	} else {
		cannyEdgeDetection(src_gray, edge);
	}
	// sobelEdgeDetection(src_gray, edge_image);
	// cv::imshow("edge image", edge);
	// cv::imshow("depth", src_gray);
	if (param_.ignore_invalid) {
		ignoreInvalid(src_gray, edge);
	}

	if (param_.debug) {
		cv::imshow("edge image", edge);
	}

	Lines lines;
	houghLine(edge, lines);
	// std::cout << "Finished hough line" << std::endl;

	if (getBoundingBox(src_rgb, lines, tmp_bounding_box)) {
		time_t end = time(NULL);
		double second = difftime(end, begin);
		if (param_.debug) {
			std::cout << "Found Stiars Time used: " << second << std::endl;
		}
		bounding_box  = tmp_bounding_box;
		return true;
	} else {
		// std::cout << "There is not a stair" << std::endl;
		bounding_box.clear();
		return false;
	}
}

void StairDetectorGeo::setParam(const StairDetectorGeoParams& param) {
	param_ = param;
	param_.merge_max_angle_diff = param_.merge_max_angle_diff * PI / 180;
	param_.filter_slope_bandwidth = param_.filter_slope_bandwidth * PI / 180;
	param_.hough_theta = param_.hough_theta * PI / 180;
	param_.filter_slope_hist_bin_width = param_.filter_slope_hist_bin_width * PI / 180;
	param_.canny_kernel_size = param_.canny_kernel_size * 2 + 1;
	if (param_.canny_kernel_size > 7) {
		param_.canny_kernel_size = 7;
	}
	if (param_.canny_kernel_size < 1) {
		param_.canny_kernel_size = 1;
	}
}

/**
* @brief      Draws xy lines.
*
* @param      image  The image
* @param[in]  lines  The lines
*
*/

void StairDetectorGeo::drawLines(cv::Mat &image, const Lines &lines, const cv::Scalar &color) {
	for (int i = 0; i < lines.size(); i++) {
		// std::cout << "Drwing lines: " << lines[i] << std::endl;
		cv::line(image, lines[i].p1, lines[i].p2, color, 3, 8);
		// putText(image, std::to_string(lines[i].t), lines[i].p1, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 0));
	}
}

/**
 * @brief      Draws line's slope.
 *
 * @param      image  The image
 * @param[in]  lines  The lines
 * @param[in]  color  The color
 */
void StairDetectorGeo::drawLinesSlope(cv::Mat &image, const Lines &lines, const cv::Scalar &color) {
	for (int i = 0; i < lines.size(); i++) {
		putText(image, std::to_string(lines[i].t), lines[i].p1, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 0));
	}
}

/**
 * @brief      Draws line's radius.
 *
 * @param      image  The image
 * @param[in]  lines  The lines
 * @param[in]  color  The color
 */
void StairDetectorGeo::drawLinesRadius(cv::Mat &image, const Lines &lines, const cv::Scalar &color) {
	for (int i = 0; i < lines.size(); i++) {
		putText(image, std::to_string(lines[i].b), lines[i].p1, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 0));
	}
}

void StairDetectorGeo::drawBox(cv::Mat& image, const std::vector<cv::Point>& bounding_box) {
	if (bounding_box.size() != 2) {
		if (param_.debug) {
			std::cout << "Can't draw boxes because the point size is not equal to 2" << std::endl;
		}
		return;
	}
	cv::rectangle(image, bounding_box[0], bounding_box[1], cv::Scalar(255, 0, 0), 3, 8);
}

/**
 * @brief      Visualize the histogram
 *
 * @param[in]  hist         The hist
 * @param[in]  height       The height
 * @param[in]  window_name  The window name
 */
void StairDetectorGeo::visualizeHist(const std::vector<int> hist, int height, std::string window_name) {
	// visualize the histogram
	cv::Mat row;
	for (int i = 0; i < hist.size(); i++) {
		row.push_back(hist[i]);
	}
	row = row.t();

	cv::Mat tmp4;
	for (int i = 0; i < height; i++) {
		tmp4.push_back(row);
	}

	double min, max;
	cv::minMaxIdx(tmp4, &min, &max);
	cv::convertScaleAbs(tmp4, tmp4, 255 / max);
	cv::Mat tmp5;
	applyColorMap(tmp4, tmp5, cv::COLORMAP_AUTUMN);
	cv::imshow(window_name, tmp5);
}


/**
 * @brief      calculate the hough lines
 *
 * @param[in]  edge_image  The edge image
 * @param      lines       The lines
 */
void StairDetectorGeo::houghLine(const cv::Mat &edge_image, Lines &lines) {
	lines.clear();
	if (param_.hough_theta == 0) {
		param_.hough_theta = 1 * PI / 180;
	}
	if (param_.hough_rho == 0) {
		param_.hough_rho = 1;
	}

	std::vector<cv::Vec4i> xy_lines;
	HoughLinesP(edge_image, xy_lines,
	            (double)param_.hough_rho / 10,
	            (double)param_.hough_theta,
	            (int)	param_.hough_threshold,
	            (double)param_.hough_min_line_length,
	            (double)param_.hough_max_line_gap);

	for (int i = 0; i < xy_lines.size(); i++) {
		Line line(xy_lines[i]);
		line.calPixels(edge_image);
		lines.push_back(line);
	}
}

void StairDetectorGeo::filterLinesBySlopeThreshold(const Lines& input_lines, Lines& filtered_lines ) {
	double target_angle = 0;
	double buff_angle = param_.filter_slope_bandwidth;

	for (int i = 0; i < input_lines.size(); i++) {
		if ( std::abs(input_lines[i].t - target_angle) < buff_angle ) {
			filtered_lines.push_back(input_lines[i]);
		}
	}
}

void StairDetectorGeo::filterLinesBySlopeHist(const Lines& input_lines, Lines& filtered_lines ) {
	// calculate histogram
	std::vector<int> slope_hist;
	std::vector<std::vector<int>> slope_hist_list;
	// 10 degree
	double bin_width = param_.filter_slope_hist_bin_width;
	int bin_num = PI / bin_width;

	// initialize the histogram
	for (int i = 0; i < bin_num; i++) {
		slope_hist.push_back(0);
		std::vector<int> empty;
		slope_hist_list.push_back(empty);
	}

	// calculate slope_hist value
	for (int i = 0; i < input_lines.size(); i++) {
		double t = input_lines[i].t;
		if (t < 0) {
			t = t + PI;
		}
		if (t > PI) {
			t = t - PI;
		}
		// std::cout << xy_linesk << std::endl;
		int id = t / bin_width;
		if (id > bin_num) {
			id = 0;
		}
		if (id < 0) {
			id = bin_num;
		}
		slope_hist[id]++;
		slope_hist_list[id].push_back(i);
	}

	// print histogram value
	// for (int i = 0; i < slope_hist.size(); i++) {
	// 	std::cout << slope_hist[i] << std::endl;
	// }

	// calcuate the maximum frequency angle
	int max_id = std::distance(slope_hist.begin(), std::max_element(slope_hist.begin(), slope_hist.end()));
	if (param_.debug) {
		std::cout << "Maximum frequency is between angle: "
		          << max_id * bin_width * 180 / PI << " and "
		          << (max_id + 1) * bin_width * 180 / PI << std::endl;
	}

	// std::cout << "Which has totoal line: " << slope_hist_list[max_id].size() <<
	// std::endl;
	// extract the filtered lines

	// Lines filtered_lines;
	for (int i = 0; i < slope_hist_list[max_id].size(); i++) {
		int id = slope_hist_list[max_id][i];
		filtered_lines.push_back(input_lines[id]);
	}
}

/**
 * @brief      Gets the bounding box.
 *
 * @param[in]  input_lines   The input lines
 * @param      bounding_box  The bounding box
 *
 * @return     The bounding box.
 */
bool StairDetectorGeo::getBoundingBox(const cv::Mat input_rgb_image, const Lines &input_lines, std::vector<cv::Point>& bounding_box) {
	bounding_box.clear();
	if (input_lines.size() < 3) {
		if (param_.debug) {
			std::cout << "There is no stair because not much lines detected" << std::endl;
		}
		return false;
	}

	// step 1 filter lines by it's slope
	Lines filtered_lines;
	filterLinesBySlopeThreshold(input_lines, filtered_lines);
	if (filtered_lines.size() < 3) {
		if (param_.debug) {
			std::cout << "There is no stair because not much lines detected after filter" << std::endl;
		}
		return false;
	}
	// step 2 merge lines after filtering
	Lines merged_lines;
	mergeLines(input_rgb_image, filtered_lines, merged_lines);
	if (merged_lines.size() < 3) {
		if (param_.debug) {
			std::cout << "There is no stair because not much lines detected after merge" << std::endl;
		}
		return false;
	}

	// draw the lines
	cv::Mat line_mark = cv::Mat::zeros(input_rgb_image.rows, input_rgb_image.cols, CV_8UC1);
	// input_rgb_image.copyTo(tmp3);
	drawLines(line_mark, merged_lines, cv::Scalar(255, 255, 255));

	// calculate the histogram of lines passing each row
	std::vector<int> col_hist;
	// int bin_num = 20;
	for (int j = 0; j < line_mark.cols; j++) {
		int count = 0;
		for (int i = 0; i < line_mark.rows; i++) {
			unsigned char intensity = line_mark.at<uchar>(i, j);
			// check if the pixel is red
			if (intensity == 255) {
				count++;
				// std::cout << intensity << std::endl;
			}
		}
		col_hist.push_back(count);
		// std::cout << count << std::endl;
	}


	int max_frequency_col = std::distance(col_hist.begin(), std::max_element(col_hist.begin(), col_hist.end()));

	if (param_.debug) {
		cv::imshow("depth", input_rgb_image);
		cv::Mat tmp1;
		input_rgb_image.copyTo(tmp1);
		drawLines(tmp1, input_lines, cv::Scalar(0, 0, 255));
		cv::imshow("before filter", tmp1);
		cv::Mat tmp2;
		input_rgb_image.copyTo(tmp2);
		drawLines(tmp2, filtered_lines, cv::Scalar(0, 0, 255));
		cv::imshow("after filter before merge", tmp2);
		std::cout << "Maximum Frequency Column is " << max_frequency_col << std::endl;
	}

	// visualize the histogram
	// visualizeHist(col_hist, 200, "column histogram");

	// find out the left and right most
	int left_most = input_rgb_image.cols;
	int right_most = -1;
	std::vector<int> valid_ids;  // store all the lines that contain the frequency
	for (int i = 0; i < merged_lines.size(); i++) {
		for (int j = 0; j < merged_lines[i].pixels.size(); j++) {
			if (merged_lines[i].pixels[j].x == max_frequency_col) {
				int x1 = merged_lines[i].p1.x;
				int x2 = merged_lines[i].p2.x;
				if (x1 < left_most) { left_most = x1; }
				if (x1 > right_most) { right_most = x1; }
				if (x2 < left_most) { left_most = x2; }
				if (x2 > right_most) { right_most = x2; }
				valid_ids.push_back(i);
				break;
			}
		}
	}
	// find out the up and down most
	int up_most = input_rgb_image.rows;
	int down_most = -1;
	for (int i = 0; i < valid_ids.size(); i++) {
		int id = valid_ids[i];
		// ignore lines if it's length is smaller than 20
		if (merged_lines[id].length < 20) {
			continue;
		}
		for (int j = 0; j < merged_lines[id].pixels.size(); j++) {
			int x = merged_lines[id].pixels[j].x;
			int y = merged_lines[id].pixels[j].y;
			if (x < left_most || x > right_most) { break; }
			if (y < up_most) { up_most = y; }
			if (y > down_most) { down_most = y; }
		}
	}

	cv::Point p1(left_most, up_most);
	cv::Point p2(right_most, down_most);

	// filter lines by checking if line is inside the bounding box
	Lines final_lines;
	for (int i = 0; i < merged_lines.size(); i++) {
		bool bad_line = false;
		for (int j = 0; j < merged_lines[i].pixels.size(); j++) {
			int x = merged_lines[i].pixels[j].x;
			int y = merged_lines[i].pixels[j].y;
			if (x < left_most || x > right_most || y < up_most || y > down_most) {
				bad_line = true;
				break;
			}
		}
		if (!bad_line) {
			final_lines.push_back(merged_lines[i]);
		}
	}

	if (param_.debug) {
		cv::Mat tmp3;
		input_rgb_image.copyTo(tmp3);
		drawLines(tmp3, merged_lines, cv::Scalar(0, 0, 255));
		cv::rectangle(tmp3, p1, p2, cv::Scalar(255, 0, 0), 3, 8);
		cv::imshow("after filter after merge", tmp3);
	}

	if (param_.debug) {
		// show the final images
		cv::Mat tmp4;
		input_rgb_image.copyTo(tmp4);
		// draw the bounding box
		cv::rectangle(tmp4, p1, p2, cv::Scalar(255, 0, 0), 3, 8);
		drawLines(tmp4, final_lines, cv::Scalar(0, 0, 255));
		// plot the maximum frequency column
		cv::Point p3(max_frequency_col, 0);
		cv::Point p4(max_frequency_col, input_rgb_image.rows);
		cv::line(tmp4, p3, p4, cv::Scalar(0, 255, 255), 3, 8);
		drawLinesSlope(tmp4, final_lines, cv::Scalar(255, 255, 0));
		cv::imshow("Result", tmp4);
	}

	if (final_lines.size() < param_.minimum_line_num || (right_most - left_most) < 20) {
		if (param_.debug) {
			std::cout << "There is no a stair because final lines size are less than " << param_.minimum_line_num << std::endl;
		}
		return false;
	} else {
		// std::cout << "Found a stair" << std::endl;
		bounding_box.push_back(p1);
		bounding_box.push_back(p2);
		return true;
	}
}
/**
 * @brief      Gets the Neighbour pixel identifier.
 *
 * @param[in]  row   The row
 * @param[in]  col   The col
 * @param      ids   The identifiers
 *
 * @return     if it's success or not
 */
bool StairDetectorGeo::getNeighbourId(int row, int col, std::vector<cv::Vec2i> &ids, const cv::Mat& input_image) {
	if (input_image.rows <= 0 || input_image.cols <= 0) {
		std::cout << "getNeighbourId: rows and cols is not setuped" << std::endl;
		return false;
	}
	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {
			int x = col - i;
			int y = row - j;
			// ignore
			if (x < 0 || y < 0 || x > input_image.cols || y > input_image.rows) {
				continue;
			}
			// ignore itself
			if (x == col && y == row) {
				continue;
			}
			cv::Vec2i id(x, y);
			ids.push_back(id);
		}
	}
	return true;
}


/**
 * @brief      Determines if lines are close.
 *
 * @param[in]  line1  The line 1
 * @param[in]  line2  The line 2
 *
 * @return     True if close, False otherwise.
 */
bool StairDetectorGeo::isLineClose(const Line &line1, const Line &line2) {
	if (cv::norm(line1.p1 - line2.p1) < param_.merge_max_dist_diff ||
	        cv::norm(line1.p1 - line2.p2) < param_.merge_max_dist_diff ||
	        cv::norm(line1.p2 - line2.p1) < param_.merge_max_dist_diff ||
	        cv::norm(line1.p2 - line2.p2) < param_.merge_max_dist_diff) {
		return true;
	}

	int count = 0;
	for (int m = 0; m < line1.pixels_num; m++) {
		for (int n = 0; n < line2.pixels_num; n++) {
			// at leaset 10 pixel that are distance less than 5
			double dist = cv::norm(line1.pixels[m] - line2.pixels[n]);
			if (dist < param_.merge_max_dist_diff) {
				count++;
				if (count > param_.merge_close_count || count > line1.pixels_num || count > line2.pixels_num) {
					// std::cout << line1 << " " << line2 <<" close" <<std::endl;
					return true;
				}
				break;
			}
		}
	}
	return false;
}

/**
 * @brief      Determines if line parallel.
 *
 * @param[in]  line1  The line 1
 * @param[in]  line2  The line 2
 *
 * @return     True if line parallel, False otherwise.
 */
bool StairDetectorGeo::isLineParallel(const Line &line1, const Line &line2) {
	double angle_diff = line1.t - line2.t;
	// double dist_diff = line1.r - line2.r;
	// check if thier center is near;
	double dist_diff = (line1.center - line2.center).y ;
	angle_diff = std::abs(angle_diff);
	dist_diff = std::abs(dist_diff);
	// if (angle_diff > PI) {
	// 	angle_diff -= PI;
	// }
	// if (angle_diff < 0) {
	// 	angle_diff += PI;
	// }
	if (dist_diff > param_.merge_max_dist_diff) {
		return false;
	}
	if (angle_diff > param_.merge_max_angle_diff) {
		return false;
	}
	return true;

}

typedef std::vector<std::vector<cv::Point>> MergePointsList;
/**
 * @brief      Get the merge list form the input lines, every line that can be
 *             merged will be put in merge point list, any line that can't be
 *             merged will be put in pre_merge_lines
 *
 * @param[in]  input_lines        The input lines
 * @param      merge_points_list  The merge points list, type
 * std::vector<std::vector<cv::Point> >
 * @param      pre_merge_lines    The pre merge lines
 *
 * @return     void
 */
void StairDetectorGeo::getMergePointList(const Lines &input_lines,
        MergePointsList &merge_points_list,
        Lines &pre_merge_lines) {
	merge_points_list.clear();
	if (input_lines.size() < 1) {
		std::cout << "can't merge lines because no lines detected" << std::endl;
		return;
	}

	std::vector<int> candidate_id;
	// consctruct the KB lines and lines iterator and loop list
	for (int i = 0; i < input_lines.size(); i++) {
		candidate_id.push_back(i);
	}

	// find out which lines should be merged
	std::vector<std::vector<int>> merge_list;
	while (candidate_id.size() > 0) {
		std::vector<int> brother;
		// push_back itself first and remove it
		int id1 = candidate_id[0];
		brother.push_back(id1);
		candidate_id.erase(candidate_id.begin());
		// check if any candidate is brother
		for (std::vector<int>::iterator it2 = candidate_id.begin(); it2 != candidate_id.end();) {
			int id2 = *it2;
			if (isLineParallel(input_lines[id1], input_lines[id2])) {
				if (isLineClose(input_lines[id1], input_lines[id2])) {
					brother.push_back(id2);
					candidate_id.erase(it2);
				} else {
					++it2;
				}
			} else {
				++it2;
			}
		}
		merge_list.push_back(brother);
	}

	// std::cout << "before merge: " << input_lines.size() << " after: " <<
	// merge_list.size() << std::endl;

	// construct merge points list, every point is a cv::Point, every line is
	// constrcuted with multiple points, so every line
	// is std::vector<cv::Point>, the list of merging line is a vector of mat
	// std::vector<std::vector<cv::Point> > merge_points_list;
	for (int i = 0; i < merge_list.size(); i++) {
		// if only one line
		if (merge_list[i].size() == 1) {
			int id = merge_list[i][0];
			pre_merge_lines.push_back(input_lines[id]);
			continue;
		}

		std::vector<cv::Point> points;
		for (int j = 0; j < merge_list[i].size(); j++) {
			int id = merge_list[i][j];
			points.push_back(input_lines[id].p1);
			points.push_back(input_lines[id].p2);
		}
		merge_points_list.push_back(points);
	}
	// std::cout << "Lines needs to be merged: " << merge_points_list.size() << std::endl;
}

/**
 * @brief      merge all lines
 *
 * @param[in]  input_lines   The input XY lines
 * @param      merged_lines  The merged lines
 */
void StairDetectorGeo::mergeLines(const cv::Mat& input_image, const Lines &input_lines, Lines &merged_lines) {
	std::vector<std::vector<cv::Point>> merge_points_list;
	getMergePointList(input_lines, merge_points_list, merged_lines);

	int merge_count = 0;
	Lines tmp_merge_lines;
	Lines pre_merge_lines;
	while (merge_points_list.size() > 0) {
		merge_count++;
		if (param_.debug) {
			std::cout << merge_count << " merge iteration," << merge_points_list.size()
			          << " lines to be merged, " << tmp_merge_lines.size()
			          << " lines already merged." << std::endl;
		}
		// this will merge all points in merge_points_list
		for (int i = 0; i < merge_points_list.size(); i++) {
			cv::Vec4f merged_vline;
			// out put of fit line is (vx, vy, x0, y0), where (vx, vy) is a normalized
			// vector collinear to the line and (x0, y0) is a point on the line
			cv::fitLine(merge_points_list[i], merged_vline, CV_DIST_L2, 1.3, 0.01, 0.01);
			std::vector<double> distances;  // distance is the distance from the point
			// to the origin point
			cv::Vec2f dir(merged_vline[0], merged_vline[1]);
			cv::Vec2f origin(merged_vline[2], merged_vline[3]);
			// std::cout << dir << " " << origin << std::endl;
			for (int j = 0; j < merge_points_list[i].size(); j++) {
				// std::cout << merge_points_list[i][j] << std::endl;
				// pv stands for point vector
				cv::Vec2f pv((double)merge_points_list[i][j].x - origin[0],
				             (double)merge_points_list[i][j].y - origin[1]);
				double dist = pv.dot(dir);
				// std::cout << "Point Vecotor: " << pv <<" Direction: " << dir << " Dot
				// Product: " << dist << std::endl;
				distances.push_back(dist);
			}

			double min = *std::min_element(distances.begin(), distances.end());
			double max = *std::max_element(distances.begin(), distances.end());

			// std::cout << min << " " << max << std::endl;
			Line merged_line(origin[0] + min * dir[0], origin[1] + min * dir[1],
			                 origin[0] + max * dir[0], origin[1] + max * dir[1]);
			merged_line.calPixels(input_image);
			tmp_merge_lines.push_back(merged_line);
		}
		// Any line in tmp_merge_lines that can't be merged will be put inside
		// pre_merge_lines.
		// Any line that can be merged will be put inside merge_points_list
		getMergePointList(tmp_merge_lines, merge_points_list, pre_merge_lines);
		// clear temperary lines
		tmp_merge_lines.clear();
		// add the pre-merged lines into the tmp lines;
		tmp_merge_lines.insert(tmp_merge_lines.begin(), pre_merge_lines.begin(), pre_merge_lines.end());
		pre_merge_lines.clear();
	}
	if (param_.debug) {
		std::cout << "Merge finished" << std::endl;
	}
	merged_lines.insert(merged_lines.end(), tmp_merge_lines.begin(),
	                    tmp_merge_lines.end());
}

void StairDetectorGeo::ignoreInvalid(const cv::Mat& input_image, cv::Mat& filter_image) {
	// If the depth is 0(which is unknow), make it's surrounding edge zero also
	// std::cout << "Ignore Invalid" << std::endl;
	NeighbourFinder nf;
	for (int i = 0; i < input_image.rows; i++) {
		for (int j = 0; j < input_image.cols; j++) {
			if (input_image.at<uchar>(i, j) == 0) {
				nf.start(j, i);
				for (int k = 0; k < 8; k++) {
					std::pair<int, int> pos = nf.next();
					if (isInbound(pos.first, pos.second, filter_image)) {
						filter_image.at<uchar>(pos.second, pos.first) = 0;
					}
				}
			}
		}
	}
	return;
	// std::cout << "Finished Ignoreing" << std::endl;
}

/**
 * @function CannyThreshold
 * @brief      Trackbar callback - Canny thresholds input with a ratio_ 1:3
 *
 * @param[in]  src   The source image should be GraySclae
 * @param      edge  The out put edge image is Gray Scale
 */
void StairDetectorGeo::cannyEdgeDetection(const cv::Mat& input_image, cv::Mat &edge) {
	/// Reduce noise with a kernel 3x3
	cv::blur(input_image, edge, cv::Size(3, 3));
	/// Canny detector
	cv::Canny(edge, edge, (double) param_.canny_low_threshold, (double) param_.canny_low_threshold * param_.canny_ratio, param_.canny_kernel_size);
}


/**
 * @brief      use soble to do edge detection
 *
 * @param      edge  The edge
 */
void StairDetectorGeo::sobelEdgeDetection(const cv::Mat& input_image, cv::Mat &edge) {
	cv::blur(input_image, edge, cv::Size(3, 3));
	cv::Sobel( input_image, edge, CV_8U, 0, 1, 3);
	cv::convertScaleAbs(edge, edge);
}

/**
 * @brief      use laplacian to do edge detection
 *
 * @param      edge  The edge
 */
void StairDetectorGeo::laplacianEdgeDetection(const cv::Mat& input_image, cv::Mat &edge) {
	cv::GaussianBlur( input_image, edge, cv::Size( 3, 3 ), 0 , 0);
	cv::Laplacian( edge, edge, CV_8U, 3, 1, 0, cv::BORDER_DEFAULT );
	cv::convertScaleAbs(edge, edge);
}

/**
 * @brief      fill the invalid value by it's nerest neighbour
 *
 * @param      input_image   The input image
 * @param      filled_image  The filled image
 */
void StairDetectorGeo::fillByNearestNeighbour(const cv::Mat& input_image, cv::Mat& filled_image) {
	// cv::Mat filled_image_tmp;
	// filled_image_tmp = input_image.clone();
	NeighbourFinder nf;
	int count = 0;
	for (int i = 0; i < input_image.rows; i++) {
		for (int j = 0; j < input_image.cols; j++) {
			if (input_image.at<uchar>(i, j) == 0) {
				nf.start(j, i);
				count = 0;
				while (1) {
					std::pair<int, int> pos = nf.next();
					++count;
					if (!isInbound(pos.first, pos.second, input_image)) {
						continue;
					}
					// y is row, x is col
					unsigned char value = input_image.at<uchar>(pos.second, pos.first);
					if (value == 0) {
						continue;
					}
					if (count > 50) {
						break;
					}
					// using temperary image will cost a lot of time when the image has a lot of invalid pixel
					// filled_image_tmp.at<uchar>(i, j) = value;
					// using the same image will help but quality is bad.
					filled_image.at<uchar>(i, j) = value;
					break;
				}
			}
		}
	}
	// filled_image = filled_image_tmp;
}

/**
 * @brief      Determines if a pixel coordinate inbound of image.
 *
 * @param[in]  x     x coordinate
 * @param[in]  y     y coordinate
 *
 * @return     True if inbound, False otherwise.
 */
bool StairDetectorGeo::isInbound(int x, int y, const cv::Mat& image) {
	if (x < 0) {
		return false;
	}
	if (y < 0) {
		return false;
	}
	if (x >= image.cols) {
		return false;
	}
	if (y >= image.rows) {
		return false;
	}
	return true;
}