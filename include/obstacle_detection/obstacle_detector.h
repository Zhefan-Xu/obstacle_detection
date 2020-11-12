#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <set>

using std::cout; using std::endl;

#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H


class obstacle_detector{
private:
	double u_depth_res;
	double max_depth;
	int image_count;
	double cutoff_thresh;
public:
	obstacle_detector();
	obstacle_detector(double _u_depth_res, double _max_depth);
	bool is_same_object_line(cv::Point l_s1, cv::Point l_e1, cv::Point l_s2, cv::Point l_e2);
	cv::Mat calculate_u_depth_map(const sensor_msgs::ImageConstPtr &depth_image_msg);
	std::vector<cv::Rect>  detect_u_depth_map(const cv::Mat & u_depth_map);

};

#endif