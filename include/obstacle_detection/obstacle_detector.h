#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cout; using std::endl;

#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H


class obstacle_detector{
private:
	double u_depth_res;
	double max_depth;
	int image_count;
public:
	obstacle_detector();
	obstacle_detector(double _u_depth_res, double _max_depth);
	void calculate_u_depth_map(const sensor_msgs::ImageConstPtr &depth_image_msg,
							   sensor_msgs::Image &u_depth_map_msg);

};

#endif