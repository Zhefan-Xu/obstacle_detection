#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using std::cout; using std::endl;

#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H


class obstacle_detector{
private:
	double u_depth_res;
public:
	obstacle_detector();
	obstacle_detector(double _u_depth_res);
	void calculate_u_depth_map(const sensor_msgs::ImageConstPtr &depth_image,
							   sensor_msgs::Image &u_depth_map);

};

#endif