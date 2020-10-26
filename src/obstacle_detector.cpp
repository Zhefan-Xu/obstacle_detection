#include <obstacle_detection/obstacle_detector.h>


obstacle_detector::obstacle_detector(){
	cout << "Initializing default Obstacle Detector!" << endl;
	u_depth_res = 10; // 10 mm per bin
}

obstacle_detector::obstacle_detector(double _u_depth_res){
	cout << "Initializing custom Obstacle Detector!" << endl;
	u_depth_res = _u_depth_res;
}

void obstacle_detector::calculate_u_depth_map(const sensor_msgs::ImageConstPtr& depth_image,
											  sensor_msgs::Image &u_depth_map)

{
	cout << "Calculating U-depth Map..." << endl;
	// Convert the depth image into opencv format
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::MONO16);
}