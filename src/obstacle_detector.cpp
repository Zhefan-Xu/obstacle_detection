#include <obstacle_detection/obstacle_detector.h>


obstacle_detector::obstacle_detector(){
	cout << "Initializing default Obstacle Detector!" << endl;
	u_depth_res = 10; // res mm per bin
	max_depth = 5000;
	image_count = 0; // for saving image
}

obstacle_detector::obstacle_detector(double _u_depth_res, 
									 double _max_depth
									 )
{
	cout << "Initializing custom Obstacle Detector!" << endl;
	u_depth_res = _u_depth_res;
	max_depth = _max_depth;
}

void obstacle_detector::calculate_u_depth_map(const sensor_msgs::ImageConstPtr& depth_image_msg,
											  sensor_msgs::Image &u_depth_map_msg)

{
	cout << "Calculating U-depth Map..." << endl;
	// Convert the depth image into opencv format
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_image_msg);
	cv::Mat depth_image = cv_ptr->image;
	int rows = depth_image.rows, cols = depth_image.cols;

	// Initialize the U-depth map
	int u_depth_rows = (int) max_depth/u_depth_res;
	cv::Mat u_depth_map = cv::Mat::zeros(cv::Size(cols, u_depth_rows), CV_32S);
	// u_depth_map = 0;


	// Iterate through the image:
	for (int row=0; row != rows; ++row){
		for (int col=0; col != cols; ++col){
			int pixel_val = depth_image.at<ushort>(row, col);
			if (pixel_val > 0 and pixel_val < max_depth){
			 	// incremnet the value into each bin
				int bin_idx = (int) pixel_val/u_depth_res;
				// cout << u_depth_map.at<int>(bin_idx, col) << endl;

				u_depth_map.at<int>(bin_idx, col) += 1;

			}
		}
	}

	cout << "Finish Processing!" << endl;
	// cout << "here" << endl;
	// cout << u_depth_map << endl;

	// Display image
	// cv::imshow("Image window", u_depth_map*1000);
	// cv::imshow("Image window", depth_image);
	// cv::waitKey(3);

	// Write image
	// std::string path = "/home/zhefan/catkin_ws/src/obstacle_detection/test_image_data/u_depth_map/";
	// std::string filename = "u_depth_" + std::to_string(image_count) + ".jpg";
	// cv::Mat output_map;
	// cv::normalize(u_depth_map, output_map, 0, 255, cv::NORM_MINMAX);
	// cv::imwrite(path+filename, output_map);
	// ++image_count;
}