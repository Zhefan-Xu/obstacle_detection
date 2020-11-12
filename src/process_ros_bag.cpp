#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <obstacle_detection/obstacle_detector.h>
#include <sensor_msgs/Image.h>

using std::cout; using std::endl;
cv::RNG rng(12345);

int main(int argcm, char** argv){
	obstacle_detector od;

	// Read Sensor Data from ros bag
	rosbag::Bag bag;
	bag.open("/home/zhefan/catkin_ws/src/obstacle_detection/rosbag/10_24.bag");

	int count_depth_image = 0;
	bool stop = false;
	for (rosbag::MessageInstance const m: rosbag::View(bag)){
		sensor_msgs::Image::ConstPtr data = m.instantiate<sensor_msgs::Image>();
		if (data != nullptr){
			std::string encoding = data->encoding;
			if (encoding == "16UC1"){
				
				// if (count_depth_image == 120){
				if (true){	
					cv::Mat u_depth_map = od.calculate_u_depth_map(data);
					cout << "image size: " << u_depth_map.rows << " x " << u_depth_map.cols << endl;
					std::vector<cv::Rect> bboxes = od.detect_u_depth_map(u_depth_map);
					// cv::Mat drawing = cv::Mat::zeros( u_depth_map.size(), CV_8UC3 );
					// cv::imwrite("/home/zhefan/catkin_ws/src/obstacle_detection/test_image_data/u_depth_detection/detect_image.jpg", u_depth_map);

					for (int i=0; i < bboxes.size(); ++i){
						cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
						cv::rectangle(u_depth_map, bboxes[i].tl(), bboxes[i].br(), color);
						
					}
					std::string file_name = "/home/zhefan/catkin_ws/src/obstacle_detection/test_image_data/u_depth_detection/detect_image" + std::to_string(count_depth_image) +".jpg";
					cv::imwrite(file_name, u_depth_map);
					
					// std::string windowName = "u depth map";

					// cv::imshow(windowName, u_depth_map);
					// cv::waitKey(0);

					// stop = true;	
				}
				


				++count_depth_image;
			}
		}
	}
	cout << "Total Depth Image: " << count_depth_image << endl;
	bag.close();








	return 0;
}