#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <obstacle_detection/obstacle_detector.h>
#include <sensor_msgs/Image.h>

using std::cout; using std::endl;

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
				sensor_msgs::Image u_depth_map;
				if (not stop){
					od.calculate_u_depth_map(data, u_depth_map);
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