#include <obstacle_detection/obstacle_detector.h>


obstacle_detector::obstacle_detector(){
	cout << "Initializing default Obstacle Detector!" << endl;
	u_depth_res = 50; // res mm per bin
	max_depth = 5000;
	image_count = 0; // for saving image
	cutoff_thresh = 0.3;
}

obstacle_detector::obstacle_detector(double _u_depth_res, 
									 double _max_depth
									 )
{
	cout << "Initializing custom Obstacle Detector!" << endl;
	u_depth_res = _u_depth_res;
	max_depth = _max_depth;
}

bool obstacle_detector::is_same_object_line(cv::Point l_s1, cv::Point l_e1, cv::Point l_s2, cv::Point l_e2){
	if (std::abs(l_s1.x - l_s2.x) == 0){ 
		int pixel_tolerance = 10;
		if (std::abs(l_s2.y-l_e1.y) <= pixel_tolerance){
			return true;
		}
		else{
			return false;
		}
	}
	else if (std::abs(l_s1.x - l_s2.x) > 1){ // Not consecutive depth
		return false;
	}
	else{
		int col_s1 = l_s1.y;
		int col_e1 = l_e1.y;
		int col_s2 = l_s2.y;
		int col_e2 = l_e2.y;

		// There are 4 conditions: if one is satisfied, then they are considered the same obejct
		// int pixel_tolerance = 10;
		bool condition = (col_s1 <= col_e2  and col_s1 >= col_s2) or
						 (col_s2 <= col_e2  and col_s2 >= col_s2) or
						 (col_s1 <= col_s2  and col_s2 >= col_e2) or
						 (col_s1 >= col_s2  and col_s2 <= col_e2);
		return condition;
	} 
}

cv::Mat obstacle_detector::calculate_u_depth_map(const sensor_msgs::ImageConstPtr& depth_image_msg)

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
	cout << "Finish Processing U-depth Map!" << endl;
	return u_depth_map;
}

std::vector<cv::Point>  obstacle_detector::detect_u_depth_map(const cv::Mat & u_depth_map){ 
	cout << "Detecting U-depth Map..." << endl;
	// Group Lines of Interests
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(u_depth_map, &minVal, &maxVal, &minLoc, &maxLoc);
	std::vector<cv::Point> lines_of_interests;
	bool first_time = true;
	int horizontal_distance_thresh = 5;
	int last_row = 0;
	int last_col = 0;
	for (int row=0; row != u_depth_map.rows; ++row){
		for (int col=0; col != u_depth_map.cols; ++col){
			int pixel_val = u_depth_map.at<int>(row, col);
			if (pixel_val > cutoff_thresh * maxVal){
				// cout << "pos: " << "(" << row << ", " << col << ")" << "  num: " << pixel_val << endl;
				if (first_time){
					last_row = row;
					last_col = col;
					cv::Point p_start;
					p_start.x = row;
					p_start.y = col;
					lines_of_interests.push_back(p_start);
					first_time = false;
				}
				else{
					if (row == last_row){
						int horizontal_distance = std::abs(col - last_col);
						// cout << horizontal_distance << endl;
						if (horizontal_distance >= horizontal_distance_thresh){
							cv::Point p_start, p_end;
							
							p_end.x = last_row;
							p_end.y = last_col;
							lines_of_interests.push_back(p_end);
		
							p_start.x = row;
							p_start.y = col;
							lines_of_interests.push_back(p_start);
						}
					}
					else{
						cv::Point p_start, p_end;

						p_end.x = last_row;
						p_end.y = last_col;
						lines_of_interests.push_back(p_end);

						p_start.x = row;
						p_start.y = col;;
						lines_of_interests.push_back(p_start);
					}
					last_row = row;
					last_col = col;
					
				}
			}
		}
	}

	for (cv::Point p: lines_of_interests){
		cout << p << endl;
	}

	// Group Lines
	
	std::vector<std::set<int>> line_group_indices;
	// std::set<int> a;
	// line_group_indices.push_back(a);
	// line_group_indices[0].insert(1);
	// line_group_indices[0].insert(2);
	// line_group_indices[0].insert(3);
	// line_group_indices[0].insert(4);
	// for (int index: line_group_indices[0]){
	// 	cout << index << endl;
	// }
	// std::vector<cv::Point> b;
	// return b;
	for (int i=0; i < lines_of_interests.size()/2 - 1; ++i){
		
		// check whether the line is in one of the group
		bool has_group = false;
		for (int itr=0; itr < line_group_indices.size(); itr++){
			bool is_in = line_group_indices[itr].find(i) != line_group_indices[itr].end();
			if (is_in){
				has_group = true;
				break;
			}
			if (has_group){break;}
		}

		// If it does not have any group, find group
		bool find_group = false;
		if (not has_group){
			cv::Point l_s1 = lines_of_interests[i*2];
			cv::Point l_e1 = lines_of_interests[i*2+1];
			for (int itr=0; itr < line_group_indices.size(); itr++){
				for (int index: line_group_indices[itr]){
					cv::Point l_s2 = lines_of_interests[index*2];
					cv::Point l_e2 = lines_of_interests[index*2+1];

					bool is_same_group = is_same_object_line(l_s1, l_e1, l_s2, l_e2);

					if (is_same_group){
						find_group = true;
						line_group_indices[itr].insert(i);
						break;
					}
				}
				if (find_group){break;}
			}
		}

		// if there is no group the line belongs to, it start a new group
		if (not find_group){
			std::set<int> new_group;
			new_group.insert(i);
			line_group_indices.push_back(new_group);
		}

	}

	int count_group = 0;
	for (std::set<int> group_indices: line_group_indices){
		++count_group;
		for (int index: group_indices){
			cout << "(" <<lines_of_interests[index*2] << lines_of_interests[index*2+1] << ")" << " "; 
			// cout << index;
		}
		cout << endl;
	}



	cout << "total: " << count_group << endl;
	std::vector<cv::Point> bboxes;

	cout << "Finish Detecting U-depth Map!" << endl;
	return bboxes;
}