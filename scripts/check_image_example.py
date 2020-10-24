#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import rospkg
import numpy as np
import sys

np.set_printoptions(threshold=sys.maxsize)

rospack = rospkg.RosPack()
package_path = rospack.get_path('obstacle_detection')
save_path = os.path.join(package_path,"test_image_data/test_depth_image") 
os.chdir(save_path)


def callback(image_message):
	global frame_id

	cv_image =bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
	# cv2.imshow("Image window", cv_image)
	# cv2.waitKey(3)

	# Save image
	if (frame_id < 1000000000):
		image_array = np.array(cv_image, dtype=np.float32)
		cv2.normalize(image_array, image_array, 0, 1, cv2.NORM_MINMAX)
		cv2.imwrite("test_image_"+str(frame_id)+".jpg", image_array*255)
		frame_id += 1

	# data = image_message.data
	# height = image_message.height
	# width = image_message.width
	# image = data
	# image = np.array(data).reshape(height, width)
	# print(image[0])
	# cv_image = np.array(cv_image)
	# print(cv_image)

def main():
	rospy.init_node('depth_image_subscriber', anonymous=True)
	# Subscribe to depth image
	
	rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback)
	rospy.spin()

if __name__ == '__main__':
	frame_id = 0
	bridge = CvBridge()
	main()