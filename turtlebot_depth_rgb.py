#!/usr/bin/env python
import roslib
import sys 
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:


	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

	def callback(self,data):
		try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				cv2.imshow("color_camera_msg.jpg", cv_image)
				cv2.waitKey(1)
				#print "image saved!"

		except CvBridgeError, e:
			print e


def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:

		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
		main(sys.argv)