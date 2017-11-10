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
		self.image_sub2 = rospy.Subscriber("/camera/depth_registered/image_raw",Image,self.callback2)

	def callback(self,data):
		try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				#cv2.imshow("color_camera_msg.jpg", cv_image)
				cv2.waitKey(1)
				#print "image saved!"

		except CvBridgeError, e:
			print e

	def callback2(self,data):
		try:
				depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
				print depth_image
				print('[0,0] ' + str(float(depth_image)/1000) + ' millimetres.')
				#depth_array = array(depth_image, dtype=float32)
				#cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX)
				cv2.imshow("depth_camera_msg.jpg", depth_image)
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