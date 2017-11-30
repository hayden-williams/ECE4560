# Stephen Williams and Edgardo Marchand
# Created 30 Nov 2017

# Homework 12
#!/usr/bin/env python
import roslib
import sys 
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 

class image_converter:
	# detecting orange

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)

	def callback(self,data):
		try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				#hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


				lower_orange = np.array([0,10,170], dtype = "uint8") #bgr
				upper_orange = np.array([120,130,255], dtype = "uint8")
				mask = cv2.inRange(cv_image, lower_orange, upper_orange)
				image = cv2.bitwise_and(cv_image,cv_image, mask= mask)
				#cv2.imshow('mask',mask)

				#greyImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
				#ret,thresh = cv2.threshold(grey,127,255,0)


				cv2.imshow('image',image)

				#cv2.imshow("color_camera_msg.jpg", cv_image)
				cv2.waitKey(3)
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