# Stephen Williams and Edgardo Marchand
# Created 3 Dec 2017

# Homework 13
#!/usr/bin/env python
import roslib
import sys 
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
from geometry_msgs.msg import Twist

class image_converter:
	# detecting orange
	
	depth_image = 10

	def __init__(self):
		rospy.init_node('image_converter', anonymous=True)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
		self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,self.callback_depth)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.move_cmd = Twist()

		self.r = rospy.Rate(10)

	def callback(self,data):
		try:

				K = 0.005


				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				#hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


				lower_orange = np.array([0,10,170], dtype = "uint8") #bgr
				upper_orange = np.array([120,130,255], dtype = "uint8")
				self.mask = cv2.inRange(cv_image, lower_orange, upper_orange)
				image = cv2.bitwise_and(cv_image,cv_image, mask= self.mask)


				#cv2.imshow('image',image)
				#cv2.waitKey(5)

				
				# Convert BGR to GReyscale
				grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


				ret,thresh = cv2.threshold(grey,127,255,0)
				contours,hierarchy = cv2.findContours(thresh, 1, 2)
				cnt = contours[0]

				M = cv2.moments(self.mask)
				height, width, channels = image.shape #grey scale channel is 1, rgb is 3

				if ( M['m00'] > 0):
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					centerOfObject = (int(cx),int(cy))
					#cv2.circle(image,centerOfObject,10,(0,255,0),-1)
					#rospy.loginfo("in if statement in callback")

					dx = cx - width/2 # +ve move left, -ve move right?
					dy = cy - height/2

					#self.move_cmd.linear.x = 0.0015*(-1)*dy
					self.move_cmd.angular.z = K*(-1)*dx
					
					#distance = self.depth_image(cx,cy)
					print self.depth_image
					print "hello"
					self.move_cmd.linear.x = 0
				
				#rospy.loginfo("in callback")
				self.cmd_vel.publish(self.move_cmd)

				self.r.sleep()
				

				#cv2.imshow('mask',mask)

				#greyImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
				#ret,thresh = cv2.threshold(grey,127,255,0)


				

				#cv2.imshow("color_camera_msg.jpg", cv_image)
				#cv2.waitKey(3)
				#print "image saved!"

		except CvBridgeError, e:
			print e

	def callback_depth(self,data):
		# process depth data
		# mask contains the actual mask if it does not equal 10
		self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
		#self.depth_image = cv2.bitwise_and(cv_image,cv_image, mask= self.mask)


	
def main(args):
	ic = image_converter()
	#rospy.init_node('image_converter', anonymous=True)
	try:

		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
		main(sys.argv)