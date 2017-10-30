# Authors: Stephen Hayden Williams and Edgardo Marchand
# Date Created: 29 Oct 2017
# Date Revised: 29 Oct 2017

# Similar to goStraight.py, but it turns to a specific orientation while not moving forward
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python turninplace.py

import rospy
import roslib
from geometry_msgs.msg import Twist#, Pose
from nav_msgs.msg import Odometry
from cmath import *
#from tf2_msgs.msg import TFMessage
#import tf

class turninplace_userinput():
	zeroAngle = 10 # should never naturally be 10, this was to give bot time to get correct error
	thetaError = 0
	kTurn = 1.5
	#desiredAngle = -1-1*1j # use complex math, 90+45 clockwise
	#desiredAngle = 0-1*1j # use complex math, 90 clockwise
	#desiredAngle = 0+1*1j # use complex math, 90 counterclockwise
	# change desirgedAngle from complex to just requesting radians
	desiredAngle = pi/2
	def __init__(self):
		# initiliaze
		rospy.init_node('turninplace_userinput', anonymous=False)

		# tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")

		# What function to call when you ctrl + c    
		rospy.on_shutdown(self.shutdown)

		#rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
		#rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)
		rospy.Subscriber('odom',Odometry,self.Orientation)

		# may need rospy.spin(); 

		
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	 
		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		r = rospy.Rate(20);

		# Twist is a datatype for velocity
		move_cmd = Twist()
		# let's go forward at 0.2 m/s
		move_cmd.linear.x = 0.0
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0

		
		# as long as you haven't ctrl + c keeping doing...
		while not rospy.is_shutdown():
		
			if self.zeroAngle == 10:
				move_cmd.linear.x = 0.0
				move_cmd.angular.z = 0
			else:
				move_cmd.angular.z = self.kTurn*self.thetaError
			# publish the velocity
			self.cmd_vel.publish(move_cmd)
			# wait for 0.1 seconds (10 HZ) and publish again
			r.sleep()

	def Orientation(self,data):
		qz = data.pose.pose.orientation.z
		qw = data.pose.pose.orientation.w
		current = qw + qz*1j
		if self.zeroAngle == 10:
			self.zeroAngle = (qw + qz*1j)**2
			# at this point, zeroAngle is our 0
			self.zeroAngle = self.zeroAngle*(cos(self.desiredAngle)+sin(self.desiredAngle)*1j)
		else:
			error = self.zeroAngle/(current**2)
			self.thetaError = phase(error) # radians from 0, -pi to pi
		
		rospy.loginfo("theta = %f"%(self.thetaError))

	def shutdown(self):
		# stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)
 
if __name__ == '__main__':
	try:
		turninplace_userinput()
	except:
		rospy.loginfo("turninplace node terminated.")
