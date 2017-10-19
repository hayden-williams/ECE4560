# Authors: Stephen Hayden Williams and Edgardo Marchand
# Date Created: 18 Oct 2017
# Date Revised: 18 Oct 2017

# A very basic TurtleBot script that moves TurtleBot forward, bumper paused the movement for 2 sec. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goStraight.py

import rospy
import roslib
from geometry_msgs.msg import Twist#, Pose
from nav_msgs.msg import Odometry
from cmath import *
#from tf2_msgs.msg import TFMessage
#import tf

class GoStraight():
	desired = 10
	thetaError = 0
	kTurn = 20
	def __init__(self):
		# initiliaze
		rospy.init_node('GoStraight', anonymous=False)

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
		r = rospy.Rate(10);

		# Twist is a datatype for velocity
		move_cmd = Twist()
		# let's go forward at 0.2 m/s
		move_cmd.linear.x = 0.0
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0

		
		# as long as you haven't ctrl + c keeping doing...
		while not rospy.is_shutdown():
		
			if self.desired == 10:
				move_cmd.linear.x = 0.0
				move_cmd.angular.z = 0
			else:
				move_cmd.linear.x = 0.2
				move_cmd.angular.z = self.kTurn*self.thetaError
			# publish the velocity
			self.cmd_vel.publish(move_cmd)
			# wait for 0.1 seconds (10 HZ) and publish again
			r.sleep()

	def Orientation(self,data):
		qz = data.pose.pose.orientation.z
		qw = data.pose.pose.orientation.w
		current = qw + qz*1j
		if self.desired == 10:
			self.desired = (qw + qz*1j)**2
		else:
			error = self.desired/(current**2)
			self.thetaError = phase(error)
		#desired = 1 + 0*1j
		#thetaDesired = 0
		#desired = cos(thetaDesired)+sin(thetaDesired)*1j
		#error = desired/current
		#thetaError = phase(error)
		#rospy.loginfo("qz: %f qw: %f"%(qz, qw))
		
		#thetaZ = qz/sqrt(1-(qw*qw))
		#euler = self.tf.transformations.euler_from_quaternion(quaternion)
		#yaw = euler[2]
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
		GoStraight()
	except:
		rospy.loginfo("GoStraight node terminated.")
