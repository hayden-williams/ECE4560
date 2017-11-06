# Authors: Stephen Hayden Williams and Edgardo Marchand
# Date Created: 5 Nov 2017
# Date Revised: 5 Nov 2017

#!/usr/bin/env python
import rospy
#import roslib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent

class Scan_msg():

	
	def __init__(self):
		'''Initializes an object of this class.
		The constructor creates a publisher, a twist message.
		3 integer variables are created to keep track of where obstacles exist.
		3 dictionaries are to keep track of the movement and log messages.'''
		rospy.init_node('navigation_sensors')
		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		#r = rospy.Rate(10);
		self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
		self.msg = Twist()
		self.sect_1 = 0 
		self.sect_2 = 0 
		self.sect_3 = 0 
		self.sect_4 = 0 
		self.sect_5 = 0 
		self.ang = {0:0,001:-1.2,10:-1.2,11:-1.2,100:1.2,101:-1.2,110:-1.2,111:-1.2,1000:1.2,1001:-1.2,1010:1.2,1011:-1.2,1100:1.2,1101:-1.2,1110:1.2,1111:-1.2,10000:1.2,10001:1.2,10010:1.2,10011:-1.2,10100:1.2,10101:1.2,10110:1.2,10111:-1.2,11000:1.2,11001:1.2,11010:1.2,11011:1.2,11100:1.2,11101:1.2,11110:1.2,11111:1.2}
		self.fwd = {0:.25,1:0,10:0,11:0,100:0,101:0,110:0,111:0,1000:0,1001:0,1010:0,1011:0,1100:0,1101:0,1110:0,1111:0,10000:0,10001:0,10010:0,10011:0,10100:0,10101:0,10110:0,10111:0,11000:0,11001:0,11010:0,11011:0,11100:0,11101:0,11110:0,11111:0}
		self.dbgmsg = {0:'Move forward',1:'Veer left',10:'Veer left',11:'Veer left',100:'Veer right',101:'Veer left',110:'Veer left',111:'Veer right',1000:'Veer right',1001:'Veer right',1010:'Veer right',1011:'Veer right',1100:'Veer right',1101:'Veer right',1110:'Veer right',1111:'Veer right',10000:'Veer right',10001:'Veer right',10010:'Veer right',10011:'Veer right',10100:'Veer right',10101:'Veer right',10110:'Veer right',10111:'Veer right',11000:'Veer right',11001:'Veer right',11010:'Veer right',11011:'Veer right',11100:'Veer right',11101:'Veer right',11110:'Veer right',11111:'Veer right'}

		#rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
		#rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)


	def reset_sect(self):
		'''Resets the below variables before each new scan message is read'''
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0
		self.sect_4 = 0
		self.sect_5 = 0

	def sort(self, laserscan):
		'''Goes through 'ranges' array in laserscan message and determines 
		where obstacles are located. The class variables sect_1, sect_2, 
		and sect_3 are updated as either '0' (no obstacles within 0.7 m)
		or '1' (obstacles within 0.7 m)
		Parameter laserscan is a laserscan message.'''
		entries = len(laserscan.ranges)
		for entry in range(0,entries):
			if 0.4 < laserscan.ranges[entry] < 0.75:
				self.sect_1 = 1 if (0 < entry < entries/5) else 0 
				self.sect_2 = 1 if (entries/5 < entry < 2*entries/5) else 0
				self.sect_3 = 1 if (2*entries/5 < entry < 3*entries/5) else 0
				self.sect_4 = 1 if (3*entries/5 < entry < 4*entries/5) else 0
				self.sect_5 = 1 if (4*entries/5 < entry < entries) else 0
		#rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3) + " sect_4: " + str(self.sect_4) + " sect_5: " + str(self.sect_5))

	def movement(self, sect1, sect2, sect3, sect4, sect5):
		'''Uses the information known about the obstacles to move robot.
		Parameters are class variables and are used to assign a value to
		variable sect and then	set the appropriate angular and linear 
		velocities, and log messages.
		These are published and the sect variables are reset.'''
		sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3) + str(self.sect_4) + str(self.sect_5))
		#rospy.loginfo("Sect = " + str(sect)) 
		
		self.msg.angular.z = self.ang[sect]
		self.msg.linear.x = self.fwd[sect]
		#rospy.loginfo(self.dbgmsg[sect])
		self.pub.publish(self.msg)

		self.reset_sect()

	def for_callback(self,laserscan):
		'''Passes laserscan onto function sort which gives the sect 
		variables the proper values.  Then the movement function is run 
		with the class sect variables as parameters.
		Parameter laserscan is received from callback function.'''
		self.sort(laserscan)
		self.movement(self.sect_1, self.sect_2, self.sect_3, self.sect_4, self.sect_5)

	def for_BumperEventCallback(self,data):
		if ( data.state == BumperEvent.PRESSED ) :
			self.msg.angular.z = 0
			self.msg.linear.x = 0
		
		rospy.loginfo("Bumper")

	def for_WheelDropEventCallback(self,data):
		if ( data.state == WheelDropEvent.DROPPED ) :
			self.msg.angular.z = 0
			self.msg.linear.x = 0
			  
		rospy.loginfo("Wheel")
	

def call_back(scanmsg):
	'''Passes laser scan message to for_callback function of sub_obj.
	Parameter scanmsg is laserscan message.'''
	rospy.loginfo("call_back")
	sub_obj.for_callback(scanmsg)

def BumperEventCallback(data):
	sub_obj.for_BumperEventCallback(data)

def WheelDropEventCallback(data):
	sub_obj.for_WheelDropEventCallback(data)

def listener():
	'''Initializes node, creates subscriber, and states callback 
	function.'''
	#rospy.init_node('navigation_sensors')
	rospy.loginfo("Subscriber Starting")
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,BumperEventCallback)
	rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,WheelDropEventCallback)
	sub = rospy.Subscriber('/scan', LaserScan, call_back)
	

	#sub = rospy.Subscriber('/scan', LaserScan, call_back)
	#sub = rospy.Subscriber('/mobile_base/sensors/bumper_pointcloud', LaserScan, call_back, queue_size=1)
	rospy.spin()

if __name__ == "__main__":
	'''A Scan_msg class object called sub_obj is created and listener
	function is run''' 
	sub_obj = Scan_msg()
	listener()