#!/usr/bin/env python

# Author(s):	Rosalind Ellis (rnellis@brandeis.edu)
#		Melanie Kaplan-Cohen (melaniek@brandeis.edu)
#		Jonas Tjahjadi (jtjahjad15@brandeis.edu)
#
# Algorithm idea:
#	Follow the left most wall until completion
#

import rospy
from stdr_msgs.msg import LaserSensorMsg
#from sensor_msgs.msg import LaserScan #for real turtlebot
from Tkinter import *

class Solver():
	
	def __init__(self, sensor_topic):
		self.sensor_topic_name = sensor_topic
		move_forward_twist = Twist() # initialize move forward, will follow left wall
		turn_twist = Twist() # initialize turning, will turn when wall right in front
		move_forward_twist.linear.x = 0.5
		turn_twist.angular.z = 0.5
		## init publisher
		cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	
	def __move__(direction):
		# put move info here
		if (direction == "FORWARD") {
			cmd_vel_pub(self.move_forward)
		} else if (direction == "TURN") {
			cmd_vel_pub(self.turn_twist)
		} else if (direction == "OUT") {
			cmd_vel_pub(0) # stop moving
		}
		
	def read_sensors_callback(self, msgs):
		#read / return information
		

	# should take in callback info and interpret it into forward, turn, out
	def read_scanners():
		# put reading scanners here?

	def start():
		root = Tk()
		rospy.Subscriber(self.sensor_topic_name, LaserScannerMsg, self.read_sensors_callback)
		root.mainloop()

def main():
	rospy.get_node('solver')
	maze_solver = Solver("/scan")
	maze_solver.start()
		
