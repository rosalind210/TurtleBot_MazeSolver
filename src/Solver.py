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

	# variables to follow wall
	min_left = .05 # in meters
	max_left = .1
	# variables for checking front
	min_front = .07
	
	def __init__(self, sensor_topic):
		self.sensor_topic_name = sensor_topic
		move_forward_twist = Twist() # initialize move forward, will follow left wall
		turn_twist = Twist() # initialize turning, will turn when wall right in front
		move_forward_twist.linear.x = 0.5
		turn_twist.angular.z = 0.5
		## init publisher
		cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	# Publishes move forward or turn
	def move(direction):
		# set up distances
		left = direction[0]
		right = direction[1]
		int_front = direction[2]
		# check if we should move forward
		bool forward = __check_move__(in_front)
		# check if robot needs to adjust 
		if (forward && (left < min_left || left > max_left)) {
			__adjust__(left)
		} else if (forward) { # move forward 
			cmd_vel_pub(self.move_forward)
		} else if (!forward) { # have to turn
			cmd_vel_pub(self.turn_twist)
		}
		
	# adjust to follow left wall
	def __adjust__(distance):
		# establish diagonal movement (only necessary here -- why its not global or init)
		go_left = Twist()
		go_left.angular.z = -0.2
		go_left.linear.x = 0.2
		# actually move
		cmd_vel_pub(go_left)

	# check if there is a wall in front
	def __check_move__(front) :
		# check front
		if (front < min_front) {
			return false
		} else {
			return true
		}

	def read_sensors_callback(self, msgs):
		ranges[] = LaserScannerMsg.getRanges()
		#read / return information
		

	# should take in callback info and interpret it into forward, turn, out
	def read_scanners(ranges):
		left = ranges[255:285]
		right = ranges[75:105]
		front = zip(ranges[345:360],ranges[0:15])
		return [min(left),min(right),min(front)]



def main():
	rospy.get_node('solver')
	maze_solver = Solver("/scan")
	maze_solver.start()
		
