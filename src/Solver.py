#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from Tinter import *

class Solver():
	def __init__(self, scan_topic):
		self.scan_topic_name = scan_topic
	
	def __move__():
		# put move info here

	def read_scanners():
		# put reading scanners here?

	def start():
		root = Tk()
		rospy.Subscriber(self.scan_topic_name, LaserScan, self.scanCallback)
		root.mainloop()

def main():
	rospy.get_mode('solver')
	maze_solver = Solver("/scan")
	maze_solver.start()
		
