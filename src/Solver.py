#!/usr/bin/env python

import rospy
from stdr_msgs.msg import LaserSensorMsg
#from sensor_msgs.msg import LaserScan #for real turtlebot
from Tkinter import *

class Solver():
	def __init__(self, solver):
		self.sensor_topic_name = solver
	
	def __move__():
		# put move info here
		
	def read_sensors_callback(self, msgs):
		#read / return information
		
		
	def start():
		root = Tk()
		rospy.Subscriber(self.sensor_topic_name, LaserScannerMsg, self.read_sensors_callback)
		root.mainloop()

def main():
	rospy.get_mode('solver')
	maze_solver = Solver("/scan")
	maze_solver.start()
		
