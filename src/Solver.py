#!/usr/bin/env python

# Author(s):	Rosalind Ellis (rnellis@brandeis.edu)
#		Melanie Kaplan-Cohen (melaniek@brandeis.edu)
#		Jonas Tjahjadi (jtjahjad15@brandeis.edu)
#
# Algorithm idea:
#	Follow the left most wall until completion
#
#When creating robot on simulator: Add Laser (laser_1), edit to make number of rays, angle span 360
#orientation 180. Add a sonar as well to see where the robot is facing on simulator
	
import rospy
#from stdr_msgs.msg import LaserSensorMsg
from sensor_msgs.msg import LaserScan #for real turtlebot
from Tkinter import *
from geometry_msgs.msg import Twist

#CMD: rosrun maze_solver Solver.py tb (or type any other single argument to use Simulation)
robotid = str(sys.argv[1])

if(robotid=="tb"):
	print("Turtlebot")
else:
	print("Simulation Robot")
	
class Solver():
	
	def __init__(self, sensor_topic):
		rospy.init_node('solver')
		self.sensor_topic_name = sensor_topic
		self.move_forward_twist = Twist() # initialize move forward, will follow left wall
		self.turn_twist = Twist() # initialize turning, will turn when wall right in front
		self.move_forward_twist.linear.x = 0.5
		self.turn_twist.angular.z = 0.5
		## init publisher
		self.cmd_vel_pub = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=1)
		# init subscriber and left forward right lists
		self.left = []
		self.front = []
		self.right = []
		
		# variables to follow wall
		self.min_left = .3 # in meters
		self.max_left = 2
		# variables for checking front
		self.min_front = .5

	# Publishes move forward or turn
	def move(self, direction):
		# set up distances
		left = direction[0]
		right = direction[1]
		in_front = direction[2]
		print("MOVE")
		print("left = " + str(left))
		print("right = " + str(right))
		print("in_front = " + str(in_front))
		# check if we should move forward
		forward = self.__check_move__(in_front)
		# check if robot needs to adjust 
		if (forward and (left < self.min_left or left > self.max_left)):
			self.__adjust__(left)
		elif (forward): # move forward 
			self.cmd_vel_pub.publish(self.move_forward_twist)
		elif (not forward): # have to turn
			self.cmd_vel_pub.publish(self.turn_twist)

	# adjust to follow left wall
	def __adjust__(self, distance):
		# establish diagonal movement (only necessary here -- why its not global or init)
		go_left = Twist()
		if (distance < self.min_left):
			go_left.angular.z = -0.02
		else:
			go_left.angular.z = 0.02
		go_left.linear.x = 0.02
		# actually move
		print("IN ADJUST")
		print(go_left)
		self.cmd_vel_pub.publish(go_left)

	# check if there is a wall in front
	def __check_move__(self, front):
		# check front
		if (front < self.min_front):
			return False
		else:
			return True

	def read_sensors_callback(self, msg):
		#read / return information
		#print("Range array has " + str(len(msg.ranges)) + " elements.")
		#print("Angle Increment is " + str(msg.angle_increment))
		#print(str(len(msg.ranges) * msg.angle_increment))
		# start movement by calling scanners to interpret information
		self.read_scanners(msg)
		

	# should take in callback info and interpret it into forward, turn, out
	def read_scanners(self, msg):
		self.left = min(msg.ranges[75:105])
		print("Min left " + str(self.left))
		self.right = min(msg.ranges[255:285])
		print("Min right " + str(self.right))
		self.front = min(msg.ranges[345:360] + msg.ranges[0:15])
		print("Min front " + str(self.front))
		# move given info
		self.move([self.left, self.right, self.front])

	def start(self):
		root = Tk()
		rospy.Subscriber(self.sensor_topic_name, LaserScan, self.read_sensors_callback)
		root.mainloop()

def main():
	rospy.init_node('solver')
	#initialize subscriber to specific topic depending on if it's running on turtlebot or simulator
	if(robotid == "tb"):
		maze_solver = Solver("/scan")
	else:
		maze_solver = Solver("/robot0/laser_1")
	maze_solver.start()
	#TODO - create shutdown method for when maze finishes


if __name__ == '__main__':
	main()
		
