#!/usr/bin/env python


# Plots hysteresis data being output by the labjack and by hysteresis_tracker.cpp
# Eric Schneider, July 2014


import rospy
from std_msgs.msg import Bool, Float64

import numpy as np
import matplotlib.pyplot as plt


class hysteresis_plotter():

	def __init__():
		topic = '/tactile'
		rospy.Subscribe(topic, Float64, self.tactile_callback)
		rospy.loginfo('hysteresis_plotter: Subscribed to %s topic', topic)
		self.tactile_count = 0

		topic = '/sensor_min'
		rospy.Subscribe(topic, Float64, self.min_callback)
		rospy.loginfo('hysteresis_plotter: Subscribed to %s topic', topic)
		self.min_count = 0

		topic = '/sensor_max'
		rospy.Subscribe(topic, Float64, self.max_callback)
		rospy.loginfo('hysteresis_plotter: Subscribed to %s topic', topic)
		self.max_count = 0

		topic = '/sensor_falltime'
		rospy.Subscribe(topic, Float64, self.falltime_callback)
		rospy.loginfo('hysteresis_plotter: Subscribed to %s topic', topic)
		self.falltime_count = 0

		# Initialize matplotlib graphs and name them
		self.x_range = 100

		# Tactile plot
		plt.figure(1)
		plt.xlabel('Data points taken')
		plt.ylabel('Tactile strength (unitless?)')
		plt.title('Compressed tactile data over time')

		plt.figure(2)
		plt.xlabel('Data points taken')
		plt.ylabel('Sensor min over the last cycle')
		plt.title('How the minimum of each cycle changes over time')

		plt.figure(3)
		plt.xlabel('Data points taken')
		plt.ylabel('Sensor max over the last cycle')
		plt.title('How the maximum of each cycle changes over time')

		plt.figure(4)
		plt.xlabel('Data points taken')
		plt.ylabel('Falltime of the last cycle (ms)')
		plt.title('Length of falltime for each cycle')

		plt.show()

	def tactile_callback(self, msg):
		plt.figure(1)
		# Plot to the correct graph
		if self.tactile_count < self.x_range:
			plt.axis([0, self.x_range, -100, 400])
		else:
			plt.axis([self.tactile_count, self.tactile_count+self.x_range, -100, 400])
		self.tactile_count += 1

	def min_callback(self, msg):
		plt.figure(2)
		# Plot to the correct graph
		
		self.min_count += 1

	def max_callback(self, msg):
		plt.figure(3)
		# Plot to the correct graph - reuse code?
		self.max_count += 1

	def falltime_callback(self, msg):
		plt.figure(4)
		# Plot to the correct graph - reuse code?
		self.falltime_count += 1


if __name__ == '__main__':
	hys_p = hysteresis_plotter()
	rospy.init_node('hysteresis_plotter')
	rospy.loginfo('hysteresis_plotter: started listening for data to plot')