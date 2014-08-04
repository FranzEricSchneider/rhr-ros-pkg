#!/usr/bin/env python

# Publishes labjack information of use to the RHR testbench
# Right now it only provides data for the finger pull test
# Eric Schneider, July 2014
# Labjack code explained here: http://labjack.com/support/labjackpython/modbus

import rospy
from std_msgs.msg import Bool, Float64

import u6
from math import pi


# Initialize the Labjack
try:
	d = u6.U6()
except:
	rospy.logerr("No Labjack device was connected, exiting")
	exit()

# Analog read variables
AIN_REGISTER = 0							# Allows reading from AIN
numReg = 8

# Photogate setup
photogate_cutoff_v = 0.25
PHOTOGATE = rospy.get_param("cycle_AIN")	# The photogate output is different for finger puller/compressor

# Wire connectivity setup
track = rospy.get_param("track_connectivity")
if track:
	connectivity_cutoff = 1.0
	CONNECTIVITY = 0						# The connectivity output should be at AIN0


def publish_labjack():
	topic = 'is_blocked_%d'%PHOTOGATE
	blocked_pub = rospy.Publisher(topic, Bool, queue_size=100)					# Boolean
	topic = 'motor_angle_%d'%PHOTOGATE
	angle_pub = rospy.Publisher('motor_angle', Float64, queue_size=10)			# Radians
	if track:
		connect_pub = rospy.Publisher('is_connected', Bool, queue_size=100)		# Boolean
	r = rospy.Rate(rospy.get_param("labjack_rate"))

	start_time = rospy.get_time()
	end_time = rospy.get_time()
	state = True
	last_state = True

	while not rospy.is_shutdown():
		state = check_gate_blocked()
		rospy.loginfo("labjack_publisher: The photogate is open: %s", state)
		blocked_pub.publish(state)

		now = rospy.get_time()
		angle = (((now - start_time) / (end_time - start_time)) * 2*pi) % (2*pi)
		rospy.loginfo("labjack_publisher: The motor is at %f radians from the photogate intersection point", angle)
		angle_pub.publish(angle)

		if last_state and not state:
			start_time = end_time
			end_time = now
		last_state = state

		if track:
			connect_state = check_connectivity()
			rospy.loginfo("labjack_publisher: The finger wire is still connected: %s", connect_state)
			connect_pub.publish(connect_state)

		r.sleep()


def check_gate_blocked():
	v_level = d.readRegister(AIN_REGISTER, numReg=numReg)[PHOTOGATE]
	if v_level > photogate_cutoff_v:
		return False
	else:
		return True


def check_connectivity():
	v_level = d.readRegister(AIN_REGISTER, numReg=numReg)[CONNECTIVITY]
	if v_level > connectivity_cutoff:
		return True
	else:
		return False


if __name__ == '__main__':
	rospy.init_node('labjack_publisher')
	publish_labjack()
	rospy.loginfo("labjack_publisher: Started publishing labjack data")