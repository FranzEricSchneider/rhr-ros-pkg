#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float.h>

#include <boost/circular_buffer.hpp>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>


/**
This code tracks the time frome 75% to 25% of a falling barometer value over time
It also records when failure occurs electrically
Eric Schneider, July 2014
**/


void blocked_callback(const std_msgs::Bool::ConstPtr& msg);
void cycle_done();
void tactile_callback(const std_msgs::Float::ConstPtr& msg);
float buff_max(boost::circular_buffer<float> buffer);
float buff_min(boost::circular_buffer<float> buffer);


bool last_blocked = false;
int cycle_counter = 0;
float start_time;
boost::circular_buffer<float> ring(1000);
float upper_cutoff_time;
float lower_cutoff_time;
bool in_ROI = false;
const float ROI_edge = 0.25;
float latest_max;
float latest_min;
float latest_falltime;

std::ofstream cycleFile;
std::ofstream falltimeFile;


// Called when the photogate data updates. If a gate is blocked and it wasn't previously, signals that a cycle finished
void blocked_callback(const std_msgs::Bool::ConstPtr& msg) {
	if ((msg->data == true) && (last_blocked == false))
		cycle_done();
	if (msg->data == true)
		last_blocked = true;
	else
		last_blocked = false;
}


void cycle_done() {
	latest_max = buff_max(ring);
	sensor_max_pub.publish(latest_max);
	ROS_INFO("\tThe MAX sensor value in the last cycle was %f", latest_max);

	latest_min = buff_min(ring);
	sensor_min_pub.publish(latest_min);
	ROS_INFO("\tThe MIN sensor value in the last cycle was %f", latest_min);

	ROS_INFO("There were %d cycles after %f seconds", ++cycle_counter, (ros::Time::now().toSec() - start_time));
	cycle_file << '\n' << cycle_counter << " cycles completed, they took " << (ros::Time::now().toSec() - start_time) << " seconds";
	
	latest_falltime = lower_cutoff_time - upper_cutoff_time;
	falltime_pub.publish(latest_falltime);
	ROS_INFO("\tThe latest falltime was %d ms", latest_falltime*1000);
	falltimeFile << "After "<< cycle_counter <<" cycles ("<< (ros::Time::now().toSec() - start_time) <<" seconds, the falltime was "<< latest_falltime*1000 <<" ms";
}


void tactile_callback(const std_msgs::Float::ConstPtr& msg) {
	float upper_cutoff = (1 - ROI_edge)*(latest_max - latest_min) + latest_min;
	float lower_cutoff = ROI_edge*(latest_max - latest_min) + latest_min;
	if (in_ROI == false && msg->data < upper_cutoff) {
		in_ROI = true;
		upper_cutoff_time = ros::Time::now().toSec();
		ROS_INFO("Identified an UPPER CUTOFF TIME at %f", upper_cutoff_time);
	}
	else if (in_ROI == true && msg->data < lower_cutoff) {
		in_ROI = false;
		lower_cutoff_time = ros::Time::now().toSec();
		ROS_INFO("Identified a LOWER CUTOFF TIME at %f", lower_cutoff_time);
	}
}


float buff_max(boost::circular_buffer<float> buffer) {
	float max = 0;
	for (int i=0; i<buffer.size(); i++) {
		if (buffer[i] > max)
			max = buffer[i];
	}
	return max;
}


float buff_min(boost::circular_buffer<float> buffer) {
	float min = 1000;
	for (int i=0; i<buffer.size(); i++) {
		if (buffer[i] < min)
			min = buffer[i];
	}
	return min;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "hysteresis_tracker");
	ros::NodeHandle n;
	start_time = ros::Time::now().toSec();

	cycleFile.open("/home/eon-alone/ros/src/rhr-ros-pkg/rhr_testbench/log/hys_cycleCounter.txt", std::ios::out|std::ios::trunc);
	falltimeFile.open("/home/eon-alone/ros/src/rhr-ros-pkg/rhr_testbench/log/hys_falltime.txt", std::ios::out|std::ios::trunc);

	ros::Publisher sensor_max_pub = n.advertise<std_msgs::Float>("/sensor_max", 1000);
	ros::Publisher sensor_min_pub = n.advertise<std_msgs::Float>("/sensor_min", 1000);
	ros::Publisher falltime_pub = n.advertise<std_msgs::Float>("/falltime", 1000);

	ros::Subscriber cycle_sub = n.subscribe("/is_blocked", 20, blocked_callback);
	// ros::Subscriber tactile_sub = n.subscribe("/tactile", 20, tactile_callback);		// Subscribe to tactile data


	cycleFile.close();
	falltimeFile.close();
	return 0;
}