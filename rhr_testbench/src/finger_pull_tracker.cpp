#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <string>

/**
This code tracks the pull count of the finger pull testbench and notes the cycles
It also records when failure occurs electrically
Eric Schneider, July 2014
**/

void blockedCallback(const std_msgs::Bool::ConstPtr& msg);
void cycleDone();
void connectionCallback(const std_msgs::Bool::ConstPtr& msg);

bool last_blocked = false;
int cycleCounter = 0;
bool connected = true;
bool everFailed = false;
ros::Time start_time;

// const char *cyclePath="~/ros/src/rhr-ros-pkg/rhr_testbench/cycleCounter.txt";
// const char *connectPath="~/ros/src/rhr-ros-pkg/rhr_testbench/connectionFailed.txt";
// std::ofstream cycleFile(cyclePath);
// std::ofstream failFile(connectPath);
std::ofstream cycleFile;
std::ofstream failFile;

void blockedCallback(const std_msgs::Bool::ConstPtr& msg) {
	if ((msg->data == true) && (last_blocked == false))
		cycleDone();
	if (msg->data == true)
		last_blocked = true;
	else
		last_blocked = false;
}

void cycleDone() {
	ROS_INFO("The test is at %d cycles. Is wire still connected: %d", ++cycleCounter, connected);
	cycleFile << "Cycle number " << cycleCounter << " was completed after " << (ros::Time::now() - start_time) << " seconds \n";
}

void connectionCallback(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data == true)
		connected = true;
	else
		connected = false;

	if ((everFailed == false) && (connected == false)) {
		everFailed = true;
		ROS_WARN("The wire failed at %d cycles", cycleCounter);
		failFile << "After " << cycleCounter << " cycles the wire connection has failed. It took " << (ros::Time::now() - start_time) << " seconds \n";
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "finger_pull_tracker");
	ros::NodeHandle n;
	start_time = ros::Time::now();

	cycleFile.open("/home/eon-alone/ros/src/rhr-ros-pkg/rhr_testbench/cycleCounter.txt", std::ios::out|std::ios::trunc);
	failFile.open("/home/eon-alone/ros/src/rhr-ros-pkg/rhr_testbench/connectionFailed.txt", std::ios::out|std::ios::trunc);

	ros::Subscriber cycleSub = n.subscribe("/is_blocked", 20, blockedCallback);
	ros::Subscriber connectSub = n.subscribe("/is_connected", 20, connectionCallback);

	ros::spin();

	cycleFile.close();
	failFile.close();
	return 0;
}