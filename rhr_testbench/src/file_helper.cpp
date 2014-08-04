/** 
These are helper functions for the rhr_testbench ROS package, they read the log files created
by various rhr_testbench applications
**/


#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
using namespace std;


// Extracts the last line of a file indicated by address, then tries to interpret the first word on that line as an integer
// The way the log files are set up, this should extract the previous number of cycle steps
int read_past_cycles(string address) {
	string buff;
	ifstream read_file(address.c_str());
	if (read_file.is_open()) {
		while ( getline (read_file, buff) ) {
			ROS_INFO("Loading old cycle data: \t%s", buff.c_str());
		}
		read_file.close();
	}
	else {
		ROS_INFO("There was no file to read past cycles from\n");
		return 0;
	}
	size_t space_pos = buff.find(" ");
	string clipped = buff.substr(0, space_pos);

	int result;
	try {
		result = atoi(clipped.c_str());
		ROS_INFO("Successfully read a previous cycle value of %d", result);
	}
	catch (int e) {
		ROS_INFO("The first characters in the line weren't integers, couldn't parse the number of previous cycles");
		return 0;
	}

	return result;
}