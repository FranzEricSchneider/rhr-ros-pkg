#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;

int main(int argc, char** argv) {
	ros::init(argc, argv, "output_angles");
	ros::NodeHandle n;
	joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Rate loop_rate(20);

	joint_state.name.resize(6);
	joint_state.position.resize(6);
	joint_state.name[0] ="base_to_prox1";
	joint_state.name[1] ="base_to_prox2";
	joint_state.name[2] ="base_to_prox3";
	joint_state.name[3] ="prox1_to_dist1";
	joint_state.name[4] ="prox2_to_dist2";
	joint_state.name[5] ="prox3_to_dist3";

	float i = 0;
	float ang = 0;
	while (ros::ok()) {
    //update joint_state
		joint_state.header.stamp = ros::Time::now();
		joint_state.position[0] = -ang;
		joint_state.position[1] = ang;
		joint_state.position[2] = ang;
		joint_state.position[3] = -ang*0.5;
		joint_state.position[4] = ang*0.5;
		joint_state.position[5] = ang*0.5;
		ang = (1.4 * sin(i/100)) + 1.4;
		i++;

    //send the joint state and transform
		joint_pub.publish(joint_state);
		loop_rate.sleep();
	}
	return 0;
}