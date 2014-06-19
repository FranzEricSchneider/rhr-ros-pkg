#include <ros/ros.h>
#include <rhr_visualization/Hand.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

ros::Publisher pub;
sensor_msgs::JointState joint_state;

/**
 * This function takes the Hand data published by the RightHandRobotics hand
 * and sends that information in joint format to an rviz visualizer
 */
void publish_to_rviz(const rhr_visualization::HandConstPtr& hand)
 {
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = -hand->finger[0].proximal;
  joint_state.position[1] = -hand->finger[1].proximal;
  joint_state.position[2] = hand->finger[2].proximal;
  joint_state.position[3] = -hand->finger[0].distal;
  joint_state.position[4] = -hand->finger[1].distal;
  joint_state.position[5] = -hand->finger[2].distal;
  joint_state.position[6] = hand->palm.preshape[0];
  joint_state.position[7] = -hand->palm.preshape[1];

  pub.publish(joint_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rhr_hand_visualizer");
  ros::NodeHandle n;
  pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  joint_state.name.resize(8);
  joint_state.position.resize(8);
  joint_state.name[0] ="prox1_joint";
  joint_state.name[1] ="prox2_joint";
  joint_state.name[2] ="prox3_joint";
  joint_state.name[3] ="dist1_joint";
  joint_state.name[4] ="dist2_joint";
  joint_state.name[5] ="dist3_joint";
  joint_state.name[6] ="pre1_joint";
  joint_state.name[7] ="pre2_joint";

  ros::Subscriber sub = n.subscribe("spoof_hand_data", 100, publish_to_rviz);

  ros::spin();
  return 0;
}