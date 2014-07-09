#include <ros/ros.h>
#include <reflex_msgs/Hand.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <string>

ros::Publisher pub;
sensor_msgs::JointState joint_state;
int num_fixed_steps = 5;
int num_flex_steps = 9;

/**
 * This function takes the Hand data published by the RightHandRobotics hand
 * and sends that information in joint format to an rviz visualizer
 */
 void publish_to_rviz(const reflex_msgs::HandConstPtr& hand)
 {
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = -hand->finger[0].proximal;
  joint_state.position[1] = -hand->finger[1].proximal;
  joint_state.position[2] = hand->finger[2].proximal;
  joint_state.position[3] = hand->palm.preshape;
  joint_state.position[4] = -hand->palm.preshape;

  int index = num_fixed_steps;
  for (int finger = 0; finger<3; finger++)
  {
    for (int i=0; i<(num_flex_steps+1); i++)
    {
      joint_state.position[index] = hand->finger[finger].distal/((float) (num_flex_steps+1));
      index++;
    }
  }

  pub.publish(joint_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rhr_hand_visualizer");
  ros::NodeHandle n;
  pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ROS_INFO("Number to resize: %d", num_fixed_steps + 3*(num_flex_steps+1));
  joint_state.name.resize(num_fixed_steps + 3*(num_flex_steps+1));
  joint_state.position.resize(num_fixed_steps + 3*(num_flex_steps+1));
  joint_state.name[0] ="prox1_joint";
  joint_state.name[1] ="prox2_joint";
  joint_state.name[2] ="prox3_joint";
  joint_state.name[3] ="pre1_joint";
  joint_state.name[4] ="pre2_joint";

  char buffer[50];
  int index = num_fixed_steps;
  for (int finger = 1; finger<4; finger++)
  {
    for (int i=1; i<(num_flex_steps+2); i++)
    {
      if (i == 1)
        sprintf(buffer, "finger[%d]/flex_joint_from_prox_to_1", finger);
      else if (i == (num_flex_steps+1))
        sprintf(buffer, "finger[%d]/flex_joint_from_%d_to_dist", finger, num_flex_steps);
      else
        sprintf(buffer, "finger[%d]/flex_joint_from_%d_to_%d", finger, i-1, i);
      joint_state.name[index] = buffer;
      index++;
    }
  }

  ros::Subscriber sub = n.subscribe("spoof_hand_data", 1, publish_to_rviz);

  ros::spin();
  return 0;
}