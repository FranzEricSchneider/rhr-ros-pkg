#include <ros/ros.h>
#include <rhr_visualization/Hand.h>
#include <cmath>
#include <string>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

void publish_to_rviz(const rhr_visualization::Hand hand);
visualization_msgs::Marker makeContactMarker(float val, int id);
visualization_msgs::Marker makePressureMarker(float val, int id);
visualization_msgs::Marker makeMarker(int id);
void finger_tactile_positions(int index, double* x, double* y);
int main( int argc, char** argv );

ros::Publisher pub;
const double contact_cutoff = 100;

/**
 * This function takes the tactile data published by the RightHandRobotics hand
 * and sends that information in joint format to an rviz visualizer
 */
void publish_to_rviz(const rhr_visualization::HandConstPtr& hand)
 {
 	visualization_msgs::MarkerArray marker_array;

 	for (int finger=0; finger<3; finger++)
 	{
 		char prox_fid[20];
 			sprintf(prox_fid, "/prox%d_tactile", (finger+1));
 		char dist_fid[20];
 			sprintf(dist_fid, "/dist%d_tactile", (finger+1));

 		for (int i=0; i<9; i++)
 		{
 			float pressure_val = hand->finger[finger].pressure[i];
 			visualization_msgs::Marker contact_marker = makeContactMarker(pressure_val, i);
 			visualization_msgs::Marker pressure_marker = makePressureMarker(pressure_val, i);
 			if (i < 5) {		// Proximal link
 				contact_marker.header.frame_id = prox_fid;
 				pressure_marker.header.frame_id = prox_fid;
 			} else {			// Distal link
 				contact_marker.header.frame_id = dist_fid;
 				pressure_marker.header.frame_id = dist_fid;
 			}
 			marker_array.markers.push_back(contact_marker);
 			marker_array.markers.push_back(pressure_marker);
 		}
 	}
  pub.publish(marker_array);
}


visualization_msgs::Marker makeContactMarker(float val, int id)
{
	visualization_msgs::Marker marker = makeMarker(id);

	marker.ns = "contact_markers";

	marker.scale.x = 0.002;
    marker.scale.y = 0.005;
    marker.scale.z = 0.002;

    marker.color.r = 0.0f;
    if (val > contact_cutoff) {
	    marker.color.g = 1.0f;
		marker.color.b = 0.0f;
	}
	else {
		marker.color.g = 0.0f;
		marker.color.b = 0.7f;
	}

    return marker;
}


visualization_msgs::Marker makePressureMarker(float val, int id)
{
	visualization_msgs::Marker marker = makeMarker(id);

	marker.ns = "pressure_markers";

	marker.scale.x = 0.007;
    marker.scale.y = 0.003;
    marker.scale.z = 0.007;

    marker.color.r = val/500 + 0.2;		// Assuming 400 is the max sensor value, that could be replaced with some MAX value
    marker.color.g = 0.0f;
    marker.color.b = 0.2f;

    return marker;
}


visualization_msgs::Marker makeMarker(int id)
{
	visualization_msgs::Marker marker;

	marker.header.stamp = ros::Time::now();
	marker.id = id;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(0.1);
	marker.type = visualization_msgs::Marker::SPHERE;

	finger_tactile_positions(id, &marker.pose.position.x, &marker.pose.position.y);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
  	marker.pose.orientation.y = 0.0;
  	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    return marker;
}

// double finger_tactile_positions(int index)
// void finger_tactile_positions(int index, double& x, double& y)
void finger_tactile_positions(int index, double* x, double* y)
{
	// double x;
	double x_gap = 0.008;
	if (index < 5) {	// Proximal link
		*x = x_gap*index + 0.015;
		*y = -0.01;
	}
	else {				// Distal link
		*x = x_gap*(index-5) + 0.006;
		*y = -0.005;
	}
	// return x;

}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "tactile_visualizer");
  ros::NodeHandle n;
  pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Subscriber sub = n.subscribe("spoof_hand_data", 100, publish_to_rviz);

  ros::spin();
}

