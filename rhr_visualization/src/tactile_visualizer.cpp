#include <ros/ros.h>
#include <rhr_visualization/Hand.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

int main( int argc, char** argv );
void publish_to_rviz(const rhr_visualization::HandConstPtr& hand);
visualization_msgs::Marker makeContactMarker(float val, int id);
visualization_msgs::Marker makePressureMarker(float val, int id);
visualization_msgs::Marker makeMarker(int id);
void finger_tactile_positions(int index, double* x, double* z);

ros::Publisher pub;
const double contact_cutoff = 100;


int main( int argc, char** argv )
{
	ros::init(argc, argv, "tactile_visualizer");
	ros::NodeHandle n;
	pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
	ros::Subscriber sub = n.subscribe("spoof_hand_data", 1, publish_to_rviz);


	ros::spin();
}


/**
 * This function takes the tactile data published by the RightHandRobotics hand
 * and sends that information in joint format to an rviz visualizer
 **/
 // void publish_to_rviz(const rhr_visualization::HandConstPtr& hand)
 void publish_to_rviz(const rhr_visualization::HandConstPtr& hand)
 {
 	float pressure_val;
	visualization_msgs::MarkerArray marker_array;

 	for (int finger=0; finger<3; finger++)
 	{
 		char prox_fid[20];
 		sprintf(prox_fid, "/prox%d_tactile", (finger+1));
 		char dist_fid[20];
 		sprintf(dist_fid, "/dist%d_tactile", (finger+1));

 		for (int i=0; i<9; i++)
 		{
 			pressure_val = hand->finger[finger].pressure[i];
 			visualization_msgs::Marker contact_marker = makeContactMarker(pressure_val, i);
 			visualization_msgs::Marker pressure_marker = makePressureMarker(pressure_val, i);
 			if (i < 5) {		// Proximal link
 				contact_marker.header.frame_id = prox_fid;
 				pressure_marker.header.frame_id = prox_fid;
 			} else {			// Distal link
 				contact_marker.header.frame_id = dist_fid;
 				pressure_marker.header.frame_id = dist_fid;
 			}
 			contact_marker.id = i + (finger*9);
 			pressure_marker.id = i + (finger*9);
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

	if (val > contact_cutoff) {
		marker.scale.x = 0.004;
		marker.scale.y = 0.004;
		marker.scale.z = 0.005;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}
	else {
		marker.scale.x = 0.002;
		marker.scale.y = 0.002;
		marker.scale.z = 0.003;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}

	return marker;
}


visualization_msgs::Marker makePressureMarker(float val, int id)
{
	visualization_msgs::Marker marker = makeMarker(id);

	marker.ns = "pressure_markers";

	marker.scale.x = 0.008;
	marker.scale.y = 0.009;
	marker.scale.z = 0.002;

	val = 0.6*(val/-400) + 0.6;
    marker.color.r = val;		// Assuming 400 is the max sensor value, that could be replaced with some MAX value
    marker.color.g = val;
    marker.color.b = 1.0;

    return marker;
}


visualization_msgs::Marker makeMarker(int id)
{
	visualization_msgs::Marker marker;
	marker.header.stamp = ros::Time();
	marker.type = visualization_msgs::Marker::SPHERE;

	finger_tactile_positions(id, &marker.pose.position.x, &marker.pose.position.z);
	marker.pose.position.y = 0;
	marker.pose.position.y = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.color.a = 1.0;
	return marker;
}


void finger_tactile_positions(int index, double* x, double* z)
{
	double x_gap = 0.008;
	if (index < 5) {	// Proximal link
		*x = x_gap*index + 0.018;
		*z = 0.0145;
	}
	else {				// Distal link
		*x = x_gap*(index-5) + 0.022;
		*z = 0.009;
	}
}