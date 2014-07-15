#include <ros/ros.h>
#include <reflex_msgs/Hand.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

int main( int argc, char** argv );
void publish_to_rviz(const reflex_msgs::HandConstPtr& hand);
visualization_msgs::Marker makeContactMarker(bool val, int id, float radius, float height, bool finger);
visualization_msgs::Marker makePressureMarker(float val, int id, float radius, float height, bool finger);
visualization_msgs::Marker makeFingerMarker(int id);
visualization_msgs::Marker makePalmMarker(int id);
void finger_tactile_positions(int index, double* x, double* z);

ros::Publisher pub;
const double contact_cutoff = 100;
const int sensors_per_finger = 9;

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
 void publish_to_rviz(const reflex_msgs::HandConstPtr& hand)
 {
 	bool contact_val;
 	float pressure_val;
 	visualization_msgs::MarkerArray marker_array;

 	for (int finger=0; finger<3; finger++)
 	{
 		char prox_fid[20];
 		sprintf(prox_fid, "/proximal_%d_tactile", (finger+1));
 		char dist_fid[20];
 		sprintf(dist_fid, "/distal_%d_tactile", (finger+1));

 		for (int i=0; i<sensors_per_finger; i++)		// Loop through tactile sensors in the fingers
 		{
 			contact_val = hand->finger[finger].contact[i];
 			pressure_val = hand->finger[finger].pressure[i];
 			visualization_msgs::Marker contact_marker = makeContactMarker(contact_val, i, 0.004, 0.005, true);
 			visualization_msgs::Marker pressure_marker = makePressureMarker(pressure_val, i, 0.008, 0.003, true);
 			if (i < 5) {		// Proximal link
 				contact_marker.header.frame_id = prox_fid;
 				pressure_marker.header.frame_id = prox_fid;
 			} else {			// Distal link
 				contact_marker.header.frame_id = dist_fid;
 				pressure_marker.header.frame_id = dist_fid;
 			}
 			contact_marker.id = i + (finger*sensors_per_finger);
 			pressure_marker.id = i + (finger*sensors_per_finger);
 			marker_array.markers.push_back(contact_marker);
 			marker_array.markers.push_back(pressure_marker);
 		}
 	}

 	for (int i=0; i<11; i++)		// Loop through tactile sensors in the palm
 	{
 		visualization_msgs::Marker contact_marker = makeContactMarker(contact_val, i, 0.004, 0.01, false);
 		visualization_msgs::Marker pressure_marker = makePressureMarker(pressure_val, i, 0.009, 0.008, false);
 		marker_array.markers.push_back(contact_marker);
 		marker_array.markers.push_back(pressure_marker);
 	}

 	pub.publish(marker_array);
 }


 visualization_msgs::Marker makeContactMarker(bool val, int id, float radius, float height, bool finger)
 {
 	visualization_msgs::Marker marker;
 	if (finger)
 		marker = makeFingerMarker(id);
 	else
 		marker = makePalmMarker(id);
 	marker.ns = "contact_markers";

 	if (val) {
 		marker.scale.x = radius;
 		marker.scale.y = radius;
 		marker.scale.z = height;
 		marker.color.r = 1.0;
 		marker.color.g = 0.0;
 		marker.color.b = 0.0;
 	}
 	else {
 		marker.scale.x = radius/2;
 		marker.scale.y = radius/2;
 		marker.scale.z = height-0.001;
 		marker.color.r = 1.0;
 		marker.color.g = 1.0;
 		marker.color.b = 0.0;
 	}
 	return marker;
 }


 visualization_msgs::Marker makePressureMarker(float val, int id, float radius, float height, bool finger)
 {
 	visualization_msgs::Marker marker;
 	if (finger)
 		marker = makeFingerMarker(id);
 	else
 		marker = makePalmMarker(id);
 	marker.ns = "pressure_markers";

 	marker.scale.x = radius;
 	marker.scale.y = radius*(4.0/5);
 	marker.scale.z = height;

 	val = 0.6*(val/-400) + 0.6;
    marker.color.r = val;		// Assuming 400 is the max sensor value, that could be replaced with some MAX value
    marker.color.g = val;
    marker.color.b = 1.0;
    return marker;
}


visualization_msgs::Marker makeFingerMarker(int id)
{
	visualization_msgs::Marker marker;
	marker.header.stamp = ros::Time();
	marker.type = visualization_msgs::Marker::SPHERE;

	finger_tactile_positions(id, &marker.pose.position.x, &marker.pose.position.z);
	marker.pose.position.y = 0;
	marker.pose.orientation.w = 1.0;
	marker.color.a = 1.0;
	return marker;
}


visualization_msgs::Marker makePalmMarker(int id)
{
	float pos1[3] = {-0.0235,	-0.0315, 	0.08};
	float pos2[3] = {-0.0235, 	0.0315, 	0.08};
	float pos3[3] = {0.026,		0, 			0.08};
	float x_gap = 0.028;
	float y_gap = 0.004;
	float x[11] = {pos1[0], pos1[0], pos1[0]+x_gap, pos1[0]+x_gap, pos2[0], pos2[0], pos2[0]+x_gap, pos2[0]+x_gap, pos3[0], pos3[0], pos3[0]+x_gap};
	float y[11] = {pos1[1], pos1[1]+y_gap, pos1[1], pos1[1]+y_gap, pos2[1], pos2[1]-y_gap, pos2[1], pos2[1]-y_gap, pos3[1]-0.5*y_gap, pos3[1]+0.5*y_gap, pos3[1]};
	float z[11] = {pos1[2], pos1[2], pos1[2], pos1[2], pos2[2], pos2[2], pos2[2], pos2[2], pos3[2], pos3[2], pos3[2]};

	visualization_msgs::Marker marker;
	marker.ns = "palm_markers";
	marker.header.stamp = ros::Time();
	marker.header.frame_id = "base_tactile";
	marker.id = id + 3*sensors_per_finger;
	marker.type = visualization_msgs::Marker::SPHERE;

	marker.pose.position.x = x[id];
	marker.pose.position.y = y[id];
	marker.pose.position.z = z[id];
	marker.pose.orientation.w = 1.0;
	marker.color.a = 1.0;

	return marker;
}


void finger_tactile_positions(int index, double* x, double* z)
{
	double x_gap = 0.008;
	if (index < 5) {	// Proximal link
		*x = x_gap*index + 0.016;
		*z = 0.0125;
	}
	else {				// Distal link
		*x = x_gap*(index-5) + 0.015;
		*z = 0.007;
	}
}