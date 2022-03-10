#include <iostream>
#include "ros/ros.h"
#include "uwb_interface/uwb_interface.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Uwb_interface uwb_interface;

	ros::init( argc, argv, "uwb_interface_node" );
	ros::NodeHandle node_handle;

	ros::Subscriber link_states_subscriber = node_handle.subscribe( "/gazebo/link_states",1, &Uwb_interface::handleGazeboState, &uwb_interface );
	ros::Publisher uwb_pos_publisher = node_handle.advertise< geometry_msgs::PoseArray >( "rolling_robot/uwb_poses", 1, true );

	double frequency = 10;
	ros::Rate timer( frequency );

	while( ros::ok() ){
		uwb_pos_publisher.publish( uwb_interface.uwb_pose_msg() );

		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
