#include <iostream>
#include "ros/ros.h"
#include "planner/planner.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Planner planner;

	ros::init( argc, argv, "planner_node" );
	ros::NodeHandle node_handle;

	ros::Subscriber link_states_subscriber = node_handle.subscribe( "/gazebo/link_states",1, &Planner::handleGazeboState, &planner );
	ros::Subscriber cmd_dir_subscriber = node_handle.subscribe( "/cmd_dir",1, &Planner::handle_cmd_dir, &planner );
	ros::Publisher faceState_publisher = node_handle.advertise< std_msgs::Int8 >( "rolling_robot/face_state", 1, true );
	ros::Publisher heading_publisher = node_handle.advertise< std_msgs::Float64 >( "/cmd_heading", 1, true );

	double frequency = 10;
	ros::Rate timer( frequency );

	while( ros::ok() ){
		faceState_publisher.publish( planner.faceState_msg() );
		heading_publisher.publish( planner.heading_msg() );

		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
