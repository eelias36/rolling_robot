#include <iostream>
#include "ros/ros.h"
#include "planner/planner.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Planner planner;

	ros::init( argc, argv, "planner_node" );
	ros::NodeHandle node_handle;

	//ros::Subscriber link_states_subscriber = node_handle.subscribe( "/gazebo/link_states",1, &Planner::handleGazeboState, &planner );
	ros::Subscriber position_subscriber = node_handle.subscribe( "/position",1, &Planner::handlePosition, &planner );
	ros::Subscriber orientation_subscriber = node_handle.subscribe( "/imu/data",1, &Planner::handleOrientation, &planner );
	ros::Subscriber roll_to_goal_subscriber = node_handle.subscribe( "/roll_to_goal",1, &Planner::handle_roll_to_goal_msg, &planner );
	//ros::Subscriber cmd_dir_subscriber = node_handle.subscribe( "/cmd_dir",1, &Planner::handle_cmd_dir, &planner );
	ros::Subscriber rolling_subscriber = node_handle.subscribe( "/rolling_state",1, &Planner::handle_rolling_msg, &planner );
	ros::Subscriber goal_subscriber = node_handle.subscribe( "/goal",1, &Planner::handle_goal_msg, &planner );
	ros::Publisher faceState_publisher = node_handle.advertise< std_msgs::Int8 >( "rolling_robot/face_state", 1, true );
	planner.heading_publisher = node_handle.advertise< std_msgs::Float64 >( "/cmd_heading", 1, true );
	ros::Publisher cmd_vel_publisher = node_handle.advertise< geometry_msgs::Twist >( "/cmd_vel", 1, true );

	double frequency = 10;
	ros::Rate timer( frequency );
	sleep(1);

	while( ros::ok() ){
		faceState_publisher.publish( planner.faceState_msg() );
		cmd_vel_publisher.publish( planner.command_msg() );
		//heading_publisher.publish( planner.heading_msg() );

		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
