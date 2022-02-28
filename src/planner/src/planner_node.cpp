#include <iostream>
#include "ros/ros.h"
#include "planner/planner.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Planner planner;

	ros::init( argc, argv, "planner_node" );
	ros::NodeHandle node_handle;

	ros::Publisher joint_command_publishers[8];

	for(int i = 1; i < 9; i++) {
		string topicName = "rolling_robot/joint" + to_string(i) + "_position_controller/command";
		joint_command_publishers[i] = node_handle.advertise< std_msgs::Float64 >( topicName, 1);
	}

	double frequency = 10;
	ros::Rate timer( frequency );

	actuators.home();
	actuators.update_command_msgs();

	ros::Duration(2.0).sleep();

	while( ros::ok() ){

		actuators.update_command_msgs();
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
