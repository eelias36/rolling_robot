#include <iostream>
#include <string>
#include "ros/ros.h"
#include "actuator_control/actuator_control.h"

using namespace std;

int
main( int argc, char* argv[] ){
	Actuators actuators;

	ros::init( argc, argv, "actuators_node" );
	ros::NodeHandle node_handle;

	ros::Publisher joint_command_publishers[16];
	ros::Subscriber command_subscriber = node_handle.subscribe( "rolling_robot/commanded_vel", 1, &Actuators::handle_command, &actuators );
	ros::Subscriber faceState_subscriber = node_handle.subscribe( "rolling_robot/face_state", 1, &Actuators::handle_faceState, &actuators );

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
		actuators.actuator_position_update();
		actuators.update_command_msgs();
		for(int i = 0; i < 8; i++) {
			joint_command_publishers[i+1].publish( actuators.command_msgs[i] );
		}
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
