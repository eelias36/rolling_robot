#include <iostream>
#include "planner/planner.h"

using namespace std;

Planner::Planner() {

}

Planner::~Planner() {

}

void Planner::handleGazeboState( const gazebo::LinkStates::ConstPtr& msg ) {
	_pose = msg->pose[1];
	return;
}
