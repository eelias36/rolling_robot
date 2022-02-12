#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "actuator_control/actuator_control.h"

using namespace std;

Actuators::Actuators() {

}

Actuators::~Actuators() {

}

void Actuators::update_command_msgs(void) {
	for(int i=0;i<11;i++) {
		command_msgs[i].data = (float) _commanded_pos[i];
	}
}

void Actuators::update_commands(void) {
	if ( _commanded_pos[0] == 0 ) {
		_commanded_pos[0] = 1;
	} else {
		_commanded_pos[0] = 0;
	}

}
