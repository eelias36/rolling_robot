#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "actuator_control/actuator_control.h"

using namespace std;

Actuators::Actuators() {
	_counter = 0;
}

Actuators::~Actuators() {

}

void Actuators::update_command_msgs(void) {
	for( int i=0;i<11;i++ ) {
		command_msgs[i].data = (float) _commanded_pos[i];
	}
}

void Actuators::update_commands(void) {
	if ( (_counter > 120) | (_counter < 0) ) {
		_counter = 0;
	} else if ( _counter < 60 ) {
		_commanded_pos[0] = _counter/60.0;
	} else {
		_commanded_pos[0] = 1 - (_counter-60)/60.0;
	}

	_counter++;

}
