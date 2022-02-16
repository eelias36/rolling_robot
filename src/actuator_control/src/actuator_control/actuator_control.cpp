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

void Actuators::home(void) {
	for( int i=0;i<11;i++ ) {
		_commanded_pos[i] = 0;
	}
}

void Actuators::update_command_msgs(void) {
	for( int i=0;i<11;i++ ) {
		command_msgs[i].data = (float) _commanded_pos[i];
	}
}

void Actuators::update_commands(void) {
	float static stroke = 0.8;

	if ( (_counter > 420) | (_counter < 0) ) {
		_counter = 0;
	} else if ( _counter < 60 ) {
		_commanded_pos[0] = _counter/60.0*stroke;
	} else if ( _counter < 120 ){
		_commanded_pos[0] = stroke - (_counter-60)/60.0*stroke;
		_commanded_pos[1] = (_counter-60)/60.0*stroke;
	} else if ( _counter < 180 ){
		_commanded_pos[1] = stroke - (_counter-120)/60.0*stroke;
		_commanded_pos[2] = (_counter-120)/60.0*stroke;
	} else if ( _counter < 240 ){
		_commanded_pos[2] = stroke - (_counter-180)/60.0*stroke;
		_commanded_pos[3] = (_counter-180)/60.0*stroke;
	} else if ( _counter < 300 ){
		_commanded_pos[3] = stroke - (_counter-240)/	60.0*stroke;
		_commanded_pos[4] = (_counter-240)/60.0*stroke;
	} else if ( _counter < 360 ){
		_commanded_pos[4] = stroke - (_counter-300)/60.0*stroke;
		_commanded_pos[5] = (_counter-300)/60.0*stroke;
	} else {
		_commanded_pos[5] = stroke - (_counter-360)/60.0*stroke;
	}

	_counter++;

}
