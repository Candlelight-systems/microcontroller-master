#include <Arduino.h>
#include "pid.h"

float PID::next( float value ) {


	float error = target - value;
	unsigned long time_ellapsed = ( millis() - lastTime );
	integral = integral + error * time_ellapsed;
	float derivative = ( error - error_prior ) / time_ellapsed / 1000.0;
	float output;

	if( mode_heating ) {
		output = (float) ( Kp_h * error + Ki_h * integral + Kd_h * derivative ) + bias_h;
	} else {
		output = (float) ( Kp_c * error + Ki_c * integral + Kd_c * derivative ) + bias_c;
	}

	if( output <  0 ) {
		output = 0;
	} else if( output > 1 ) {
		output = 1;
	}

	lastTime = millis();
	error_prior = error;
	return output;
}


float readSensor( ) {
	return 1.0;
}

