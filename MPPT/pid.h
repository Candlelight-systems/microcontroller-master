/*
 * pid.h
 *
 *  Created on: 14 Apr 2018
 *      Author: normanpellet
 */

#ifndef PID_H_
#define PID_H_


class PID {

	float error_prior;
	float integral;


	unsigned long lastTime;

	public:

	bool mode_heating;
	float next( float );
	float target;
	float Kp_h;
	float Ki_h;
	float Kd_h;
	float bias_h = 0.0;

	float Kp_c;
	float Ki_c;
	float Kd_c;
	float bias_c = 0.0;

	unsigned long interval = 5000;

};



#endif /* PID_H_ */
