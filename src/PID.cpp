
// Arduino PID implementation by Niels Moeller

#include "PID.h"


void PID::SetTunings(double p, double i, double d){
	kp = p;
	ki = i;
	kd = d;
}

void PID::compute(){
	currenttime = millis();
	double dt = (double)(currenttime - lasttime);

	//Calculate the current error.
	error = (setpoint - input);
	errSum = error * dt;
	//The derivative of the error with respect to time.
	derE = (error - lasterr) / dt;

	output = kp * error + ki * errSum + kd * derE;

	lastErr = error;
	lastTime = currenttime;
}

