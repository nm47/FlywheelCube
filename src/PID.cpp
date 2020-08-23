
// Arduino PID implementation by Niels Moeller
#include "PID.h"


void PID::SetTunings(float p, float i, float d){
	kp = p;
	ki = i * ((float)sampletime / 1000); //convert sample time to seconds
	kd = d / ((float)sampletime / 1000);
}

void PID::compute(){
	currenttime = millis();
	float dt = (float)(currenttime - lasttime);

	if (dt >= sampletime) {
		//Calculate the current error.
		error = (setpoint - input);
		errSum += error;

		//change in input since last timestep
		float dinp = input - lastinput;

		output = kp * error + ki * errSum - kd * dinp;

		lastinput = input;
		lastTime = currenttime;
	}
}
void PID::SetSampleTime(const int stime) {
	if (stime > 0) {
		sampletime = (unsigned long)stime;
		float ratio = (float)stime / (float)sampletime;
		ki *= ratio;
		kd /= ratio;
	}
}

