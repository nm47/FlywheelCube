// Arduino PID implementation by Niels Moeller

class PID{
	private:
		float input, float  output, float setpoint;
		float kp, float ki, float kd;
		float errSum, lastErr, lastinput;
		int sampletime = 500; 
		unsigned long lasttime, currenttime;
	public:
		PID(double &inp,  &outp, float &setp){
			currenttime = 0;
			input = inp;
			output = outp;
			setpoint = setp;
		};
		void SetTunings(float kp, float ki, float kd);
		void SetSampleTime(const int stime);
		void compute();
};
