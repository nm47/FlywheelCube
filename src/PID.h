// Arduino PID implementation by Niels Moeller

class PID{
	private:
		double input, double  output, double setpoint;
		double kp, double ki, double kd;
		double errSum, lastErr;
		unsigned long lasttime, currenttime;
	public:
		PID(double &inp, double &outp, double &setp){
			currenttime = 0;
			input = inp;
			output = outp;
			setpoint = setp;
		};
		void SetTunings(double kp, double ki, double kd);
		void compute();
};
