#include "PID.hpp"

PID::PID(float _Kp, float _Ki, float _Kd, float _iMax, float _iMin, float _uMax, float _uMin, float _f) {
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
	this->iMax = _iMax;
	this->iMin = _iMin;
	this->uMax = _uMax;
	this->uMin = _uMin;
	this->f = _f;
};


float PID::sqrt_controller(float alpha_des, float _alpha_des, unsigned short angle_counter, float Kff) {
	float e_angle = alpha_des - _alpha_des;
	e_angle = e_angle/((angle_counter+1)*st);
	float abs_e = abs(e_angle);
	unsigned short sign_e = sgn(e_angle);
	float sqrt_e = sqrt((int)(100*abs_e));
	float P = Kff * sqrt_e/10;
	P = P * sign_e;
	return P;
}


float PID::Run(double e) {

  float e_int = e;

  if((int)pid_buf != (int)pid_sat_buf) {
    if(sgn(e) == sgn(pid_sat_buf)) {
      e_int = 0;
    }
  }

  	de_filt = N * (Kd * e - de_int);
  	de_int += de_filt*st;

	de = e - _e;
	_e = e;

	ie += e_int*st;

	ie_sat = ie;
	

	P = Kp*e; D = de_filt; I = Ki * ie_sat;
	//D = lpf.update(D);
	pid = P + I + D;
  	pid_buf = pid;
	pid  = Sat(pid,  uMax, uMin);
	pid_sat_buf = pid;
    return pid;

}

void PID::reset() {
	ie = 0;
	ie_rate = 0;
	de_filt = 0;
	de_int = 0;
}


unsigned short PID::sgn(float v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}

 float PID::Sat(float pwm, int max, int min, int thr) {
	float pwm_out;

	if(thr > 1020) {
		if(pwm > max) {
			pwm_out = max;
		}

		else if (pwm < min) {
			pwm_out = min;
		}

		else {
			pwm_out = pwm;
		}


	}

	else {
		pwm_out = 1000;
	}
	return pwm_out;
}

 float PID::Sat(float pwm, int max, int min) {
	float pwm_out;

		if(pwm > max) {
			pwm_out = max;
		}

		else if (pwm < min) {
			pwm_out = min;
		}

		else {
			pwm_out = pwm;
		}




	return pwm_out;
}

float PID::pwm2ang(unsigned short int pwm) {
	int dead_zone = 5;

	int in_min  = 1000;
	int in_max  = 2000;

	/*
	int in_min  = 1160;
	int in_max  = 1850;
	*/
	int out_min = -30;
	int out_max  = 30;
	unsigned short int pwm_out;

	if(pwm > 1500 - dead_zone && pwm < 1500 + dead_zone) {
		pwm_out = 1500;
	}

	else {
		pwm_out = pwm;
	}

	return (pwm_out - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float PID::pwm2rate(unsigned short int pwm) {

	int in_min  = 1000;
	int in_max  = 2000;

	/*
	int in_min  = 1160;
	int in_max  = 1850;
	 */
	int out_min = -100;
	int out_max  = 100;

	return -1 * ((pwm - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

//Convert pwm to motor speed for simulation
float PID::pwm2mot(unsigned short int pwm, int dir) {
	float in_min  = 1000;
	float in_max  = 2000;
	float out_min = 0;
	float out_max  = 1326;

	return (float)(dir) * ((float)pwm - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned int PID::F2thr(float F) {
	float kf = 5.074714371861032e-08;
	float max_rpm = 17591;
	float Fm = F/4;
	float wh = sqrt(Fm/kf);

	unsigned int thr = (wh - 0) * (2000 - 1000) / (max_rpm - 0) + 1000;
	return thr;
}

PID::~PID() {};
