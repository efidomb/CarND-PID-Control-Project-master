#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	//double int_cte = 0;
	//bool first_time = true;
	//double prev_cte;
}

void PID::UpdateError(double cte) {
	int_cte += cte;
	if (first_time) {
		diff_cte = 0;
		first_time = false;
	}
	else {
		diff_cte = cte - prev_cte;
	}
	prev_cte = cte;
}

double PID::TotalError(double cte) {
	return -Kp * cte - Ki * int_cte - Kd * diff_cte;
}

