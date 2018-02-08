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
}

double PID::TotalError() {
}

