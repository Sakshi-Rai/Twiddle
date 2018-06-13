#include "PID.h"
#include<cfloat>
#include<cmath>
#include<fstream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::initTwiddle(){
	flag = -1;
	fsign = 0;
	fchange = 0;
	n = 0;
	for (unsigned int i =0; i < 3; i++) {
		p.push_back(0.0);
		dp.push_back(0.5);
	}
	ctr = 0;
}

int PID::Twiddle(){
	if (n == 250) {
		if (flag == -1 ) {
			best_err = TotalError();
			flag = 0;
			p[flag] += dp[flag];
		}
	    else if (flag < p.size()) {
		    err = TotalError();
		    //Checking if err improves and increasing param change factor
			if (err < best_err) {
				  best_err = err;
				  dp[flag] *= 1.1;
				  fchange = 1;
			}
			//Checking for decrement
			else if(fsign == 0) {
				p[flag] -= 2 * dp[flag];
				fsign = 1;
			}
			//Changing param change factor
			else {
				p[flag] += dp[flag];
				dp[flag] *= 0.9;
				fsign = 0;
				fchange = 1;
			}
		  //Changing to next parameter
		if (fchange == 1) {
			  flag += 1;
			  if (flag == p.size()) {
				  flag = 0;
			  }
			  p[flag] += dp[flag];
			  fchange = 0;
		    }
	    }
		Init(p[0], p[1], p[2]);
		myfile.open("Data.txt", ios::out | ios::app);
		myfile<<"Best Error : "<<best_err<<" Error : "<<err<<" Params : "<<p[0]<<" "<<p[1]<<" "<<p[2]<<std::endl;
		myfile.close();
		ctr ++;
		if (ctr == 1000) {
			exit(0);
		}
		return 1;
	}
	
	// int sum_dp = 0;
	// for (unsigned int i =0; i<dp.size(); i++) {
		// sum_dp += dp[i];
	// }
	// if (sum_dp < 0.2 ) {
		// exit(0);
	// }
	return 0;
}

void PID::Init(double Kp, double Ki, double Kd) {
	//initialising self variables using this
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = DBL_MAX;
	i_error = 0;
	twi_err = 0;
}

void PID::UpdateError(double cte) {
	if (p_error == DBL_MAX) {
		d_error = 0;
	}
	else {
		d_error = cte - p_error;
	}
	p_error = cte;
	i_error += cte;
	
	//Incrementing counter
	n++;
	
	//Incrementing Twiddle error
	
	twi_err += pow(p_error, 2.0);
}

double PID::TotalError() {
	return twi_err;
}

