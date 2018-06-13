#include "PID.h"

using namespace std;
#include <math.h>
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_,double dpp_, double dpi_, double dpd_) {
	Kp=Kp_;
	Ki=Ki_;
	Kd=Kd_;
	
	dpp=dpp_;
	dpi=dpi_;
	dpd=dpd_;
	
	p_error=0;
	i_error=0;
	d_error=0;
	prev_cte=0;
	
	flag=false;
	
	
}
void PID::InitTwiddle() {
	best_err=0;
    err=0;
    init_flag=false;
    flag_a=true;
    flag_b=false;
    i=0;
}
void PID::Twiddle(double ctc) {
		if(init_flag==false)
				best_err+=pow(cte,2);
			else
				err+=pow(cte,2);
		 if(counter==200)
			{
				if(init_flag==false)
					best_err/=200;
			init_flag=true;
			}
		
		if(init_flag==true && counter==200)
		{
				//err/=200;
				if(flag_a==true){
				p[i] += dp[i];
				counter=0;
				pid.Init(p[0],p[1],p[2],dp[0],dp[1],dp[2]);
				reset_simulator(ws);
				flag_a=false;
				flag_b=true;
				}
				else if(flag_b==true){
					err/=200;
					if (err < best_err){
						best_err = err;
						dp[i] *= 1.1;}
					else{
						p[i] -= 2 * dp[i];
						counter=0;
						pid.Init(p[0],p[1],p[2],dp[0],dp[1],dp[2]);
						reset_simulator(ws);
						if (err < best_err){
							best_err = err;
							dp[i] *= 1.1;}
						else{
							p[i] += dp[i];
							dp[i] *= 0.9;
							}
						}
				}
			i+=1;
		  }
}
void PID::UpdateError(double cte) {
	p_error=cte;
	if(flag==false){
		i_error=0;
		flag=true
	}
	else
		i_error=cte-prev_cte;
	d_error=d_error+cte;
	prev_cte=cte;
	
}

double PID::TotalError() {
	return Kp*p_error + Ki*i_error + Kd*d_error;
}

