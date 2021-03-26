#include "PID.h"
#include <iostream>
#include <stdio.h>
# include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
 }

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}


void PID::Twiddle() {
  double p[3] = {Kp, Ki, Kd};
  double d[3] = {-0.1,-0.1,-0.1};
  double best_err = p[0]*p_error + p[1]*i_error + p[2]*d_error;
  double err;
  int argc;
  std::cout << "best_err=" << best_err << std::endl;
  while ((d[0]+d[1]+d[2]) > 0.00001) {
    for (int i=0; i<3; i++) {
	  // try increment first 
	  p[i] += d[i];
	  err = p[0]*p_error + p[1]*i_error + p[2]*d_error;
	  std::cout << "a) added : p[" << i << "]+=d[i] : err=" << err << std::endl;
	  
	  // b)
	  if (err < best_err) {
	    best_err = err;
		d[i] *= 1.1;
	    std::cout << "a) best err update=" << best_err << "new i=" << i << " p=" << p[0] << "|" << p[1] << "|" << p[2] << std::endl;
	  }
	  else {
		p[i] -= 2*d[i];
        err = p[0]*p_error + p[1]*i_error + p[2]*d_error;
	    std::cout << "b) subtracted : p[" << i << "]-=2xd[i]: err=" << err << std::endl;
	  
	    // b)
	    if (err < best_err) {
		  best_err = err;
		  d[i] *=0.87;
		  std::cout << "b) best err update=" << best_err << "new i=" << i << " p=" << p[0] << "|" << p[1] << "|" << p[2] << std::endl;
		}
		else {
		  // d)
		  p[i] += d[i];
		  std::cout << "c) unchange : i=" << i << " p=" << p[0] << "|" << p[1] << "|" << p[2] << std::endl;
		}
	  }
	}
	std::cin >> argc;
  }
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
}