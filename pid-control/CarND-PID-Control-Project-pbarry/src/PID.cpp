#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  this->Kp = Kp;
  this->Ki =  Ki;
  this->Kd = Kd;

  p_error = i_error = d_error = 0.0;
  cte_prev = 0;

}

/*
* Update the PID error variables given cross track error.
*/
void PID::UpdateError(double cte) {

   p_error = cte;
   i_error += cte;
   d_error = cte - cte_prev;
   cte_prev = cte;
}
/*
* Calculate the total PID error.
*/
double PID::TotalError() {
  return (  (p_error * Kp) + (i_error * Ki) + (d_error * Kd) );
}
