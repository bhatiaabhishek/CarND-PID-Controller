#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool tune_param) {

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->tune_param = tune_param;

    p_error = 0 ;
    i_error = 0 ;
    d_error = 0 ;

    err = 0;
    best_err = 1000000;
    tolerance = 0.02;
    span = 1;
    opt_track = 0;
    index = 0;
    restart_sim = 0;


    d_p.push_back(1);
    d_p.push_back(0.01);
    d_p.push_back(1);
    K_vec.push_back(Kp);
    K_vec.push_back(Ki);
    K_vec.push_back(Kd);

}

double rolling_avg(double roll_avg, double new_steer, int samples) {


    
    roll_avg -= roll_avg/samples;
    roll_avg += new_steer/samples;
    return roll_avg;


}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    
    if (span > 100) {err += pow(cte,2);}

    span += 1;
    if ( tune_param and ((span == 800) or (fabs(cte) > 2.5))){
        if (fabs(cte) > 2.5) { err = 10000000000000000; }
        err = err/700;
        double sum_k_e = d_p[0] + d_p[1] + d_p[2];
        std::cout << "span : " << span << std::endl;
        std::cout << "dP dI dD :" << d_p[0] <<  " "<< d_p[1] << " " << d_p[2]<< std::endl;
        std::cout << "KP KI KD :" << Kp <<  " "<< Ki << " " << Kd<< std::endl;
        std::cout << "track : " << opt_track << std::endl;
        std::cout << "index : " << index << std::endl;
        if (sum_k_e > tolerance) {


               if (opt_track == 0) { 

                   K_vec[index] += d_p[index];
                   opt_track = 1;
                   best_err = err;
                   
                   }
               else if (opt_track == 1) {
                  if (err < best_err) {
                      best_err = err;
                      d_p[index] *= 1.1;
                      opt_track = 0;
                      index = (index + 1)%3;
                    }
                  else { 
                  K_vec[index] -= 2*d_p[index];
                  opt_track = 2; }
                } 
               else if (opt_track == 2) {
                   if (err < best_err) { 
                       best_err = err;
                       d_p[index] *= 1.1;
                   }
                   else {
                       K_vec[index] += d_p[index];
                       d_p[index] *= 0.9;
                    }
                    opt_track = 0;
                    index = (index + 1)%3;
                   
               }
               
               


       Kp = K_vec[0];
       Ki = K_vec[1];
       Kd = K_vec[2];
      
       restart_sim =1;
     }
     else { std::cout << "I am done!!!!!" << std::endl; 
            restart_sim = 0;
           }
     span = 1;
     err = 0;

     }
        

    
    
}

double PID::TotalError() {

     steer = ((-Kp*p_error) - (Kd*d_error) - (Ki*i_error));
     return rolling_avg(roll_avg, steer,10);

     
}

