#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
public:

  /*
  * Errors
  */
  double prev_cte_;  // Previous cte
  double int_cte_ = 0.0;  // Integral cte
  double cum_root_square_error_ = 0.0; // Cumulative root square error

  /*
  * Coefficients
  */
  double p_[3];  // Kp, Kd, Ki
    
  /*
  * Step counter
  */
  int n_ = 0;
    
  /*
  * Parameters for twiddle
  */
    
  bool twiddle_flag_;  // Do twiddle if true
  int n_twiddle_steps_;  // Number of steps for twiddle
    
  int best_n_ = 0;  // The largest number of steps the car has gone
  double best_rmse_;  // The smallest RMSE value that has be recorded
    
  double dp_[3]={0.1, 1, 0.0001};  // Step changes on parameters
    
  int flag_p_ = 0;  // Indicate which parameter is being twiddled.
                    // There are 3 parameters in total.
    
  int flag_d_ = 0;  // Indicate the twiddle step on one parameter.
  /* 
  Note: Twiddle on one parameter can be divided as 3 steps.
        0: Twiddle up the parameter by the step change,
           and set the twiddle step as 1, and test run.
        1: Check if RMSE of the test run is smaller than the best RMSE.
           If positive, update the best RMSE and the best steps,
             increase step change, set the flag of twiddle parameter 
             as the next parameter, and set the twiddle step as 0.
           Otherwise, twiddle down the parameter by the double of step change,
             and set the twiddle step as 2, and test run.
        2: Check if RMSE of the test run is smaller than the best RMSE.
           If positive, update the best RMSE and the best steps,
             increase step change.
           Otherwise, twiddle up back to the previous parameter value,
             decrease step change.
           Set the flag of twiddle parameter as the next parameter,
             and set the twiddle step as 0.
  */
    
  /*
  * Counter for twiddle iterations
  */
  int it_ = 0;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double p[], bool twiddle_flag, const int n_twiddle_steps);

  /*
  * Update the PID error variables given cross track error.
  * Also increment the number of steps.
  */
  void UpdateError(double cte);

  /*
  * Return the total PID root square error.
  */
  double TotalError();
    
  /*
  * Restart the simulator.
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
    
  /*
  * Twiddle
  */
  void Twiddle(double rmse);
};

#endif /* PID_H */











