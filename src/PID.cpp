#include "PID.h"
#include <uWS/uWS.h>
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double p[], bool twiddle_flag, const int n_twiddle_steps) {
    // Initialize Kp, Kd, Ki
    p_[0] = p[0];
    p_[1] = p[1];
    p_[2] = p[2];
    
    // Set flag and step number for twiddle
    twiddle_flag_ = twiddle_flag;
    n_twiddle_steps_ = n_twiddle_steps;
}

void PID::UpdateError(double cte) {
    // Increment steps
    n_ += 1;
    
    // Update errors
    prev_cte_ = cte;
    int_cte_ += cte;
    cum_root_square_error_ += sqrt(cte * cte);
}

double PID::TotalError() {
    return cum_root_square_error_;
}

void PID::Twiddle(double rmse){
    // Do a particular twiddle step according to
    // the flag of twiddle step, flag_d_
    switch (flag_d_) {
            
        // Twiddle up a particular parameter, indicated as
        // the flag of parameter flag_p_, by a step change dp,
        // and set the twiddle step as 1.
        case 0:
            p_[flag_p_] += dp_[flag_p_];
            flag_d_ = 1;
            break;
            
        // Check if RMSE of the test run is smaller than the best RMSE.
        // If positive, update the best RMSE and the best steps,
        // increase step change, set the flag of twiddle parameter
        // as the next parameter, and set the twiddle step as 0.
        // Otherwise, twiddle down the parameter by the double of step change,
        // and set the twiddle step as 2.
        case 1:
            if (n_ > best_n_ || (n_ == best_n_ && rmse < best_rmse_)) {
                best_n_ = n_;
                best_rmse_ = rmse;
                dp_[flag_p_] *= 1.1;
                flag_p_ = fmod(flag_p_ + 1, 3);
                flag_d_ = 0;
                
                // Echo when a better parameter is found
                cout << "***** Find new best RMSE: " << best_rmse_ << " *****" << endl;
            } else {
                p_[flag_p_] -= 2 * dp_[flag_p_];
                flag_d_ = 2;
            }
            break;
            
        // Check if RMSE of the test run is smaller than the best RMSE.
        // If positive, update the best RMSE and the best steps,
        // increase step change.
        // Otherwise, twiddle up back to the previous parameter value,
        // decrease step change.
        // Set the flag of twiddle parameter as the next parameter,
        // and set the twiddle step as 0.
        case 2:
            if (n_ > best_n_ || (n_ == best_n_ && rmse < best_rmse_)) {
                best_n_ = n_;
                best_rmse_ = rmse;
                dp_[flag_p_] *= 1.1;
                
                // Echo when a better parameter is found
                cout << "***** Find new best RMSE: " << best_rmse_ << " *****" << endl;
            } else {
                p_[flag_p_] += dp_[flag_p_];
                dp_[flag_p_] *= 0.9;
            }
            flag_p_ = fmod(flag_p_ + 1, 3);
            flag_d_ = 0;
            
        default:
            break;
    }
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
    // Reset step number, integral cte and cumulative root square error.
    n_ = 0;
    int_cte_ = 0;
    cum_root_square_error_ = 0;
    
    // Send reset message and restart simulator
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}















