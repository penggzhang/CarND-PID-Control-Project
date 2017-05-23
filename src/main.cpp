#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;
    
    
    PID pid;
    
    // TODO: Initialize the pid variables.
    
    // Parameters for controller: Kp, Kd, Ki
    double p[3] = {0.993922, 11.0414, 0.00043469};
    
    // Flag for twiddle. Set as true if do twiddle, otherwise false.
    bool twiddle_flag = true;
    
    // Number of steps for twiddle.
    // After 1500 steps, the car will go over the bridge.
    const int n_twiddle_steps = 1500;
    
    // Initialize pid with variables
    pid.Init(p, twiddle_flag, n_twiddle_steps);
    
    
    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;
                    /*
                     * TODO: Calcuate steering value here, remember the steering value is
                     * [-1, 1].
                     * NOTE: Feel free to play around with the throttle and speed. Maybe use
                     * another PID controller to control the speed!
                     */
                    
                    // Declare temporal difference and integral of cross track error.
                    double diff_cte, int_cte;
                    
                    // Calculate the temporal difference of cte.
                    // If the car just starts to run, i.e. step counter of the pid is 0,
                    // the difference is set as 0.
                    diff_cte = pid.n_ == 0 ? 0 : (cte - pid.prev_cte_);
                    
                    // Update pid with the cte of this time.
                    // Step counter, previous cte, integral cte, and cumulative root square error
                    // will be updated accordingly.
                    pid.UpdateError(cte);
                    
                    // Get the value of integral cte.
                    int_cte = pid.int_cte_;
                    
                    // Restart the simulator if the car goes off the track.
                    // Set the allowance of cte being still on track as 2.2(m).
                    // Do twiddle, if corresponding flag is set and specified steps are met.
                    // Following twiddle work, the simulator will also be restarted.
                    if (fabs(cte)>2.2 || (pid.twiddle_flag_==true && pid.n_==pid.n_twiddle_steps_)) {
                        
                        // Check the steps the car has gone to avoid sporadic off-track cte message.
                        // Note: Sometimes, 1 more message with off-track cte is found to be sent
                        // by the simulator after it is restarted. Therefore, to skip this message,
                        // check the number of steps again here.
                        if (pid.n_ == pid.n_twiddle_steps_) {
                            
                            // Increment the iteration counter for twiddle.
                            pid.it_ += 1;
                            
                            // Echo the PID parameters of the just-completed run
                            std::cout << "Params of iteration " << pid.it_ << ": " << std::endl;
                            std::cout << "Kp = " << pid.p_[0] << ", Kd = " << pid.p_[1] << ", Ki = " << pid.p_[2] << std::endl;
                            
                            // Calculate the root mean square (cross track) error.
                            // Cumulated square error is averaged by the number of steps
                            // by which the car has gone.
                            double rmse = pid.TotalError() / pid.n_;
                            std::cout << "RMSE: " << rmse << std::endl;
                            
                            // If it is the first time to do twiddle, then initialize the best
                            // number of steps and RMSE accordingly.
                            // Otherwise, do twiddle.
                            if (pid.best_n_ == 0) {
                                pid.best_n_ = pid.n_;
                                pid.best_rmse_ = rmse;
                            } else {
                                pid.Twiddle(rmse);
                            }
                            
                            // Echo the best RMSE obtained up to now.
                            std::cout << "Current best RMSE: " << pid.best_rmse_ << std::endl << std::endl;
                        }
                        
                        // Reset and restart simulator.
                        pid.Restart(ws);
                        
                    } else { // The car still on track and not to be switched off to do twiddle
                    
                        // Calculate steering value within the range [-1, 1].
                        double steer_temp = - pid.p_[0] * cte - pid.p_[1] * diff_cte - pid.p_[2] * int_cte;
                        steer_temp = fmod(steer_temp, 2 * M_PI);
                        steer_value = steer_temp > 1 ? 1 : steer_temp;
                        steer_value = steer_temp < -1 ? -1 : steer_temp;
                        
                        // DEBUG
                        //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
                    
                        json msgJson;
                        msgJson["steering_angle"] = steer_value;
                        msgJson["throttle"] = 0.3;
                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        std::cout << msg << std::endl;
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        
                    }
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
