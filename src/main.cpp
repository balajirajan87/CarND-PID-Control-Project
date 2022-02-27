#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
    double p[] = {0.194384, 0.0, 1.84692, 0.5, 0.001, 0.0};
    double dp[] = {0.01, 0.0, 0.1, 0.0, 0.0, 0.0};
  pid.Init(p[0], p[1], p[2], p[3], p[4], p[5]);
  h.onMessage([&pid,&p, &dp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
            pid.cte_mod += 1.0 * (cte - pid.cte_mod);
            double Limit_Steer_Angle = 57.2958; // --> 1 rad
            //double tar_spd = pid.calc_tar(30, speed);           // --> mph
            double tar_spd = pid.calc_traj(30);           // --> mph
            double err_spd = tar_spd - speed;
            double throttle_value;
            vector<double> control_op;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
            control_op = pid.twiddle(p, dp, pid.cte_mod, err_spd);
            //pid.UpdateError(pid.cte_mod,err_spd);
            steer_value = std::max(std::min(control_op[0],deg2rad(Limit_Steer_Angle)),deg2rad(-Limit_Steer_Angle));
            throttle_value = fmax(control_op[1],0);
          // DEBUG
            /*
          std::cout << "CTE: " << pid.cte_mod << " Steering Value: " << steer_value
                    << " Steering Angle in Degrees: " << angle << " modified target speed" << tar_spd << " thottle value" << throttle_value << " iter count: " << pid.n_iter <<std::endl;
             */
            std::cout << " iter count: " << pid.n_iter <<std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
