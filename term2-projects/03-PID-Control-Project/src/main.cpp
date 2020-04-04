#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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

// Reset the simulator
void reset_simulator(uWS::WebSocket<uWS::SERVER> ws) {
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


int main() {
  uWS::Hub h;

  PID pid;
  //Initialize the pid variable. (lecture p=0,2, i=0,004, d=3) (twiddle: p = 0.313373, i = 0.004, d = 2.81048)
  double K_p_0 = 0.2;//0.30351;
  double K_i_0 = 0.004;//0.001;
  double K_d_0 = 3;//2.66123;

  bool twiddle = true;
  pid.Init(K_p_0, K_i_0, K_d_0, twiddle);

  int n_iteration = 0;

  h.onMessage([&pid, &n_iteration](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

          //Calculate steering value in range [-1, 1].
          pid.UpdateError(cte);
          steer_value = pid.ComputeControl();
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          // Constant throttle
          msgJson["throttle"] = 0.3;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // Perform PID parameter tuning if twiddle is active
          if (pid.twiddle.active)
          {
            if (n_iteration > pid.twiddle.ignore_initial_iterations)
            {
              double avrg_error = pid.TotalAverageError(cte, n_iteration);
              std::cout << "avrg_error = " << avrg_error << std::endl;
              if (avrg_error > 0.01)
              {
                pid.Twiddle(avrg_error);
                reset_simulator(ws);
                n_iteration = 0.0;
              }
              else
              {
                n_iteration++;
              }
            }
            else
            {
              n_iteration++;
            }
          }
        }  // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
  {
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