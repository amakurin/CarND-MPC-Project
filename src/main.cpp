#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// for convenience
using json = nlohmann::json;

const double MPH_TO_MPS = 1609.344/3600;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void sim_course_model(int steps, double dt 
                    , double steering, double velocity
                    , double Lf
                    , std::vector<double> &x_vals
                    , std::vector<double> &y_vals
                    , std::vector<double> &psi_vals
                    ){

  for (int i = 1; i< steps; ++i){
    double psi = psi_vals[i-1];
    x_vals[i] = x_vals[i-1] + velocity * cos(psi) * dt;
    y_vals[i] = y_vals[i-1] + velocity * sin(psi) * dt;
    psi_vals[i] = psi + velocity * steering * dt / Lf;
  }
}

void sim_car_model(int steps, double dt
                    , double steering, double velocity
                    , double Lf, double Lr
                    , std::vector<double> &x_vals
                    , std::vector<double> &y_vals
                    , std::vector<double> &psi_vals
                    ){

  for (int i = 1; i< steps; ++i){
    double beta = atan(Lr * tan(steering) / (Lf + Lr));
    double psi = psi_vals[i-1];
    x_vals[i] = x_vals[i-1] + velocity * cos(psi + beta) * dt;
    y_vals[i] = y_vals[i-1] + velocity * sin(psi + beta) * dt;
    psi_vals[i] = psi + velocity * sin(beta) * dt / Lr;
  }
}

void LfLrExperiment(MPC_params params){
  int steps = (int) params.get_util("EX_STEPS", 1000);
  double steering = params.get_util("EX_STEER", 0.436332);
  double velocity = params.get_util("EX_V", 5.0);
  double dt = params.get_util("EX_DT", 0.01);

  std::vector<double> t(steps, 0.0);
  for (int i = 1; i< steps; ++i){
    t[i] = dt*i;
  }
  
  std::vector<double> x_vals0(steps, 0.0);
  std::vector<double> y_vals0(steps, 0.0);
  std::vector<double> psi_vals0(steps, 0.0);


  double Lf0 = params.get_util("EX_LF0", 2.67);;
  sim_course_model(steps, dt, steering, velocity, Lf0, x_vals0, y_vals0, psi_vals0);

  std::vector<double> x_vals1(steps, 0.0);
  std::vector<double> y_vals1(steps, 0.0);
  std::vector<double> psi_vals1(steps, 0.0);
  double Lf1 = params.LF;
  double Lr1 = params.LR;
  sim_car_model(steps, dt, steering, velocity, Lf1, Lr1, x_vals1, y_vals1, psi_vals1);
  plt::subplot(2, 1, 1);
  plt::title("y(x) Red - simplified, Blue - original");
  plt::axis("equal");
  plt::plot(x_vals0, y_vals0, "r-"
          , x_vals1, y_vals1, "b-");
  plt::subplot(2, 1, 2);
  plt::title("psi(t) Red - simplified, Blue - original");
  plt::plot(t, psi_vals0, "r-"
          , t, psi_vals1, "b-");
  plt::show();
}


int main(int argc, char *argv[]) {

  // MPC is initialized here!
  MPC mpc;
  mpc.params.from_args(argc, argv);

  std::cout << mpc.params;
  if ((int) mpc.params.get_util("EX_DO", 0.0) != 0) 
    LfLrExperiment(mpc.params);
  
  if ((int) mpc.params.get_util("EX_ONLY", 0.0) != 0) 
    return 0;

  uWS::Hub h;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];

          // Convert speed from mph to meters per second
          v = v * MPH_TO_MPS;

          // Update vehicle state using simulator data and prediction to compensate latency
          mpc.predict_vehicle_state({px, py, psi, v, -steering_angle, throttle});
          
          // Transform faypoints into vehicle coordinates, fit polynomial and store results.
          mpc.use_waypoints(ptsx, ptsy);

          // Calculate steering angle, throttle and trajectory using MPC.
          auto vars = mpc.Solve();

          // Normalize steering to [-1, 1]
          double steer_value = -vars[0]/deg2rad(25);
          double throttle_value = vars[1];

          json msgJson;

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the waypoints/reference line
          // the points are in vehicle's space  
          // and are connected by a Yellow line in the simulator       
          msgJson["next_x"] = mpc.waypoints_x;
          msgJson["next_y"] = mpc.waypoints_y;

          // Display the MPC predicted trajectory 
          // the points are in vehicle's space  
          // and are connected by a Green line in the simulator       
          msgJson["mpc_x"] = mpc.trajectory_x;
          msgJson["mpc_y"] = mpc.trajectory_y;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
