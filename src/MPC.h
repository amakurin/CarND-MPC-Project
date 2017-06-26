#ifndef MPC_H
#define MPC_H

#include <chrono>
#include <map>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class MPC_params
{

public:
  double LF        ;
  double LR        ;
  double K_CTE     ;
  double K_EPSI    ;
  double K_ESPEED  ;
  double K_STEER   ;
  double K_SMOOTH  ;
  double V_MIN     ;
  double V_SAFE    ;
  double V_MAX     ;
  double LIM_CURV  ;
  double LIM_EPSI  ;
  double V_SETPOINT;
  double P_DT      ;
  double S_DT      ;
  size_t S_N       ;

  size_t x_start    ;
  size_t y_start    ;
  size_t psi_start  ;
  size_t v_start    ;
  size_t cte_start  ;
  size_t epsi_start ;
  size_t delta_start;
  size_t a_start    ;

  // Map to store all utility parameters not used 
  // extensively during updates
  std::map<std::string, double> util;

  MPC_params();
  
  // Sets parameter value with val by its name
  void set_by_name(std::string name, double val);

  // Special case for N (solver steps). 
  // We need to calculate _start indices here as well.
  void set_S_N(int val);
  
  // Gets utility parameter value by its name
  double get_util(std::string name, double default_val);
  
  // Merge parameters with main args, all unknown parameters goes to _util
  void from_args(int argc, char *argv[]);

};
// Overloded for convenies of param output
std::ostream& operator << ( std::ostream& os, const MPC_params& mpc_params );


class MPC {
  // Timepoint of previous update, used to estimate latency
  std::chrono::time_point<std::chrono::high_resolution_clock> _prev_time_point;
  // Initialization flag
  bool _initialized = false;

public:
  // Parameters object
  MPC_params params;
  // Vehicle state in world coordinates
  vector<double> vehicle_state;
  
  // Waypoints in vehicle coordinates
  vector<double> waypoints_x;
  vector<double> waypoints_y;

  // Polynomial fitted with waypoints
  Eigen::VectorXd poly_coeffs;

  // Estimated trajectory
  vector<double> trajectory_x;
  vector<double> trajectory_y;

  MPC();

  virtual ~MPC();

  // Stores current timestamp, calculates latency and returnes it
  double obtain_latency();

  // Updates vehicle state using new_state from simulator
  // and prediction on period of latency 
  void predict_vehicle_state(const vector<double> &new_state);
  
  // Transforms waypoints from simulator, to current vehicle's state space
  // updates waypoints_x, waypoints_y with resulting data  
  // updates poly_coeffs with polinomial fitted with new waypoints  
  void use_waypoints(const vector<double> &world_x, const vector<double> &world_y);

  // Fits polynomial of 'order' with poins specified by pts_x and pts_y
  Eigen::VectorXd polyfit(std::vector<double> &pts_x, std::vector<double> &pts_y, int order);

  // Calculates value of polinomial specified by coeffs in point x
  double polyeval(const Eigen::VectorXd &coeffs, double x);

  // Calculates synthetic parameter characterizing
  // how close is vehicle to rough turn
  double predict_turn();

  // Calculates setpoint using linear dependency of err
  double linear_setpoint(double err, double err_limit,
                         double min_limit, double max_limit);

  // Calculates setpoint using quadratic dependency of err
  double quadratic_setpoint(double err, double err_limit,
                            double min_limit, double max_limit);

  // Calculates speed setpoint, according to current state 
  double calc_v_setpoint(double epsi);

  

  // Solve the model according to current state and waypoints
  // Return the first actuations.
  vector<double> Solve();
};

#endif /* MPC_H */
