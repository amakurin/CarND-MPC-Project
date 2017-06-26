#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
//const double Lf = 2.67;

//-----------------------
// NOTE: LF gone to MPC_params, as i experimented with different models 
//-----------------------


MPC_params::MPC_params(){
  // Default values, could be overriten by args to main
  // Length of vehicle from front to CoG
  LF         =   2.67;
  // Length of vehicle from rear to CoG
  LR         =  0.185;
  
  // Functional cost coefficients K_ prefix
  K_CTE      =    1.0;
  // Cost of EPsi
  K_EPSI     =   40.0;
  // Cost of Speed error
  K_ESPEED   =   10.0;
  // Cost of steering
  K_STEER    = 1300.0;
  // Cost of tough steering
  K_SMOOTH   = 1000.0;

  // Minimum speed in MPS to use when vehicle become highly unstable
  V_MIN      =   20.0;
  // Safe speed in MPS to use on tough turns 
  V_SAFE     =   30.0;
  // Maximum speed in MPS to use when stable on straight portion of track 
  V_MAX      =   70.0;
  // Threshold for 'tough turn' coefficient called CURV (but it is not a curvature radius)
  // used for speed setpoint estimation 
  LIM_CURV   =   80.0;
  // Threshold for epsi during speed setpoint estimation 
  LIM_EPSI   =    0.3;
  // Speed setpoint for solver (estimated dynamically before solving) 
  V_SETPOINT =   30.0;
  // Timestep to use in latency compensation  
  P_DT       =   0.01;
  // Timestep to use with solver  
  S_DT       =   0.05;
  // Count of steps to use with solver  
  set_S_N(20);
}

void MPC_params::set_S_N(int val){
  S_N = val;
  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we should to establish
  // when one variable starts and another ends to make our lifes easier.
  x_start = 0;
  y_start = x_start + S_N;
  psi_start = y_start + S_N;
  v_start = psi_start + S_N;
  cte_start = v_start + S_N;
  epsi_start = cte_start + S_N;
  delta_start = epsi_start + S_N;
  a_start = delta_start + S_N - 1;
}
  
void MPC_params::set_by_name(std::string name, double val){
  if      (name == "LF"        ) LF         = val;
  else if (name == "LR"        ) LR         = val;
  else if (name == "K_CTE"     ) K_CTE      = val;
  else if (name == "K_EPSI"    ) K_EPSI     = val;
  else if (name == "K_ESPEED"  ) K_ESPEED   = val;
  else if (name == "K_STEER"   ) K_STEER    = val;
  else if (name == "K_SMOOTH"  ) K_SMOOTH   = val;
  else if (name == "V_MIN"     ) V_MIN      = val;
  else if (name == "V_SAFE"    ) V_SAFE     = val;
  else if (name == "V_MAX"     ) V_MAX      = val;
  else if (name == "LIM_CURV"  ) LIM_CURV   = val;
  else if (name == "LIM_EPSI"  ) LIM_EPSI   = val;
  else if (name == "V_SETPOINT") V_SETPOINT = val;
  else if (name == "P_DT"      ) P_DT       = val;
  else if (name == "S_DT"      ) S_DT       = val;
  else if (name == "S_N"       ) set_S_N((int) val);
  else { 
    util[name] = val;
    std::cout<< "Unknown argument: \"" << name 
    << "\" with value {"<< val << "} put to util \n";
  } 
};

double MPC_params::get_util(std::string name, double default_val){
  auto it = util.find(name);
  if( it == util.end()){ 
    std::cout << "Requested util param not found: " << name 
    << "default value used: "<< default_val <<"\n";
    return default_val;
  }
  return it->second;
}

// quick and dirty params parser
void MPC_params::from_args(int argc, char *argv[]){
  for (int i = 1; i < argc; ++i)
  {
    std::string arg(argv[i]);
    std::size_t found = arg.find("=");
    if (found == std::string::npos){
      std::cout << "Ignoring argument \"" << arg << "\", arg format name=double_val\n";
      continue;
    }

    std::string name = arg.substr(0, found);
    double val = stod(arg.substr(found+1, arg.size()));
    set_by_name(name, val);
  }
};

std::ostream& operator << ( std::ostream& os, const MPC_params& mpc_params ) {
    os << "******* MPC runtime Parameters *******\n"
      << "LF         = " << mpc_params.LF        << '\n'
      << "LR         = " << mpc_params.LR        << '\n'
      << "K_CTE      = " << mpc_params.K_CTE     << '\n'
      << "K_EPSI     = " << mpc_params.K_EPSI    << '\n'
      << "K_ESPEED   = " << mpc_params.K_ESPEED  << '\n'
      << "K_STEER    = " << mpc_params.K_STEER   << '\n'
      << "K_SMOOTH   = " << mpc_params.K_SMOOTH  << '\n'
      << "V_MIN      = " << mpc_params.V_MIN     << '\n'
      << "V_SAFE     = " << mpc_params.V_SAFE    << '\n'
      << "V_MAX      = " << mpc_params.V_MAX     << '\n'
      << "LIM_CURV   = " << mpc_params.LIM_CURV  << '\n'
      << "LIM_EPSI   = " << mpc_params.LIM_EPSI  << '\n'
      << "P_DT       = " << mpc_params.P_DT      << '\n'
      << "S_DT       = " << mpc_params.S_DT      << '\n'
      << "S_N        = " << mpc_params.S_N       << '\n'
      << "---------- and util params -----------\n";
      for(auto const &map_entry : mpc_params.util) {
        os << map_entry.first << "\t=\t" << map_entry.second << '\n';
      }  
      os << "**************************************\n";    
    return os;
}


class FG_eval {
public:
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  MPC_params params;

  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs, MPC_params params){ 
    this->coeffs = coeffs;
    this->params = params;
  }

  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

    size_t N           = params.S_N        ;
    size_t x_start     = params.x_start    ;
    size_t y_start     = params.y_start    ;
    size_t psi_start   = params.psi_start  ;
    size_t v_start     = params.v_start    ;
    size_t cte_start   = params.cte_start  ;
    size_t epsi_start  = params.epsi_start ;
    size_t delta_start = params.delta_start;
    size_t a_start     = params.a_start    ;
    double dt = params.S_DT;
    double Lf = params.LF;
    //==============================
    // Сost function
    //==============================
    fg[0] = 0.0;
    for (int t = 0; t< N; ++t){
      // Minimize CTE
      fg[0] += params.K_CTE * CppAD::pow(vars[cte_start + t], 2);
      // Minimize EPsi
      fg[0] += params.K_EPSI * CppAD::pow(vars[epsi_start + t], 2);
      // Maximize speed
      fg[0] += params.K_ESPEED * CppAD::pow(vars[v_start + t] - params.V_SETPOINT, 2);
    }

    for (int t = 0; t < N - 1; t++) {
      // Minimize steering
      fg[0] += params.K_STEER * CppAD::pow(vars[delta_start + t], 2);
      // Minimize throttle 
      // Does not give much, commented to reduce computations
      //fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    for (int t = 0; t < N - 2; t++) {
      // Smooth steering
      fg[0] += params.K_SMOOTH * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      // Smooth throttle
      // Does not give much, commented to reduce computations
      //fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //==============================
    // Constraints
    //==============================
    // Initial conditions
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
 
      // Y setpoint at time t.
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 
                    + coeffs[2] * CppAD::pow(x0, 2) 
                    + coeffs[3] * CppAD::pow(x0, 3);

      // Psi setpoint at time t.
      AD<double> psi_des0 = CppAD::atan(
                               coeffs[1] 
                             + coeffs[2] * 2 * x0
                             + coeffs[3] * 3 * CppAD::pow(x0, 2)
                             );
      // Apply model equations 
      // x[t+1]
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      // y[t+1]   (*)
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      // psi[t+1] (**)
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 * dt / Lf);
      // v[t+1]
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      // Equation (*) for cte
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      // Equation (**) for epsi
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) + v0 * delta0 * dt / Lf);
    }
  }
};


//
// MPC class definition implementation.
//
MPC::MPC() {
}

MPC::~MPC() {}

double MPC::obtain_latency(){
  double latency = 0.0;
  // Get current timestamp
  auto now = std::chrono::high_resolution_clock::now();
  // If initialized get difference with previous timestamp
  if (_initialized){
    std::chrono::duration<double> time_diff = now - _prev_time_point;
    latency = time_diff.count();
  }
  else{
    _initialized = true;
  }
  // Remember current timestamp
  _prev_time_point = now;
  return latency;
}

void MPC::predict_vehicle_state(const vector<double> &new_state){
  // Update state
  vehicle_state = new_state;

  // Get current latency 
  double latency = obtain_latency();

  // Set time step to use in simulation 
  double dt = params.P_DT;

  // Calculate steps count 
  long steps = (long) round(latency/dt);

  // Simulate vehicle motion  
  for (int i = 0; i< steps; ++i){
    double x      = vehicle_state[0];
    double y      = vehicle_state[1];
    double psi    = vehicle_state[2];
    double v      = vehicle_state[3];
    double delta  = vehicle_state[4];
    double a      = vehicle_state[5];
    
    // Incrementally update vehicle's state according to model equations 
    vehicle_state[0] = x + v * cos(psi) * dt;
    vehicle_state[1] = y + v * sin(psi) * dt;
    vehicle_state[2] = psi + v * delta * dt / params.LF;
    vehicle_state[3] = v + a * dt;
  }
}

void MPC::use_waypoints(const std::vector<double> &world_x, 
                        const std::vector<double> &world_y){

    int points_count = world_x.size();

    double vehicle_x  = vehicle_state[0];
    double vehicle_y  = vehicle_state[1];
    double psi        = vehicle_state[2];
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);
    
    waypoints_x.resize(points_count, 0.0);
    waypoints_y.resize(points_count, 0.0);
    
    // Trasform waypoints coordinates from world's to vehicle's space 
    for (int i=0; i< points_count; ++i){
      double x_diff = world_x[i] - vehicle_x;
      double y_diff = world_y[i] - vehicle_y;

      waypoints_x[i] = y_diff * sin_psi + x_diff * cos_psi;
      waypoints_y[i] = y_diff * cos_psi - x_diff * sin_psi;
    }

    // Fit 3'd order polynomial with waypoints in vehicle's space
    poly_coeffs = polyfit(waypoints_x, waypoints_y, 3);
}

// Evaluate a polynomial.
double MPC::polyeval(const Eigen::VectorXd &coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPC::polyfit(std::vector<double> &pts_x,
                             std::vector<double> &pts_y,
                             int order){

  int x_size = pts_x.size();
  
  assert(x_size == pts_y.size());
  assert(order >= 1 && order <= x_size - 1);
  
  Eigen::VectorXd yvals = Eigen::Map<Eigen::VectorXd>(pts_y.data(), x_size);

  Eigen::MatrixXd A(x_size, order + 1);

  for (int i = 0; i < x_size; i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < x_size; j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * pts_x[j];
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  return result;
}

double MPC::predict_turn(){
  double epsi_max = 0.0;
  double distance = 0.0;
  // Iterate over waypoints
  for (int i = 0; i < waypoints_x.size(); ++i)
  {
    double x = waypoints_x[i];
    double y = waypoints_y[i];
    // Calculate epsi = psi - psides, where psi=0
    double epsi = fabs( atan(poly_coeffs[1] 
                  + poly_coeffs[2] * 2 * x
                  + poly_coeffs[3] * 3 * x * x));
    // Get max epsi over waypoints and 
    // manhattan distance to waypoint with max epsi
    if (epsi > epsi_max){
      epsi_max = epsi;
      distance = fabs(x) + fabs(y);            
    }
  }
  // Zero check
  if (epsi_max < 0.0001)
    return 1000.0; 
  else 
    // Return synthetic parameter as distance to epsi ratio
    return distance/epsi_max; 
}

double MPC::linear_setpoint(double err, double err_limit,
                       double min_limit, double max_limit){
  double abs_err = fabs(err);
  if (abs_err > err_limit)
    abs_err = err_limit;
  return max_limit + (min_limit - max_limit) * abs_err / err_limit;
}

double MPC::quadratic_setpoint(double err, double err_limit,
                          double min_limit, double max_limit){
  double abs_err = fabs(err);
  if (abs_err > err_limit)
    abs_err = err_limit;
  double arg =  abs_err - err_limit;
  return min_limit + (max_limit - min_limit) * arg * arg / (err_limit * err_limit);
}

double MPC::calc_v_setpoint(double epsi){
  double v_min    = params.V_MIN;
  double v_safe   = params.V_SAFE;
  double v_max    = params.V_MAX;
  double lim_curv = params.LIM_CURV;
  double curv     = predict_turn();
  double lim_epsi = params.LIM_EPSI;
  // Correct maximum speed according to closest turn estimation
  // if parameter less then threshold
  if (curv < lim_curv)
    v_max = linear_setpoint(curv, lim_curv, v_safe, v_max);
  return linear_setpoint(epsi, lim_epsi, v_min, v_max);
}


vector<double> MPC::Solve() {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  size_t N           = params.S_N        ;
  size_t x_start     = params.x_start    ;
  size_t y_start     = params.y_start    ;
  size_t psi_start   = params.psi_start  ;
  size_t v_start     = params.v_start    ;
  size_t cte_start   = params.cte_start  ;
  size_t epsi_start  = params.epsi_start ;
  size_t delta_start = params.delta_start;
  size_t a_start     = params.a_start    ;

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial state (vehicle's space)
  double x    = 0;
  double y    = 0;
  double psi  = 0;
  double v    = vehicle_state[3];
  // cte = f(x)-y, x=0, y=0 => it is just zero coeff of poly
  double cte  = polyeval(poly_coeffs, 0);
  // epsi = psi - psides, psi=0, tan=f'(0) => it is just atan of first coeff of poly 
  double epsi = - atan(poly_coeffs[1]);

  // Set initial value of all the independent variables to zero.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Except state variables at time t0
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Initial state constraints
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // Calculate velocity setpoint
  params.V_SETPOINT = calc_v_setpoint(epsi);
  
  // object that computes objective and constraints
  FG_eval fg_eval(poly_coeffs, params);
 
  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;
  
 
  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout<< " Cost " << cost << std::endl;
 
  if (ok){
    // Update trajectory with newly estimated values
    trajectory_x.resize(N, 0.0);
    trajectory_y.resize(N, 0.0);
    for (int t = 0; t < N; ++t){
        trajectory_x[t] = solution.x[x_start + t];
        trajectory_y[t] = solution.x[y_start + t];
    }

    // Return the first actuator values
    return {solution.x[delta_start], solution.x[a_start]};
  } else {
    std::cout<< "WARN: Failed to solve for trajectory! Zero actuations sent.\n";
    return {0.0, 0.0};
  }

}


