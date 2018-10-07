#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//Set the timestep length and duration
size_t N = 10;
double dt = 0.1;
size_t latency_cnt = 1;

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
const double Lf = 2.67;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t strt_x = 0;
size_t strt_y = strt_x + N;
size_t strt_psi = strt_y + N;
size_t strt_v = strt_psi + N;
size_t strt_cte = strt_v + N;
size_t psi_estrt = strt_cte + N;
size_t strt_delta = psi_estrt + N;
size_t strt_a = strt_delta + N - 1;


double throttle_prv=0;
double Steerting_prv=0;

double v_ref = 70;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // MPC Implementation
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

    //Calculate cost
    fg[0] = 0;
    for(size_t t=0;t<N;t++){
      //increase the weight by multiplying with a scalar
      fg[0] += 900*CppAD::pow(vars[strt_cte + t], 2); // matain the trajectory with low error 
      fg[0] += 125*CppAD::pow(vars[psi_estrt + t], 2); // orienation  cost maintanace
      fg[0] += CppAD::pow(vars[strt_v + t] - v_ref, 2); //velocity maintaining
    }
	
    //cost based on actuators- 
	//Minimize the use
    for(size_t t=0;t<N-1;t++){
      fg[0] += CppAD::pow(vars[strt_delta + t], 2);
      fg[0] += CppAD::pow(vars[strt_a + t], 2);
    }
    
	
	//reduce the value between the sequential actuations
    for(size_t t=0;t<N-2;t++){
    
	
    // increase the weights by multplying the weights with a constant 
    // reduce the fluctuations	
	 fg[0] += 500*CppAD::pow(vars[strt_delta + t + 1] - vars[strt_delta + t], 2);
      fg[0] += CppAD::pow(vars[strt_a + t + 1] - vars[strt_a + t], 2);
    }
    
	// psi and velocity,  change in angle vs the velocity.
    for(size_t t=0;t<N-1;t++){
      fg[0] += 150*CppAD::pow(vars[strt_delta + t]*vars[strt_v + t], 2);
      fg[0] += 10*CppAD::pow(vars[psi_estrt + t]*vars[strt_v + t], 2);
    }

    //
    // Setiing up 
    //
    // Initial constraints
    //Add one to each of the string indcies as cost is at index 0. This shifts the position of all other value.

	
    fg[1 + strt_x] = vars[strt_x];
    fg[1 + strt_y] = vars[strt_y];
    fg[1 + strt_psi] = vars[strt_psi];
    fg[1 + strt_v] = vars[strt_v];
    fg[1 + strt_cte] = vars[strt_cte];
    fg[1 + psi_estrt] = vars[psi_estrt];


    // for the remianing constrains
	
    for (size_t t = 1; t < N; t++) {
      // at time t -1 state
      AD<double> x0 = vars[strt_x + t - 1];
      AD<double> y0 = vars[strt_y + t - 1];
      AD<double> psi0 = vars[strt_psi + t - 1];
      AD<double> v0 = vars[strt_v + t - 1];
      AD<double> cte0 = vars[strt_cte +t -1];
      AD<double> epsi0 = vars[psi_estrt +t -1];

      // at time t state
      AD<double> x1 = vars[strt_x + t];
      AD<double> y1 = vars[strt_y + t];
      AD<double> psi1 = vars[strt_psi + t];
      AD<double> v1 = vars[strt_v + t];
      AD<double> cte1 = vars[strt_cte +t];
      AD<double> epsi1 = vars[psi_estrt +t];

      //  At t -1 acutuators value 
      AD<double> delta0 = vars[strt_delta + t -1];
      AD<double> a0 = vars[strt_a+t -1];

      //calculating error evaluate psi and y
      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);


      
      //Constraints
      fg[1 + strt_x + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + strt_y + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + strt_psi + t] = psi1 - (psi0 + v0 * (delta0 / Lf) * dt);
      fg[1 + strt_v + t] = v1 - (v0 + a0 * dt);
      fg[1 + strt_cte + t] = cte1-((f0 - y0) + (v0*CppAD::sin(epsi0)*dt));
      fg[1 + psi_estrt + t] = epsi1 - ((psi0 - psides0) + v0 * (delta0/Lf) * dt);
    }
  }
};

//
// Implementation of MPC class
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  //Initial State
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];


  // Number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N*6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // Set the initial variable values
  vars[strt_x] = x;
  vars[strt_y] = y;
  vars[strt_psi] = psi;
  vars[strt_v] = v;
  vars[strt_cte] = cte;
  vars[psi_estrt] = epsi;
  //vars[strt_delta] = current_steer;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < strt_delta; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = strt_delta; i < strt_delta+latency_cnt; i++) {
    vars_lowerbound[i] = Steerting_prv;
    vars_upperbound[i] = Steerting_prv;
  }
  for (size_t i = strt_delta+latency_cnt; i < strt_a; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = strt_a; i < strt_a+latency_cnt; i++) {
    vars_lowerbound[i] = throttle_prv;
    vars_upperbound[i] = throttle_prv;
  }
  for (size_t i = strt_a+latency_cnt; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[strt_x] = x;
  constraints_lowerbound[strt_y] = y;
  constraints_lowerbound[strt_psi] = psi;
  constraints_lowerbound[strt_v] = v;
  constraints_lowerbound[strt_cte] = cte;
  constraints_lowerbound[psi_estrt] = epsi;

  constraints_upperbound[strt_x] = x;
  constraints_upperbound[strt_y] = y;
  constraints_upperbound[strt_psi] = psi;
  constraints_upperbound[strt_v] = v;
  constraints_upperbound[strt_cte] = cte;
  constraints_upperbound[psi_estrt] = epsi;



  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  //Update the values for next reference to simulate 100ms delay in actuation
  Steerting_prv = solution.x[strt_delta+latency_cnt];
  throttle_prv = solution.x[strt_a+latency_cnt];
  //copy required data to return vector
  std::vector<double> ret = {solution.x[strt_delta+latency_cnt],solution.x[strt_a+latency_cnt]};
  for (size_t i = 0; i < N; i++) {
    ret.push_back(solution.x[strt_x + 1 +i]);
    ret.push_back(solution.x[strt_y + 1 +i]);
  }
  return ret;
}
