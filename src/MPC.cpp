#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
// N, dt, and T are hyperparameters you will need to tune for each model predictive controller you build.
// However, there are some general guidelines. T should be as large as possible, while dt should be as small as possible
// If the N is too high, there would be high computational cost and the compute required for MPC::slove() may cause in delayed response
// to the actuators

// A good approach to setting N, dt, and T is to first determine a reasonable range for T and then tune dt and N appropriately,
// keeping the effect of each in mind.
// Keeping the T to be 2 seconds - why?
size_t N = 30;
double dt = 0.1;  //seconds

// Set the reference valocity to 40
double ref_v = 40;

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

// Add multipliers for different cost components
int nCostMultiplierCTE = 2000;
int nCostMultiplierEPSI = 1500;
int nCostMultiplierV = 10;
int nCostMultiplierDELTA = 20000;
int nCostMultiplierACC = 10;
int nCostMultiplierDELTA_diff = 100;
int nCostMultiplierACC_diff = 10;

//define the offsets for each component in vars Vector
// Offsets when N = 5 as given in the lesson https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/d3df10cc-797a-47ae-82c9-a39a597870d9
// Note Ipopt expects all the constraints and variables as vectors. For example, suppose N is 5, then the structure of vars a 38-element vector:
/*
vars[0],...,vars[4] -> [x ​1 ​​ ,....,x ​5 ​​ ]
vars[5],...,vars[9] -> [y ​1 ​​ ,....,y ​5 ​​ ]
vars[10],...,vars[14] -> [ψ ​1 ​​ ,....,ψ ​5 ​​ ]
vars[15],...,vars[19] -> [v ​1 ​​ ,....,v ​5 ​​ ]
vars[20],...,vars[24] -> [cte ​1 ​​ ,....,cte ​5 ​​ ]
vars[25],...,vars[29] -> [eψ ​1 ​​ ,....,eψ ​5 ​​ ]
vars[30],...,vars[33] -> [δ ​1 ​​ ,....,δ ​4 ​​ ]
vars[34],...,vars[37] -> [a ​1 ​​ ,....,a ​4 ​​ ]
*/
const int x_start = 0;
const int y_start = x_start + N;
const int psi_start = y_start + N;
const int v_start = psi_start + N;
const int cte_start = v_start + N;
const int epsi_start = cte_start + N;
const int delta_start = epsi_start + N;
const int a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;


    // Reference State Cost
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.
    for (uint t = 1; t < N; t++) {
        // Add cost related to reference state (cte and espilon error)
        // https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/1528b1f9-3373-44bd-b431-b6f61668bed2
        fg[0] += nCostMultiplierCTE*CppAD::pow(vars[cte_start+t], 2);
        fg[0] += nCostMultiplierEPSI*CppAD::pow(vars[epsi_start+t], 2);
        // Add the cost dealing with stopping (incentive to keep moving the vehicle at desired speed)
        //Ref https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/554a16a1-cdf1-408b-a431-4a84689a4fba
        fg[0] += nCostMultiplierV*CppAD::pow(vars[v_start+t] - ref_v , 2);
    }

    // Cost related to control input

    // Use of actuators (only N-1 measures)
    // Only take care of delta (steeing angle) and accleration
    // Ref https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/5df9cd1c-b111-48e5-857c-7547f82dac0c
    for (uint t = 1; t < N-1; t++) {
        fg[0] += nCostMultiplierDELTA*100*CppAD::pow(vars[delta_start+t], 2);
        fg[0] += nCostMultiplierACC*20*CppAD::pow(vars[a_start + t], 2);
    }

    // Only take care of change in delta and a (current vs. prev measurement)
    // Ref https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/5df9cd1c-b111-48e5-857c-7547f82dac0c
    for (uint t = 1; t < N-2; t++) {
        // Ref https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/d6909689-a76f-4eb9-9439-0e885679ba59
        fg[0] += nCostMultiplierDELTA_diff*CppAD::pow(vars[delta_start+t+1] - vars[delta_start+t], 2);
        fg[0] += nCostMultiplierACC_diff*CppAD::pow(vars[a_start+t+1] - vars[a_start+t], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (uint t = 1; t < N; t++) {

      // Values at time t
      // ref https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/d26b8460-653f-4479-bc24-68bb62c146ba
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // Values at time 0
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t -1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuations (steering angle, throttle) at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 +  + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // 2nd degree polynomial may reduce errors for cte
      //AD<double> f0 = coeffs[0] + coeffs[1] * x0 +  + coeffs[2] * CppAD::pow(x0, 2) ;
      //AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.

      // TODO: Setup the rest of the model constraints
      // Ref https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/ee21948d-7fad-4821-b61c-0d69bbfcc425
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (uint i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  //Fill in x, y, psi, v, cte, and epsi from state vector from transformed waypoints
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  // All od x, y, psi, v values
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians  -25*pi/180 to 25*pi/180).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  // -1: Full brake, 1: Full acceleration
  for (uint i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (uint i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

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

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> x1 = {solution.x[delta_start], solution.x[a_start]};

  // Now add the mpc x and y, so that they can be used to display the way points on the emulator
  for (uint i = 0; i < N-1; i++){
      x1.push_back(solution.x[x_start+i+1]);
      x1.push_back(solution.x[y_start+i+1]);
  }

  return x1;
}

/*
 * Convert the global map pos to car map
 * http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node153.html
   https://www.miniphysics.com/coordinate-transformation-under-rotation.html
*/
vector<double> MPC::MapToCarPos(vector<double> globalPos, vector<double> carPos){
    double car_x = carPos[0];
    double car_y = carPos[1];
    double car_psi = carPos[2];

    double global_x = globalPos[0];
    double global_y = globalPos[1];

    double x = global_x - car_x;
    double y = global_y - car_y;

    double x_dot = x*cos(car_psi)+y*sin(car_psi);
    double y_dot = -x*sin(car_psi)+y*cos(car_psi);

    auto ret = {x_dot, y_dot};

    return ret;
}
