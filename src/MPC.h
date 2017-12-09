#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  // Function to transform waypoints from global coordinates to vehicle coordinates
  vector<double> MapToCarPos(vector<double> globalPos, vector<double> carPos);

  // Function to reduce the throttle if the steering angle is more than 15 degress
  double MapThrottleToSteeringAngle(double absSteeringAngle);
};

#endif /* MPC_H */
