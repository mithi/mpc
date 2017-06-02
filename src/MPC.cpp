#include "MPC.h"

using CppAD::AD;
using namespace std;

class FG_eval {

  public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    Eigen::VectorXd K; // Fitted road curve polynomial coefficients

    FG_eval(Eigen::VectorXd Kin) : K(Kin) {}

    void operator()(ADvector& fg, const ADvector& x) {
      // fg a vector containing the cost and all constraints
      // x is a vector containing all states and actuations for N "lookahead" states and actuations.

      //*********************************************************
      //* COST DEFINED HERE
      //*********************************************************

      fg[0] = 0.0;

      for (int i = 0; i < N; ++i) {

        const auto cte = x[ID_FIRST_cte + i];
        const auto epsi = x[ID_FIRST_epsi + i];
        const auto v = x[ID_FIRST_v + i] - VELOCITY_MAX;

        fg[0] += (W_cte * cte * cte + W_epsi * epsi * epsi + W_v * v * v);
      }

      for (int i = 0; i < N - 1; ++i) {

        const auto delta = x[ID_FIRST_delta + i];
        const auto a = x[ID_FIRST_a + i];

        fg[0] += (W_delta * delta * delta + W_a * a * a);
      }

      for (int i = 0; i < N - 2; ++i) {

        const auto ddelta = x[ID_FIRST_delta + i + 1] - x[ID_FIRST_delta + i];
        const auto da = x[ID_FIRST_a + i + 1] - x[ID_FIRST_a + i];

        fg[0] += (W_ddelta * ddelta * ddelta + W_da * da * da);
      }

      //*********************************************************
      //* CONSTRAINTS DEFINED HERE
      //*********************************************************

      // given state does not vary
      fg[ID_FIRST_px + 1] = x[ID_FIRST_px];
      fg[ID_FIRST_py + 1] = x[ID_FIRST_py];
      fg[ID_FIRST_psi + 1] = x[ID_FIRST_psi];
      fg[ID_FIRST_v + 1] = x[ID_FIRST_v];
      fg[ID_FIRST_cte + 1] = x[ID_FIRST_cte];
      fg[ID_FIRST_epsi + 1] = x[ID_FIRST_epsi];

      // constraints based on our kinematic model
      for (int i = 0; i < N - 1; ++i) {

        // where the current state variables of interest are stored
        // stored for readability
        const int ID_CURRENT_px = ID_FIRST_px + i;
        const int ID_CURRENT_py = ID_FIRST_py + i;
        const int ID_CURRENT_psi = ID_FIRST_psi + i;
        const int ID_CURRENT_v = ID_FIRST_v + i;
        const int ID_CURRENT_cte = ID_FIRST_cte + i;
        const int ID_CURRENT_epsi = ID_FIRST_epsi + i;
        const int ID_CURRENT_delta = ID_FIRST_delta + i;
        const int ID_CURRENT_a = ID_FIRST_a + i;

        //current state and actuations
        const auto px0 = x[ID_CURRENT_px];
        const auto py0 = x[ID_CURRENT_py];
        const auto psi0 = x[ID_CURRENT_psi];
        const auto v0 = x[ID_CURRENT_v];
        const auto cte0 = x[ID_CURRENT_cte];
        const auto epsi0 = x[ID_CURRENT_epsi];
        const auto delta0 = x[ID_CURRENT_delta];
        const auto a0 = x[ID_CURRENT_a];

        // next state
        const auto px1 = x[ID_CURRENT_px + 1];
        const auto py1 = x[ID_CURRENT_py + 1];
        const auto psi1 = x[ID_CURRENT_psi + 1];
        const auto v1 = x[ID_CURRENT_v + 1];
        const auto cte1 = x[ID_CURRENT_cte + 1];
        const auto epsi1 = x[ID_CURRENT_epsi + 1];

        // desired py and psi
        const auto py_desired = K[3] * px0 * px0 * px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
        const auto psi_desired = CppAD::atan(3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]);

        // relationship of current state + actuations and next state
        // based on our kinematic model
        const auto px1_f = px0 + v0 * CppAD::cos(psi0) * dt;
        const auto py1_f = py0 + v0 * CppAD::sin(psi0) * dt;
        const auto psi1_f = psi0 + v0 * (-delta0) / Lf * dt;
        const auto v1_f = v0 + a0 * dt;
        const auto cte1_f = py_desired - py0 + v0 * CppAD::sin(epsi0) * dt;
        const auto epsi1_f = psi0 - psi_desired + v0 * (-delta0) / Lf * dt;

        // store the constraint expression of two consecutive states
        fg[ID_CURRENT_px + 2] = px1 - px1_f;
        fg[ID_CURRENT_py + 2] = py1 - py1_f;
        fg[ID_CURRENT_psi + 2] = psi1 - psi1_f;
        fg[ID_CURRENT_v + 2] = v1 - v1_f;
        fg[ID_CURRENT_cte + 2] = cte1 - cte1_f;
        fg[ID_CURRENT_epsi + 2] = epsi1 - epsi1_f;
      }
    }
};

MPC::MPC() {

  //**************************************************************
  //* SET INITIAL VALUES OF VARIABLES
  //**************************************************************
  this->x.resize(NX);

  // all states except the ID_FIRST are set to zero
  // the aformentioned states will be initialized when solve() is called

  for (int i = 0; i < NX; ++i) {
    this->x[i] = 0.0;
  }

  //**************************************************************
  //* SET UPPER AND LOWER LIMITS OF VARIABLES
  //**************************************************************

  this->x_lowerbound.resize(NX);
  this->x_upperbound.resize(NX);

  // all other values large values the computer can handle
  for (int i = 0; i < ID_FIRST_delta; ++i) {
    this->x_lowerbound[i] = -1.0e10;
    this->x_upperbound[i] = 1.0e10;
  }

  // all actuation inputs (steering, acceleration) should have values between [-1, 1]
  for (int i = ID_FIRST_delta; i < ID_FIRST_a; ++i) {
    this->x_lowerbound[i] = -0.75;
    this->x_upperbound[i] = 0.75;
  }

  for (int i = ID_FIRST_a; i < NX; ++i) {
    this->x_lowerbound[i] = -0.5;
    this->x_upperbound[i] = 1.0;
  }

  //**************************************************************
  //* SET UPPER AND LOWER LIMITS OF CONSTRAINTS
  //**************************************************************
  this->g_lowerbound.resize(NG);
  this->g_upperbound.resize(NG);

  // the first constraint for each state veriable
  // refer to the initial state conditions
  // this will be initialized when solve() is called
  // the succeeding constraints refer to the relationship
  // between succeeding states based on our kinematic model of the system

  for (int i = 0; i < NG; ++i) {
    this->g_lowerbound[i] = 0.0;
    this->g_upperbound[i] = 0.0;
  }
}

MPC::~MPC() {}

void MPC::solve(Eigen::VectorXd state, Eigen::VectorXd K) {

  const double px = state[0];
  const double py = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  this->x[ID_FIRST_px] = px;
  this->x[ID_FIRST_py] = py;
  this->x[ID_FIRST_psi] = psi;
  this->x[ID_FIRST_v] = v;
  this->x[ID_FIRST_cte] = cte;
  this->x[ID_FIRST_epsi] = epsi;

  this->g_lowerbound[ID_FIRST_px] = px;
  this->g_lowerbound[ID_FIRST_py] = py;
  this->g_lowerbound[ID_FIRST_psi] = psi;
  this->g_lowerbound[ID_FIRST_v] = v;
  this->g_lowerbound[ID_FIRST_cte] = cte;
  this->g_lowerbound[ID_FIRST_epsi] = epsi;

  this->g_upperbound[ID_FIRST_px] = px;
  this->g_upperbound[ID_FIRST_py] = py;
  this->g_upperbound[ID_FIRST_psi] = psi;
  this->g_upperbound[ID_FIRST_v] = v;
  this->g_upperbound[ID_FIRST_cte] = cte;
  this->g_upperbound[ID_FIRST_epsi] = epsi;

  //**************************************************************
  //* SOLVE
  //**************************************************************

  // object that computes objective and constraints
  FG_eval fg_eval(K);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options,
      x,
      x_lowerbound,
      x_upperbound,
      g_lowerbound,
      g_upperbound,
      fg_eval,
      solution);

  // comment out the lines below to debug!
  /*
  bool ok = true;
  auto cost = solution.obj_value;

  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  if (ok) {
    std::cout << "OK! Cost:" << cost << std::endl;
  } else {
    std::cout << "SOMETHING IS WRONG!" << cost << std::endl;
  }
  */

  //**************************************************************
  //* STORE RELEVANT INFORMATION FROM SOLUTION
  //**************************************************************

  this->steer = solution.x[ID_FIRST_delta];
  this->throttle = solution.x[ID_FIRST_a];

  this->future_xs = {};
  this->future_ys = {};

  for (int i = 0; i < N; ++i) {

    const double px = solution.x[ID_FIRST_px + i];
    const double py = solution.x[ID_FIRST_py + i];

    this->future_xs.emplace_back(px);
    this->future_ys.emplace_back(py);
  }
}
