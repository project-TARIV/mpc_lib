#include <mpc_lib/dDrive_MPC.h>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <utility>

using CppAD::AD;
using Dvector = CppAD::vector<double>; // Can also use std::vector or std::valarray

void mpc_lib::VarIndices::setIndices(int N) {
    x_start = 0;
    y_start = x_start + N;
    theta_start = y_start + N;
    v_start = theta_start + N;
    cte_start = v_start + N;
    etheta_start = cte_start + N;
    omega_start = etheta_start + N;
    acc_start = omega_start + N - 1;
}

struct mpc_lib::VarIndices indices;

class FG_eval {
    mpc_lib::Params &_params;
public:
    using ADvector = CppAD::vector<CppAD::AD<double>>; // Can also use std::vector or std::valarray

    // Fitted polynomial coefficients
    Eigen::VectorXd &coeffs;

    FG_eval(Eigen::VectorXd &coeffs, mpc_lib::Params &params) : _params(params), coeffs(coeffs) {}

    void operator()(ADvector &fg, const ADvector &vars) {
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // Reference State Cost
        for (int t = 0; t < _params.N; t++) {
            fg[0] += _params.W_CTE * CppAD::pow(vars[indices.cte_start + t] - _params.ref_cte, 2);
            fg[0] += _params.W_ETHETA * CppAD::pow(vars[indices.etheta_start + t] - _params.ref_etheta, 2);
            fg[0] += _params.W_VEL * CppAD::pow(vars[indices.v_start + t] - _params.ref_v, 2);
        }
        for (int t = 0; t < _params.N - 1; t++) {
            fg[0] += _params.W_OMEGA * CppAD::pow(vars[indices.omega_start + t], 2);
            fg[0] += _params.W_ACC * CppAD::pow(vars[indices.acc_start + t], 2);
        }
        // Smoother transitions (less jerks)
        for (int t = 0; t < _params.N - 2; t++) {
            fg[0] += _params.W_ACC_D *
                     CppAD::pow(vars[indices.acc_start + t + 1] - vars[indices.acc_start + t], 2);
            fg[0] += _params.W_OMEGA_D *
                     CppAD::pow(vars[indices.omega_start + t + 1] - vars[indices.omega_start + t], 2);
        }
        //
        // Setup Constraints
        //

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + indices.x_start] = vars[indices.x_start];
        fg[1 + indices.y_start] = vars[indices.y_start];
        fg[1 + indices.theta_start] = vars[indices.theta_start];
        fg[1 + indices.v_start] = vars[indices.v_start];
        fg[1 + indices.cte_start] = vars[indices.cte_start];
        fg[1 + indices.etheta_start] = vars[indices.etheta_start];

        // The rest of the constraints
        for (int t = 0; t < _params.N - 1; t++) {

            // Time : T + 1
            AD<double> x1 = vars[indices.x_start + t + 1];
            AD<double> y1 = vars[indices.y_start + t + 1];
            AD<double> theta1 = vars[indices.theta_start + t + 1];
            AD<double> v1 = vars[indices.v_start + t + 1];
            AD<double> cte1 = vars[indices.cte_start + t + 1];
            AD<double> etheta1 = vars[indices.etheta_start + t + 1];

            // Time : T
            AD<double> x0 = vars[indices.x_start + t];
            AD<double> y0 = vars[indices.y_start + t];
            AD<double> theta0 = vars[indices.theta_start + t];
            AD<double> v0 = vars[indices.v_start + t];
            AD<double> cte0 = vars[indices.cte_start + t];
            AD<double> etheta0 = vars[indices.etheta_start + t];

            AD<double> w0 = vars[indices.omega_start + t];
            AD<double> a0 = vars[indices.acc_start + t];

            AD<double> f0 = 0.0;
            for (int i = 0; i < coeffs.size(); i++) {
                f0 += coeffs[i] * CppAD::pow(x0, i);
            }

            AD<double> traj_grad0 = 0.0;
            for (int i = 1; i < coeffs.size(); i++) {
                traj_grad0 += i * coeffs[i] * CppAD::pow(x0, i - 1);
            }
            traj_grad0 = CppAD::atan(traj_grad0);

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            fg[2 + indices.x_start + t] = x1 - (x0 + v0 * CppAD::cos(theta0) * _params.dt);
            fg[2 + indices.y_start + t] = y1 - (y0 + v0 * CppAD::sin(theta0) * _params.dt);
            fg[2 + indices.theta_start + t] = theta1 - (theta0 + w0 * _params.dt);
            fg[2 + indices.v_start + t] = v1 - (v0 + a0 * _params.dt);

            fg[2 + indices.cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(etheta0) * _params.dt));
            fg[2 + indices.etheta_start + t] = etheta1 - ((theta0 - traj_grad0) + w0 * _params.dt);
        }
    }
};

mpc_lib::MPC::MPC() = default;

mpc_lib::MPC::~MPC() = default;

bool mpc_lib::MPC::Solve(const mpc_lib::State &state, Eigen::VectorXd coeffs, std::vector<double> &result) {

    indices.setIndices(params.N); // TODO: move to set params

    // bool ok = true;


    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    const size_t n_vars = 6u * params.N + 2u * (params.N - 1u);

    // TODO: Set the number of constraints
    const size_t n_constraints = 6u * params.N;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) { // probably a better way
        vars[i] = 0;
    }

    // set the initial variable values
    vars[indices.x_start] = state.x;
    vars[indices.y_start] = state.y;
    vars[indices.theta_start] = state.theta;
    vars[indices.v_start] = state.v;
    vars[indices.cte_start] = state.cte;
    vars[indices.etheta_start] = state.etheta;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // TODO: Set lower and upper limits for variables.
    for (auto i = 0; i < indices.omega_start; i++) {
        vars_lowerbound[i] = -params.BOUND_VALUE;
        vars_upperbound[i] = params.BOUND_VALUE;
    }
    for (auto i = indices.omega_start; i < indices.acc_start; i++) {
        vars_lowerbound[i] = -params.MAX_OMEGA;
        vars_upperbound[i] = params.MAX_OMEGA;
    }
    for (auto i = indices.acc_start; i < n_vars; i++) {
        vars_lowerbound[i] = -params.MAX_THROTTLE;
        vars_upperbound[i] = params.MAX_THROTTLE;
    }
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[indices.x_start] = state.x;
    constraints_lowerbound[indices.y_start] = state.y;
    constraints_lowerbound[indices.theta_start] = state.theta;
    constraints_lowerbound[indices.v_start] = state.v;
    constraints_lowerbound[indices.cte_start] = state.cte;
    constraints_lowerbound[indices.etheta_start] = state.etheta;

    constraints_upperbound[indices.x_start] = state.x;
    constraints_upperbound[indices.y_start] = state.y;
    constraints_upperbound[indices.theta_start] = state.theta;
    constraints_upperbound[indices.v_start] = state.v;
    constraints_upperbound[indices.cte_start] = state.cte;
    constraints_upperbound[indices.etheta_start] = state.etheta;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs, params);

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
    if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
        std::cout << "NOT OKAY " << solution.status << std::endl;
        /*
           enum status_type {
                not_defined,
                success,
                maxiter_exceeded,
                stop_at_tiny_step,
                stop_at_acceptable_point,
                local_infeasibility,
                user_requested_stop,
                feasible_point_found,
                diverging_iterates,
                restoration_failure,
                error_in_step_computation,
                invalid_number_detected,
                too_few_degrees_of_freedom,
                internal_error,
                unknown
            };
         */
        return false;
    }

    // Cost
    std::cout << "COST  : " << solution.obj_value << std::endl;

    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    std::cout << solution.x.size() << std::endl;

    result = {solution.x[indices.omega_start], solution.x[indices.acc_start]};
    // Add "future" solutions (where MPC is going)
    for (int i = 0; i < params.N - 1; ++i) {
        result.push_back(solution.x[indices.x_start + i + 1]);
        result.push_back(solution.x[indices.y_start + i + 1]);
    }
    return true;
}