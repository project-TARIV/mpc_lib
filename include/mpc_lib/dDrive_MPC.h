#ifndef D_MPC
#define D_MPC

#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Core>
#include <cppad/cppad.hpp>

#include <vector>


namespace mpc_lib {

    struct Params {
        int N{};
        double dt{};

        double ref_cte{};
        double ref_etheta{};
        double ref_v{};

        double MAX_OMEGA{};
        double MAX_THROTTLE{};
        double BOUND_VALUE{1.0e3};

        double W_CTE{};
        double W_ETHETA{};
        double W_VEL{};
        double W_OMEGA{};
        double W_ACC{};
        double W_OMEGA_D{};
        double W_ACC_D{};
    };
    struct State {
        double x, y, theta, v, cte, etheta;
    };

    class FG_eval {
        mpc_lib::Params &_params;
    public:

        // Below decleration required by ipopt
        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;
        using ADvector = CppAD::vector<CppAD::AD<double>>; // Can also use std::vector or std::valarray

        explicit FG_eval(mpc_lib::Params &params);

        void operator()(ADvector &fg, const ADvector &vars);
    };

    class MPC {
        FG_eval fg_eval;

    public:
        MPC();

        virtual ~MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuations.
        bool solve(const State &state, const Eigen::VectorXd &coeffs, std::vector<double> &result);


        Params params;
    };
}
#endif