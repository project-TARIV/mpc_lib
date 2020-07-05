#ifndef D_MPC
#define D_MPC

#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Core>

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

    class MPC {
    public:
        MPC();

        virtual ~MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuations.
        bool solve(const State &state, Eigen::VectorXd coeffs, std::vector<double> &result);


        Params params;
    };
}
#endif