#ifndef MPC_LIB_MPC_H
#define MPC_LIB_MPC_H

#include <cppad/cppad.hpp>
//#include <eigen3/Eigen/Core>
#include <map>

#include "mpc_lib/helpers.h"

/*
 * Using MPC:
 *
 * Initialize MPC object with a Params object
 *
 * For every iteration:
 *   Set the state and global plan member variables.
 *   Call solve to get the acceleration.
 * Profit
 *
 * Intended to be use by mpc_local_planner
 *
 * Written for a differential drive vehicle path planning task.
 *
 */

/*
 * Following are many helper type definitions.
 * Then there is the actual class
 */
namespace mpc_lib {

    // Uncomment only ONE of the following
    template<typename T>
    // using vector = std::vector<T>;
    // using vector = std::valarray<T>;
    using vector = CppAD::vector<T>;
    // using vector = eigen::vector<T>;
    // TODO: Choose vector container.

    // Non-differentiable vector of doubles, specialises vector
    using Dvector = vector<double>;

    // Structure that stores the current state
    template<typename T>
    struct State_ {
        T x, y, theta, v_r, v_l;
    };

    // Non-differentiable State type
    using State = State_<Dvector::value_type>;


    // Stores a pair of values named 'low' and 'high'
    template<typename T>
    struct LH {
        T low, high;
    };


    struct Params {
        // For forward simulation of the robots state
        struct Forward {
            double frequency;   // Hz
            size_t steps;       // number of time steps
        } forward;

        // Max and Min values of velocity and acc
        struct Limits {
            LH<double> vel, acc;
        } limits;

        // NOTE: Params all have different scales
        // e.g. etheta is in the range -1 to 1 and cte is in the range -5 tot 5
        // So the same weights values actually mean different things on different parameters
        struct Weights {
            double acc, vel, omega, cte, etheta, obs;
        } wt;

        // Target velocity to be maintained
        double v_ref;

        // Distance between the wheels
        /*unsigned*/ double wheel_dist; // meters
    };


    class MPC {
        // Type definitions for differentiable values.
    public:
        using ADvector = vector<CppAD::AD<double>>; // Exposed publicallyfor ipopt
    private:
        // required in  void operator()(...)
        using ADState = State_<ADvector::value_type &>;


        /*
         * TODO: Spit the constructor into a constructor and initialize method
         * Allow params to be updated via the initilize method.
         */
        // Parameters
        const Params params;
        const double dt; /* = 1 / params.forward.frequency */
        const size_t &steps;

        std::string options; // TODO: Use the Params to build the options string


        // The following class sucks to declare, but is very easy to use in code
        // Ignore this class definition if you are going through this

        // TODO: Better declaration format
        /* If you want to add more variables or constraints please ping me (suraj) */

        /*
         * Stores indices of variables and constraints
         * Usage:
         *   To get a_r (acceleration_right), call a_r
         *   The resulting object acts like an iterator,
         *   i.e: ++ to get go to the next timestep,
         *   and * to get the indice at the timestep..
         *
         *   Can be used as:
         *   Indices indices(5);
         *
         *   auto ar = indices.a_r()
         *   for (auto & indice : indices) {
         *       std::cout << vars[indice] << std::endl;
         *   }
         *
         *   // OR:
         *   auto ar = indices.a_r();
         *   while(<condition>) {
         *       std::cout << vars[*ar] << std::endl;
         *       ++ar;
         *   }
         *
         *   Use indices.a_r(2) to start from the second timestep.
         *
         *   You can access the indice at the nth time step by:
         *   indices.a_r()[n];
         *
         * See Range.
        */
        const class Indices {
            const size_t time_steps; // Do we need to store this?

            // Indices for variables
        private:
            const size_t _a_r, _a_l;
        public:
            const size_t num_variables;

            [[nodiscard]] Range a_r(size_t offset = 0) const { return {0 + offset, _a_r}; }

            [[nodiscard]] Range a_l(size_t offset = 0) const { return {_a_r + offset, _a_l}; }


            // Indices for constraints
        private:
            const size_t _v_r, _v_l;
        public:
            const size_t num_constraints;

            [[nodiscard]] Range v_r(size_t offset = 0) const { return {0 + offset, _v_r}; }

            [[nodiscard]] Range v_l(size_t offset = 0) const { return {_v_r + offset, _v_l}; }


            // Constructor
        public:
            explicit Indices(const size_t ts /* time steps*/) :
                    time_steps{ts},

                    // Variables
                    _a_r{ts}, _a_l{_a_r + ts},
                    num_variables{_a_l},

                    // Constraints
                    _v_r{ts}, _v_l{_v_r + ts},
                    num_constraints{_v_l} {}

        } indices;


        // Variables
        Dvector _vars;
        LH<Dvector> vars_b, cons_b;

        // Calculates the State at each timestep from the velocities (stored in the constraints) and initial state.
        // Store in the given vector
        void get_states(const Dvector &constraints, const State &initial, std::vector<State> &states) const;

    public:

        explicit MPC(Params p);

        // NOTE: You must update these before invoking solve(...)
        State state;
        Dvector global_plan;
        std::vector<Dvector> obstacles;
        CppAD::AD<double> directionality{1}; // Should be +- 1 ONLY
        /*  ^^
         * If we go along the global plan by going in the +x direction, this shou;d be set as +1.
         * If going along the +x direction causes us to go backwards (away from goal), this should be -1
         *
         * Currently not used.
         */

        // Result of solve
        struct Result {
            size_t status;
            std::pair<double, double> acc;
            std::vector<State> path;
        };

        // Calculate's optimal acceleration for given state and constraints.
        // TODO: take previous acceleration?
        bool solve(Result &result, bool get_path = false);


        // Get human readable string from Result::status
        const static std::map<size_t, std::string> error_string;


        /*
         * This calculates the cost function and constraints from variables
         * You should not branch based on variable values (i think),
         * i.e no if-else or while conditions based on variable value.
         * Only use mathematical operators and CppAD mathematical functions.
         * Intermediate values should be of CppAD::AD<> type.
         * ONLY MEANT TO BE CALLED BY IPOPT.
         *
         */
        void operator()(ADvector &outputs, ADvector &vars) const;
    };

}

#endif //MPC_LIB_MPC_H
