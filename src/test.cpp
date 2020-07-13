#include <mpc_lib/mpc.h>
#include <iostream>

int main() {
    mpc_lib::Params p{};
    p.forward.steps = 20;
    p.forward.frequency = 20;
    p.limits.vel = {-1, 1};
    p.limits.acc = {-0.1, 0.1};
    p.wheel_dist = 0.65; //meters
    p.v_ref = 1;
    p.wt = {100, 200, 400};

    mpc_lib::MPC mpc(p);

    mpc_lib::Dvector vec{2};
    vec[0] = -0.5;
    vec[1] = 1;
    mpc.global_plan = vec;
    mpc.state.v_r = 0.3;
    mpc.state.v_l = 0.7;

    mpc_lib::MPC::Result res;
    mpc.solve(res, false);
    return 0;
}