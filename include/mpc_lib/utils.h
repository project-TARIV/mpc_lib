#ifndef MPC_U
#define MPC_U

#include <cmath>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Core>

double pi() { return M_PI; }

// For converting back and forth between radians and degrees.
double deg2rad(double x) {
    return x * pi() / 180;
}

double rad2deg(double x) {
    return x * 180 / pi();
}

double polyeval(Eigen::VectorXd coeffs, double x);

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif
