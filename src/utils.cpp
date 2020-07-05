#include <mpc_lib/utils.h>

double mpc_lib::polyeval(Eigen::VectorXd coeffs, double x) {

    double result = 0.0;

    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }

    return result;
}

Eigen::VectorXd mpc_lib::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    Eigen::VectorXd result = A.householderQr().solve(yvals);

    return result;
}
