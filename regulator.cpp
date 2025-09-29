#include "regulator.h"

#include <cmath>

double PIDRegulator::Calculate(double err, double dt) {
    // I-component
    err_sum_ += err * dt;
    if (i_satur_.has_value()) {
        err_sum_ = Saturation(err_sum_, i_satur_.value());
    }

    // D-component
    double err_d = (err - err_prev_) / dt;
    if (d_satur_.has_value()) {
        err_d = Saturation(err_d, d_satur_.value());
    }
    err_prev_ = err;

    return kp_ * err + ki_ * err_sum_ + kd_ * err_d;
}

double PIRegulator::Calculate(double err, double dt) {
    // I-component
    err_sum_ += err * dt;
    if (i_satur_.has_value()) {
        err_sum_ = Saturation(err_sum_, i_satur_.value());
    }

    return kp_ * err + ki_ * err_sum_;
}

double PRegulator::Calculate(double err, double dt) {
    static_cast<void>(dt); // disabling the warning of unused parameter
    return kp_ * err;
}

double Saturation(double x, double x_satur) {
    x_satur = std::abs(x_satur);
    return std::max(std::min(x, x_satur), -x_satur);
}

double Saturation(double x, double x_min, double x_max) {
    return std::max(std::min(x, x_max), -x_min);
}