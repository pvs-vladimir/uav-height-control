#pragma once

#include <optional>

// Set the interface for all regulators
class Regulator {
public:
    virtual ~Regulator() = default;
    virtual double Calculate(double err, double dt) = 0;
};

class PIDRegulator : public Regulator {
public:
    PIDRegulator(double kp, double ki, double kd,
                 std::optional<double> i_saturation = std::nullopt,
                 std::optional<double> d_saturation = std::nullopt)
        : kp_(kp), ki_(ki), kd_(kd),
        i_satur_(i_saturation), d_satur_(d_saturation) {}
    
    double Calculate(double err, double dt) override;

private:
    double kp_, ki_, kd_;
    std::optional<double> i_satur_;
    std::optional<double> d_satur_;
    double err_sum_ = 0.0;
    double err_prev_ = 0.0;
};

class PIRegulator : public Regulator {
public:
    PIRegulator(double kp, double ki, std::optional<double> i_saturation = std::nullopt)
        : kp_(kp), ki_(ki), i_satur_(i_saturation) {}
    
    double Calculate(double err, double dt) override;

private:
    double kp_, ki_;
    std::optional<double> i_satur_;
    double err_sum_ = 0.0;
};

class PRegulator : public Regulator {
public:
    PRegulator(double kp) : kp_(kp) {}
    
    double Calculate(double err, double dt) override;

private:
    double kp_;
};

double Saturation(double x, double x_satur);
double Saturation(double x, double x_min, double x_max);