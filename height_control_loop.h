#pragma once

#include "regulator.h"

// The error of the height position within which it is considered stable reached.
constexpr double HEIGHT_ERROR_THRESHOLD = 0.1; // [m]
constexpr double DEFAULT_THRUST = 0.5;
constexpr double MAX_THRUST_MODUL = 1.0;

struct HeightControlCoef {
    // Default coefficients of HeightControlLoop
    double Kp_H = 1.5;
    double Ki_H = 0.01;
    double Kp_VH = 0.05;
    double i_saturation_H = 50.0; // [m]
    double saturation_VH = 10.0;  // [m/s]
    double saturation_U = MAX_THRUST_MODUL;
};

class HeightControlLoop {
public:
    HeightControlLoop(const HeightControlCoef& coefs, double H0 = 0.0);
    void SetTargetHeight(double target_H);
    double GetTargetHeight() const;
    bool IsTargetHeightReached() const;
    double CalcControl(double H, double VH, double dt);

private:
    PIRegulator controller_H_;
    PRegulator controller_VH_;
    double VH_satur_;
    double U_satur_;
    double target_H_;
    double H_;
};