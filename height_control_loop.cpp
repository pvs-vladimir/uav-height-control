#include "height_control_loop.h"

#include <cmath>

HeightControlLoop::HeightControlLoop(const HeightControlCoef& coefs, double H0) 
    : controller_H_(coefs.Kp_H, coefs.Ki_H, coefs.i_saturation_H),
      controller_VH_(coefs.Kp_VH),
      VH_satur_(coefs.saturation_VH),
      target_H_(H0), H_(H0) {
}

void HeightControlLoop::SetTargetHeight(double target_H) {
    target_H_ = target_H;
}

double HeightControlLoop::GetTargetHeight() const {
    return target_H_;
}

bool HeightControlLoop::IsTargetHeightReached() const {
    return std::abs(target_H_ - H_) < HEIGHT_ERROR_THRESHOLD;
}

double HeightControlLoop::CalcControl(double H, double VH, double dt) {
    H_ = H;
    double err_H = target_H_ - H;
    double target_VH = controller_H_.Calculate(err_H, dt);
    target_VH = Saturation(target_VH, VH_satur_);
    double err_VH = target_VH - VH;
    double control_U = controller_VH_.Calculate(err_VH, dt);
    return Saturation(control_U, U_satur_);
}