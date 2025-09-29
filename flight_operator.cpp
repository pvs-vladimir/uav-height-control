#include "flight_operator.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace mavsdk;

FlightOperator::FlightOperator(System& system, double dt) 
    : system_(system),
      telemetry_(Telemetry{system}),
      action_(Action{system}),
      offboard_(Offboard{system}),
      controller_(HeightControlCoef()),
      dt_(dt) {
    std::cout << std::fixed << std::setprecision(2);
}

bool FlightOperator::IsSystemArmed() const {
    return telemetry_.armed();
}

void FlightOperator::ArmSystem() {
    if (IsSystemArmed()) return;
    auto arm_result = action_.arm();
    if (arm_result != Action::Result::Success) {
        throw std::runtime_error("Arming failed!");
    }
    std::cout << "System armed successfully!\n";
    auto start_result = offboard_.start();
    if (start_result != Offboard::Result::Success) {
        throw std::runtime_error("Offboard connection failed!");
    }
    std::cout << "Offboard connected successfully!\n";
}

void FlightOperator::DisarmSystem() {
    if (!IsSystemArmed()) return;
    auto stop_result = offboard_.stop();
    if (stop_result != Offboard::Result::Success) {
        throw std::runtime_error("Offboard stopping failed!");
    }
    std::cout << "Offboard stopped successfully!\n";
    auto disarm_result = action_.disarm();
    if (disarm_result != Action::Result::Success) {
        throw std::runtime_error("Disarming failed!");
    }
    std::cout << "System disarmed successfully!\n";
}

void FlightOperator::TakeoffToPosition(double pos) {
    if (!IsSystemArmed()) ArmSystem();
    std::cout << "Taking off to height of " << pos << "meters ...\n";
    controller_.SetTargetHeight(pos);
    while (!controller_.IsTargetHeightReached()) {
        MakeControlTact();
    }
    std::cout << "Height of " << pos << "meters reached!\n";
}

void FlightOperator::HoldPosition(double time) {
    double H = (controller_.IsTargetHeightReached())
             ? controller_.GetTargetHeight()
             : telemetry_.position().relative_altitude_m;
    controller_.SetTargetHeight(H);
    std::cout << "Holding the height of " << H << " meters for " << time << " seconds ...\n";
    for (int i = 0; i < static_cast<int>(time / dt_); ++i) {
        MakeControlTact();
    }
    std::cout << "Hold duration complete.\n";
}

void FlightOperator::Land() {
    if (telemetry_.in_air()) {
        std::cout << "Landing ...\n";
        controller_.SetTargetHeight(0.0);
        while (telemetry_.in_air()) {
            MakeControlTact();
        }
        std::cout << "System is landed!\n";
    }
}

void FlightOperator::MakeControlTact() {
    double H = telemetry_.position().relative_altitude_m;
    double VH = telemetry_.velocity_ned().down_m_s;
    double U = controller_.CalcControl(H, VH, dt_);
    SetSystemControl(U);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)));
}

void FlightOperator::SetSystemControl(double U) {
    float thrust = DEFAULT_THRUST + U / 2.0;
    Offboard::ActuatorControl actuator_control;
    for (int i = 0; i < 4; ++i) {
        actuator_control.groups[0].controls[i] = thrust;
    }
    offboard_.set_actuator_control(actuator_control);
}