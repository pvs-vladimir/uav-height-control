#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "height_control_loop.h"

constexpr double CONTROL_DT_SECOND = 0.1;
constexpr double HOLD_DURATION_SECOND = 10.0;

class FlightOperator {
public:
    FlightOperator(mavsdk::System& system, double dt = CONTROL_DT_SECOND);
    bool IsSystemArmed() const;
    void ArmSystem();
    void DisarmSystem();
    void TakeoffToPosition(double pos);
    void HoldPosition(double time = HOLD_DURATION_SECOND);
    void Land();

private:
    mavsdk::System& system_;
    mavsdk::Telemetry telemetry_;
    mavsdk::Action action_;
    mavsdk::Offboard offboard_;
    HeightControlLoop controller_;
    double dt_;

    void MakeControlTact();
    void SetSystemControl(double U);
};