#include <iostream>
#include <stdexcept>
#include <string>

#include <mavsdk/mavsdk.h>

#include "flight_operator.h"

using namespace mavsdk;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <connection_url> <target_height>" << std::endl;
        return 1;
    }
    auto connection_url = std::string(argv[1]);
    auto target_height = std::stof(argv[2]);

    Mavsdk uav{Mavsdk::Configuration{ComponentType::GroundStation}};
    ConnectionResult connection_result = uav.add_any_connection(connection_url.c_str());
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Error: Connection with " << connection_url << " failed: " << connection_result << std::endl;
        return 1;
    }
    std::cout << "The flight controller is successfully connected!\n";

    auto system = uav.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Error: Timed out waiting for system" << std::endl;
        return 1;
    }

    try {
        FlightOperator uav_operator(*system.value());
        uav_operator.ArmSystem();
        uav_operator.TakeoffToPosition(target_height);
        uav_operator.HoldPosition();
        uav_operator.Land();
        uav_operator.DisarmSystem();
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}