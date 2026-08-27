#include "safety_monitor.hpp"

#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    const arl::RoverPose pose{100.0, 50.0, 90.0};
    const arl::SafetyConfig config{0.60, 40.0, 1.50, 1.0, 5.0};
    const double speedKph = 36.0;

    const std::vector<arl::Detection> detections{
        {"barrier", 18.0, 1.0, 0.90},
        {"crate", 25.0, -0.5, 0.75},
        {"sign", 30.0, -8.0, 0.55},
        {"debris", 45.0, 0.0, 0.99},
        {"cone", 12.0, 4.0, 0.95},
    };

    const auto obstacles = arl::processDetections(detections, pose, config);
    const auto nearest = arl::findNearestObstacle(obstacles);
    const double stoppingDistance = arl::calculateStoppingDistance(speedKph, config);
    const bool emergencyBrake = arl::shouldEmergencyBrake(obstacles, speedKph, config);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "ARL Rover Safety Monitor\n";
    std::cout << "Valid detections: " << obstacles.size() << '\n';

    if (nearest) {
        std::cout << "Nearest obstacle: " << nearest->id << '\n';
        std::cout << "World position: (" << nearest->worldX << ", "
                  << nearest->worldY << ")\n";
    } else {
        std::cout << "Nearest obstacle: none\n";
    }

    std::cout << "Stopping distance: " << stoppingDistance << " m\n";
    std::cout << "Emergency brake: " << (emergencyBrake ? "ENGAGE" : "clear") << '\n';

    return 0;
}
