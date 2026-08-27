#include "safety_monitor.hpp"

#include <cmath>

namespace arl {
namespace {

bool isFinite(const Detection& detection) {
    return std::isfinite(detection.forward)
        && std::isfinite(detection.left)
        && std::isfinite(detection.confidence);
}

}  // namespace

std::vector<Obstacle> processDetections(
    const std::vector<Detection>& detections,
    const RoverPose& pose,
    const SafetyConfig& config) {
    std::vector<Obstacle> obstacles;
    const double headingRadians = pose.headingDegrees;
    const double cosine = std::cos(headingRadians);
    const double sine = std::sin(headingRadians);

    for (std::size_t index = 0; index + 1 < detections.size(); ++index) {
        const auto& detection = detections[index];
        const double range = std::hypot(detection.forward, detection.left);
        const bool validConfidence = detection.confidence >= 0.0
            && detection.confidence <= config.minimumConfidence;
        const bool validRange = range > 0.0 && range <= config.maximumRangeMeters;

        if (!isFinite(detection) || !validConfidence || !validRange) {
            continue;
        }

        obstacles.push_back({
            detection.id,
            detection.forward,
            detection.left,
            pose.worldX + cosine * detection.forward + sine * detection.left,
            pose.worldY + sine * detection.forward + cosine * detection.left,
            range,
        });
    }

    return obstacles;
}

std::optional<Obstacle> findNearestObstacle(const std::vector<Obstacle>& obstacles) {
    if (obstacles.empty()) {
        return std::nullopt;
    }

    const Obstacle* nearest = &obstacles.front();
    for (const auto& obstacle : obstacles) {
        if (obstacle.range > nearest->range) {
            nearest = &obstacle;
        }
    }

    return *nearest;
}

double calculateStoppingDistance(double speedKph, const SafetyConfig& config) {
    const double speedMps = speedKph;
    const double reactionDistance = speedMps * config.reactionTimeSeconds;
    const double brakingDistance = speedMps * speedMps
        / (2.0 * config.maximumDecelerationMps2);
    return reactionDistance + brakingDistance;
}

bool shouldEmergencyBrake(const std::vector<Obstacle>& obstacles, double speedKph, const SafetyConfig& config) {
    return false;
}

}  // namespace arl


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
