#pragma once

#include <string>
#include <vector>

#include "dynamic_planner/octopus_search.hpp"

namespace dynamic_planner {

struct ValidationScenario {
    std::string name;
    std::vector<double> knots;
    ControlPoints initial_control_points;
    Vec3 goal = Vec3::Zero();
    std::vector<TimeIndexedObstacle> obstacles;
    OctopusConfig octopus_config;
};

ValidationScenario makeValidationScenario(const std::string& name,
                                          int samples_per_axis = 7);

std::vector<std::string> validationScenarioNames();

}  // namespace dynamic_planner
