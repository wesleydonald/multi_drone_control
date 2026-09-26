#include "tejen_dynamic_planner/reference_message.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "dynamic_planner/reference_window.hpp"

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected));
    }
}

double durationSeconds(const builtin_interfaces::msg::Duration& duration) {
    return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

}  // namespace

int main() {
    try {
        dynamic_planner::TrajectoryPiece piece;
        piece.valid_from = 100.0;
        piece.valid_until = 104.0;
        piece.knots = dynamic_planner::openUniformKnots(piece.valid_from, piece.valid_until, 1);
        piece.control_points = dynamic_planner::ControlPoints::Zero(4, 3);
        piece.control_points.row(0) = dynamic_planner::Vec3(0.0, 0.0, 1.5).transpose();
        piece.control_points.row(1) = dynamic_planner::Vec3(0.5, 0.3, 1.6).transpose();
        piece.control_points.row(2) = dynamic_planner::Vec3(1.4, -0.2, 1.4).transpose();
        piece.control_points.row(3) = dynamic_planner::Vec3(2.0, 0.4, 1.5).transpose();

        dynamic_planner::CommittedTrajectory committed;
        require(committed.replaceSuffix(piece.valid_from, piece).accepted,
                "failed to initialise moving committed trajectory");
        const dynamic_planner::ReferenceWindowConfig config;
        const auto window = dynamic_planner::sampleCommittedReferenceWindow(
            committed, 100.50, config);

        builtin_interfaces::msg::Time stamp;
        stamp.sec = 123;
        stamp.nanosec = 456U;

        const auto message = tejen_dynamic_planner::makeReferenceMessage(
            window, stamp, "world", "drone_0");

        require(message.header.stamp.sec == 123 && message.header.stamp.nanosec == 456U,
                "header timestamp changed during encoding");
        require(message.header.frame_id == "world", "frame_id mismatch");
        require(message.joint_names.size() == 1U && message.joint_names.front() == "drone_0",
                "vehicle joint name mismatch");
        require(message.points.size() == 61U, "MPC message must contain exactly 61 points");
        require(window.samples.front().state.velocity.norm() > 1e-6,
                "test trajectory must exercise nonzero velocity encoding");
        require(window.samples.front().state.acceleration.norm() > 1e-6,
                "test trajectory must exercise nonzero acceleration encoding");

        for (std::size_t k = 0; k < message.points.size(); ++k) {
            const auto& point = message.points[k];
            const auto& expected = window.samples[k].state;
            require(point.transforms.size() == 1U, "point must contain one transform");
            require(point.velocities.size() == 1U, "point must contain one velocity");
            require(point.accelerations.size() == 1U, "point must contain one acceleration");

            const auto& transform = point.transforms.front();
            requireNear(transform.translation.x, expected.position.x(), 1e-15, "position x changed");
            requireNear(transform.translation.y, expected.position.y(), 1e-15, "position y changed");
            requireNear(transform.translation.z, expected.position.z(), 1e-15, "position z changed");
            requireNear(transform.rotation.w, 1.0, 0.0, "orientation w is not identity");
            requireNear(transform.rotation.x, 0.0, 0.0, "orientation x is not identity");
            requireNear(transform.rotation.y, 0.0, 0.0, "orientation y is not identity");
            requireNear(transform.rotation.z, 0.0, 0.0, "orientation z is not identity");

            const auto& velocity = point.velocities.front().linear;
            requireNear(velocity.x, expected.velocity.x(), 1e-15, "velocity x changed");
            requireNear(velocity.y, expected.velocity.y(), 1e-15, "velocity y changed");
            requireNear(velocity.z, expected.velocity.z(), 1e-15, "velocity z changed");

            const auto& acceleration = point.accelerations.front().linear;
            requireNear(acceleration.x, expected.acceleration.x(), 1e-15, "acceleration x changed");
            requireNear(acceleration.y, expected.acceleration.y(), 1e-15, "acceleration y changed");
            requireNear(acceleration.z, expected.acceleration.z(), 1e-15, "acceleration z changed");

            const double expected_offset = static_cast<double>(k) / 30.0;
            requireNear(durationSeconds(point.time_from_start), expected_offset, 1.1e-9,
                        "time_from_start does not match 30 Hz sample offset");
        }

        // The current MPC samples every third 30 Hz point. Verify those stage
        // epochs are exactly the intended 0.1 s apart after ROS duration rounding.
        for (std::size_t stage = 0; stage <= 20U; ++stage) {
            const std::size_t sample_index = stage * 3U;
            requireNear(durationSeconds(message.points[sample_index].time_from_start),
                        0.1 * static_cast<double>(stage), 1.1e-9,
                        "MPC stage sample timing mismatch");
        }

        std::cout << "C.1a ROS reference-message contract: PASS\n";
        std::cout << "points: " << message.points.size() << "\n";
        std::cout << "stage spacing: 0.1 s; terminal: 2.0 s\n";
        std::cout << "p/v/a copied without resmoothing: true\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_reference_message FAILED: " << exc.what() << '\n';
        return 1;
    }
}
