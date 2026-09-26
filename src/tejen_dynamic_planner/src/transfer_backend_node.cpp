#include "tejen_dynamic_planner/c1e_scene.hpp"
#include "tejen_dynamic_planner/kinematics_filter.hpp"
#include "tejen_dynamic_planner/moving_basket_geometry.hpp"
#include "tejen_dynamic_planner/reference_message.hpp"

#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/committed_trajectory.hpp"
#include "dynamic_planner/ego_collision_model.hpp"
#include "dynamic_planner/minimum_time.hpp"
#include "dynamic_planner/receding_horizon_planner.hpp"
#include "dynamic_planner/reference_window.hpp"
#include "dynamic_planner/target_predictor.hpp"
#include "dynamic_planner/trajectory_safety_checker.hpp"
#include "dynamic_planner/world_snapshot.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <interfaces/msg/motion_capture_state.hpp>
#include <interfaces/msg/committed_trajectory.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace tejen_dynamic_planner {
namespace {

using dynamic_planner::AttachedTetherGeometry;
using dynamic_planner::CommittedTrajectory;
using dynamic_planner::CooperativeObstacleTrajectory;
using dynamic_planner::LocalPlannerConfig;
using dynamic_planner::RecedingHorizonConfig;
using dynamic_planner::RecedingHorizonPlanner;
using dynamic_planner::ReplanResult;
using dynamic_planner::State;
using dynamic_planner::SuspendedGeometry;
using dynamic_planner::TargetPrediction;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::TrajectoryTimeSource;
using dynamic_planner::Vec3;
using dynamic_planner::VersionedWorld;
using dynamic_planner::WorldSnapshot;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kControlReferenceHz = 30.0;
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kMpcAuthorityTopic = "/join_planner/reference";

SuspendedGeometry suspendedGeometry(double max_swing_angle_rad) {
    SuspendedGeometry geometry;
    geometry.enabled = true;
    geometry.cable_length_m = 0.50;
    geometry.cable_radius_m = 0.0025;
    geometry.max_swing_angle_rad = max_swing_angle_rad;
    geometry.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    geometry.magnet_center_below_cable_end_m = 0.025;
    // Payload geometry is configured below and activated from the live
    // /magnet/object_attached state only when the C1F.3 loaded-geometry mode is enabled.
    geometry.payload_attached = false;
    geometry.payload_half_extents = Vec3(0.05, 0.01, 0.003);
    geometry.payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.028);
    return geometry;
}

RecedingHorizonConfig recedingConfig(
    double initial_timing_seed_s,
    double fixed_splice_lookahead_s,
    double spline_time_factor,
    double factor_alloc,
    double factor_alloc_close,
    double close_to_goal_m,
    double incumbent_failure_recheck_horizon_s,
    double terminal_hold_guard_s,
    double terminal_hold_guard_interval_s,
    bool require_terminal_hold_for_nominal,
    bool require_rendezvous_backup_for_nominal,
    bool defer_incumbent_conflict_until_reaction_horizon,
    double emergency_panic_horizon_s,
    bool enable_octopus_useful_deadline,
    double octopus_post_search_reserve_s,
    bool moving_rendezvous_enabled,
    const Vec3& moving_rendezvous_terminal_velocity,
    bool terminal_time_target_prediction_enabled,
    int target_time_fixed_point_iterations,
    double target_time_fixed_point_tolerance_s,
    double target_prediction_low_confidence_horizon_s,
    double target_prediction_high_confidence_horizon_s,
    double target_prediction_low_confidence_max_distance_m) {
    RecedingHorizonConfig config;
    config.dc_s = 1.0 / kControlReferenceHz;
    config.planning_radius_m = 2.0;
    config.factor_alpha = 2.5;
    config.min_splice_lookahead_s = 0.05;
    config.max_splice_lookahead_s = 1.0;
    config.fixed_splice_lookahead_s = fixed_splice_lookahead_s;
    config.factor_alloc = factor_alloc;
    config.factor_alloc_close = factor_alloc_close;
    config.spline_time_factor = spline_time_factor;
    config.close_to_goal_m = close_to_goal_m;
    config.goal_tolerance_m = 0.05;
    config.continuity_tolerance = 1e-7;
    config.separator_validation_tolerance = 1e-7;
    config.initial_splice_timing_s = std::max(0.0, initial_timing_seed_s);
    config.enable_octopus_useful_deadline = enable_octopus_useful_deadline;
    config.octopus_post_search_reserve_s = octopus_post_search_reserve_s;
    config.incumbent_failure_recheck_horizon_s = incumbent_failure_recheck_horizon_s;
    config.terminal_hold_guard_s = terminal_hold_guard_s;
    config.terminal_hold_guard_interval_s = terminal_hold_guard_interval_s;
    config.require_terminal_hold_for_nominal = require_terminal_hold_for_nominal;
    config.require_rendezvous_backup_for_nominal = require_rendezvous_backup_for_nominal;
    config.defer_incumbent_conflict_until_reaction_horizon =
        defer_incumbent_conflict_until_reaction_horizon;
    config.emergency_panic_horizon_s = emergency_panic_horizon_s;
    config.moving_rendezvous_enabled = moving_rendezvous_enabled;
    config.moving_rendezvous_terminal_velocity = moving_rendezvous_terminal_velocity;
    config.terminal_time_target_prediction_enabled =
        terminal_time_target_prediction_enabled;
    config.target_time_fixed_point_iterations = target_time_fixed_point_iterations;
    config.target_time_fixed_point_tolerance_s = target_time_fixed_point_tolerance_s;
    config.target_prediction_low_confidence_horizon_s =
        target_prediction_low_confidence_horizon_s;
    config.target_prediction_high_confidence_horizon_s =
        target_prediction_high_confidence_horizon_s;
    config.target_prediction_low_confidence_max_distance_m =
        target_prediction_low_confidence_max_distance_m;
    config.rendezvous_backup_brake_accel_limit = Vec3(1.0, 1.0, 1.5);
    config.rendezvous_backup_min_duration_s = 0.25;
    config.v_max = Vec3::Ones();
    config.a_max = Vec3(1.0, 1.0, 1.5);
    return config;
}

LocalPlannerConfig localConfig(
    const Vec3& start,
    const Vec3& goal,
    double minimum_search_z_m,
    double smoother_jerk_weight,
    double smoother_goal_weight,
    double octopus_max_runtime_s) {
    LocalPlannerConfig config;
    config.num_segments = 4;
    config.octopus.samples_per_axis = {7, 7, 7};
    config.octopus.alpha_shrink = 0.9;
    config.octopus.voxel_fraction = 0.10;
    config.octopus.heuristic_bias = 1.0;
    config.octopus.max_runtime_s = octopus_max_runtime_s;
    const Vec3 padding(1.0, 1.0, 1.0);
    config.octopus.xyz_min = start.cwiseMin(goal) - padding;
    config.octopus.xyz_max = start.cwiseMax(goal) + padding;
    config.octopus.xyz_min.z() = std::max(minimum_search_z_m, config.octopus.xyz_min.z());
    if (config.octopus.xyz_max.z() <= config.octopus.xyz_min.z() + 0.2) {
        config.octopus.xyz_max.z() = config.octopus.xyz_min.z() + 0.2;
    }
    config.octopus.random_seed = 1;
    config.refinement.xyz_min = config.octopus.xyz_min;
    config.refinement.xyz_max = config.octopus.xyz_max;
    config.refinement.j_max = Vec3::Constant(4.0);
    config.refinement.jerk_weight = smoother_jerk_weight;
    config.refinement.goal_weight = smoother_goal_weight;
    config.refinement.max_working_set_recalculations = 500;
    return config;
}

struct CooperativeStateSample {
    bool received = false;
    Vec3 position = Vec3::Zero();
    Vec3 velocity = Vec3::Zero();
    double receive_ros_s = std::numeric_limits<double>::quiet_NaN();
};

struct CooperativeTrajectorySample {
    bool received = false;
    std::uint64_t sequence = 0U;
    CommittedTrajectory trajectory;
    double receive_ros_s = std::numeric_limits<double>::quiet_NaN();
};

struct CooperativeAttachedPlateSample {
    bool received = false;
    int plate_id = -1;
    double receive_ros_s = std::numeric_limits<double>::quiet_NaN();
};

struct RingPoseSample {
    bool received = false;
    Vec3 position = Vec3::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    double receive_ros_s = std::numeric_limits<double>::quiet_NaN();
};

CommittedTrajectory trajectoryFromMessage(
    const interfaces::msg::CommittedTrajectory& msg,
    const Vec3& position_offset = Vec3::Zero()) {
    if (msg.pieces.empty()) {
        throw std::invalid_argument("shared cooperative commitment has no spline pieces");
    }
    CommittedTrajectory trajectory;
    for (const auto& msg_piece : msg.pieces) {
        TrajectoryPiece piece;
        piece.valid_from = msg_piece.valid_from_s;
        piece.valid_until = msg_piece.valid_until_s;
        piece.knots.assign(msg_piece.knots.begin(), msg_piece.knots.end());
        piece.control_points.resize(
            static_cast<Eigen::Index>(msg_piece.control_points.size()), 3);
        for (std::size_t i = 0; i < msg_piece.control_points.size(); ++i) {
            const auto& p = msg_piece.control_points[i];
            piece.control_points.row(static_cast<Eigen::Index>(i)) =
                Vec3(p.x, p.y, p.z).transpose();
        }
        piece.validate();
        if (position_offset.squaredNorm() > 0.0) {
            piece = translatedCollisionTrajectoryPiece(piece, position_offset);
        }
        const auto diagnostics = trajectory.replaceSuffix(
            piece.valid_from, piece, 1e-5);
        if (!diagnostics.accepted) {
            throw std::invalid_argument(
                "shared cooperative commitment pieces are not C2-continuous");
        }
    }
    if (msg.terminal_hold && !trajectory.endsInStoppedHold(1e-6)) {
        throw std::invalid_argument(
            "shared commitment declares terminal hold but does not end stopped");
    }
    return trajectory;
}

CommittedTrajectory constantVelocityPrediction(
    const Vec3& position,
    const Vec3& velocity,
    double t0_s,
    double horizon_s) {
    if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(t0_s) ||
        !std::isfinite(horizon_s) || !(horizon_s > 0.0)) {
        throw std::invalid_argument("invalid cooperative constant-velocity prediction");
    }
    const double t1_s = t0_s + horizon_s;
    TrajectoryPiece piece;
    piece.knots = dynamic_planner::openUniformKnots(t0_s, t1_s, 1);
    piece.control_points.resize(4, 3);
    piece.control_points.row(0) = position.transpose();
    piece.control_points.row(1) = (position + velocity * (horizon_s / 3.0)).transpose();
    piece.control_points.row(2) = (position + velocity * (2.0 * horizon_s / 3.0)).transpose();
    piece.control_points.row(3) = (position + velocity * horizon_s).transpose();
    piece.valid_from = t0_s;
    piece.valid_until = t1_s;

    CommittedTrajectory trajectory;
    const auto diagnostics = trajectory.replaceSuffix(t0_s, piece);
    if (!diagnostics.accepted) {
        throw std::runtime_error("failed to construct cooperative CV trajectory");
    }
    return trajectory;
}

std::string safeCsvField(std::string text) {
    std::replace(text.begin(), text.end(), ',', ';');
    std::replace(text.begin(), text.end(), '\n', ' ');
    return text;
}

geometry_msgs::msg::Point markerPoint(const Vec3& point) {
    geometry_msgs::msg::Point message;
    message.x = point.x();
    message.y = point.y();
    message.z = point.z();
    return message;
}

std_msgs::msg::ColorRGBA markerColour(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA colour;
    colour.r = r;
    colour.g = g;
    colour.b = b;
    colour.a = a;
    return colour;
}

Marker baseMarker(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& marker_namespace,
    int id,
    int type) {
    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
}

Marker lineStripMarker(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const std::vector<Vec3>& points,
    float width,
    const std_msgs::msg::ColorRGBA& colour) {
    Marker marker = baseMarker(frame_id, stamp, marker_namespace, id, Marker::LINE_STRIP);
    marker.scale.x = width;
    marker.color = colour;
    marker.points.reserve(points.size());
    for (const auto& point : points) marker.points.push_back(markerPoint(point));
    return marker;
}

struct FinalCheckFailureWitness {
    std::string obstacle_name;
    int interval_index = -1;
    std::optional<double> geometric_gap_m;
};

std::optional<FinalCheckFailureWitness> finalCheckFailureWitness(
    const ReplanResult& result) {
    if (!result.local_plan.has_value()) return std::nullopt;
    const auto& search = result.local_plan->search;
    for (auto it = search.separators.rbegin(); it != search.separators.rend(); ++it) {
        if (!it->result.feasible) {
            return FinalCheckFailureWitness{
                it->obstacle_name, it->interval_index, it->result.geometric_gap_m};
        }
    }
    return std::nullopt;
}

Marker pointsMarker(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const std::vector<Vec3>& points,
    float scale,
    const std_msgs::msg::ColorRGBA& colour) {
    Marker marker = baseMarker(frame_id, stamp, marker_namespace, id, Marker::POINTS);
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.color = colour;
    marker.points.reserve(points.size());
    for (const auto& point : points) marker.points.push_back(markerPoint(point));
    return marker;
}

Marker sphereMarker(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const Vec3& centre,
    float diameter,
    const std_msgs::msg::ColorRGBA& colour) {
    Marker marker = baseMarker(frame_id, stamp, marker_namespace, id, Marker::SPHERE);
    marker.pose.position = markerPoint(centre);
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.color = colour;
    return marker;
}

Marker cubeMarker(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const Vec3& centre,
    const Vec3& half_extents,
    const std_msgs::msg::ColorRGBA& colour) {
    Marker marker = baseMarker(frame_id, stamp, marker_namespace, id, Marker::CUBE);
    marker.pose.position = markerPoint(centre);
    marker.scale.x = 2.0 * half_extents.x();
    marker.scale.y = 2.0 * half_extents.y();
    marker.scale.z = 2.0 * half_extents.z();
    marker.color = colour;
    return marker;
}

Marker cylinderMarker(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const Vec3& centre,
    double diameter,
    double height,
    const std_msgs::msg::ColorRGBA& colour) {
    Marker marker = baseMarker(frame_id, stamp, marker_namespace, id, Marker::CYLINDER);
    marker.pose.position = markerPoint(centre);
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = height;
    marker.color = colour;
    return marker;
}

Marker componentAabbMarker(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const dynamic_planner::ConvexComponent& component,
    const Vec3& origin,
    const Vec3& extra_half_extents,
    const std_msgs::msg::ColorRGBA& colour) {
    Vec3 center;
    Vec3 half_extents;
    dynamic_planner::componentAabb(component, &center, &half_extents);
    return cubeMarker(
        frame_id, stamp, marker_namespace, id, origin + center,
        half_extents + extra_half_extents, colour);
}

Marker cableEnvelopeSurfaceMarker(
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const dynamic_planner::ConvexComponent& cable,
    const Vec3& origin,
    const std_msgs::msg::ColorRGBA& colour) {
    if (cable.vertices.rows() != 8 || cable.vertices.cols() != 3) {
        throw std::invalid_argument("cable envelope visualization expects 8 vertices");
    }
    Marker marker = baseMarker(
        frame_id, stamp, marker_namespace, id, Marker::TRIANGLE_LIST);
    marker.color = colour;
    // makeCableComponent() orders the first four vertices on the top square and
    // the final four on the wide bottom square. Draw the exact frustum surface
    // used by the collision component rather than an unrelated bounding cylinder.
    static constexpr int kTriangles[12][3] = {
        {0, 2, 3}, {0, 3, 1},  // top
        {4, 5, 7}, {4, 7, 6},  // bottom
        {0, 4, 6}, {0, 6, 2},  // -y
        {1, 3, 7}, {1, 7, 5},  // +y
        {0, 1, 5}, {0, 5, 4},  // -x
        {2, 6, 7}, {2, 7, 3},  // +x
    };
    marker.points.reserve(36U);
    for (const auto& triangle : kTriangles) {
        for (const int index : triangle) {
            marker.points.push_back(markerPoint(
                origin + cable.vertices.row(index).transpose()));
        }
    }
    return marker;
}

std::string optionalSafetyStatus(
    const std::optional<dynamic_planner::TrajectorySafetyResult>& result) {
    return result.has_value() ? result->status : std::string();
}

struct PlannerOutcome {
    std::uint64_t sequence = 0U;
    std::uint64_t epoch = 0U;
    double request_ros_time_s = 0.0;
    double octopus_wall_budget_s = std::numeric_limits<double>::quiet_NaN();
    State request_state;
    Vec3 target = Vec3::Zero();
    std::unique_ptr<RecedingHorizonPlanner> planner;
    ReplanResult result;
    bool worker_exception = false;
    std::string worker_error;
};

struct ScopedSteadyDurationRecorder {
    using Clock = std::chrono::steady_clock;

    double* last_ms = nullptr;
    double* max_ms = nullptr;
    std::size_t* overrun_count = nullptr;
    double budget_ms = std::numeric_limits<double>::infinity();
    Clock::time_point started = Clock::now();

    ~ScopedSteadyDurationRecorder() {
        const double elapsed_ms = 1e3 *
            std::chrono::duration<double>(Clock::now() - started).count();
        if (last_ms) *last_ms = elapsed_ms;
        if (max_ms) *max_ms = std::max(*max_ms, elapsed_ms);
        if (overrun_count && elapsed_ms > budget_ms) ++(*overrun_count);
    }
};

}  // namespace

class TransferBackendNode final : public rclcpp::Node {
public:
    TransferBackendNode()
        : Node("dynamic_planner_transfer_backend"),
          kinematics_(declare_parameter<double>("acceleration_filter_tau_s", 0.15)) {
        state_topic_ = declare_parameter<std::string>("state_topic", "/motion_capture_state");
        target_topic_ = declare_parameter<std::string>(
            "target_topic", "/dynamic_planner/transfer_target");
        target_velocity_topic_ = declare_parameter<std::string>(
            "target_velocity_topic", "/dynamic_planner/transfer_target_velocity");
        object_attached_topic_ = declare_parameter<std::string>(
            "object_attached_topic", "/magnet/object_attached");
        enable_topic_ = declare_parameter<std::string>(
            "enable_topic", "/dynamic_planner/transfer_enable");
        authority_topic_ = declare_parameter<std::string>(
            "authority_topic", "/dynamic_planner/transfer_authority");
        authority_ack_topic_ = declare_parameter<std::string>(
            "authority_ack_topic", "/dynamic_planner/transfer_authority_ack");
        shadow_reference_topic_ = declare_parameter<std::string>(
            "shadow_reference_topic", "/dynamic_planner/transfer_reference");
        diagnostics_topic_ = declare_parameter<std::string>(
            "diagnostics_topic", "/dynamic_planner/transfer_status");
        marker_topic_ = declare_parameter<std::string>(
            "marker_topic", "/dynamic_planner/markers");
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        vehicle_id_ = declare_parameter<std::string>("vehicle_id", "drone_0");
        csv_path_ = declare_parameter<std::string>("csv_path", "");
        replan_csv_path_ = declare_parameter<std::string>("replan_csv_path", "");

        state_timeout_s_ = declare_parameter<double>("state_timeout_s", 0.25);
        object_attached_timeout_s_ = declare_parameter<double>(
            "object_attached_timeout_s", 0.25);
        planner_rate_hz_ = declare_parameter<double>("planner_rate_hz", 5.0);
        marker_rate_hz_ = declare_parameter<double>("marker_rate_hz", 5.0);
        minimum_search_z_m_ = declare_parameter<double>("minimum_search_z_m", 0.20);
        initial_splice_timing_s_ = declare_parameter<double>("initial_splice_timing_s", 0.16);
        fixed_splice_lookahead_s_ = declare_parameter<double>(
            "fixed_splice_lookahead_s", 0.0);
        octopus_max_runtime_s_ = declare_parameter<double>(
            "octopus_max_runtime_s", 2.0);
        octopus_pre_authority_max_runtime_s_ = declare_parameter<double>(
            "octopus_pre_authority_max_runtime_s", octopus_max_runtime_s_);
        reuse_prepared_commit_on_authority_ = declare_parameter<bool>(
            "reuse_prepared_commit_on_authority", false);
        spline_time_factor_ = declare_parameter<double>("spline_time_factor", 3.0);
        factor_alloc_ = declare_parameter<double>("factor_alloc", 1.0);
        factor_alloc_close_ = declare_parameter<double>("factor_alloc_close", 2.5);
        close_to_goal_m_ = declare_parameter<double>("close_to_goal_m", 0.20);
        smoother_jerk_weight_ = declare_parameter<double>("smoother_jerk_weight", 1.0);
        smoother_goal_weight_ = declare_parameter<double>("smoother_goal_weight", 10.0);
        incumbent_failure_recheck_horizon_s_ = declare_parameter<double>(
            "incumbent_failure_recheck_horizon_s", 2.0);
        terminal_hold_guard_s_ = declare_parameter<double>(
            "terminal_hold_guard_s", 0.0);
        terminal_hold_guard_interval_s_ = declare_parameter<double>(
            "terminal_hold_guard_interval_s", 0.50);
        planning_policy_profile_ = declare_parameter<std::string>(
            "planning_policy_profile", "conservative");
        require_terminal_hold_for_nominal_ = declare_parameter<bool>(
            "require_terminal_hold_for_nominal", true);
        require_rendezvous_backup_for_nominal_ = declare_parameter<bool>(
            "require_rendezvous_backup_for_nominal", true);
        defer_incumbent_conflict_until_reaction_horizon_ = declare_parameter<bool>(
            "defer_incumbent_conflict_until_reaction_horizon", false);
        emergency_panic_horizon_s_ = declare_parameter<double>(
            "emergency_panic_horizon_s", 0.50);
        enable_octopus_useful_deadline_ = declare_parameter<bool>(
            "enable_octopus_useful_deadline", true);
        octopus_post_search_reserve_s_ = declare_parameter<double>(
            "octopus_post_search_reserve_s", 1.0 / kControlReferenceHz);

        require_stationary_activation_ = declare_parameter<bool>(
            "require_stationary_activation", true);
        activation_speed_tolerance_mps_ = declare_parameter<double>(
            "activation_speed_tolerance_mps", 0.10);
        activation_accel_tolerance_mps2_ = declare_parameter<double>(
            "activation_accel_tolerance_mps2", 0.50);
        fixed_target_change_tolerance_m_ = declare_parameter<double>(
            "fixed_target_change_tolerance_m", 1e-4);
        allow_live_target_updates_ = declare_parameter<bool>(
            "allow_live_target_updates", false);
        live_target_replan_threshold_m_ = declare_parameter<double>(
            "live_target_replan_threshold_m", 0.01);
        require_target_velocity_ = declare_parameter<bool>(
            "require_target_velocity", false);
        target_lead_enabled_ = declare_parameter<bool>(
            "target_lead_enabled", false);
        target_lead_nominal_speed_mps_ = declare_parameter<double>(
            "target_lead_nominal_speed_mps", 0.35);
        target_lead_min_s_ = declare_parameter<double>(
            "target_lead_min_s", 0.0);
        target_lead_max_s_ = declare_parameter<double>(
            "target_lead_max_s", 2.0);
        target_lead_high_confidence_max_s_ = declare_parameter<double>(
            "target_lead_high_confidence_max_s", 10.0);
        target_lead_max_distance_m_ = declare_parameter<double>(
            "target_lead_max_distance_m", 0.0);
        target_predictor_type_ = declare_parameter<std::string>(
            "target_predictor_type", "constant_velocity");
        scripted_target_center_ = Vec3(
            declare_parameter<double>("scripted_target_center_x_m", 0.0),
            declare_parameter<double>("scripted_target_center_y_m", 0.0),
            declare_parameter<double>("scripted_target_center_z_m", 0.0));
        scripted_target_radius_m_ = declare_parameter<double>(
            "scripted_target_radius_m", 0.5);
        scripted_target_omega_rad_s_ = declare_parameter<double>(
            "scripted_target_omega_rad_s", 0.25);
        target_lead_initial_solve_runtime_s_ = declare_parameter<double>(
            "target_lead_initial_solve_runtime_s", 0.50);
        target_lead_solve_runtime_scale_ = declare_parameter<double>(
            "target_lead_solve_runtime_scale", 1.0);
        target_lead_solve_runtime_ema_alpha_ = declare_parameter<double>(
            "target_lead_solve_runtime_ema_alpha", 0.35);
        arrival_time_target_lead_enabled_ = declare_parameter<bool>(
            "arrival_time_target_lead_enabled", false);
        const std::int64_t requested_target_lead_iterations =
            declare_parameter<std::int64_t>("target_lead_fixed_point_iterations", 3);
        if (requested_target_lead_iterations > std::numeric_limits<int>::max()) {
            throw std::invalid_argument(
                "target_lead_fixed_point_iterations exceeds the supported integer range");
        }
        target_lead_fixed_point_iterations_ = static_cast<int>(
            std::max<std::int64_t>(1, requested_target_lead_iterations));
        terminal_time_target_prediction_enabled_ = declare_parameter<bool>(
            "terminal_time_target_prediction_enabled", false);
        const std::int64_t requested_target_time_iterations =
            declare_parameter<std::int64_t>("target_time_fixed_point_iterations", 6);
        if (requested_target_time_iterations > std::numeric_limits<int>::max()) {
            throw std::invalid_argument(
                "target_time_fixed_point_iterations exceeds the supported integer range");
        }
        target_time_fixed_point_iterations_ = static_cast<int>(
            std::max<std::int64_t>(1, requested_target_time_iterations));
        target_time_fixed_point_tolerance_s_ = declare_parameter<double>(
            "target_time_fixed_point_tolerance_s", 0.01);
        moving_rendezvous_enabled_ = declare_parameter<bool>(
            "moving_rendezvous_enabled", false);
        live_target_velocity_replan_threshold_mps_ = declare_parameter<double>(
            "live_target_velocity_replan_threshold_mps", 0.01);
        estimated_solve_runtime_s_ = target_lead_initial_solve_runtime_s_;
        seed_stationary_on_activation_ = declare_parameter<bool>(
            "seed_stationary_on_activation", true);

        max_swing_angle_rad_ = declare_parameter<double>(
            "max_swing_angle_rad", 15.0 * kPi / 180.0);
        payload_collision_enabled_ = declare_parameter<bool>(
            "payload_collision_enabled", false);
        require_payload_attached_for_enable_ = declare_parameter<bool>(
            "require_payload_attached_for_enable", false);
        payload_half_extents_ = Vec3(
            declare_parameter<double>("payload_half_x_m", 0.05),
            declare_parameter<double>("payload_half_y_m", 0.01),
            declare_parameter<double>("payload_half_z_m", 0.003));
        payload_center_from_magnet_center_ = Vec3(
            declare_parameter<double>("payload_center_from_magnet_x_m", 0.0),
            declare_parameter<double>("payload_center_from_magnet_y_m", 0.0),
            declare_parameter<double>("payload_center_from_magnet_z_m", -0.028));
        static_scene_enabled_ = declare_parameter<bool>("static_scene_enabled", true);
        require_scene_witness_ = declare_parameter<bool>("require_scene_witness", true);
        obstacle_name_ = declare_parameter<std::string>(
            "obstacle_name", "c1e_commissioning_obstacle");
        obstacle_centre_ = Vec3(
            declare_parameter<double>("obstacle_center_x_m", 0.50),
            declare_parameter<double>("obstacle_center_y_m", 0.06),
            declare_parameter<double>("obstacle_center_z_m", 0.68));
        obstacle_half_extents_ = Vec3(
            declare_parameter<double>("obstacle_half_x_m", 0.12),
            declare_parameter<double>("obstacle_half_y_m", 0.25),
            declare_parameter<double>("obstacle_half_z_m", 0.10));
        obstacle_yaw_rad_ = declare_parameter<double>(
            "obstacle_yaw_rad", 20.0 * kPi / 180.0);

        cooperative_scene_enabled_ = declare_parameter<bool>(
            "cooperative_scene_enabled", false);
        cooperative_drone_state_topics_ = declare_parameter<std::vector<std::string>>(
            "cooperative_drone_state_topics", std::vector<std::string>{});
        cooperative_prediction_mode_ = declare_parameter<std::string>(
            "cooperative_prediction_mode", "constant_velocity");
        cooperative_committed_trajectory_topics_ = declare_parameter<std::vector<std::string>>(
            "cooperative_committed_trajectory_topics", std::vector<std::string>{});
        cooperative_vehicle_ids_ = declare_parameter<std::vector<std::string>>(
            "cooperative_vehicle_ids", std::vector<std::string>{});
        cooperative_state_timeout_s_ = declare_parameter<double>(
            "cooperative_state_timeout_s", 0.30);
        cooperative_prediction_horizon_s_ = declare_parameter<double>(
            "cooperative_prediction_horizon_s", 20.0);
        cooperative_physical_half_extents_ = Vec3(
            declare_parameter<double>("cooperative_body_half_x_m", 0.105),
            declare_parameter<double>("cooperative_body_half_y_m", 0.105),
            declare_parameter<double>("cooperative_body_half_z_m", 0.060));
        cooperative_tracking_error_half_extents_ = Vec3(
            declare_parameter<double>("cooperative_tracking_half_x_m", 0.10),
            declare_parameter<double>("cooperative_tracking_half_y_m", 0.10),
            declare_parameter<double>("cooperative_tracking_half_z_m", 0.10));
        cooperative_attached_tether_enabled_ = declare_parameter<bool>(
            "cooperative_attached_tether_enabled", false);
        cooperative_attached_plate_topics_ = declare_parameter<std::vector<std::string>>(
            "cooperative_attached_plate_topics", std::vector<std::string>{});
        cooperative_ring_pose_topic_ = declare_parameter<std::string>(
            "cooperative_ring_pose_topic", "/model/payload_model/pose");
        cooperative_ring_pose_index_ = declare_parameter<int>(
            "cooperative_ring_pose_index", 1);
        cooperative_ring_pose_timeout_s_ = declare_parameter<double>(
            "cooperative_ring_pose_timeout_s", 0.50);
        cooperative_ring_plate_count_ = declare_parameter<int>(
            "cooperative_ring_plate_count", 12);
        cooperative_ring_plate_pitch_diameter_m_ = declare_parameter<double>(
            "cooperative_ring_plate_pitch_diameter_m", 0.50);
        cooperative_ring_angle_zero_rad_ = declare_parameter<double>(
            "cooperative_ring_angle_zero_rad", 0.0);
        cooperative_attached_tether_radius_m_ = declare_parameter<double>(
            "cooperative_attached_tether_radius_m", 0.015);
        cooperative_tether_anchor_from_body_ = Vec3(
            declare_parameter<double>("cooperative_tether_anchor_x_m", 0.0),
            declare_parameter<double>("cooperative_tether_anchor_y_m", 0.0),
            declare_parameter<double>("cooperative_tether_anchor_z_m", -0.04));

        // Moving-ring collision geometry is deliberately separate from target
        // prediction. The commitment always remains the true attachment-plane ring
        // centre. M1 keeps its legacy solid net envelope; M2D selects the 24-bar
        // segmented fixture model while consuming the same commitment.
        moving_basket_scene_enabled_ = declare_parameter<bool>(
            "moving_basket_scene_enabled", false);
        moving_basket_committed_trajectory_topic_ = declare_parameter<std::string>(
            "moving_basket_committed_trajectory_topic", "/fake_payload/committed_trajectory");
        moving_basket_collision_mode_ = declare_parameter<std::string>(
            "moving_basket_collision_mode", "solid_box");
        moving_basket_geometry_.outer_diameter_m = declare_parameter<double>(
            "moving_basket_collision_outer_diameter_m", 0.56);
        moving_basket_geometry_.top_offset_m = declare_parameter<double>(
            "moving_basket_collision_top_offset_m", 0.0);
        moving_basket_geometry_.bottom_offset_m = declare_parameter<double>(
            "moving_basket_collision_bottom_offset_m", -0.27);
        moving_basket_geometry_.validate();
        moving_basket_physical_half_extents_ = moving_basket_geometry_.halfExtents();
        moving_basket_tracking_error_half_extents_ = Vec3(
            declare_parameter<double>("moving_basket_tracking_half_x_m", 0.05),
            declare_parameter<double>("moving_basket_tracking_half_y_m", 0.05),
            declare_parameter<double>("moving_basket_tracking_half_z_m", 0.05));
        segmented_ring_geometry_.segment_count = declare_parameter<int>(
            "moving_ring_segment_count", 24);
        segmented_ring_geometry_.segment_center_radius_m = declare_parameter<double>(
            "moving_ring_segment_center_radius_m", 0.25);
        segmented_ring_geometry_.tangential_length_m = declare_parameter<double>(
            "moving_ring_segment_tangential_length_m", 0.068067840828);
        segmented_ring_geometry_.radial_width_m = declare_parameter<double>(
            "moving_ring_segment_radial_width_m", 0.060);
        segmented_ring_geometry_.height_m = declare_parameter<double>(
            "moving_ring_segment_height_m", 0.030);
        segmented_ring_geometry_.center_z_offset_m = declare_parameter<double>(
            "moving_ring_segment_center_z_offset_m", -0.020);
        segmented_ring_geometry_.padding_m = declare_parameter<double>(
            "moving_ring_segment_padding_m", 0.005);
        segmented_ring_geometry_.validate();

        // C1F.4 closed-loop execution certificate. These bounds serve two roles:
        // (1) inflate planned ego geometry so admitted tracking error is part of
        // the collision certificate, and (2) invalidate the temporal certificate
        // if measured tracking leaves that tube during authority.
        execution_tracking_safety_enabled_ = declare_parameter<bool>(
            "execution_tracking_safety_enabled", false);
        ego_tracking_error_half_extents_ = Vec3(
            declare_parameter<double>("ego_tracking_half_x_m", 0.0),
            declare_parameter<double>("ego_tracking_half_y_m", 0.0),
            declare_parameter<double>("ego_tracking_half_z_m", 0.0));
        execution_tracking_violation_samples_ = declare_parameter<int>(
            "execution_tracking_violation_samples", 3);
        execution_velocity_error_limit_mps_ = declare_parameter<double>(
            "execution_velocity_error_limit_mps", 0.0);
        execution_brake_accel_limit_ = Vec3(
            declare_parameter<double>("execution_brake_accel_x_mps2", 1.0),
            declare_parameter<double>("execution_brake_accel_y_mps2", 1.0),
            declare_parameter<double>("execution_brake_accel_z_mps2", 1.5));
        execution_brake_min_duration_s_ = declare_parameter<double>(
            "execution_brake_min_duration_s", 0.25);

        validateParameters();
        planner_period_s_ = 1.0 / planner_rate_hz_;

        reference_config_.reference_rate_hz = kControlReferenceHz;
        reference_config_.mpc_horizon_stages = 20U;
        reference_config_.mpc_skip_steps = 3U;
        reference_config_.validate();
        if (reference_config_.requiredSampleCount() != 61U) {
            throw std::logic_error("C1F.0 expected the inspected 61-sample MPC contract");
        }

        suspended_geometry_ = suspendedGeometry(max_swing_angle_rad_);
        suspended_geometry_.payload_half_extents = payload_half_extents_;
        suspended_geometry_.payload_center_from_magnet_center =
            payload_center_from_magnet_center_;
        // Fail-safe startup: payload geometry is not activated until the live
        // attachment-state topic explicitly reports object_attached=true.
        suspended_geometry_.payload_attached = false;
        WorldSnapshot initial_world;
        initial_world.captured_at_s = 0.0;
        initial_world.ego_half_extents = body_half_extents_;
        initial_world.ego_tracking_error_half_extents = ego_tracking_error_half_extents_;
        initial_world.ego_suspended_geometry = suspended_geometry_;
        if (static_scene_enabled_) {
            physical_obstacle_ = makeYawedCuboidObstacle(
                obstacle_name_, obstacle_centre_, obstacle_half_extents_, obstacle_yaw_rad_);
            initial_world.physical_static_obstacles.push_back(*physical_obstacle_);
        }
        world_ = std::make_shared<VersionedWorld>(std::move(initial_world));
        scene_witness_passed_ = !require_scene_witness_;
        scene_witness_summary_ = require_scene_witness_ ? "PENDING" : "NOT_REQUIRED";

        state_sub_ = create_subscription<interfaces::msg::MotionCaptureState>(
            state_topic_, rclcpp::QoS(10),
            std::bind(&TransferBackendNode::stateCallback, this, std::placeholders::_1));
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            target_topic_, rclcpp::QoS(10),
            std::bind(&TransferBackendNode::targetCallback, this, std::placeholders::_1));
        target_velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            target_velocity_topic_, rclcpp::QoS(10),
            std::bind(&TransferBackendNode::targetVelocityCallback, this, std::placeholders::_1));
        object_attached_sub_ = create_subscription<std_msgs::msg::Bool>(
            object_attached_topic_, rclcpp::QoS(10),
            std::bind(&TransferBackendNode::objectAttachedCallback, this, std::placeholders::_1));
        cooperative_state_samples_.resize(cooperative_drone_state_topics_.size());
        cooperative_state_subs_.reserve(cooperative_drone_state_topics_.size());
        for (std::size_t i = 0; i < cooperative_drone_state_topics_.size(); ++i) {
            cooperative_state_subs_.push_back(
                create_subscription<interfaces::msg::MotionCaptureState>(
                    cooperative_drone_state_topics_[i], rclcpp::QoS(10),
                    [this, i](const interfaces::msg::MotionCaptureState::SharedPtr msg) {
                        cooperativeStateCallback(i, msg);
                    }));
        }
        cooperative_trajectory_samples_.resize(cooperative_drone_state_topics_.size());
        cooperative_attached_plate_samples_.resize(cooperative_drone_state_topics_.size());
        if (cooperative_attached_tether_enabled_) {
            rclcpp::QoS attached_plate_qos(1);
            attached_plate_qos.reliable().transient_local();
            cooperative_attached_plate_subs_.reserve(cooperative_attached_plate_topics_.size());
            for (std::size_t i = 0; i < cooperative_attached_plate_topics_.size(); ++i) {
                cooperative_attached_plate_subs_.push_back(
                    create_subscription<std_msgs::msg::Int32>(
                        cooperative_attached_plate_topics_[i], attached_plate_qos,
                        [this, i](const std_msgs::msg::Int32::SharedPtr msg) {
                            cooperativeAttachedPlateCallback(i, msg);
                        }));
            }
        }
        if (cooperative_attached_tether_enabled_ || moving_basket_collision_mode_ == "segmented_ring") {
            cooperative_ring_pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
                cooperative_ring_pose_topic_, rclcpp::QoS(10),
                std::bind(&TransferBackendNode::cooperativeRingPoseCallback, this, std::placeholders::_1));
        }
        if (cooperative_prediction_mode_ == "shared_trajectory") {
            rclcpp::QoS trajectory_qos(1);
            trajectory_qos.reliable().transient_local();
            cooperative_trajectory_subs_.reserve(cooperative_committed_trajectory_topics_.size());
            for (std::size_t i = 0; i < cooperative_committed_trajectory_topics_.size(); ++i) {
                cooperative_trajectory_subs_.push_back(
                    create_subscription<interfaces::msg::CommittedTrajectory>(
                        cooperative_committed_trajectory_topics_[i], trajectory_qos,
                        [this, i](const interfaces::msg::CommittedTrajectory::SharedPtr msg) {
                            cooperativeTrajectoryCallback(i, msg);
                        }));
            }
        }
        if (moving_basket_scene_enabled_) {
            rclcpp::QoS basket_qos(1);
            basket_qos.reliable().transient_local();
            moving_basket_trajectory_sub_ =
                create_subscription<interfaces::msg::CommittedTrajectory>(
                    moving_basket_committed_trajectory_topic_, basket_qos,
                    std::bind(&TransferBackendNode::movingBasketTrajectoryCallback,
                              this, std::placeholders::_1));
        }
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            enable_topic_, rclcpp::QoS(10),
            std::bind(&TransferBackendNode::enableCallback, this, std::placeholders::_1));
        authority_sub_ = create_subscription<std_msgs::msg::Bool>(
            authority_topic_, rclcpp::QoS(10),
            std::bind(&TransferBackendNode::authorityCallback, this, std::placeholders::_1));
        shadow_reference_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            shadow_reference_topic_, rclcpp::QoS(10));
        diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
            diagnostics_topic_, rclcpp::QoS(10));
        authority_ack_pub_ = create_publisher<std_msgs::msg::Bool>(
            authority_ack_topic_, rclcpp::QoS(10));
        rclcpp::QoS marker_qos(1);
        marker_qos.reliable().transient_local();
        marker_pub_ = create_publisher<MarkerArray>(marker_topic_, marker_qos);

        reference_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(1.0 / kControlReferenceHz),
            std::bind(&TransferBackendNode::referenceTimer, this));
        replan_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(planner_period_s_),
            std::bind(&TransferBackendNode::replanTimer, this));
        diagnostics_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(1.0),
            std::bind(&TransferBackendNode::diagnosticsTimer, this));
        marker_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(1.0 / marker_rate_hz_),
            [this]() { publishMarkers(); });

        openLogs();
        RCLCPP_WARN(get_logger(),
            "Transfer backend never publishes the MPC authority topic directly. Internal reference=%s; authority topic %s is untouched.",
            shadow_reference_topic_.c_str(), kMpcAuthorityTopic);
        RCLCPP_INFO(get_logger(),
            "BODY target=%s target_velocity=%s enable=%s authority=%s ack=%s planner=%.1f Hz reference=30 Hz spline_factor=%.2f far/close=%.2f/%.2f close_m=%.2f smoother(jerk/goal)=%.2f/%.2f octopus_active/preauth=%.1f/%.1f ms useful_deadline=%s reserve=%.1f ms live_retarget=%s lead=%s lead_cap_m=%.2f solve_ema_init=%.2f s payload_collision=%s require_attached=%s cooperative=%s coop_count=%zu exec_tracking=%s exec_bounds=[%.2f %.2f %.2f] markers=%s",
            target_topic_.c_str(), target_velocity_topic_.c_str(), enable_topic_.c_str(), authority_topic_.c_str(), authority_ack_topic_.c_str(), planner_rate_hz_,
            spline_time_factor_, factor_alloc_, factor_alloc_close_, close_to_goal_m_,
            smoother_jerk_weight_, smoother_goal_weight_,
            1e3 * octopus_max_runtime_s_, 1e3 * octopus_pre_authority_max_runtime_s_,
            enable_octopus_useful_deadline_ ? "ON" : "OFF",
            1e3 * octopus_post_search_reserve_s_, allow_live_target_updates_ ? "ON" : "OFF",
            target_lead_enabled_ ? "ON" : "OFF", target_lead_max_distance_m_, estimated_solve_runtime_s_,
            payload_collision_enabled_ ? "ON" : "OFF",
            require_payload_attached_for_enable_ ? "ON" : "OFF",
            cooperative_scene_enabled_ ? "ON" : "OFF", cooperative_drone_state_topics_.size(),
            execution_tracking_safety_enabled_ ? "ON" : "OFF",
            ego_tracking_error_half_extents_.x(), ego_tracking_error_half_extents_.y(),
            ego_tracking_error_half_extents_.z(), marker_topic_.c_str());
    }

    ~TransferBackendNode() override {
        if (planner_future_.valid()) planner_future_.wait();
        if (csv_) csv_.flush();
        if (replan_csv_) replan_csv_.flush();
    }

private:
    void validateParameters() const {
        if (shadow_reference_topic_ == kMpcAuthorityTopic) {
            throw std::invalid_argument(
                "C1F.0 is PASSIVE ONLY: shadow_reference_topic must not be /join_planner/reference");
        }
        if (state_topic_.empty() || target_topic_.empty() || target_velocity_topic_.empty() || enable_topic_.empty() ||
            authority_topic_.empty() || authority_ack_topic_.empty() ||
            shadow_reference_topic_.empty() || diagnostics_topic_.empty() || marker_topic_.empty() ||
            frame_id_.empty() || vehicle_id_.empty() || !(state_timeout_s_ > 0.0) ||
            !(object_attached_timeout_s_ > 0.0) ||
            !(planner_rate_hz_ > 0.0) || planner_rate_hz_ > kControlReferenceHz ||
            !(marker_rate_hz_ > 0.0) || marker_rate_hz_ > kControlReferenceHz ||
            !std::isfinite(minimum_search_z_m_) || !std::isfinite(initial_splice_timing_s_) ||
            !(initial_splice_timing_s_ >= 0.0) ||
            !std::isfinite(fixed_splice_lookahead_s_) ||
            !(fixed_splice_lookahead_s_ >= 0.0) || fixed_splice_lookahead_s_ > 1.0 ||
            !std::isfinite(octopus_max_runtime_s_) || !(octopus_max_runtime_s_ > 0.0) ||
            !std::isfinite(octopus_pre_authority_max_runtime_s_) ||
            !(octopus_pre_authority_max_runtime_s_ > 0.0) ||
            !std::isfinite(spline_time_factor_) ||
            !(spline_time_factor_ > 0.0) || !std::isfinite(factor_alloc_) ||
            !(factor_alloc_ > 0.0) || !std::isfinite(factor_alloc_close_) ||
            !(factor_alloc_close_ > 0.0) || !std::isfinite(close_to_goal_m_) ||
            !(close_to_goal_m_ >= 0.05) || !std::isfinite(smoother_jerk_weight_) ||
            !(smoother_jerk_weight_ >= 0.0) || !std::isfinite(smoother_goal_weight_) ||
            !(smoother_goal_weight_ > 0.0) ||
            !std::isfinite(incumbent_failure_recheck_horizon_s_) ||
            !(incumbent_failure_recheck_horizon_s_ > 0.0) ||
            !std::isfinite(octopus_post_search_reserve_s_) ||
            !(octopus_post_search_reserve_s_ >= 0.0) ||
            (enable_octopus_useful_deadline_ &&
             (!(octopus_post_search_reserve_s_ > 0.0) ||
              !(octopus_post_search_reserve_s_ < 1.0))) ||
            !(activation_speed_tolerance_mps_ >= 0.0) ||
            !(activation_accel_tolerance_mps2_ >= 0.0) ||
            !(fixed_target_change_tolerance_m_ >= 0.0) ||
            !(live_target_replan_threshold_m_ >= 0.0) ||
            !std::isfinite(target_lead_nominal_speed_mps_) || !(target_lead_nominal_speed_mps_ > 0.0) ||
            !std::isfinite(target_lead_min_s_) || !(target_lead_min_s_ >= 0.0) ||
            !std::isfinite(target_lead_max_s_) || target_lead_max_s_ < target_lead_min_s_ ||
            !std::isfinite(target_lead_high_confidence_max_s_) ||
            target_lead_high_confidence_max_s_ < target_lead_min_s_ ||
            !std::isfinite(target_lead_max_distance_m_) || !(target_lead_max_distance_m_ >= 0.0) ||
            !std::isfinite(target_lead_initial_solve_runtime_s_) ||
            !(target_lead_initial_solve_runtime_s_ >= 0.0) ||
            !std::isfinite(target_lead_solve_runtime_scale_) ||
            !(target_lead_solve_runtime_scale_ >= 0.0) ||
            !std::isfinite(target_lead_solve_runtime_ema_alpha_) ||
            !(target_lead_solve_runtime_ema_alpha_ > 0.0) ||
            !(target_lead_solve_runtime_ema_alpha_ <= 1.0) ||
            target_lead_fixed_point_iterations_ < 1 ||
            target_time_fixed_point_iterations_ < 1 ||
            !std::isfinite(target_time_fixed_point_tolerance_s_) ||
            !(target_time_fixed_point_tolerance_s_ > 0.0) ||
            !std::isfinite(live_target_velocity_replan_threshold_mps_) ||
            !(live_target_velocity_replan_threshold_mps_ >= 0.0) ||
            !std::isfinite(max_swing_angle_rad_) || !(max_swing_angle_rad_ > 0.0) ||
            max_swing_angle_rad_ >= 0.5 * kPi ||
            (payload_collision_enabled_ && object_attached_topic_.empty()) ||
            !payload_half_extents_.allFinite() ||
            (payload_half_extents_.array() <= 0.0).any() ||
            !payload_center_from_magnet_center_.allFinite() ||
            !std::isfinite(cooperative_state_timeout_s_) || !(cooperative_state_timeout_s_ > 0.0) ||
            !std::isfinite(terminal_hold_guard_s_) || !(terminal_hold_guard_s_ >= 0.0) ||
            !std::isfinite(terminal_hold_guard_interval_s_) || !(terminal_hold_guard_interval_s_ > 0.0) ||
            !std::isfinite(emergency_panic_horizon_s_) || !(emergency_panic_horizon_s_ > 0.0) ||
            !std::isfinite(cooperative_prediction_horizon_s_) || !(cooperative_prediction_horizon_s_ > 0.0) ||
            !cooperative_physical_half_extents_.allFinite() ||
            (cooperative_physical_half_extents_.array() <= 0.0).any() ||
            !cooperative_tracking_error_half_extents_.allFinite() ||
            (cooperative_tracking_error_half_extents_.array() < 0.0).any() ||
            !std::isfinite(cooperative_ring_pose_timeout_s_) || !(cooperative_ring_pose_timeout_s_ > 0.0) ||
            cooperative_ring_pose_index_ < 0 || cooperative_ring_plate_count_ <= 0 ||
            !std::isfinite(cooperative_ring_plate_pitch_diameter_m_) || !(cooperative_ring_plate_pitch_diameter_m_ > 0.0) ||
            !std::isfinite(cooperative_ring_angle_zero_rad_) ||
            !std::isfinite(cooperative_attached_tether_radius_m_) || !(cooperative_attached_tether_radius_m_ > 0.0) ||
            !cooperative_tether_anchor_from_body_.allFinite() ||
            !moving_basket_physical_half_extents_.allFinite() ||
            (moving_basket_physical_half_extents_.array() <= 0.0).any() ||
            !moving_basket_tracking_error_half_extents_.allFinite() ||
            (moving_basket_tracking_error_half_extents_.array() < 0.0).any() ||
            !ego_tracking_error_half_extents_.allFinite() ||
            (ego_tracking_error_half_extents_.array() < 0.0).any() ||
            execution_tracking_violation_samples_ < 1 ||
            !std::isfinite(execution_velocity_error_limit_mps_) ||
            !(execution_velocity_error_limit_mps_ >= 0.0) ||
            !execution_brake_accel_limit_.allFinite() ||
            (execution_brake_accel_limit_.array() <= 0.0).any() ||
            !std::isfinite(execution_brake_min_duration_s_) ||
            !(execution_brake_min_duration_s_ > 0.0)) {
            throw std::invalid_argument("invalid C1F transfer-backend parameters");
        }
        if (planning_policy_profile_ != "conservative" &&
            planning_policy_profile_ != "simulation_cage") {
            throw std::invalid_argument(
                "planning_policy_profile must be conservative or simulation_cage");
        }
        if (target_predictor_type_ != "constant_velocity" &&
            target_predictor_type_ != "scripted_circle" &&
            target_predictor_type_ != "committed_trajectory") {
            throw std::invalid_argument(
                "target_predictor_type must be constant_velocity, scripted_circle, or committed_trajectory");
        }
        if (target_predictor_type_ == "scripted_circle" &&
            (!scripted_target_center_.allFinite() ||
             !std::isfinite(scripted_target_radius_m_) || !(scripted_target_radius_m_ > 0.0) ||
             !std::isfinite(scripted_target_omega_rad_s_))) {
            throw std::invalid_argument("invalid scripted target predictor parameters");
        }
        if (target_predictor_type_ == "committed_trajectory" && !moving_basket_scene_enabled_) {
            throw std::invalid_argument(
                "committed_trajectory target prediction requires moving_basket_scene_enabled");
        }
        if (moving_basket_collision_mode_ != "solid_box" &&
            moving_basket_collision_mode_ != "segmented_ring") {
            throw std::invalid_argument(
                "moving_basket_collision_mode must be solid_box or segmented_ring");
        }
        if (moving_basket_collision_mode_ == "segmented_ring" &&
            cooperative_ring_pose_topic_.empty()) {
            throw std::invalid_argument(
                "segmented ring collision geometry requires a measured ring pose topic");
        }
        if (static_scene_enabled_ &&
            (obstacle_name_.empty() || !obstacle_centre_.allFinite() ||
             !obstacle_half_extents_.allFinite() ||
             (obstacle_half_extents_.array() <= 0.0).any() ||
             !std::isfinite(obstacle_yaw_rad_))) {
            throw std::invalid_argument("invalid C1F static-scene parameters");
        }
        if (require_scene_witness_ && !static_scene_enabled_) {
            throw std::invalid_argument("require_scene_witness needs static_scene_enabled");
        }
        if (execution_tracking_safety_enabled_ &&
            (ego_tracking_error_half_extents_.array() <= 0.0).any()) {
            throw std::invalid_argument(
                "execution tracking safety requires positive axis-wise ego tracking bounds");
        }
        if (moving_basket_scene_enabled_ &&
            moving_basket_committed_trajectory_topic_.empty()) {
            throw std::invalid_argument(
                "moving basket scene requires a committed-trajectory topic");
        }
        if (cooperative_scene_enabled_) {
            if (cooperative_drone_state_topics_.empty()) {
                throw std::invalid_argument("cooperative scene requires at least one state topic");
            }
            if (cooperative_prediction_mode_ != "constant_velocity" &&
                cooperative_prediction_mode_ != "shared_trajectory") {
                throw std::invalid_argument(
                    "cooperative_prediction_mode must be constant_velocity or shared_trajectory");
            }
            for (const auto& topic : cooperative_drone_state_topics_) {
                if (topic.empty()) {
                    throw std::invalid_argument("cooperative drone state topic must not be empty");
                }
            }
            if (!cooperative_vehicle_ids_.empty() &&
                cooperative_vehicle_ids_.size() != cooperative_drone_state_topics_.size()) {
                throw std::invalid_argument(
                    "cooperative_vehicle_ids must be empty or contain one ID per cooperative state topic");
            }
            for (const auto& vehicle_id : cooperative_vehicle_ids_) {
                if (vehicle_id.empty()) {
                    throw std::invalid_argument("cooperative vehicle ID must not be empty");
                }
            }
            if (cooperative_attached_tether_enabled_) {
                if (cooperative_attached_plate_topics_.size() != cooperative_drone_state_topics_.size()) {
                    throw std::invalid_argument(
                        "attached-tether mode requires one attached-plate topic per cooperative state topic");
                }
                for (const auto& topic : cooperative_attached_plate_topics_) {
                    if (topic.empty()) {
                        throw std::invalid_argument("cooperative attached-plate topic must not be empty");
                    }
                }
                if (cooperative_ring_pose_topic_.empty()) {
                    throw std::invalid_argument("attached-tether mode requires a ring pose topic");
                }
            }
            if (cooperative_prediction_mode_ == "shared_trajectory") {
                if (cooperative_committed_trajectory_topics_.size() !=
                    cooperative_drone_state_topics_.size()) {
                    throw std::invalid_argument(
                        "shared_trajectory mode requires one commitment topic per cooperative state topic");
                }
                for (const auto& topic : cooperative_committed_trajectory_topics_) {
                    if (topic.empty()) {
                        throw std::invalid_argument(
                            "cooperative committed-trajectory topic must not be empty");
                    }
                }
                if (require_terminal_hold_for_nominal_ && !(terminal_hold_guard_s_ > 0.0)) {
                    throw std::invalid_argument(
                        "shared cooperative C1F.5 mode requires a positive terminal_hold_guard_s");
                }
            }
        }
    }

    void openLogs() {
        if (!csv_path_.empty()) {
            csv_.open(csv_path_);
            if (csv_) {
                csv_ << "ros_time,enable_requested,enabled,state_fresh,state_age_ms,target_received,"
                        "target_x,target_y,target_z,target_velocity_received,target_vx,target_vy,target_vz,"
                        "planning_goal_x,planning_goal_y,planning_goal_z,target_lead_s,target_lead_distance_m,"
                        "target_travel_lead_s,target_solve_lead_s,target_rendezvous_time_unclamped_s,"
                        "estimated_solve_runtime_s,arrival_time_target_lead_enabled,moving_rendezvous_enabled,"
                        "active_terminal_vx,active_terminal_vy,active_terminal_vz,has_commit,"
                        "commit_start_s,commit_end_s,prepared_candidate_ready,authority_fresh_plan_pending,"
                        "authority_granted,authority_fresh_commit_count,authority_rebase_count,"
                        "last_authority_grant_ros_s,latest_status,reference_publish_count,reference_errors,"
                        "terminal_hold_reference_recoveries,last_reference_period_ms,last_reference_callback_ms,"
                        "max_reference_callback_ms,reference_callback_overrun_count,last_planner_result_processing_ms,"
                        "max_planner_result_processing_ms,backend_incumbent_recheck_count,"
                        "backend_incumbent_recheck_skipped_same_world_count,last_reference_sample_ros_s,"
                        "last_reference_error_ros_s,last_reference_error,accepted,failed,late,coalesced,"
                        "stale_state_replan_skips,stale_state_reference_skips,worker_errors,"
                        "payload_collision_enabled,object_attached_received,object_attached,payload_attachment_age_ms,"
                        "payload_geometry_active,static_scene_enabled,obstacle_x,obstacle_y,obstacle_z,"
                        "obstacle_half_x,obstacle_half_y,obstacle_half_z,scene_witness_passed,scene_witness_summary,"
                        "cooperative_scene_enabled,cooperative_required_count,cooperative_fresh_count,"
                        "cooperative_max_state_age_ms,cooperative_prediction_mode,cooperative_shared_fresh_count,"
                        "cooperative_min_future_coverage_s,cooperative_max_tracking_ratio,"
                        "cooperative_shared_updates,cooperative_shared_rejections,"
                        "cooperative_world_refresh_count,stale_cooperative_replan_skips,"
                        "coop0_x,coop0_y,coop0_z,coop0_vx,coop0_vy,coop0_vz,coop0_age_ms,"
                        "coop1_x,coop1_y,coop1_z,coop1_vx,coop1_vy,coop1_vz,coop1_age_ms,"
                        "execution_tracking_enabled,ego_tracking_half_x_m,ego_tracking_half_y_m,ego_tracking_half_z_m,"
                        "execution_reference_valid,execution_error_x_m,execution_error_y_m,execution_error_z_m,"
                        "execution_error_norm_m,execution_velocity_error_norm_mps,execution_time_lag_s,"
                        "execution_consecutive_violations,execution_fallback_active,execution_fallback_certified,"
                        "execution_fallback_count,execution_fallback_recovery_count,execution_fallback_mode,"
                        "execution_fallback_reason,execution_fallback_restart_blocked,execution_fallback_restart_z_m,"
                        "world_version\n";
            } else {
                RCLCPP_WARN(get_logger(), "Could not open C1F.0 backend CSV: %s", csv_path_.c_str());
            }
        }
        if (!replan_csv_path_.empty()) {
            replan_csv_.open(replan_csv_path_);
            if (replan_csv_) {
                replan_csv_ << "sequence,request_ros_time,status,attempted,accepted,candidate_late,"
                               "splice_lookahead_ms,octopus_termination,octopus_search_ms,qp_solve_ms,"
                               "local_plan_total_ms,replan_runtime_ms,world_snapshot_ms,obstacle_build_ms,"
                               "safety_check_ms,atomic_commit_ms,octopus_separator_lp_calls,octopus_aabb_skips,"
                               "terminal_hold_separator_lp_calls,partial_fallback_retained,partial_fallback_tested,"
                               "partial_fallback_selected_rank,safety_separator_lp_calls,safety_aabb_skips,"
                               "post_search_reserve_ms,useful_budget_ms,octopus_wall_budget_ms,authority_margin_ms,"
                               "moving_target_terminal_active,local_continuation_active,moving_rendezvous_active,"
                               "terminal_target_vx,terminal_target_vy,terminal_target_vz,"
                               "target_time_prediction_used,target_time_prediction_valid,target_time_high_confidence,"
                               "target_time_fixed_point_iterations,target_time_fixed_point_residual_ms,"
                               "predicted_target_time_s,predicted_target_horizon_s,"
                               "predicted_target_x,predicted_target_y,predicted_target_z,"
                               "predicted_target_vx,predicted_target_vy,predicted_target_vz,"
                               "terminal_velocity_error_mps,rendezvous_backup_duration_s,"
                               "planner_policy_profile,candidate_kind,candidate_collision_safe,"
                               "terminal_hold_required_for_acceptance,incumbent_time_to_conflict_s,"
                               "estimated_brake_time_s,fallback_trigger_horizon_s,"
                               "initial_q0_x,initial_q0_y,initial_q0_z,initial_q2_x,initial_q2_y,initial_q2_z,"
                               "search_min_x,search_min_y,search_min_z,search_max_x,search_max_y,search_max_z,"
                               "initial_q2_radius_m,initial_q2_box_valid,initial_q2_radius_valid,"
                               "committed_endpoint_distance_m,prefix_safety,candidate_safety,backup_safety,terminal_hold_safety,"
                               "terminal_hold_rejections,incumbent_remaining_unsafe,"
                               "planning_world_version,recheck_world_version,final_recheck_world_version,recheck_error,"
                               "incumbent_recheck_performed,incumbent_recheck_world_version,"
                               "worker_exception,worker_error\n";
            } else {
                RCLCPP_WARN(get_logger(), "Could not open C1F.0 replan CSV: %s", replan_csv_path_.c_str());
            }
        }
    }

    void stateCallback(const interfaces::msg::MotionCaptureState::SharedPtr msg) {
        const double now_s = get_clock()->now().seconds();
        const Vec3 position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        const Vec3 velocity(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
        if (last_state_receive_ros_s_.has_value() && now_s < *last_state_receive_ros_s_ - 1e-6) {
            disableBackend("ROS_TIME_BACKJUMP_DISABLED");
            kinematics_.reset();
        }
        try {
            latest_state_ = kinematics_.update(position, velocity, now_s);
            last_state_receive_ros_s_ = now_s;
        } catch (const std::exception& exc) {
            RCLCPP_WARN(get_logger(), "Rejected C1F.0 kinematics sample: %s", exc.what());
        }
    }

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
            latest_status_ = "TARGET_FRAME_REJECTED";
            return;
        }
        const Vec3 target(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        if (!target.allFinite()) {
            latest_status_ = "TARGET_NONFINITE_REJECTED";
            return;
        }
        latest_target_ = target;
        if (enabled_ && active_source_target_.has_value() &&
            (target - *active_source_target_).norm() > fixed_target_change_tolerance_m_) {
            if (allow_live_target_updates_) {
                latest_status_ = "LIVE_TARGET_UPDATE_PENDING";
            } else {
                disableBackend("TARGET_CHANGED_REENABLE_REQUIRED");
                // Preserve the newly received target while requiring explicit re-enable.
                latest_target_ = target;
                RCLCPP_WARN(get_logger(),
                    "C1F.0 is fixed-target only. Target changed; backend disabled and re-enable is required.");
            }
        }
    }

    void targetVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
            latest_status_ = "TARGET_VELOCITY_FRAME_REJECTED";
            return;
        }
        const Vec3 velocity(
            msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
        if (!velocity.allFinite()) {
            latest_status_ = "TARGET_VELOCITY_NONFINITE_REJECTED";
            return;
        }
        latest_target_velocity_ = velocity;
    }

    void objectAttachedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        const double now_s = get_clock()->now().seconds();
        latest_object_attached_ = msg->data;
        last_object_attached_receive_ros_s_ = now_s;
        if (!payload_collision_enabled_) return;

        if (suspended_geometry_.payload_attached != msg->data) {
            suspended_geometry_.payload_attached = msg->data;
            world_->setEgoSuspendedGeometry(suspended_geometry_);
            RCLCPP_WARN(
                get_logger(),
                "C1F payload collision geometry %s from %s (world version %llu).",
                msg->data ? "ENABLED" : "DISABLED",
                object_attached_topic_.c_str(),
                static_cast<unsigned long long>(world_->currentVersion()));
        }

        if (!msg->data && enabled_ && require_payload_attached_for_enable_) {
            disableBackend("PAYLOAD_DETACHED_DISABLED");
        }
    }

    void cooperativeStateCallback(
        std::size_t index,
        const interfaces::msg::MotionCaptureState::SharedPtr msg) {
        if (index >= cooperative_state_samples_.size()) return;
        if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Rejected cooperative drone %zu state in frame '%s' (expected '%s').",
                index, msg->header.frame_id.c_str(), frame_id_.c_str());
            return;
        }
        const Vec3 position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        const Vec3 velocity(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
        if (!position.allFinite() || !velocity.allFinite()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Rejected non-finite cooperative drone %zu state.", index);
            return;
        }
        const double now_s = get_clock()->now().seconds();
        auto& sample = cooperative_state_samples_[index];
        if (sample.received && std::isfinite(sample.receive_ros_s) &&
            now_s < sample.receive_ros_s - 1e-6) {
            sample = CooperativeStateSample{};
            if (enabled_) disableBackend("COOPERATIVE_ROS_TIME_BACKJUMP_DISABLED");
            return;
        }
        sample.received = true;
        sample.position = position;
        sample.velocity = velocity;
        sample.receive_ros_s = now_s;
        if (cooperative_prediction_mode_ == "shared_trajectory") {
            refreshSharedCooperativeObstacle(index, now_s, false);
        }
    }

    bool ringPoseSampleFresh(double now_s) const {
        if (!cooperative_ring_pose_sample_.received ||
            !std::isfinite(cooperative_ring_pose_sample_.receive_ros_s)) return false;
        const double age_s = now_s - cooperative_ring_pose_sample_.receive_ros_s;
        return std::isfinite(age_s) && age_s >= -0.02 && age_s <= cooperative_ring_pose_timeout_s_;
    }

    bool cooperativeRingPoseFresh(double now_s) const {
        if (!cooperative_attached_tether_enabled_) return true;
        return ringPoseSampleFresh(now_s);
    }

    Vec3 cooperativePlateWorldPosition(int plate_id) const {
        if (plate_id < 0 || plate_id >= cooperative_ring_plate_count_) {
            throw std::invalid_argument("cooperative attached plate id is out of range");
        }
        const double theta = cooperative_ring_angle_zero_rad_ +
            2.0 * kPi * static_cast<double>(plate_id) /
                static_cast<double>(cooperative_ring_plate_count_);
        const double radius = 0.5 * cooperative_ring_plate_pitch_diameter_m_;
        const Vec3 plate_ring(radius * std::cos(theta), radius * std::sin(theta), 0.0);
        return cooperative_ring_pose_sample_.position +
            cooperative_ring_pose_sample_.rotation * plate_ring;
    }

    AttachedTetherGeometry cooperativeAttachedTetherGeometry(
        std::size_t index,
        const Vec3& body_position,
        double now_s,
        bool require_fresh) const {
        AttachedTetherGeometry geometry;
        if (!cooperative_attached_tether_enabled_ ||
            index >= cooperative_attached_plate_samples_.size()) return geometry;
        const auto& plate = cooperative_attached_plate_samples_[index];
        if (!plate.received || plate.plate_id < 0) return geometry;
        if (!cooperativeRingPoseFresh(now_s)) {
            if (require_fresh) {
                throw std::invalid_argument("attached peer requires fresh measured ring pose");
            }
            return geometry;
        }
        geometry.enabled = true;
        geometry.anchor_from_body = cooperative_tether_anchor_from_body_;
        geometry.plate_from_body = cooperativePlateWorldPosition(plate.plate_id) - body_position;
        geometry.radius_m = cooperative_attached_tether_radius_m_;
        geometry.validate();
        return geometry;
    }

    bool refreshSharedCooperativeObstacle(
        std::size_t index,
        double now_s,
        bool require_tether_inputs) {
        if (index >= cooperative_trajectory_samples_.size()) return false;
        const auto& trajectory_sample = cooperative_trajectory_samples_[index];
        if (!trajectory_sample.received || trajectory_sample.trajectory.empty()) return false;
        CooperativeObstacleTrajectory obstacle;
        obstacle.name = "cooperative_drone_" + std::to_string(index);
        obstacle.trajectory = trajectory_sample.trajectory;
        obstacle.physical_half_extents = cooperative_physical_half_extents_;
        obstacle.tracking_error_half_extents = cooperative_tracking_error_half_extents_;
        obstacle.suspended_geometry.enabled = false;
        Vec3 body_position = obstacle.trajectory.evaluate(
            std::clamp(now_s, obstacle.trajectory.startTime(), obstacle.trajectory.endTime())).position;
        if (index < cooperative_state_samples_.size() && cooperative_state_samples_[index].received) {
            body_position = cooperative_state_samples_[index].position;
        }
        try {
            obstacle.attached_tether_geometry = cooperativeAttachedTetherGeometry(
                index, body_position, now_s, require_tether_inputs);
        } catch (const std::exception&) {
            return false;
        }
        world_->upsertCooperativeTrajectory(std::move(obstacle));
        return true;
    }

    void cooperativeAttachedPlateCallback(
        std::size_t index,
        const std_msgs::msg::Int32::SharedPtr msg) {
        if (index >= cooperative_attached_plate_samples_.size()) return;
        const int plate_id = msg->data;
        if (plate_id < -1 || plate_id >= cooperative_ring_plate_count_) {
            RCLCPP_ERROR(
                get_logger(), "Rejected cooperative attached plate %d for peer %zu.",
                plate_id, index);
            return;
        }
        auto& sample = cooperative_attached_plate_samples_[index];
        sample.received = true;
        sample.plate_id = plate_id;
        sample.receive_ros_s = get_clock()->now().seconds();
        if (cooperative_prediction_mode_ == "shared_trajectory") {
            refreshSharedCooperativeObstacle(index, sample.receive_ros_s, false);
        }
    }

    void cooperativeRingPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        const std::size_t index = static_cast<std::size_t>(cooperative_ring_pose_index_);
        if (index >= msg->poses.size()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "M2D ring pose index %zu unavailable in PoseArray size %zu.",
                index, msg->poses.size());
            return;
        }
        const auto& pose = msg->poses[index];
        const Vec3 position(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond q(
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        if (!position.allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-9) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Rejected invalid M2D ring pose.");
            return;
        }
        q.normalize();
        cooperative_ring_pose_sample_.received = true;
        cooperative_ring_pose_sample_.position = position;
        cooperative_ring_pose_sample_.rotation = q.toRotationMatrix();
        cooperative_ring_pose_sample_.receive_ros_s = get_clock()->now().seconds();
        if (moving_basket_collision_mode_ == "segmented_ring" &&
            moving_basket_trajectory_sample_.received &&
            (!moving_basket_collision_sequence_.has_value() ||
             moving_basket_collision_sequence_.value() != moving_basket_trajectory_sample_.sequence)) {
            refreshMovingBasketCollisionObstacles(cooperative_ring_pose_sample_.receive_ros_s);
        }
        if (cooperative_prediction_mode_ == "shared_trajectory") {
            for (std::size_t i = 0; i < cooperative_trajectory_samples_.size(); ++i) {
                refreshSharedCooperativeObstacle(i, cooperative_ring_pose_sample_.receive_ros_s, false);
            }
        }
    }

    void cooperativeTrajectoryCallback(
        std::size_t index,
        const interfaces::msg::CommittedTrajectory::SharedPtr msg) {
        if (index >= cooperative_trajectory_samples_.size()) return;
        if (!cooperative_vehicle_ids_.empty()) {
            const auto& expected_vehicle_id = cooperative_vehicle_ids_.at(index);
            if (msg->vehicle_id != expected_vehicle_id) {
                ++cooperative_shared_commitment_rejections_;
                latest_status_ = "COOPERATIVE_SHARED_TRAJECTORY_REJECTED";
                RCLCPP_ERROR(
                    get_logger(),
                    "Rejected cooperative shared trajectory %zu: vehicle_id '%s' != expected '%s'.",
                    index, msg->vehicle_id.c_str(), expected_vehicle_id.c_str());
                return;
            }
        }
        if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
            RCLCPP_WARN(
                get_logger(),
                "Rejected shared cooperative trajectory %zu in frame '%s' (expected '%s').",
                index, msg->header.frame_id.c_str(), frame_id_.c_str());
            return;
        }
        try {
            CommittedTrajectory trajectory = trajectoryFromMessage(*msg);
            const double now_s = get_clock()->now().seconds();
            auto& sample = cooperative_trajectory_samples_[index];
            if (sample.received && msg->sequence <= sample.sequence) {
                return;
            }
            if (trajectory.endTime() < now_s - dynamic_planner::kTrajectoryTimeToleranceS &&
                !trajectory.endsInStoppedHold()) {
                throw std::invalid_argument("shared cooperative trajectory is already expired");
            }
            sample.received = true;
            sample.sequence = msg->sequence;
            sample.trajectory = std::move(trajectory);
            sample.receive_ros_s = now_s;
            if (!refreshSharedCooperativeObstacle(index, now_s, false)) {
                CooperativeObstacleTrajectory obstacle;
                obstacle.name = "cooperative_drone_" + std::to_string(index);
                obstacle.trajectory = sample.trajectory;
                obstacle.physical_half_extents = cooperative_physical_half_extents_;
                obstacle.tracking_error_half_extents = cooperative_tracking_error_half_extents_;
                obstacle.suspended_geometry.enabled = false;
                world_->upsertCooperativeTrajectory(std::move(obstacle));
            }
            ++cooperative_shared_commitment_updates_;
            RCLCPP_DEBUG(
                get_logger(),
                "Accepted cooperative shared trajectory %zu seq=%llu coverage=[%.3f, %.3f] world=%llu.",
                index, static_cast<unsigned long long>(msg->sequence),
                sample.trajectory.startTime(), sample.trajectory.endTime(),
                static_cast<unsigned long long>(world_->currentVersion()));
        } catch (const std::exception& exc) {
            ++cooperative_shared_commitment_rejections_;
            latest_status_ = "COOPERATIVE_SHARED_TRAJECTORY_REJECTED";
            RCLCPP_ERROR(
                get_logger(), "Rejected cooperative shared trajectory %zu: %s",
                index, exc.what());
        }
    }

    bool refreshMovingBasketCollisionObstacles(double now_s) {
        const auto& sample = moving_basket_trajectory_sample_;
        if (!sample.received || sample.trajectory.empty()) return false;

        if (moving_basket_collision_mode_ == "solid_box") {
            CooperativeObstacleTrajectory obstacle;
            obstacle.name = "moving_drop_basket";
            obstacle.trajectory = translatedCollisionTrajectory(
                sample.trajectory, moving_basket_geometry_.centerOffset());
            obstacle.physical_half_extents = moving_basket_physical_half_extents_;
            obstacle.tracking_error_half_extents = moving_basket_tracking_error_half_extents_;
            obstacle.suspended_geometry.enabled = false;
            world_->upsertCooperativeTrajectory(std::move(obstacle));
            moving_basket_collision_sequence_ = sample.sequence;
            return true;
        }

        if (!ringPoseSampleFresh(now_s)) return false;

        std::vector<CooperativeObstacleTrajectory> segments;
        segments.reserve(static_cast<std::size_t>(segmented_ring_geometry_.segment_count));
        for (int i = 0; i < segmented_ring_geometry_.segment_count; ++i) {
            CooperativeObstacleTrajectory obstacle;
            obstacle.name = segmented_ring_geometry_.obstacleName(i);
            const Vec3 offset = segmented_ring_geometry_.segmentWorldCenterOffset(
                cooperative_ring_pose_sample_.rotation, i);
            obstacle.trajectory = translatedCollisionTrajectory(sample.trajectory, offset);
            obstacle.physical_half_extents = segmented_ring_geometry_.segmentWorldHalfExtents(
                cooperative_ring_pose_sample_.rotation, i);
            obstacle.tracking_error_half_extents = moving_basket_tracking_error_half_extents_;
            obstacle.suspended_geometry.enabled = false;
            segments.push_back(std::move(obstacle));
        }
        world_->upsertCooperativeTrajectories(std::move(segments));
        moving_basket_collision_sequence_ = sample.sequence;
        return true;
    }

    void movingBasketTrajectoryCallback(
        const interfaces::msg::CommittedTrajectory::SharedPtr msg) {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
            RCLCPP_WARN(
                get_logger(),
                "Rejected moving basket trajectory in frame '%s' (expected '%s').",
                msg->header.frame_id.c_str(), frame_id_.c_str());
            return;
        }
        try {
            CommittedTrajectory trajectory = trajectoryFromMessage(*msg);
            const double now_s = get_clock()->now().seconds();
            if (moving_basket_trajectory_sample_.received &&
                msg->sequence <= moving_basket_trajectory_sample_.sequence) {
                return;
            }
            if (trajectory.endTime() < now_s - dynamic_planner::kTrajectoryTimeToleranceS &&
                !trajectory.endsInStoppedHold()) {
                throw std::invalid_argument("moving basket trajectory is already expired");
            }

            // Preserve the unmodified ring-centre commitment for target prediction.
            // Collision geometry is a separate world-model concern and may be either
            // the legacy M1 solid envelope or the M2D fixture-faithful ring segments.
            moving_basket_trajectory_sample_.received = true;
            moving_basket_trajectory_sample_.sequence = msg->sequence;
            moving_basket_trajectory_sample_.trajectory = std::move(trajectory);
            moving_basket_trajectory_sample_.receive_ros_s = now_s;
            const bool collision_ready = refreshMovingBasketCollisionObstacles(now_s);
            ++moving_basket_commitment_updates_;
            RCLCPP_DEBUG(
                get_logger(),
                "Accepted moving basket seq=%llu coverage=[%.3f, %.3f] collision=%s world=%llu.",
                static_cast<unsigned long long>(msg->sequence),
                moving_basket_trajectory_sample_.trajectory.startTime(),
                moving_basket_trajectory_sample_.trajectory.endTime(),
                collision_ready ? moving_basket_collision_mode_.c_str() : "PENDING_RING_POSE",
                static_cast<unsigned long long>(world_->currentVersion()));
        } catch (const std::exception& exc) {
            ++moving_basket_commitment_rejections_;
            latest_status_ = "MOVING_BASKET_TRAJECTORY_REJECTED";
            RCLCPP_ERROR(get_logger(), "Rejected moving basket trajectory: %s", exc.what());
        }
    }

    bool movingBasketTrajectoryFresh(double now_s) const {
        if (!moving_basket_scene_enabled_) return true;
        const auto& sample = moving_basket_trajectory_sample_;
        if (!sample.received || sample.trajectory.empty()) return false;
        if (!moving_basket_collision_sequence_.has_value() ||
            moving_basket_collision_sequence_.value() != sample.sequence) {
            return false;
        }
        if (moving_basket_collision_mode_ == "segmented_ring" &&
            !ringPoseSampleFresh(now_s)) {
            return false;
        }
        const auto& trajectory = sample.trajectory;
        if (now_s < trajectory.startTime() - dynamic_planner::kTrajectoryTimeToleranceS) {
            return false;
        }
        if (!trajectory.endsInStoppedHold() &&
            trajectory.endTime() < now_s - dynamic_planner::kTrajectoryTimeToleranceS) {
            return false;
        }
        return true;
    }

    std::size_t cooperativeFreshCount(double now_s, double* max_age_s = nullptr) const {
        if (max_age_s) *max_age_s = 0.0;
        if (!cooperative_scene_enabled_) return 0U;
        std::size_t fresh = 0U;
        double maximum_age = 0.0;
        for (const auto& sample : cooperative_state_samples_) {
            if (!sample.received || !std::isfinite(sample.receive_ros_s)) continue;
            const double age = now_s - sample.receive_ros_s;
            if (!std::isfinite(age) || age < -0.02 || age > cooperative_state_timeout_s_) continue;
            ++fresh;
            maximum_age = std::max(maximum_age, std::max(0.0, age));
        }
        if (max_age_s) *max_age_s = maximum_age;
        return fresh;
    }

    bool cooperativeStatesFresh(double now_s, double* max_age_s = nullptr) const {
        if (!cooperative_scene_enabled_) {
            if (max_age_s) *max_age_s = 0.0;
            return true;
        }
        return cooperativeFreshCount(now_s, max_age_s) == cooperative_state_samples_.size();
    }

    std::size_t cooperativeSharedFreshCount(
        double now_s, double* minimum_future_coverage_s = nullptr,
        double* maximum_tracking_ratio = nullptr) const {
        if (minimum_future_coverage_s) {
            *minimum_future_coverage_s = std::numeric_limits<double>::infinity();
        }
        if (maximum_tracking_ratio) *maximum_tracking_ratio = 0.0;
        if (!cooperative_scene_enabled_ || cooperative_prediction_mode_ != "shared_trajectory") {
            return 0U;
        }
        std::size_t fresh = 0U;
        double min_coverage = std::numeric_limits<double>::infinity();
        double max_ratio = 0.0;
        for (std::size_t i = 0; i < cooperative_trajectory_samples_.size(); ++i) {
            const auto& trajectory_sample = cooperative_trajectory_samples_[i];
            if (!trajectory_sample.received || trajectory_sample.trajectory.empty()) continue;
            const auto& trajectory = trajectory_sample.trajectory;
            if (now_s < trajectory.startTime() - dynamic_planner::kTrajectoryTimeToleranceS) continue;
            double coverage = std::numeric_limits<double>::infinity();
            if (!trajectory.endsInStoppedHold()) {
                coverage = trajectory.endTime() - now_s;
                if (coverage < -dynamic_planner::kTrajectoryTimeToleranceS) continue;
            }
            if (i >= cooperative_state_samples_.size()) continue;
            const auto& state = cooperative_state_samples_[i];
            if (!state.received || !std::isfinite(state.receive_ros_s)) continue;
            const double state_age = now_s - state.receive_ros_s;
            if (!std::isfinite(state_age) || state_age < -0.02 ||
                state_age > cooperative_state_timeout_s_) continue;
            State advertised;
            try {
                advertised = trajectory.evaluate(state.receive_ros_s);
            } catch (const std::exception&) {
                continue;
            }
            const Vec3 error = (state.position - advertised.position).cwiseAbs();
            double ratio = 0.0;
            bool inside = true;
            for (int axis = 0; axis < 3; ++axis) {
                const double bound = cooperative_tracking_error_half_extents_(axis);
                if (bound <= 1e-12) {
                    inside = inside && error(axis) <= 1e-6;
                } else {
                    ratio = std::max(ratio, error(axis) / bound);
                    inside = inside && error(axis) <= bound + 1e-6;
                }
            }
            if (!inside) continue;
            ++fresh;
            min_coverage = std::min(min_coverage, coverage);
            max_ratio = std::max(max_ratio, ratio);
        }
        if (minimum_future_coverage_s) *minimum_future_coverage_s = min_coverage;
        if (maximum_tracking_ratio) *maximum_tracking_ratio = max_ratio;
        return fresh;
    }

    bool cooperativeInputsFresh(double now_s) const {
        if (cooperative_scene_enabled_) {
            if (!cooperativeStatesFresh(now_s)) return false;
            if (cooperative_prediction_mode_ == "shared_trajectory" &&
                cooperativeSharedFreshCount(now_s) != cooperative_state_samples_.size()) {
                return false;
            }
        }
        return movingBasketTrajectoryFresh(now_s);
    }

    const char* cooperativeWaitStatus(double now_s) const {
        if (!cooperative_scene_enabled_ && moving_basket_scene_enabled_ &&
            !movingBasketTrajectoryFresh(now_s)) {
            return "WAITING_FOR_VALID_MOVING_BASKET_COMMITMENT";
        }
        return cooperative_prediction_mode_ == "shared_trajectory"
            ? "WAITING_FOR_VALID_COOPERATIVE_SHARED_TRAJECTORIES"
            : "WAITING_FOR_FRESH_COOPERATIVE_STATES";
    }

    const char* cooperativeAuthorityRejectStatus(double now_s) const {
        if (!cooperative_scene_enabled_ && moving_basket_scene_enabled_ &&
            !movingBasketTrajectoryFresh(now_s)) {
            return "AUTHORITY_GRANT_REJECTED_INVALID_MOVING_BASKET_COMMITMENT";
        }
        return cooperative_prediction_mode_ == "shared_trajectory"
            ? "AUTHORITY_GRANT_REJECTED_INVALID_COOPERATIVE_SHARED_TRAJECTORY"
            : "AUTHORITY_GRANT_REJECTED_STALE_COOPERATIVE_STATE";
    }

    const char* cooperativeDisableStatus(double now_s) const {
        if (!cooperative_scene_enabled_ && moving_basket_scene_enabled_ &&
            !movingBasketTrajectoryFresh(now_s)) {
            return "MOVING_BASKET_INPUT_INVALID_DISABLED";
        }
        return cooperative_prediction_mode_ == "shared_trajectory"
            ? "COOPERATIVE_SHARED_INPUT_INVALID_DISABLED"
            : "COOPERATIVE_STATE_STALE_DISABLED";
    }

    bool refreshCooperativeWorld(double now_s) {
        if (!movingBasketTrajectoryFresh(now_s)) return false;
        if (!cooperative_scene_enabled_) {
            ++cooperative_world_refresh_count_;
            return true;
        }
        double max_age_s = 0.0;
        if (!cooperativeStatesFresh(now_s, &max_age_s)) return false;
        if (cooperative_prediction_mode_ == "shared_trajectory") {
            if (cooperativeSharedFreshCount(now_s) != cooperative_state_samples_.size()) {
                return false;
            }
            for (std::size_t i = 0; i < cooperative_trajectory_samples_.size(); ++i) {
                if (!refreshSharedCooperativeObstacle(i, now_s, true)) return false;
            }
            ++cooperative_world_refresh_count_;
            return true;
        }

        for (std::size_t i = 0; i < cooperative_state_samples_.size(); ++i) {
            const auto& sample = cooperative_state_samples_[i];
            CooperativeObstacleTrajectory obstacle;
            obstacle.name = "cooperative_drone_" + std::to_string(i);
            const double sample_age_s = std::max(0.0, now_s - sample.receive_ros_s);
            const Vec3 prediction_start = sample.position + sample.velocity * sample_age_s;
            obstacle.trajectory = constantVelocityPrediction(
                prediction_start, sample.velocity, now_s, cooperative_prediction_horizon_s_);
            obstacle.physical_half_extents = cooperative_physical_half_extents_;
            obstacle.tracking_error_half_extents = cooperative_tracking_error_half_extents_;
            obstacle.suspended_geometry.enabled = false;
            try {
                obstacle.attached_tether_geometry = cooperativeAttachedTetherGeometry(
                    i, sample.position, now_s, true);
            } catch (const std::exception&) {
                return false;
            }
            world_->upsertCooperativeTrajectory(std::move(obstacle));
        }
        ++cooperative_world_refresh_count_;
        return true;
    }

    std::shared_ptr<const dynamic_planner::TargetPredictor> targetPredictorSnapshot(
        double reference_time_s) const {
        if (!latest_target_.has_value()) {
            return {};
        }
        try {
            if (target_predictor_type_ == "committed_trajectory") {
                if (!moving_basket_scene_enabled_ ||
                    !moving_basket_trajectory_sample_.received ||
                    moving_basket_trajectory_sample_.trajectory.empty()) {
                    return {};
                }
                const State advertised =
                    moving_basket_trajectory_sample_.trajectory.evaluate(reference_time_s);
                const Vec3 body_target_offset = *latest_target_ - advertised.position;
                if (!body_target_offset.allFinite()) return {};
                return std::make_shared<dynamic_planner::CommittedTrajectoryTargetPredictor>(
                    moving_basket_trajectory_sample_.trajectory, body_target_offset);
            }
            if (!latest_target_velocity_.has_value()) {
                return {};
            }
            if (target_predictor_type_ == "scripted_circle") {
                return std::make_shared<dynamic_planner::ScriptedCircleTargetPredictor>(
                    scripted_target_center_, scripted_target_radius_m_,
                    scripted_target_omega_rad_s_, *latest_target_, reference_time_s);
            }
            return std::make_shared<dynamic_planner::ConstantVelocityTargetPredictor>(
                *latest_target_, *latest_target_velocity_, reference_time_s);
        } catch (const std::exception&) {
            return {};
        }
    }

    TargetPrediction targetPredictionAt(double future_time_s) {
        const double now_s = get_clock()->now().seconds();
        const auto predictor = targetPredictorSnapshot(now_s);
        if (!predictor) {
            return {{}, {}, false, "TARGET_PREDICTOR_SNAPSHOT_UNAVAILABLE"};
        }
        return predictor->evaluate(future_time_s);
    }

    Vec3 planningGoalForState(const State& measured) {
        if (!latest_target_.has_value()) {
            throw std::logic_error("planning goal requested without target");
        }
        last_target_lead_s_ = 0.0;
        last_target_lead_distance_m_ = 0.0;
        last_target_travel_lead_s_ = 0.0;
        last_target_solve_lead_s_ = 0.0;
        last_target_rendezvous_time_unclamped_s_ = 0.0;
        last_target_prediction_valid_ = false;
        last_target_prediction_horizon_s_ = 0.0;
        if (terminal_time_target_prediction_enabled_ && moving_rendezvous_enabled_) {
            // C1F.8c: the backend no longer freezes a separately pre-led point.
            // The core receives x_T(t) and evaluates it at the candidate's own
            // terminal absolute time after the future splice is chosen. For the
            // commissioned committed-trajectory provider, velocity also comes
            // from that same advertised future instead of the duplicate live
            // target-velocity topic.
            const auto current_prediction = targetPredictionAt(get_clock()->now().seconds());
            if (current_prediction.valid) {
                last_target_prediction_velocity_ = current_prediction.velocity;
                last_target_prediction_valid_ = true;
            } else {
                last_target_prediction_velocity_ = latest_target_velocity_.value_or(Vec3::Zero());
                last_target_prediction_valid_ = false;
            }
            return *latest_target_;
        }
        if (!target_lead_enabled_ || !latest_target_velocity_.has_value()) {
            last_target_prediction_velocity_ = latest_target_velocity_.value_or(Vec3::Zero());
            return *latest_target_;
        }

        const bool high_confidence_future = target_predictor_type_ == "scripted_circle";
        const double prediction_max_s = high_confidence_future
            ? target_lead_high_confidence_max_s_
            : target_lead_max_s_;
        const Vec3 target_velocity = *latest_target_velocity_;
        const double target_speed_mps = target_velocity.norm();
        if (!arrival_time_target_lead_enabled_) {
            const double distance_m = (*latest_target_ - measured.position).norm();
            last_target_travel_lead_s_ = distance_m / target_lead_nominal_speed_mps_;
            last_target_solve_lead_s_ =
                target_lead_solve_runtime_scale_ * estimated_solve_runtime_s_;
            last_target_rendezvous_time_unclamped_s_ =
                last_target_travel_lead_s_ + last_target_solve_lead_s_;
            last_target_lead_s_ = std::clamp(
                last_target_rendezvous_time_unclamped_s_,
                target_lead_min_s_, prediction_max_s);
        } else {
            // C1F.7: solve the same small fixed-point intercept-time problem,
            // but evaluate a provider-specific future. The simulation uses its
            // known scripted circle; lower-confidence IRL providers may retain
            // bounded CV/estimated prediction. This remains receding-horizon:
            // every 5 Hz transaction refreshes the estimate.
            double lead_s = std::clamp(
                std::max(target_lead_min_s_,
                         target_lead_solve_runtime_scale_ * estimated_solve_runtime_s_),
                target_lead_min_s_, prediction_max_s);
            for (int iteration = 0; iteration < target_lead_fixed_point_iterations_; ++iteration) {
                const TargetPrediction prediction = targetPredictionAt(
                    get_clock()->now().seconds() + lead_s);
                if (!prediction.valid) {
                    throw std::runtime_error("TARGET_PREDICTOR_INVALID:" + prediction.detail);
                }
                const Vec3 predicted = prediction.position;
                const Vec3 terminal_velocity = moving_rendezvous_enabled_
                    ? prediction.velocity : Vec3::Zero();
                const double minimum_s = dynamic_planner::minimumTimeDoubleIntegrator3D(
                    measured.position, measured.velocity, predicted, terminal_velocity,
                    Vec3::Ones(), Vec3(1.0, 1.0, 1.5));
                const double predicted_distance_m =
                    (predicted - measured.position).norm();
                const double rmader_factor = predicted_distance_m <= close_to_goal_m_
                    ? factor_alloc_close_ : factor_alloc_;
                const double allocation_factor = std::max(rmader_factor, spline_time_factor_);
                last_target_travel_lead_s_ = allocation_factor * minimum_s;
                last_target_solve_lead_s_ =
                    target_lead_solve_runtime_scale_ * estimated_solve_runtime_s_;
                last_target_rendezvous_time_unclamped_s_ =
                    last_target_travel_lead_s_ + last_target_solve_lead_s_;
                lead_s = std::clamp(
                    last_target_rendezvous_time_unclamped_s_,
                    target_lead_min_s_, prediction_max_s);
                if (!high_confidence_future && target_lead_max_distance_m_ > 0.0 &&
                    target_speed_mps > 1e-9) {
                    lead_s = std::min(
                        lead_s, target_lead_max_distance_m_ / target_speed_mps);
                }
            }
            last_target_lead_s_ = lead_s;
        }

        if (!high_confidence_future && target_lead_max_distance_m_ > 0.0 && target_speed_mps > 1e-9) {
            last_target_lead_s_ = std::min(
                last_target_lead_s_, target_lead_max_distance_m_ / target_speed_mps);
        }
        const TargetPrediction final_prediction = targetPredictionAt(
            get_clock()->now().seconds() + last_target_lead_s_);
        if (!final_prediction.valid) {
            throw std::runtime_error("TARGET_PREDICTOR_INVALID:" + final_prediction.detail);
        }
        last_target_prediction_valid_ = true;
        last_target_prediction_horizon_s_ = last_target_lead_s_;
        last_target_prediction_velocity_ = final_prediction.velocity;
        last_target_lead_distance_m_ =
            (final_prediction.position - *latest_target_).norm();
        return final_prediction.position;
    }

    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            disableBackend("DISABLED_BY_COMMAND");
            return;
        }
        enable_requested_ = true;
        if (!enabled_) latest_status_ = "ENABLE_REQUESTED";
        tryActivate();
    }

    void authorityCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        // Authority is a positive, latched request for the current enable epoch.
        // False is intentionally not a revocation command; transfer_enable=false
        // resets the backend atomically when the commissioned phase ends.
        if (!msg->data || authority_granted_ || authority_fresh_plan_pending_) return;
        pollPlannerFuture();
        if (!enabled_ || !prepared_candidate_ready_ || !shadow_commit_.has_value() ||
            !active_target_.has_value() || planner_future_.valid()) {
            latest_status_ = "AUTHORITY_GRANT_REJECTED_NOT_READY";
            return;
        }

        const double now_s = get_clock()->now().seconds();
        if (payload_collision_enabled_ && require_payload_attached_for_enable_) {
            bool object_attached = false;
            double attachment_age_s = 0.0;
            if (!currentObjectAttached(now_s, &object_attached, &attachment_age_s)) {
                latest_status_ = "AUTHORITY_GRANT_REJECTED_STALE_PAYLOAD_ATTACHMENT_STATE";
                return;
            }
            if (!object_attached || !suspended_geometry_.payload_attached) {
                latest_status_ = "AUTHORITY_GRANT_REJECTED_PAYLOAD_NOT_ATTACHED";
                return;
            }
        }
        if ((cooperative_scene_enabled_ || moving_basket_scene_enabled_) &&
            !cooperativeInputsFresh(now_s)) {
            latest_status_ = cooperativeAuthorityRejectStatus(now_s);
            return;
        }
        State measured;
        double age_s = 0.0;
        if (!currentState(now_s, &measured, &age_s)) {
            latest_status_ = "AUTHORITY_GRANT_REJECTED_STALE_STATE";
            return;
        }
        if (require_stationary_activation_ &&
            (measured.velocity.norm() > activation_speed_tolerance_mps_ ||
             measured.acceleration.norm() > activation_accel_tolerance_mps2_)) {
            latest_status_ = "AUTHORITY_GRANT_REJECTED_NOT_STATIONARY";
            return;
        }
        if ((cooperative_scene_enabled_ || moving_basket_scene_enabled_) &&
            !refreshCooperativeWorld(now_s)) {
            latest_status_ = cooperativeAuthorityRejectStatus(now_s);
            return;
        }

        try {
            active_source_target_ = *latest_target_;
            active_target_ = planningGoalForState(measured);
            active_target_velocity_ = last_target_prediction_velocity_;

            // M2D liveness path: preserve the already accepted prepared trajectory
            // shape instead of throwing it away at the authority boundary. The old
            // dynamic-world collision certificate is NOT reused. Shift the same
            // p/v/a trajectory so its frozen sample-0 begins at the current grant
            // time, then run the ordinary continuous collision checker against a
            // fresh current-world snapshot. Time shifting preserves the previously
            // certified kinematics; only the time-dependent collision certificate
            // needs to be renewed. Other C1F.8c users keep the commissioned fresh-
            // solve behavior unless they explicitly opt in.
            if (reuse_prepared_commit_on_authority_) {
                const CommittedTrajectory rebased = shadow_commit_->shiftedToStartTime(now_s);
                const WorldSnapshot latest_world = world_->snapshot(now_s);
                dynamic_planner::TrajectorySafetyChecker checker(1e-7);
                const auto safety = checker.checkCommitted(
                    rebased, now_s, rebased.endTime(), latest_world);
                if (safety.safe) {
                    auto normal_planner = std::make_unique<RecedingHorizonPlanner>(
                        *active_target_,
                        recedingConfig(
                            0.0, fixed_splice_lookahead_s_, spline_time_factor_, factor_alloc_,
                            factor_alloc_close_, close_to_goal_m_,
                            incumbent_failure_recheck_horizon_s_,
                            terminal_hold_guard_s_, terminal_hold_guard_interval_s_,
                            require_terminal_hold_for_nominal_, require_rendezvous_backup_for_nominal_,
                            defer_incumbent_conflict_until_reaction_horizon_, emergency_panic_horizon_s_,
                            enable_octopus_useful_deadline_, octopus_post_search_reserve_s_,
                            moving_rendezvous_enabled_, active_target_velocity_.value_or(Vec3::Zero()),
                            terminal_time_target_prediction_enabled_, target_time_fixed_point_iterations_,
                            target_time_fixed_point_tolerance_s_, target_lead_max_s_,
                            target_lead_high_confidence_max_s_, target_lead_max_distance_m_),
                        localConfig(
                            measured.position, *active_target_, minimum_search_z_m_,
                            smoother_jerk_weight_, smoother_goal_weight_, octopus_max_runtime_s_));
                    normal_planner->initializeCommittedTrajectory(rebased);
                    const bool committed = world_->runIfVersionCurrent(
                        latest_world.version,
                        [&]() {
                            planner_ = std::move(normal_planner);
                            shadow_commit_ = rebased;
                            prepared_candidate_ready_ = false;
                            authority_fresh_plan_pending_ = false;
                            authority_granted_ = true;
                            execution_tracking_consecutive_violations_ = 0;
                            execution_fallback_active_ = false;
                            execution_fallback_certified_ = true;
                            execution_fallback_reason_.clear();
                            execution_fallback_mode_.clear();
                            execution_fallback_pending_reason_.clear();
                            execution_fallback_restart_blocked_ = false;
                            execution_fallback_restart_z_m_ = std::numeric_limits<double>::quiet_NaN();
                            last_authority_grant_ros_s_ = now_s;
                            ++authority_rebase_count_;
                            latest_status_ = "AUTHORITY_GRANTED_REBASED_PREPARED";
                        });
                    if (!committed) {
                        RCLCPP_WARN(
                            get_logger(),
                            "Prepared trajectory passed fresh collision certification but the "
                            "world changed before authority commit; falling back to fresh solve.");
                    } else {
                        std_msgs::msg::Bool ack;
                        ack.data = true;
                        authority_ack_pub_->publish(ack);
                        RCLCPP_WARN(
                            get_logger(),
                            "M2D authority granted from rebased prepared trajectory after fresh "
                            "current-world collision certification; duration=%.3f s.",
                            rebased.endTime() - rebased.startTime());
                        return;
                    }
                } else {
                    RCLCPP_WARN(
                        get_logger(),
                        "Prepared trajectory rebase failed fresh collision certification "
                        "(%s obstacle=%s interval=[%.6f, %.6f]); falling back to fresh solve.",
                        safety.status.c_str(), safety.obstacle_name.c_str(),
                        safety.unsafe_interval_start_s, safety.unsafe_interval_end_s);
                }
            }

            // Default C1F.8c fallback: the prepared moving trajectory was
            // certified at earlier absolute times. Re-seed from the current
            // stationary grant state and obtain a fresh post-request commitment.

            // Give the one-time authority handoff enough deterministic lead for:
            // the bounded Octopus solve plus several 30 Hz ROS/reference callbacks.
            // This is a handshake contract, not an adaptive solver-runtime heuristic.
            const double authority_handoff_splice_s = preAuthoritySpliceLookaheadS();
            auto fresh_planner = std::make_unique<RecedingHorizonPlanner>(
                *active_target_,
                recedingConfig(
                    0.0, authority_handoff_splice_s, spline_time_factor_, factor_alloc_,
                    factor_alloc_close_, close_to_goal_m_,
                    incumbent_failure_recheck_horizon_s_,
                    terminal_hold_guard_s_, terminal_hold_guard_interval_s_,
                    require_terminal_hold_for_nominal_, require_rendezvous_backup_for_nominal_,
                    defer_incumbent_conflict_until_reaction_horizon_, emergency_panic_horizon_s_,
                    enable_octopus_useful_deadline_, octopus_post_search_reserve_s_,
                    moving_rendezvous_enabled_, active_target_velocity_.value_or(Vec3::Zero()),
                    terminal_time_target_prediction_enabled_, target_time_fixed_point_iterations_,
                    target_time_fixed_point_tolerance_s_, target_lead_max_s_,
                    target_lead_high_confidence_max_s_, target_lead_max_distance_m_),
                localConfig(
                    measured.position, *active_target_, minimum_search_z_m_,
                    smoother_jerk_weight_, smoother_goal_weight_,
                    octopus_pre_authority_max_runtime_s_));
            // Keep a stationary seed longer than one 5 Hz timer period. The first
            // candidate splices at authority_handoff_splice_s; this extra tail is
            // only a handshake buffer if scheduling is briefly delayed or a solve
            // is rejected and must be retried while Python continues to hold.
            const double fresh_authority_hold_s = std::max(
                0.50, fixed_splice_lookahead_s_ + 2.0 / std::max(1.0, planner_rate_hz_));
            CommittedTrajectory stationary_seed = dynamic_planner::makeStationaryHoverTrajectory(
                measured.position, now_s, fresh_authority_hold_s);
            fresh_planner->initializeCommittedTrajectory(stationary_seed);
            planner_ = std::move(fresh_planner);
            shadow_commit_ = std::move(stationary_seed);
            prepared_candidate_ready_ = false;
            authority_fresh_plan_pending_ = true;
            execution_tracking_consecutive_violations_ = 0;
            execution_fallback_active_ = false;
            execution_fallback_certified_ = true;
            execution_fallback_reason_.clear();
            execution_fallback_mode_.clear();
            execution_fallback_pending_reason_.clear();
            execution_fallback_restart_blocked_ = false;
            execution_fallback_restart_z_m_ = std::numeric_limits<double>::quiet_NaN();
            last_authority_grant_ros_s_ = now_s;
            latest_status_ = "AUTHORITY_REQUESTED_WAITING_FRESH_COMMIT";
            RCLCPP_WARN(
                get_logger(),
                "C1F.8c authority requested at ROS time %.6f; prepared reuse unavailable or "
                "rejected, seeded stationary hold, and launching a fresh post-grant solve immediately "
                "with %.1f ms handoff splice lead.",
                now_s, 1e3 * authority_handoff_splice_s);

            // Do not wait up to one 5 Hz timer period before the first post-request solve.
            // replanTimer() sees prepared_candidate_ready_=false and starts the normal
            // asynchronous transaction immediately from the stationary seed.
            replanTimer();
        } catch (const std::exception& exc) {
            authority_fresh_plan_pending_ = false;
            latest_status_ = std::string("AUTHORITY_FRESH_PLAN_SETUP_FAILED:") + exc.what();
            ++planner_worker_errors_;
            RCLCPP_ERROR(get_logger(), "C1F.8c authority fresh-plan setup failed: %s", exc.what());
        }
    }

    void disableBackend(const std::string& reason) {
        ++epoch_;
        enable_requested_ = false;
        enabled_ = false;
        prepared_candidate_ready_ = false;
        authority_granted_ = false;
        authority_fresh_plan_pending_ = false;
        planner_.reset();
        shadow_commit_.reset();
        latest_reference_positions_.clear();
        active_target_.reset();
        active_target_velocity_.reset();
        active_source_target_.reset();
        last_target_lead_s_ = 0.0;
        last_target_lead_distance_m_ = 0.0;
        last_target_travel_lead_s_ = 0.0;
        last_target_solve_lead_s_ = 0.0;
        last_target_rendezvous_time_unclamped_s_ = 0.0;
        execution_tracking_reference_valid_ = false;
        execution_tracking_consecutive_violations_ = 0;
        execution_fallback_active_ = false;
        execution_fallback_certified_ = true;
        execution_fallback_reason_.clear();
        execution_fallback_mode_.clear();
        execution_fallback_pending_reason_.clear();
        execution_fallback_restart_blocked_ = false;
        execution_fallback_restart_z_m_ = std::numeric_limits<double>::quiet_NaN();
        scene_witness_passed_ = !require_scene_witness_;
        scene_witness_summary_ = require_scene_witness_ ? "PENDING" : "NOT_REQUIRED";
        std_msgs::msg::Bool ack;
        ack.data = false;
        authority_ack_pub_->publish(ack);
        latest_status_ = reason;
    }

    bool currentState(double now_s, State* state, double* age_s) const {
        if (!latest_state_.has_value() || !last_state_receive_ros_s_.has_value()) return false;
        const double age = now_s - *last_state_receive_ros_s_;
        if (!std::isfinite(age) || age < -0.02 || age > state_timeout_s_) return false;
        *state = *latest_state_;
        *age_s = std::max(0.0, age);
        return true;
    }

    bool currentObjectAttached(double now_s, bool* attached, double* age_s) const {
        if (!latest_object_attached_.has_value() ||
            !last_object_attached_receive_ros_s_.has_value()) {
            return false;
        }
        const double age = now_s - *last_object_attached_receive_ros_s_;
        if (!std::isfinite(age) || age < -0.02 || age > object_attached_timeout_s_) {
            return false;
        }
        *attached = *latest_object_attached_;
        *age_s = std::max(0.0, age);
        return true;
    }

    double estimateExecutionTimeLagS(
        const CommittedTrajectory& committed,
        double now_s,
        const Vec3& measured_position) const {
        if (committed.empty() || !std::isfinite(now_s) || !measured_position.allFinite()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double search_start = std::max(committed.startTime(), now_s - 2.0);
        const double search_end = std::max(
            search_start, std::min(committed.endTime() + 0.5, now_s + 0.5));
        double best_time = now_s;
        double best_distance = std::numeric_limits<double>::infinity();
        const double dt = 1.0 / kControlReferenceHz;
        for (double t = search_start; t <= search_end + 1e-12; t += dt) {
            const double distance = (committed.evaluate(t).position - measured_position).norm();
            if (distance < best_distance) {
                best_distance = distance;
                best_time = t;
            }
        }
        return now_s - best_time;
    }

    bool activateExecutionFallback(
        const std::string& reason,
        double now_s,
        const State& measured) {
        if (!authority_granted_ || !active_target_.has_value()) return false;

        try {
            // Refresh the cooperative world immediately before certifying the
            // fallback. Replan-time world refreshes run at only 5 Hz and can be
            // older than the 30 Hz execution-certificate event that brought us
            // here. If fresh cooperative state is unavailable, still replace
            // the invalid advancing timeline with a stop, but diagnose it as
            // uncertified rather than claiming a dynamic-world guarantee.
            const bool cooperative_world_fresh =
                (!cooperative_scene_enabled_ && !moving_basket_scene_enabled_) ||
                refreshCooperativeWorld(now_s);
            const WorldSnapshot latest_world = world_->snapshot(now_s);
            dynamic_planner::TrajectorySafetyChecker checker(1e-7);

            CommittedTrajectory fallback = dynamic_planner::makeBrakingHoverTrajectory(
                measured, now_s, execution_brake_accel_limit_,
                execution_brake_min_duration_s_);
            const double smooth_check_until =
                fallback.endTime() + incumbent_failure_recheck_horizon_s_;
            auto safety = checker.checkCommitted(
                fallback, now_s, smooth_check_until, latest_world);

            bool certified = cooperative_world_fresh && safety.safe;
            std::string mode = cooperative_world_fresh
                ? "SMOOTH_BRAKE" : "UNCERTIFIED_SMOOTH_BRAKE_STALE_WORLD";
            if (cooperative_world_fresh && !safety.safe) {
                // If the smooth stopping arc is already obstructed, do not keep
                // advancing the invalid old timeline. Try a measured-position
                // hover as an emergency stop. This is intentionally a last-resort
                // path and is prominently diagnosed if it cannot itself be certified.
                fallback = dynamic_planner::makeStationaryHoverTrajectory(
                    measured.position, now_s, 0.10);
                const double hold_check_until = now_s + incumbent_failure_recheck_horizon_s_;
                safety = checker.checkCommitted(
                    fallback, now_s, hold_check_until, latest_world);
                // A stationary reference is not a physically certified transition
                // from nonzero measured velocity. Even if the hold point itself is
                // collision-free, label this last-resort discontinuous fallback as
                // uncertified rather than overclaiming a safe stop.
                certified = false;
                mode = "UNCERTIFIED_HARD_HOVER";
            }

            // Invalidate any in-flight worker. It cannot be cancelled, but its
            // epoch is now stale and pollPlannerFuture() will discard it without
            // overwriting this fallback planner/commit.
            ++epoch_;
            auto fallback_planner = std::make_unique<RecedingHorizonPlanner>(
                *active_target_,
                recedingConfig(
                    initial_splice_timing_s_, fixed_splice_lookahead_s_, spline_time_factor_, factor_alloc_,
                    factor_alloc_close_, close_to_goal_m_,
                    incumbent_failure_recheck_horizon_s_,
                    terminal_hold_guard_s_, terminal_hold_guard_interval_s_,
                    require_terminal_hold_for_nominal_, require_rendezvous_backup_for_nominal_,
                    defer_incumbent_conflict_until_reaction_horizon_, emergency_panic_horizon_s_,
                    enable_octopus_useful_deadline_, octopus_post_search_reserve_s_,
                    moving_rendezvous_enabled_, active_target_velocity_.value_or(Vec3::Zero()),
                    terminal_time_target_prediction_enabled_, target_time_fixed_point_iterations_,
                    target_time_fixed_point_tolerance_s_, target_lead_max_s_,
                    target_lead_high_confidence_max_s_, target_lead_max_distance_m_),
                localConfig(
                    fallback.endState().position, *active_target_, minimum_search_z_m_,
                    smoother_jerk_weight_, smoother_goal_weight_, octopus_max_runtime_s_));
            fallback_planner->initializeCommittedTrajectory(fallback);
            const State fallback_restart_state = fallback.endState();
            execution_fallback_restart_blocked_ =
                fallback_restart_state.position.z() < minimum_search_z_m_ - 1e-6;
            execution_fallback_restart_z_m_ = fallback_restart_state.position.z();
            planner_ = std::move(fallback_planner);
            shadow_commit_ = std::move(fallback);
            prepared_candidate_ready_ = false;
            execution_fallback_active_ = true;
            execution_fallback_certified_ = certified;
            execution_fallback_reason_ = reason;
            execution_fallback_mode_ = mode;
            execution_tracking_consecutive_violations_ = 0;
            ++execution_fallback_count_;
            latest_status_ = "EXECUTION_FALLBACK_" + mode + ":" + reason;
            RCLCPP_ERROR(
                get_logger(),
                "C1F.4 execution certificate invalid (%s). Installed %s fallback; certified=%s.",
                reason.c_str(), mode.c_str(), certified ? "true" : "false");
            return certified;
        } catch (const std::exception& exc) {
            execution_fallback_active_ = true;
            execution_fallback_certified_ = false;
            execution_fallback_reason_ = reason;
            execution_fallback_mode_ = "INSTALL_FAILED";
            ++execution_fallback_count_;
            latest_status_ = std::string("EXECUTION_FALLBACK_INSTALL_FAILED:") + exc.what();
            RCLCPP_ERROR(
                get_logger(), "C1F.4 failed to install execution fallback: %s", exc.what());
            return false;
        }
    }

    void updateExecutionTrackingDiagnostics(double now_s, const State& measured) {
        execution_tracking_reference_valid_ = false;
        execution_position_error_ = Vec3::Constant(
            std::numeric_limits<double>::quiet_NaN());
        execution_velocity_error_ = Vec3::Constant(
            std::numeric_limits<double>::quiet_NaN());
        execution_time_lag_s_ = std::numeric_limits<double>::quiet_NaN();
        if (!shadow_commit_.has_value() || !authority_granted_) return;
        try {
            const State expected = shadow_commit_->evaluate(now_s);
            execution_position_error_ = measured.position - expected.position;
            execution_velocity_error_ = measured.velocity - expected.velocity;
            execution_time_lag_s_ = estimateExecutionTimeLagS(
                *shadow_commit_, now_s, measured.position);
            execution_tracking_reference_valid_ = true;
        } catch (const std::exception&) {
            execution_position_error_ = Vec3::Constant(
                std::numeric_limits<double>::quiet_NaN());
            execution_velocity_error_ = Vec3::Constant(
                std::numeric_limits<double>::quiet_NaN());
            execution_time_lag_s_ = std::numeric_limits<double>::quiet_NaN();
        }
    }

    bool executionTrackingCertificateValid() const {
        if (!execution_tracking_safety_enabled_ || !execution_tracking_reference_valid_) {
            return true;
        }
        const bool position_valid =
            (execution_position_error_.cwiseAbs().array() <=
             ego_tracking_error_half_extents_.array()).all();
        const bool velocity_valid = execution_velocity_error_limit_mps_ <= 0.0 ||
            execution_velocity_error_.norm() <= execution_velocity_error_limit_mps_;
        return position_valid && velocity_valid;
    }

    double preAuthoritySpliceLookaheadS() const {
        // Preserve the commissioned timing exactly unless a caller explicitly
        // gives stationary pre-authority search more wall-clock budget than the
        // active-flight planner. In that diagnostic mode Python still owns the
        // stationary hover, so the candidate splice must also move farther into
        // the future or a useful long search could reject itself as late.
        if (octopus_pre_authority_max_runtime_s_ <= octopus_max_runtime_s_ + 1e-12) {
            return fixed_splice_lookahead_s_;
        }
        return std::min(
            1.0,
            std::max(fixed_splice_lookahead_s_, octopus_pre_authority_max_runtime_s_));
    }

    bool tryActivate() {
        if (!enable_requested_ || enabled_) return enabled_;
        pollPlannerFuture();
        if (planner_future_.valid()) {
            latest_status_ = "WAITING_OLD_WORKER";
            return false;
        }
        if (!latest_target_.has_value()) {
            latest_status_ = "WAITING_FOR_FIXED_TARGET";
            return false;
        }
        if (require_target_velocity_ && !latest_target_velocity_.has_value()) {
            latest_status_ = "WAITING_FOR_TARGET_VELOCITY";
            return false;
        }

        const double now_s = get_clock()->now().seconds();
        if (payload_collision_enabled_ && require_payload_attached_for_enable_) {
            bool object_attached = false;
            double attachment_age_s = 0.0;
            if (!currentObjectAttached(now_s, &object_attached, &attachment_age_s)) {
                latest_status_ = "WAITING_FOR_FRESH_PAYLOAD_ATTACHMENT_STATE";
                return false;
            }
            if (!object_attached) {
                latest_status_ = "WAITING_FOR_PAYLOAD_ATTACHED";
                return false;
            }
            if (!suspended_geometry_.payload_attached) {
                latest_status_ = "WAITING_FOR_PAYLOAD_GEOMETRY_SYNC";
                return false;
            }
        }
        if ((cooperative_scene_enabled_ || moving_basket_scene_enabled_) &&
            !cooperativeInputsFresh(now_s)) {
            latest_status_ = cooperativeWaitStatus(now_s);
            return false;
        }
        State measured;
        double age_s = 0.0;
        if (!currentState(now_s, &measured, &age_s)) {
            latest_status_ = "WAITING_FOR_FRESH_STATE";
            return false;
        }
        if (measured.position.z() < minimum_search_z_m_ - 1e-9) {
            latest_status_ = "WAITING_ABOVE_SEARCH_FLOOR";
            return false;
        }
        if (require_stationary_activation_ &&
            (measured.velocity.norm() > activation_speed_tolerance_mps_ ||
             measured.acceleration.norm() > activation_accel_tolerance_mps2_)) {
            latest_status_ = "WAITING_STATIONARY_ACTIVATION";
            return false;
        }

        if (require_scene_witness_) {
            try {
                const auto witness = evaluateC1eSceneWitness(
                    measured.position, *latest_target_, body_half_extents_,
                    suspended_geometry_, *physical_obstacle_);
                scene_witness_passed_ = witness.passed();
                scene_witness_summary_ = witness.summary();
            } catch (const std::exception& exc) {
                scene_witness_passed_ = false;
                scene_witness_summary_ = std::string("EXCEPTION:") + exc.what();
            }
            if (!scene_witness_passed_) {
                latest_status_ = "SCENE_WITNESS_FAILED";
                return false;
            }
        } else {
            scene_witness_passed_ = true;
            scene_witness_summary_ = "NOT_REQUIRED";
        }

        ++epoch_;
        prepared_candidate_ready_ = false;
        authority_granted_ = false;
        authority_fresh_plan_pending_ = false;
        active_source_target_ = *latest_target_;
        active_target_ = planningGoalForState(measured);
        // Position and velocity must describe the same predicted rendezvous
        // instant. This matters for curved targets: current velocity is not the
        // terminal velocity several seconds around the scripted circle.
        active_target_velocity_ = last_target_prediction_velocity_;
        planner_ = std::make_unique<RecedingHorizonPlanner>(
            *active_target_,
            recedingConfig(
                initial_splice_timing_s_, preAuthoritySpliceLookaheadS(), spline_time_factor_, factor_alloc_,
                factor_alloc_close_, close_to_goal_m_,
                incumbent_failure_recheck_horizon_s_,
                terminal_hold_guard_s_, terminal_hold_guard_interval_s_,
                require_terminal_hold_for_nominal_, require_rendezvous_backup_for_nominal_,
                defer_incumbent_conflict_until_reaction_horizon_, emergency_panic_horizon_s_,
                enable_octopus_useful_deadline_, octopus_post_search_reserve_s_,
                moving_rendezvous_enabled_, active_target_velocity_.value_or(Vec3::Zero()),
                    terminal_time_target_prediction_enabled_, target_time_fixed_point_iterations_,
                    target_time_fixed_point_tolerance_s_, target_lead_max_s_,
                    target_lead_high_confidence_max_s_, target_lead_max_distance_m_),
            localConfig(
                measured.position, *active_target_, minimum_search_z_m_,
                smoother_jerk_weight_, smoother_goal_weight_,
                octopus_pre_authority_max_runtime_s_));
        // When the pre-authority Octopus budget is intentionally longer than
        // the active-flight budget, preAuthoritySpliceLookaheadS() moves the
        // splice into the future. A future splice only exists if the core has an
        // incumbent to evaluate there. Seed a stationary hold for this diagnostic
        // mode even when the normal C1F.8c profile disables startup seeding.
        // Python still owns the measured stationary hover, so this changes only
        // planning time alignment and does not grant C++ execution authority.
        const bool extended_pre_authority_seed =
            octopus_pre_authority_max_runtime_s_ > octopus_max_runtime_s_ + 1e-12;
        if (seed_stationary_on_activation_ || extended_pre_authority_seed) {
            CommittedTrajectory stationary_seed = dynamic_planner::makeStationaryHoverTrajectory(
                measured.position, now_s, 0.10);
            planner_->initializeCommittedTrajectory(stationary_seed);
            shadow_commit_ = std::move(stationary_seed);
        } else {
            shadow_commit_.reset();
        }
        enabled_ = true;
        latest_status_ = (seed_stationary_on_activation_ || extended_pre_authority_seed)
            ? "ENABLED_STATIONARY_SEED"
            : "ENABLED_WAITING_FIRST_COMMIT";
        RCLCPP_WARN(get_logger(),
            "C1F.0 enabled at [%.3f %.3f %.3f] -> body target [%.3f %.3f %.3f]. PASSIVE shadow only.",
            measured.position.x(), measured.position.y(), measured.position.z(),
            active_target_->x(), active_target_->y(), active_target_->z());
        return true;
    }

    void pollPlannerFuture() {
        if (!planner_future_.valid()) return;
        if (planner_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;

        ScopedSteadyDurationRecorder processing_timer{
            &last_planner_result_processing_ms_,
            &max_planner_result_processing_ms_,
            nullptr,
            std::numeric_limits<double>::infinity()};
        PlannerOutcome outcome = planner_future_.get();
        const bool discarded = outcome.epoch != epoch_;
        writeReplanCsvRow(outcome, discarded);
        if (discarded) return;

        planner_ = std::move(outcome.planner);
        if (!terminal_time_target_prediction_enabled_ &&
            !outcome.worker_exception && outcome.result.attempted &&
            std::isfinite(outcome.result.replan_runtime_s) &&
            outcome.result.replan_runtime_s >= 0.0) {
            const double alpha = target_lead_solve_runtime_ema_alpha_;
            estimated_solve_runtime_s_ =
                alpha * outcome.result.replan_runtime_s +
                (1.0 - alpha) * estimated_solve_runtime_s_;
        }
        if (outcome.worker_exception) {
            ++planner_worker_errors_;
            latest_status_ = "WORKER_EXCEPTION:" + outcome.worker_error;
            return;
        }
        latest_replan_result_ = outcome.result;
        latest_status_ = outcome.result.status;
        if (outcome.result.candidate_late) ++late_candidates_;
        if (!outcome.result.accepted && outcome.result.attempted) {
            const auto witness = finalCheckFailureWitness(outcome.result);
            if (witness.has_value()) {
                if (witness->geometric_gap_m.has_value()) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "C1F final-check witness: obstacle=%s interval=%d gap_m=%.6f status=%s",
                        witness->obstacle_name.c_str(), witness->interval_index,
                        *witness->geometric_gap_m, outcome.result.status.c_str());
                } else {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "C1F final-check witness: obstacle=%s interval=%d gap_m=unavailable status=%s",
                        witness->obstacle_name.c_str(), witness->interval_index,
                        outcome.result.status.c_str());
                }
            }
        }

        if (outcome.result.incumbent_prefix_unsafe && authority_granted_) {
            if (outcome.result.attempted) ++failed_candidates_;
            State measured;
            double age_s = 0.0;
            const double now_s = get_clock()->now().seconds();
            if (currentState(now_s, &measured, &age_s)) {
                (void)activateExecutionFallback(
                    "INCUMBENT_REVALIDATION_UNSAFE", now_s, measured);
            } else {
                execution_fallback_pending_reason_ = "INCUMBENT_REVALIDATION_UNSAFE";
                latest_status_ = "INCUMBENT_UNSAFE_STATE_STALE_FALLBACK_PENDING";
            }
            return;
        }

        // C1F.8c moving-target commitments carry a certified short brake-to-stop
        // contingency, but no arbitrary multi-second hold. If a later replan
        // still exposes an unresolved incumbent conflict inside the measured
        // reaction horizon, install the existing measured-state emergency layer.
        if (!outcome.result.accepted &&
            outcome.result.incumbent_reaction_horizon_reached && authority_granted_) {
            State measured;
            double age_s = 0.0;
            const double now_s = get_clock()->now().seconds();
            if (currentState(now_s, &measured, &age_s)) {
                (void)activateExecutionFallback(
                    "INCUMBENT_REACTION_HORIZON_REACHED", now_s, measured);
            } else {
                execution_fallback_pending_reason_ = "INCUMBENT_REACTION_HORIZON_REACHED";
                latest_status_ = "INCUMBENT_REACTION_HORIZON_STATE_STALE_FALLBACK_PENDING";
            }
            return;
        }

        if (outcome.result.accepted) {
            const bool recovering_from_fallback = execution_fallback_active_;
            ++accepted_candidates_;
            if (planner_ && !planner_->committedTrajectory().empty()) {
                shadow_commit_ = planner_->committedTrajectory();
                if (!authority_granted_) {
                    if (authority_fresh_plan_pending_) {
                        // The first post-request solve uses a slightly longer handoff splice
                        // so Python can receive the ACK/reference while still on the stationary
                        // prefix. Restore the normal 100 ms commissioned splice for every
                        // subsequent receding-horizon transaction without changing the accepted
                        // commitment itself.
                        try {
                            auto normal_planner = std::make_unique<RecedingHorizonPlanner>(
                                *active_target_,
                                recedingConfig(
                                    0.0, fixed_splice_lookahead_s_, spline_time_factor_, factor_alloc_,
                                    factor_alloc_close_, close_to_goal_m_,
                                    incumbent_failure_recheck_horizon_s_,
                                    terminal_hold_guard_s_, terminal_hold_guard_interval_s_,
                                    require_terminal_hold_for_nominal_, require_rendezvous_backup_for_nominal_,
                                    defer_incumbent_conflict_until_reaction_horizon_, emergency_panic_horizon_s_,
                                    enable_octopus_useful_deadline_, octopus_post_search_reserve_s_,
                                    moving_rendezvous_enabled_, active_target_velocity_.value_or(Vec3::Zero()),
                                    terminal_time_target_prediction_enabled_, target_time_fixed_point_iterations_,
                                    target_time_fixed_point_tolerance_s_, target_lead_max_s_,
                                    target_lead_high_confidence_max_s_, target_lead_max_distance_m_),
                                localConfig(
                                    outcome.request_state.position, *active_target_, minimum_search_z_m_,
                                    smoother_jerk_weight_, smoother_goal_weight_, octopus_max_runtime_s_));
                            normal_planner->initializeCommittedTrajectory(*shadow_commit_);
                            planner_ = std::move(normal_planner);
                        } catch (const std::exception& exc) {
                            ++planner_worker_errors_;
                            authority_fresh_plan_pending_ = false;
                            disableBackend(std::string("AUTHORITY_NORMAL_PLANNER_REBUILD_FAILED:") + exc.what());
                            return;
                        }

                        authority_fresh_plan_pending_ = false;
                        authority_granted_ = true;
                        prepared_candidate_ready_ = false;
                        ++authority_fresh_commit_count_;
                        latest_status_ = "AUTHORITY_GRANTED_FRESH_COMMIT:" + outcome.result.status;
                        std_msgs::msg::Bool ack;
                        ack.data = true;
                        authority_ack_pub_->publish(ack);
                        RCLCPP_WARN(
                            get_logger(),
                            "C1F.8c execution authority granted from a fresh post-request commit "
                            "certified in the current dynamic world; normal %.1f ms splice restored.",
                            1e3 * fixed_splice_lookahead_s_);
                    } else {
                        prepared_candidate_ready_ = true;
                        latest_status_ = "PREPARED_WAITING_AUTHORITY";
                    }
                } else if (recovering_from_fallback) {
                    execution_fallback_active_ = false;
                    execution_fallback_certified_ = true;
                    execution_fallback_reason_.clear();
                    execution_fallback_mode_.clear();
                    execution_fallback_pending_reason_.clear();
                    execution_fallback_restart_blocked_ = false;
                    execution_fallback_restart_z_m_ = std::numeric_limits<double>::quiet_NaN();
                    ++execution_fallback_recovery_count_;
                    latest_status_ = "EXECUTION_FALLBACK_RECOVERED:" + outcome.result.status;
                }
            }
        } else if (outcome.result.attempted) {
            ++failed_candidates_;

            // C1F.8: the core planner already performs the C1F.4 failed-replan
            // incumbent revalidation. Reuse that exact certificate when the
            // world is still byte-semantically represented by the same
            // VersionedWorld version. This avoids running the same continuous
            // safety check synchronously from the 30 Hz ROS callback.
            //
            // Constant-velocity cooperative prediction is intentionally excluded
            // from this shortcut because fresh state samples are only folded into
            // VersionedWorld by refreshCooperativeWorld(), so unchanged version
            // alone does not prove unchanged live prediction inputs. Shared
            // commitments and the moving basket update VersionedWorld directly.
            if (authority_granted_ && shadow_commit_.has_value()) {
                const double certificate_now_s = get_clock()->now().seconds();
                const bool version_tracks_live_dynamic_inputs =
                    !cooperative_scene_enabled_ ||
                    cooperative_prediction_mode_ == "shared_trajectory";
                const bool live_dynamic_inputs_still_valid =
                    (!cooperative_scene_enabled_ ||
                     (cooperativeStatesFresh(certificate_now_s) &&
                      cooperativeSharedFreshCount(certificate_now_s) ==
                          cooperative_state_samples_.size())) &&
                    movingBasketTrajectoryFresh(certificate_now_s);
                const std::uint64_t current_world_version = world_->currentVersion();
                const bool core_recheck_is_current =
                    version_tracks_live_dynamic_inputs &&
                    live_dynamic_inputs_still_valid &&
                    outcome.result.incumbent_recheck_performed &&
                    outcome.result.incumbent_recheck_world_version == current_world_version;

                if (core_recheck_is_current) {
                    ++backend_incumbent_recheck_skipped_same_world_count_;
                } else {
                    ++backend_incumbent_recheck_count_;
                    const double now_s = get_clock()->now().seconds();
                    State measured;
                    double age_s = 0.0;
                    if (currentState(now_s, &measured, &age_s) &&
                        ((!cooperative_scene_enabled_ && !moving_basket_scene_enabled_) ||
                         refreshCooperativeWorld(now_s))) {
                        try {
                            const WorldSnapshot latest_world = world_->snapshot(now_s);
                            dynamic_planner::TrajectorySafetyChecker checker(1e-7);
                            const double check_until_s = shadow_commit_->endsInStoppedHold()
                                ? std::max(shadow_commit_->endTime(),
                                           now_s + incumbent_failure_recheck_horizon_s_)
                                : std::min(shadow_commit_->endTime(),
                                           now_s + incumbent_failure_recheck_horizon_s_);
                            const auto incumbent = checker.checkCommitted(
                                *shadow_commit_, now_s, check_until_s, latest_world);
                            if (!incumbent.safe) {
                                const double time_to_conflict_s = std::max(
                                    0.0, incumbent.unsafe_interval_start_s - now_s);
                                // Keep the runtime trigger consistent with the
                                // actual clamped-cubic braking fallback, whose peak
                                // initial acceleration is 2|v0|/T.
                                const double brake_time_s =
                                    (2.0 * measured.velocity.cwiseAbs().array() /
                                     execution_brake_accel_limit_.array()).maxCoeff();
                                const double fallback_horizon_s = std::max(
                                    emergency_panic_horizon_s_,
                                    brake_time_s + 1.0 / kControlReferenceHz);
                                const bool defer_conflict =
                                    defer_incumbent_conflict_until_reaction_horizon_ &&
                                    time_to_conflict_s > fallback_horizon_s;
                                if (!defer_conflict) {
                                    (void)activateExecutionFallback(
                                        "FAILED_REPLAN_LIVE_INCUMBENT_IMMINENT:" +
                                        incumbent.obstacle_name,
                                        now_s, measured);
                                } else {
                                    latest_status_ =
                                        "FAILED_REPLAN_FUTURE_CONFLICT_REPLAN_CONTINUES:" +
                                        incumbent.obstacle_name;
                                }
                            }
                        } catch (const std::exception& exc) {
                            execution_fallback_pending_reason_ =
                                std::string("FAILED_REPLAN_INCUMBENT_RECHECK_EXCEPTION:") + exc.what();
                            latest_status_ = "FAILED_REPLAN_INCUMBENT_RECHECK_EXCEPTION_FALLBACK_PENDING";
                        }
                    }
                }
            }
        }
    }

    void replanTimer() {
        pollPlannerFuture();
        if (!enabled_) {
            tryActivate();
            return;
        }
        if (planner_future_.valid()) {
            ++coalesced_replan_triggers_;
            return;
        }
        // During the one-time stationary ownership handshake, the first accepted
        // moving candidate is a prepared trajectory, not an executing trajectory.
        // Freeze it until Python explicitly grants authority. This prevents the
        // shadow planner from "flying away" in time while the real vehicle holds.
        if (!authority_granted_ && prepared_candidate_ready_) {
            latest_status_ = "PREPARED_WAITING_AUTHORITY";
            return;
        }
        if (!planner_ || !active_target_.has_value()) {
            ++planner_worker_errors_;
            latest_status_ = "ENABLED_WITHOUT_PLANNER";
            return;
        }

        const double now_s = get_clock()->now().seconds();
        if (payload_collision_enabled_ && require_payload_attached_for_enable_) {
            bool object_attached = false;
            double attachment_age_s = 0.0;
            if (!currentObjectAttached(now_s, &object_attached, &attachment_age_s)) {
                disableBackend("PAYLOAD_ATTACHMENT_STATE_STALE_DISABLED");
                return;
            }
            if (!object_attached || !suspended_geometry_.payload_attached) {
                disableBackend("PAYLOAD_NOT_ATTACHED_DISABLED");
                return;
            }
        }
        if ((cooperative_scene_enabled_ || moving_basket_scene_enabled_) &&
            !cooperativeInputsFresh(now_s)) {
            ++stale_cooperative_replan_skips_;
            disableBackend(cooperativeDisableStatus(now_s));
            return;
        }
        State measured;
        double age_s = 0.0;
        if (!currentState(now_s, &measured, &age_s)) {
            ++stale_state_replan_skips_;
            latest_status_ = "STATE_STALE_REPLAN_SKIPPED";
            return;
        }

        // C1F.5p2 recursive-feasibility contract: once emergency fallback has
        // authority, execute it to its stopped terminal state before asking
        // Octopus for a normal recovery. This avoids replanning from halfway
        // through a braking maneuver and makes the restart state identical to
        // the authoritative fallback endpoint used to build local bounds.
        if (execution_fallback_active_ && shadow_commit_.has_value()) {
            if (execution_fallback_restart_blocked_) {
                latest_status_ = "EXECUTION_FALLBACK_RESTART_BLOCKED_BELOW_MIN_SEARCH_Z";
                return;
            }
            if (now_s < shadow_commit_->endTime() - 1e-6) {
                latest_status_ = "EXECUTION_FALLBACK_EXECUTING_TO_RESTART_STATE";
                return;
            }
        }

        if (allow_live_target_updates_ && latest_target_.has_value()) {
            const Vec3 desired_goal = planningGoalForState(measured);
            const bool source_changed = !active_source_target_.has_value() ||
                (*latest_target_ - *active_source_target_).norm() > live_target_replan_threshold_m_;
            const bool goal_changed = !active_target_.has_value() ||
                (desired_goal - *active_target_).norm() > live_target_replan_threshold_m_;
            // planningGoalForState() also evaluates/stores the predictor's
            // velocity at the same future rendezvous instant as desired_goal.
            const Vec3 desired_terminal_velocity = last_target_prediction_velocity_;
            const bool velocity_changed = !active_target_velocity_.has_value() ||
                (desired_terminal_velocity - *active_target_velocity_).norm()
                    > live_target_velocity_replan_threshold_mps_;
            if (source_changed || goal_changed || velocity_changed) {
                const double carried_timing_s = planner_
                    ? planner_->previousSpliceTiming()
                    : initial_splice_timing_s_;
                active_source_target_ = *latest_target_;
                active_target_velocity_ = desired_terminal_velocity;
                active_target_ = desired_goal;
                const Vec3 local_bounds_start =
                    (execution_fallback_active_ && shadow_commit_.has_value())
                        ? shadow_commit_->endState().position
                        : measured.position;
                const bool rebuilding_pre_authority = !authority_granted_;
                const double rebuild_splice_lookahead_s = rebuilding_pre_authority
                    ? preAuthoritySpliceLookaheadS()
                    : fixed_splice_lookahead_s_;
                planner_ = std::make_unique<RecedingHorizonPlanner>(
                    *active_target_,
                    recedingConfig(
                        carried_timing_s, rebuild_splice_lookahead_s, spline_time_factor_, factor_alloc_,
                        factor_alloc_close_, close_to_goal_m_,
                        incumbent_failure_recheck_horizon_s_,
                        terminal_hold_guard_s_, terminal_hold_guard_interval_s_,
                        require_terminal_hold_for_nominal_, require_rendezvous_backup_for_nominal_,
                        defer_incumbent_conflict_until_reaction_horizon_, emergency_panic_horizon_s_,
                        enable_octopus_useful_deadline_, octopus_post_search_reserve_s_,
                        moving_rendezvous_enabled_, active_target_velocity_.value_or(Vec3::Zero()),
                    terminal_time_target_prediction_enabled_, target_time_fixed_point_iterations_,
                    target_time_fixed_point_tolerance_s_, target_lead_max_s_,
                    target_lead_high_confidence_max_s_, target_lead_max_distance_m_),
                    localConfig(
                        local_bounds_start, *active_target_, minimum_search_z_m_,
                        smoother_jerk_weight_, smoother_goal_weight_,
                        rebuilding_pre_authority
                            ? octopus_pre_authority_max_runtime_s_
                            : octopus_max_runtime_s_));
                if (shadow_commit_.has_value()) {
                    planner_->initializeCommittedTrajectory(*shadow_commit_);
                }
                ++live_target_rebuild_count_;
                latest_status_ = "LIVE_TARGET_RETARGETED";
            }
        }

        if (!refreshCooperativeWorld(now_s)) {
            ++stale_cooperative_replan_skips_;
            disableBackend(cooperativeDisableStatus(now_s));
            return;
        }

        auto planner = std::move(planner_);
        const std::uint64_t epoch = epoch_;
        const std::uint64_t sequence = ++replan_sequence_;
        const double submitted_octopus_wall_budget_s = authority_granted_
            ? octopus_max_runtime_s_
            : octopus_pre_authority_max_runtime_s_;
        const Vec3 target = *active_target_;
        const auto target_predictor = terminal_time_target_prediction_enabled_
            ? targetPredictorSnapshot(now_s)
            : std::shared_ptr<const dynamic_planner::TargetPredictor>{};
        if (terminal_time_target_prediction_enabled_ && !target_predictor) {
            planner_ = std::move(planner);
            latest_status_ = "TARGET_PREDICTOR_SNAPSHOT_UNAVAILABLE";
            return;
        }
        auto world = world_;
        auto ros_clock = get_clock();
        planner_future_ = std::async(
            std::launch::async,
            [sequence, epoch, planner = std::move(planner), target, target_predictor, world,
             ros_clock, measured, now_s, submitted_octopus_wall_budget_s]() mutable {
                PlannerOutcome outcome;
                outcome.sequence = sequence;
                outcome.epoch = epoch;
                outcome.request_ros_time_s = now_s;
                outcome.octopus_wall_budget_s = submitted_octopus_wall_budget_s;
                outcome.request_state = measured;
                outcome.target = target;
                outcome.planner = std::move(planner);
                try {
                    const TrajectoryTimeSource trajectory_clock = [ros_clock]() {
                        return ros_clock->now().seconds();
                    };
                    outcome.result = outcome.planner->replan(
                        now_s, measured, *world, trajectory_clock,
                        target_predictor.get());
                } catch (const std::exception& exc) {
                    outcome.worker_exception = true;
                    outcome.worker_error = exc.what();
                } catch (...) {
                    outcome.worker_exception = true;
                    outcome.worker_error = "unknown exception";
                }
                return outcome;
            });
    }

    void referenceTimer() {
        ScopedSteadyDurationRecorder callback_timer{
            &last_reference_callback_ms_,
            &max_reference_callback_ms_,
            &reference_callback_overrun_count_,
            1e3 / kControlReferenceHz};
        pollPlannerFuture();
        if (!enabled_ || !shadow_commit_.has_value()) return;

        const double now_s = get_clock()->now().seconds();
        State measured;
        double age_s = 0.0;
        if (!currentState(now_s, &measured, &age_s)) {
            ++stale_state_reference_skips_;
            latest_status_ = "STATE_STALE_REFERENCE_SUPPRESSED";
            return;
        }

        if (!execution_fallback_pending_reason_.empty() && authority_granted_ &&
            !execution_fallback_active_) {
            const std::string pending_reason = execution_fallback_pending_reason_;
            execution_fallback_pending_reason_.clear();
            (void)activateExecutionFallback(pending_reason, now_s, measured);
        }

        updateExecutionTrackingDiagnostics(now_s, measured);
        if (execution_tracking_safety_enabled_ && authority_granted_ &&
            !execution_fallback_active_) {
            if (!executionTrackingCertificateValid()) {
                ++execution_tracking_consecutive_violations_;
            } else {
                execution_tracking_consecutive_violations_ = 0;
            }
            if (execution_tracking_consecutive_violations_ >=
                execution_tracking_violation_samples_) {
                std::ostringstream reason;
                reason << "TRACKING_TUBE_VIOLATION pos=["
                       << execution_position_error_.x() << ' '
                       << execution_position_error_.y() << ' '
                       << execution_position_error_.z() << "] vel_norm="
                       << execution_velocity_error_.norm();
                (void)activateExecutionFallback(reason.str(), now_s, measured);
                // Re-evaluate diagnostics against the newly installed fallback
                // before publishing its first rolling window.
                updateExecutionTrackingDiagnostics(now_s, measured);
            }
        }

        auto publish_window = [&](
            const dynamic_planner::ReferenceWindow& window, double sampled_at_s) {
            const rclcpp::Time publish_time = get_clock()->now();
            const builtin_interfaces::msg::Time publish_stamp = publish_time;
            const auto message = makeReferenceMessage(
                window, publish_stamp, frame_id_, vehicle_id_);
            if (message.points.size() != 61U) {
                throw std::logic_error("C1F.2b generated a non-61-sample reference");
            }
            latest_reference_positions_.clear();
            latest_reference_positions_.reserve(window.samples.size());
            for (const auto& sample : window.samples) {
                latest_reference_positions_.push_back(sample.state.position);
            }
            shadow_reference_pub_->publish(message);
            if (last_reference_ros_s_.has_value()) {
                last_reference_period_ms_ = 1e3 * (now_s - *last_reference_ros_s_);
            }
            last_reference_ros_s_ = now_s;
            last_reference_sample_ros_s_ = sampled_at_s;
            ++reference_publish_count_;
        };

        try {
            // A pre-authority prepared trajectory is frozen at its original sample-0
            // only as readiness evidence. During the fresh-plan handshake shadow_commit_
            // is the stationary seed. Once the freshly certified commitment is accepted
            // and acknowledged, ordinary rolling-now sampling begins. No moving trajectory
            // is ever time-shifted across an authority boundary.
            const double sample_time_s = (!authority_granted_ && prepared_candidate_ready_)
                ? shadow_commit_->startTime()
                : now_s;
            const auto window = dynamic_planner::sampleCommittedReferenceWindow(
                *shadow_commit_, sample_time_s, reference_config_);
            publish_window(window, sample_time_s);
        } catch (const std::exception& exc) {
            ++reference_errors_;
            last_reference_error_ = exc.what();
            last_reference_error_ros_s_ = now_s;

            bool recovered_with_terminal_hold = false;
            try {
                if (now_s >= shadow_commit_->endTime() - dynamic_planner::kTrajectoryTimeToleranceS &&
                    shadow_commit_->endsInStoppedHold(1e-5)) {
                    const State endpoint = shadow_commit_->endState();
                    const CommittedTrajectory terminal_hold =
                        dynamic_planner::makeStationaryHoverTrajectory(
                            endpoint.position, now_s, 0.10);
                    const auto hold_window = dynamic_planner::sampleCommittedReferenceWindow(
                        terminal_hold, now_s, reference_config_);
                    publish_window(hold_window, now_s);
                    ++terminal_hold_reference_recoveries_;
                    latest_status_ = "TERMINAL_HOLD_REFERENCE_RECOVERY";
                    recovered_with_terminal_hold = true;
                }
            } catch (const std::exception& recovery_exc) {
                last_reference_error_ += std::string(" | terminal-hold recovery: ") +
                    recovery_exc.what();
            }

            if (!recovered_with_terminal_hold) {
                latest_status_ = "REFERENCE_GENERATION_EXCEPTION";
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "C1F.2b reference generation failed: %s", last_reference_error_.c_str());
            }
        }
    }

    void publishMarkers() noexcept {
        try {
            const rclcpp::Time now = get_clock()->now();
            const double now_s = now.seconds();
            const builtin_interfaces::msg::Time stamp = now;
            MarkerArray array;

            Marker clear;
            clear.action = Marker::DELETEALL;
            array.markers.push_back(std::move(clear));

            if (physical_obstacle_.has_value()) {
                Marker obstacle = baseMarker(
                    frame_id_, stamp, "physical_obstacle", 0, Marker::CUBE);
                obstacle.pose.position = markerPoint(obstacle_centre_);
                obstacle.pose.orientation.z = std::sin(0.5 * obstacle_yaw_rad_);
                obstacle.pose.orientation.w = std::cos(0.5 * obstacle_yaw_rad_);
                obstacle.scale.x = 2.0 * obstacle_half_extents_.x();
                obstacle.scale.y = 2.0 * obstacle_half_extents_.y();
                obstacle.scale.z = 2.0 * obstacle_half_extents_.z();
                obstacle.color = markerColour(0.95F, 0.35F, 0.05F, 0.72F);
                array.markers.push_back(std::move(obstacle));

                const auto components = dynamic_planner::assemblyComponents(
                    body_half_extents_, suspended_geometry_);
                int component_id = 0;
                for (const auto& component : components) {
                    const Eigen::MatrixXd cspace =
                        dynamic_planner::physicalObstacleToConfigurationSpace(
                            physical_obstacle_->vertices, component);
                    std::vector<Vec3> points;
                    points.reserve(static_cast<std::size_t>(cspace.rows()));
                    for (Eigen::Index row = 0; row < cspace.rows(); ++row) {
                        points.push_back(cspace.row(row).transpose());
                    }
                    auto colour = markerColour(0.20F, 0.80F, 1.00F, 0.48F);
                    if (component.name == "cable") {
                        colour = markerColour(1.00F, 0.85F, 0.10F, 0.48F);
                    } else if (component.name == "magnet") {
                        colour = markerColour(1.00F, 0.10F, 0.80F, 0.48F);
                    } else if (component.name == "payload") {
                        colour = markerColour(0.70F, 0.20F, 1.00F, 0.58F);
                    }
                    array.markers.push_back(pointsMarker(
                        frame_id_, stamp, "cspace_" + component.name,
                        component_id++, points, 0.025F, colour));
                }
            }

            if (latest_state_.has_value()) {
                const Vec3 ego = latest_state_->position;
                const auto components = dynamic_planner::assemblyComponents(
                    body_half_extents_, suspended_geometry_);

                // Physical geometry: opaque markers answer "where is the hardware?".
                array.markers.push_back(cubeMarker(
                    frame_id_, stamp, "ego_physical_body", 0, ego, body_half_extents_,
                    markerColour(0.20F, 0.85F, 1.00F, 0.90F)));
                array.markers.push_back(cubeMarker(
                    frame_id_, stamp, "ego_tracking_body", 0, ego,
                    body_half_extents_ + ego_tracking_error_half_extents_,
                    markerColour(0.20F, 0.85F, 1.00F, 0.18F)));

                if (suspended_geometry_.enabled) {
                    const Vec3 cable_center = ego + Vec3(
                        0.0, 0.0, -0.5 * suspended_geometry_.cable_length_m);
                    array.markers.push_back(cylinderMarker(
                        frame_id_, stamp, "ego_physical_cable", 0, cable_center,
                        2.0 * suspended_geometry_.cable_radius_m,
                        suspended_geometry_.cable_length_m,
                        markerColour(1.00F, 0.88F, 0.05F, 1.00F)));

                    const Vec3 magnet_center = ego + Vec3(
                        0.0, 0.0,
                        -suspended_geometry_.cable_length_m -
                        suspended_geometry_.magnet_center_below_cable_end_m);
                    array.markers.push_back(cylinderMarker(
                        frame_id_, stamp, "ego_physical_magnet", 0, magnet_center,
                        2.0 * suspended_geometry_.magnet_half_extents.x(),
                        2.0 * suspended_geometry_.magnet_half_extents.z(),
                        markerColour(1.00F, 0.15F, 0.78F, 0.95F)));

                    if (suspended_geometry_.payload_attached) {
                        const Vec3 payload_center = magnet_center +
                            suspended_geometry_.payload_center_from_magnet_center;
                        array.markers.push_back(cubeMarker(
                            frame_id_, stamp, "ego_physical_payload", 0, payload_center,
                            suspended_geometry_.payload_half_extents,
                            markerColour(0.65F, 0.20F, 1.00F, 0.95F)));
                    }
                }

                // Collision geometry: transparent markers are generated from the
                // same assemblyComponents() objects used by planner safety checks.
                // The exact cable swing frustum is shown separately from the
                // tracking-expanded AABB used by cooperative component-pair checks.
                // Magnet/payload components are already AABBs, so adding the ego
                // tracking allowance visualizes their protected instantaneous volume.
                for (const auto& component : components) {
                    if (component.name == "cable") {
                        array.markers.push_back(cableEnvelopeSurfaceMarker(
                            frame_id_, stamp, "ego_collision_cable", 0, component, ego,
                            markerColour(1.00F, 0.88F, 0.05F, 0.22F)));
                        array.markers.push_back(componentAabbMarker(
                            frame_id_, stamp, "ego_tracking_cable", 0, component, ego,
                            ego_tracking_error_half_extents_,
                            markerColour(1.00F, 0.88F, 0.05F, 0.08F)));
                    } else if (component.name == "magnet") {
                        array.markers.push_back(componentAabbMarker(
                            frame_id_, stamp, "ego_collision_magnet", 0, component, ego,
                            ego_tracking_error_half_extents_,
                            markerColour(1.00F, 0.15F, 0.78F, 0.16F)));
                    } else if (component.name == "payload") {
                        array.markers.push_back(componentAabbMarker(
                            frame_id_, stamp, "ego_collision_payload", 0, component, ego,
                            ego_tracking_error_half_extents_,
                            markerColour(0.65F, 0.20F, 1.00F, 0.16F)));
                    }
                }
            }

            if (cooperative_scene_enabled_) {
                const double preview_s = std::min(3.0, cooperative_prediction_horizon_s_);
                for (std::size_t i = 0; i < cooperative_state_samples_.size(); ++i) {
                    const auto& sample = cooperative_state_samples_[i];
                    if (!sample.received) continue;
                    const double age_s = now_s - sample.receive_ros_s;
                    const bool fresh = std::isfinite(age_s) && age_s >= -0.02 &&
                        age_s <= cooperative_state_timeout_s_;
                    const auto physical_colour = fresh
                        ? markerColour(0.10F, 0.75F, 1.00F, 0.90F)
                        : markerColour(1.00F, 0.15F, 0.10F, 0.90F);
                    const auto tube_colour = fresh
                        ? markerColour(0.10F, 0.75F, 1.00F, 0.22F)
                        : markerColour(1.00F, 0.15F, 0.10F, 0.22F);

                    Marker physical = baseMarker(
                        frame_id_, stamp, "cooperative_physical", static_cast<int>(i), Marker::CUBE);
                    physical.pose.position = markerPoint(sample.position);
                    physical.scale.x = 2.0 * cooperative_physical_half_extents_.x();
                    physical.scale.y = 2.0 * cooperative_physical_half_extents_.y();
                    physical.scale.z = 2.0 * cooperative_physical_half_extents_.z();
                    physical.color = physical_colour;
                    array.markers.push_back(std::move(physical));

                    Marker tube = baseMarker(
                        frame_id_, stamp, "cooperative_tracking_tube", static_cast<int>(i), Marker::CUBE);
                    tube.pose.position = markerPoint(sample.position);
                    const Vec3 tube_half = cooperative_physical_half_extents_ +
                                           cooperative_tracking_error_half_extents_;
                    tube.scale.x = 2.0 * tube_half.x();
                    tube.scale.y = 2.0 * tube_half.y();
                    tube.scale.z = 2.0 * tube_half.z();
                    tube.color = tube_colour;
                    array.markers.push_back(std::move(tube));

                    // M2D attached-peer geometry: visualize both the physical
                    // anchor-to-plate segment and the exact conservative AABB
                    // component used by cooperative collision checking.
                    if (cooperative_attached_tether_enabled_) {
                        try {
                            const AttachedTetherGeometry tether =
                                cooperativeAttachedTetherGeometry(i, sample.position, now_s, false);
                            if (tether.enabled) {
                                const Vec3 tether_start = sample.position + tether.anchor_from_body;
                                const Vec3 tether_end = sample.position + tether.plate_from_body;
                                array.markers.push_back(lineStripMarker(
                                    frame_id_, stamp, "cooperative_attached_tether_physical",
                                    static_cast<int>(i), {tether_start, tether_end},
                                    static_cast<float>(2.0 * tether.radius_m),
                                    markerColour(1.00F, 0.78F, 0.08F, 0.95F)));
                                const auto peer_components =
                                    dynamic_planner::cooperativeAssemblyComponents(
                                        cooperative_physical_half_extents_, SuspendedGeometry{}, tether);
                                for (const auto& component : peer_components) {
                                    if (component.name != "attached_tether") continue;
                                    array.markers.push_back(componentAabbMarker(
                                        frame_id_, stamp,
                                        "cooperative_attached_tether_collision",
                                        static_cast<int>(i), component, sample.position,
                                        cooperative_tracking_error_half_extents_,
                                        markerColour(1.00F, 0.35F, 0.05F, 0.18F)));
                                }
                            }
                        } catch (const std::exception& exc) {
                            RCLCPP_WARN_THROTTLE(
                                get_logger(), *get_clock(), 2000,
                                "M2D tether marker skipped for peer %zu: %s", i, exc.what());
                        }
                    }

                    std::vector<Vec3> predicted_points;
                    std::string preview_namespace = "cooperative_cv_preview";
                    if (cooperative_prediction_mode_ == "shared_trajectory" &&
                        i < cooperative_trajectory_samples_.size() &&
                        cooperative_trajectory_samples_[i].received) {
                        const auto& shared = cooperative_trajectory_samples_[i].trajectory;
                        const double explicit_remaining = shared.endsInStoppedHold()
                            ? preview_s
                            : std::max(0.0, shared.endTime() - now_s);
                        const double horizon = std::min(preview_s, explicit_remaining);
                        for (double dt = 0.0; dt < horizon - 1e-9; dt += 0.20) {
                            predicted_points.push_back(shared.evaluate(now_s + dt).position);
                        }
                        if (horizon > 0.0) {
                            predicted_points.push_back(shared.evaluate(now_s + horizon).position);
                        }
                        preview_namespace = "cooperative_shared_preview";
                    } else {
                        predicted_points = {
                            sample.position,
                            sample.position + sample.velocity * preview_s};
                    }
                    if (predicted_points.size() >= 2U) {
                        array.markers.push_back(lineStripMarker(
                            frame_id_, stamp, preview_namespace, static_cast<int>(i),
                            predicted_points, 0.018F, physical_colour));
                    }
                }
            }

            if (moving_basket_scene_enabled_ &&
                moving_basket_collision_mode_ == "segmented_ring" &&
                cooperative_ring_pose_sample_.received &&
                moving_basket_trajectory_sample_.received &&
                !moving_basket_trajectory_sample_.trajectory.empty()) {
                const auto& ring_trajectory = moving_basket_trajectory_sample_.trajectory;
                const double ring_marker_time = std::clamp(
                    now_s, ring_trajectory.startTime(), ring_trajectory.endTime());
                const Vec3 committed_ring_centre =
                    ring_trajectory.evaluate(ring_marker_time).position;
                for (int i = 0; i < segmented_ring_geometry_.segment_count; ++i) {
                    const Vec3 centre = committed_ring_centre +
                        segmented_ring_geometry_.segmentWorldCenterOffset(
                            cooperative_ring_pose_sample_.rotation, i);
                    const Vec3 physical_half = segmented_ring_geometry_.segmentWorldHalfExtents(
                        cooperative_ring_pose_sample_.rotation, i);
                    array.markers.push_back(cubeMarker(
                        frame_id_, stamp, "m2d_ring_segment_collision", i,
                        centre, physical_half,
                        markerColour(0.95F, 0.45F, 0.08F, 0.72F)));
                    array.markers.push_back(cubeMarker(
                        frame_id_, stamp, "m2d_ring_segment_tracking", i,
                        centre, physical_half + moving_basket_tracking_error_half_extents_,
                        markerColour(0.95F, 0.45F, 0.08F, 0.16F)));
                }
            }

            if (!latest_reference_positions_.empty()) {
                array.markers.push_back(lineStripMarker(
                    frame_id_, stamp, "cpp_reference", 0,
                    latest_reference_positions_, 0.025F,
                    markerColour(0.10F, 1.00F, 0.95F, 0.95F)));
            }
            if (latest_target_.has_value()) {
                array.markers.push_back(sphereMarker(
                    frame_id_, stamp, "moving_target", 0, *latest_target_, 0.11F,
                    markerColour(0.20F, 1.00F, 0.20F, 0.95F)));
            }
            if (active_target_.has_value()) {
                array.markers.push_back(sphereMarker(
                    frame_id_, stamp, "planning_goal", 0, *active_target_, 0.095F,
                    markerColour(1.00F, 0.35F, 0.85F, 0.95F)));
            }
            if (latest_replan_result_.attempted) {
                array.markers.push_back(sphereMarker(
                    frame_id_, stamp, "splice_point", 0,
                    latest_replan_result_.splice_state.position, 0.075F,
                    markerColour(1.00F, 1.00F, 0.10F, 0.95F)));
                array.markers.push_back(sphereMarker(
                    frame_id_, stamp, "local_goal", 0,
                    latest_replan_result_.local_goal, 0.085F,
                    markerColour(1.00F, 0.55F, 0.10F, 0.95F)));

                const auto witness = finalCheckFailureWitness(latest_replan_result_);
                if (witness.has_value() && latest_replan_result_.local_plan.has_value()) {
                    const auto& local_plan = *latest_replan_result_.local_plan;
                    const auto& diagnostic_cp =
                        local_plan.search.diagnostic_control_points;
                    if (diagnostic_cp.has_value() && !local_plan.knots.empty()) {
                        const double t_start =
                            local_plan.knots.at(static_cast<std::size_t>(dynamic_planner::kCubicDegree));
                        const double t_end = local_plan.knots.at(
                            static_cast<std::size_t>(diagnostic_cp->rows()));
                        std::vector<Vec3> failed_path;
                        constexpr int kFailureSamples = 48;
                        failed_path.reserve(kFailureSamples + 1);
                        for (int sample_index = 0; sample_index <= kFailureSamples; ++sample_index) {
                            const double alpha = static_cast<double>(sample_index) /
                                static_cast<double>(kFailureSamples);
                            const double t = t_start + alpha * (t_end - t_start);
                            failed_path.push_back(dynamic_planner::evaluateBSpline(
                                *diagnostic_cp, local_plan.knots,
                                dynamic_planner::kCubicDegree, t));
                        }
                        array.markers.push_back(lineStripMarker(
                            frame_id_, stamp, "failed_candidate_path", 0,
                            failed_path, 0.035F,
                            markerColour(1.00F, 0.08F, 0.08F, 0.95F)));

                        std::vector<Vec3> control_points;
                        control_points.reserve(static_cast<std::size_t>(diagnostic_cp->rows()));
                        for (Eigen::Index row = 0; row < diagnostic_cp->rows(); ++row) {
                            control_points.push_back(diagnostic_cp->row(row).transpose());
                        }
                        array.markers.push_back(pointsMarker(
                            frame_id_, stamp, "failed_candidate_control_points", 0,
                            control_points, 0.045F,
                            markerColour(1.00F, 0.40F, 0.10F, 0.95F)));
                    }

                    Marker failure_text = baseMarker(
                        frame_id_, stamp, "final_check_witness", 0, Marker::TEXT_VIEW_FACING);
                    failure_text.pose.position = markerPoint(
                        latest_replan_result_.local_goal + Vec3(0.0, 0.0, 0.18));
                    failure_text.scale.z = 0.055;
                    failure_text.color = markerColour(1.00F, 0.20F, 0.15F, 1.00F);
                    std::ostringstream witness_text;
                    witness_text << "FINAL CHECK FAIL i=" << witness->interval_index
                                 << "\n" << witness->obstacle_name;
                    if (witness->geometric_gap_m.has_value()) {
                        witness_text << "\ngap=" << std::fixed << std::setprecision(4)
                                     << *witness->geometric_gap_m << " m";
                    }
                    failure_text.text = witness_text.str();
                    array.markers.push_back(std::move(failure_text));
                }
            }

            Marker status = baseMarker(
                frame_id_, stamp, "c1f3_status", 0, Marker::TEXT_VIEW_FACING);
            const Vec3 status_anchor = physical_obstacle_.has_value()
                ? obstacle_centre_ + Vec3(0.0, 0.0, obstacle_half_extents_.z() + 0.22)
                : (latest_state_.has_value()
                    ? latest_state_->position + Vec3(0.0, 0.0, 0.30)
                    : Vec3(0.0, 0.0, 0.30));
            status.pose.position = markerPoint(status_anchor);
            status.scale.z = 0.065;
            status.color = markerColour(1.00F, 1.00F, 1.00F, 1.00F);
            bool attached = false;
            double attachment_age_s = 0.0;
            const bool attachment_fresh = currentObjectAttached(
                now.seconds(), &attached, &attachment_age_s);
            std::ostringstream status_text;
            status_text << "C1F transfer backend"
                        << "\nscene=" << (static_scene_enabled_ ? "ON" : "OFF")
                        << " witness=" << (scene_witness_passed_ ? "PASS" : "PENDING/FAIL")
                        << "\npayload_collision=" << (payload_collision_enabled_ ? "ON" : "OFF")
                        << " geom=" << (suspended_geometry_.payload_attached ? "ACTIVE" : "INACTIVE")
                        << " attached=";
            if (!attachment_fresh) {
                status_text << "STALE";
            } else {
                status_text << (attached ? "true" : "false");
            }
            double cooperative_max_age_s = 0.0;
            const std::size_t cooperative_fresh = cooperativeFreshCount(
                now.seconds(), &cooperative_max_age_s);
            status_text << "\ncooperative="
                        << (cooperative_scene_enabled_ ? "ON" : "OFF")
                        << " fresh=" << cooperative_fresh << "/"
                        << cooperative_state_samples_.size();
            if (cooperative_scene_enabled_) {
                status_text << " max_age_ms=" << 1e3 * cooperative_max_age_s;
            }
            status_text << "\nstatus=" << latest_status_;
            if (const auto witness = finalCheckFailureWitness(latest_replan_result_);
                witness.has_value()) {
                status_text << "\nwitness=i" << witness->interval_index
                            << " " << witness->obstacle_name;
            }
            status.text = status_text.str();
            array.markers.push_back(std::move(status));

            marker_pub_->publish(array);
        } catch (const std::exception& exc) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "C1F transfer visualization update skipped: %s", exc.what());
        } catch (...) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "C1F transfer visualization update skipped: unknown exception");
        }
    }

    void writeReplanCsvRow(const PlannerOutcome& outcome, bool discarded) {
        if (!replan_csv_) return;
        const ReplanResult& result = outcome.result;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        double octopus_ms = nan;
        double qp_ms = nan;
        double local_plan_total_ms = nan;
        std::string termination;
        std::size_t terminal_hold_rejections = 0U;
        std::size_t octopus_separator_lp_calls = 0U;
        std::size_t octopus_aabb_skips = 0U;
        std::size_t terminal_hold_separator_lp_calls = 0U;
        std::size_t partial_fallback_retained = 0U;
        std::size_t partial_fallback_tested = 0U;
        std::size_t partial_fallback_selected_rank = 0U;
        Vec3 initial_q0 = Vec3::Constant(nan);
        Vec3 initial_q2 = Vec3::Constant(nan);
        Vec3 search_xyz_min = Vec3::Constant(nan);
        Vec3 search_xyz_max = Vec3::Constant(nan);
        double initial_q2_radius_m = nan;
        int initial_q2_box_valid = -1;
        int initial_q2_radius_valid = -1;
        if (result.local_plan.has_value()) {
            octopus_ms = 1e3 * result.local_plan->search.search_time_s;
            local_plan_total_ms = 1e3 * result.local_plan->total_solve_time_s;
            termination = result.local_plan->search.termination_reason;
            terminal_hold_rejections = result.local_plan->search.terminal_hold_rejections;
            octopus_separator_lp_calls = result.local_plan->search.separator_lp_calls;
            octopus_aabb_skips = result.local_plan->search.aabb_separation_skips;
            terminal_hold_separator_lp_calls =
                result.local_plan->search.terminal_hold_separator_lp_calls;
            partial_fallback_retained =
                result.local_plan->search.partial_fallback_candidates_retained;
            partial_fallback_tested =
                result.local_plan->search.partial_fallback_candidates_tested;
            partial_fallback_selected_rank =
                result.local_plan->search.partial_fallback_selected_rank;
            initial_q0 = result.local_plan->search.initial_q0;
            initial_q2 = result.local_plan->search.initial_q2;
            search_xyz_min = result.local_plan->search.search_xyz_min;
            search_xyz_max = result.local_plan->search.search_xyz_max;
            initial_q2_radius_m = result.local_plan->search.initial_q2_radius_m;
            initial_q2_box_valid = result.local_plan->search.initial_q2_box_valid ? 1 : 0;
            initial_q2_radius_valid = result.local_plan->search.initial_q2_radius_valid ? 1 : 0;
            if (result.local_plan->refinement.has_value()) {
                qp_ms = 1e3 * result.local_plan->refinement->solve_time_s;
            }
        }
        std::size_t safety_separator_lp_calls = 0U;
        std::size_t safety_aabb_skips = 0U;
        const auto addSafetyDiagnostics = [&](const std::optional<dynamic_planner::TrajectorySafetyResult>& safety) {
            if (!safety.has_value()) return;
            safety_separator_lp_calls += safety->separator_lp_calls;
            safety_aabb_skips += safety->aabb_separation_skips;
        };
        addSafetyDiagnostics(result.prefix_safety);
        addSafetyDiagnostics(result.candidate_safety);
        addSafetyDiagnostics(result.candidate_backup_safety);
        addSafetyDiagnostics(result.candidate_terminal_hold_safety);
        const double margin_ms = result.attempted
            ? 1e3 * (result.splice_time_s - result.authority_decision_time_s)
            : nan;
        replan_csv_ << std::setprecision(15)
                    << outcome.sequence << ',' << outcome.request_ros_time_s << ','
                    << safeCsvField(discarded ? "DISCARDED_EPOCH:" + result.status : result.status) << ','
                    << result.attempted << ',' << result.accepted << ',' << result.candidate_late << ','
                    << 1e3 * result.splice_lookahead_s << ',' << safeCsvField(termination) << ','
                    << octopus_ms << ',' << qp_ms << ',' << local_plan_total_ms << ','
                    << 1e3 * result.replan_runtime_s << ','
                    << 1e3 * result.world_snapshot_s << ','
                    << 1e3 * result.obstacle_build_s << ','
                    << 1e3 * result.safety_check_s << ','
                    << 1e3 * result.atomic_commit_s << ','
                    << octopus_separator_lp_calls << ',' << octopus_aabb_skips << ','
                    << terminal_hold_separator_lp_calls << ','
                    << partial_fallback_retained << ',' << partial_fallback_tested << ','
                    << partial_fallback_selected_rank << ','
                    << safety_separator_lp_calls << ',' << safety_aabb_skips << ','
                    << 1e3 * result.octopus_post_search_reserve_s << ','
                    << 1e3 * result.octopus_useful_budget_s << ','
                    << 1e3 * outcome.octopus_wall_budget_s << ',' << margin_ms << ','
                    << result.moving_target_terminal_active << ','
                    << result.local_continuation_active << ','
                    << result.moving_rendezvous_active << ','
                    << result.terminal_target_velocity.x() << ','
                    << result.terminal_target_velocity.y() << ','
                    << result.terminal_target_velocity.z() << ','
                    << result.target_time_prediction_used << ','
                    << result.target_time_prediction_valid << ','
                    << result.target_time_prediction_high_confidence << ','
                    << result.target_time_fixed_point_iterations << ','
                    << 1e3 * result.target_time_fixed_point_residual_s << ','
                    << result.predicted_target_time_s << ','
                    << result.predicted_target_horizon_s << ','
                    << result.predicted_target_position.x() << ','
                    << result.predicted_target_position.y() << ','
                    << result.predicted_target_position.z() << ','
                    << result.predicted_target_velocity.x() << ','
                    << result.predicted_target_velocity.y() << ','
                    << result.predicted_target_velocity.z() << ','
                    << result.terminal_velocity_error_mps << ','
                    << result.rendezvous_backup_duration_s << ','
                    << safeCsvField(planning_policy_profile_) << ','
                    << dynamic_planner::candidateKindName(result.candidate_kind) << ','
                    << (result.candidate_safety.has_value() && result.candidate_safety->safe) << ','
                    << require_terminal_hold_for_nominal_ << ','
                    << result.incumbent_time_to_conflict_s << ','
                    << result.estimated_brake_time_s << ','
                    << result.fallback_trigger_horizon_s << ','
                    << initial_q0.x() << ',' << initial_q0.y() << ',' << initial_q0.z() << ','
                    << initial_q2.x() << ',' << initial_q2.y() << ',' << initial_q2.z() << ','
                    << search_xyz_min.x() << ',' << search_xyz_min.y() << ',' << search_xyz_min.z() << ','
                    << search_xyz_max.x() << ',' << search_xyz_max.y() << ',' << search_xyz_max.z() << ','
                    << initial_q2_radius_m << ',' << initial_q2_box_valid << ','
                    << initial_q2_radius_valid << ','
                    << result.committed_endpoint_distance_m << ','
                    << safeCsvField(optionalSafetyStatus(result.prefix_safety)) << ','
                    << safeCsvField(optionalSafetyStatus(result.candidate_safety)) << ','
                    << safeCsvField(optionalSafetyStatus(result.candidate_backup_safety)) << ','
                    << safeCsvField(optionalSafetyStatus(result.candidate_terminal_hold_safety)) << ','
                    << terminal_hold_rejections << ','
                    << result.incumbent_prefix_unsafe << ','
                    << result.planning_world_version << ','
                    << result.recheck_world_version << ','
                    << result.final_recheck_world_version << ','
                    << safeCsvField(result.recheck_error) << ','
                    << result.incumbent_recheck_performed << ','
                    << result.incumbent_recheck_world_version << ','
                    << outcome.worker_exception << ',' << safeCsvField(outcome.worker_error) << '\n';
        replan_csv_.flush();
    }

    void diagnosticsTimer() {
        pollPlannerFuture();
        if (!enabled_) tryActivate();

        const double now_s = get_clock()->now().seconds();
        State measured;
        double age_s = std::numeric_limits<double>::quiet_NaN();
        const bool state_fresh = currentState(now_s, &measured, &age_s);
        bool object_attached = false;
        double attachment_age_s = std::numeric_limits<double>::quiet_NaN();
        const bool attachment_fresh = currentObjectAttached(
            now_s, &object_attached, &attachment_age_s);
        double cooperative_max_age_s = 0.0;
        const std::size_t cooperative_fresh_count = cooperativeFreshCount(
            now_s, &cooperative_max_age_s);
        double cooperative_min_future_coverage_s = std::numeric_limits<double>::infinity();
        double cooperative_max_tracking_ratio = 0.0;
        const std::size_t cooperative_shared_fresh_count = cooperativeSharedFreshCount(
            now_s, &cooperative_min_future_coverage_s, &cooperative_max_tracking_ratio);

        std::ostringstream text;
        text << std::fixed << std::setprecision(3)
             << (allow_live_target_updates_
                    ? "C1F.2 LIVE-TARGET TRANSFER BACKEND\n"
                    : "C1F.0 PASSIVE FIXED-TARGET TRANSFER BACKEND\n")
             << "enable_requested/enabled: " << std::boolalpha
             << enable_requested_ << " / " << enabled_ << "\n"
             << "state_fresh: " << state_fresh
             << " age_ms: " << (state_fresh ? 1e3 * age_s : -1.0) << "\n"
             << "payload collision/geometry active: " << payload_collision_enabled_ << " / "
             << suspended_geometry_.payload_attached
             << " attachment_fresh/value/age_ms: " << attachment_fresh << " / "
             << object_attached << " / "
             << (attachment_fresh ? 1e3 * attachment_age_s : -1.0) << "\n"
             << "static scene: " << static_scene_enabled_;
        if (physical_obstacle_.has_value()) {
            text << " obstacle=[" << obstacle_centre_.x() << ' ' << obstacle_centre_.y() << ' '
                 << obstacle_centre_.z() << "] half=[" << obstacle_half_extents_.x() << ' '
                 << obstacle_half_extents_.y() << ' ' << obstacle_half_extents_.z()
                 << "] yaw_rad=" << obstacle_yaw_rad_;
        }
        text << "\ncooperative scene/fresh/required: " << cooperative_scene_enabled_
             << " / " << cooperative_fresh_count << " / "
             << cooperative_state_samples_.size()
             << " max_age_ms=" << (cooperative_scene_enabled_ ? 1e3 * cooperative_max_age_s : 0.0)
             << " refreshes=" << cooperative_world_refresh_count_
             << " stale_disables=" << stale_cooperative_replan_skips_;
        text << "\ntarget_received: " << latest_target_.has_value();
        if (latest_target_.has_value()) {
            text << " target=[" << latest_target_->x() << ' ' << latest_target_->y()
                 << ' ' << latest_target_->z() << ']';
        }
        if (latest_target_velocity_.has_value()) {
            text << " velocity=[" << latest_target_velocity_->x() << ' ' << latest_target_velocity_->y()
                 << ' ' << latest_target_velocity_->z() << ']';
        }
        if (active_target_.has_value()) {
            text << " planning_goal=[" << active_target_->x() << ' ' << active_target_->y()
                 << ' ' << active_target_->z() << "] lead_s=" << last_target_lead_s_
                 << " lead_distance_m=" << last_target_lead_distance_m_
                 << " (travel=" << last_target_travel_lead_s_
                 << " solve=" << last_target_solve_lead_s_
                 << " rendezvous_unclamped=" << last_target_rendezvous_time_unclamped_s_
                 << " solve_ema=" << estimated_solve_runtime_s_ << ')';
            if (active_target_velocity_.has_value()) {
                text << " terminal_v=[" << active_target_velocity_->x() << ' '
                     << active_target_velocity_->y() << ' ' << active_target_velocity_->z() << ']';
            }
        }
        text << "\ntarget predictor/type/valid/horizon_s/pred_v: "
             << target_predictor_type_ << " / " << last_target_prediction_valid_ << " / "
             << last_target_prediction_horizon_s_ << " / ["
             << last_target_prediction_velocity_.x() << ' '
             << last_target_prediction_velocity_.y() << ' '
             << last_target_prediction_velocity_.z() << ']';
        text << "\nmoving basket enabled/fresh/mode/updates/rejections: "
             << moving_basket_scene_enabled_ << " / " << movingBasketTrajectoryFresh(now_s)
             << " / " << moving_basket_collision_mode_
             << " / " << moving_basket_commitment_updates_ << " / "
             << moving_basket_commitment_rejections_;
        text << "\nexecution tracking enabled/ref_valid/error_xyz/lag_s: "
             << execution_tracking_safety_enabled_ << " / "
             << execution_tracking_reference_valid_ << " / ["
             << execution_position_error_.x() << ' ' << execution_position_error_.y() << ' '
             << execution_position_error_.z() << "] / " << execution_time_lag_s_
             << " bounds=[" << ego_tracking_error_half_extents_.x() << ' '
             << ego_tracking_error_half_extents_.y() << ' '
             << ego_tracking_error_half_extents_.z() << "]"
             << " fallback=" << execution_fallback_active_
             << " certified=" << execution_fallback_certified_
             << " mode='" << execution_fallback_mode_ << "'"
             << " reason='" << execution_fallback_reason_ << "'"
             << " count/recovered=" << execution_fallback_count_ << "/"
             << execution_fallback_recovery_count_;
        text << "\nshadow_reference: " << shadow_reference_topic_
             << " published=" << reference_publish_count_
             << " last_period_ms=" << last_reference_period_ms_
             << " callback_ms(last/max/overruns)=" << last_reference_callback_ms_ << "/"
             << max_reference_callback_ms_ << "/" << reference_callback_overrun_count_
             << " result_processing_ms(last/max)=" << last_planner_result_processing_ms_ << "/"
             << max_planner_result_processing_ms_
             << " incumbent_rechecks(run/skipped_same_world)="
             << backend_incumbent_recheck_count_ << "/"
             << backend_incumbent_recheck_skipped_same_world_count_ << "\n"
             << "latest_status: " << latest_status_ << "\n"
             << "reference errors/terminal-hold recoveries: " << reference_errors_
             << " / " << terminal_hold_reference_recoveries_
             << " last_error='" << last_reference_error_ << "'\n"
             << "replans accepted/failed/late: " << accepted_candidates_ << " / "
             << failed_candidates_ << " / " << late_candidates_ << "\n"
             << "live-target updates enabled/rebuilds: " << allow_live_target_updates_
             << " / " << live_target_rebuild_count_ << "\n"
             << "cooperative prediction/shared fresh: " << cooperative_prediction_mode_
             << " / " << cooperative_shared_fresh_count << "/"
             << cooperative_state_samples_.size() << " min_future_s="
             << (std::isfinite(cooperative_min_future_coverage_s)
                    ? cooperative_min_future_coverage_s : -1.0)
             << " max_track_ratio=" << cooperative_max_tracking_ratio << "\n"
             << "terminal-hold guard s: " << terminal_hold_guard_s_ << "\n"
             << "coalesced/stale-replan/stale-reference/worker-errors: "
             << coalesced_replan_triggers_ << " / " << stale_state_replan_skips_ << " / "
             << stale_state_reference_skips_ << " / " << planner_worker_errors_ << "\n"
             << "prepared/fresh_pending/authority_granted/fresh_commits/rebases: "
             << prepared_candidate_ready_ << " / " << authority_fresh_plan_pending_ << " / "
             << authority_granted_ << " / " << authority_fresh_commit_count_ << " / "
             << authority_rebase_count_ << "\n"
             << "scene_witness: " << scene_witness_summary_ << "\n"
             << "MPC authority publisher unchanged: true";

        std_msgs::msg::String message;
        message.data = text.str();
        diagnostics_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "%s", message.data.c_str());

        if (csv_) {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            const double commit_start_s = shadow_commit_.has_value()
                ? shadow_commit_->startTime() : nan;
            const double commit_end_s = shadow_commit_.has_value()
                ? shadow_commit_->endTime() : nan;
            csv_ << std::setprecision(15)
                 << now_s << ',' << enable_requested_ << ',' << enabled_ << ','
                 << state_fresh << ',' << (state_fresh ? 1e3 * age_s : -1.0) << ','
                 << latest_target_.has_value() << ','
                 << (latest_target_.has_value() ? latest_target_->x() : nan) << ','
                 << (latest_target_.has_value() ? latest_target_->y() : nan) << ','
                 << (latest_target_.has_value() ? latest_target_->z() : nan) << ','
                 << latest_target_velocity_.has_value() << ','
                 << (latest_target_velocity_.has_value() ? latest_target_velocity_->x() : nan) << ','
                 << (latest_target_velocity_.has_value() ? latest_target_velocity_->y() : nan) << ','
                 << (latest_target_velocity_.has_value() ? latest_target_velocity_->z() : nan) << ','
                 << (active_target_.has_value() ? active_target_->x() : nan) << ','
                 << (active_target_.has_value() ? active_target_->y() : nan) << ','
                 << (active_target_.has_value() ? active_target_->z() : nan) << ','
                 << last_target_lead_s_ << ',' << last_target_lead_distance_m_ << ','
                 << last_target_travel_lead_s_ << ','
                 << last_target_solve_lead_s_ << ','
                 << last_target_rendezvous_time_unclamped_s_ << ','
                 << estimated_solve_runtime_s_ << ','
                 << arrival_time_target_lead_enabled_ << ',' << moving_rendezvous_enabled_ << ','
                 << (active_target_velocity_.has_value() ? active_target_velocity_->x() : nan) << ','
                 << (active_target_velocity_.has_value() ? active_target_velocity_->y() : nan) << ','
                 << (active_target_velocity_.has_value() ? active_target_velocity_->z() : nan) << ','
                 << shadow_commit_.has_value() << ','
                 << commit_start_s << ',' << commit_end_s << ','
                 << prepared_candidate_ready_ << ',' << authority_fresh_plan_pending_ << ','
                 << authority_granted_ << ',' << authority_fresh_commit_count_ << ','
                 << authority_rebase_count_ << ',' << last_authority_grant_ros_s_ << ','
                 << safeCsvField(latest_status_) << ',' << reference_publish_count_ << ','
                 << reference_errors_ << ',' << terminal_hold_reference_recoveries_ << ','
                 << last_reference_period_ms_ << ',' << last_reference_callback_ms_ << ','
                 << max_reference_callback_ms_ << ',' << reference_callback_overrun_count_ << ','
                 << last_planner_result_processing_ms_ << ',' << max_planner_result_processing_ms_ << ','
                 << backend_incumbent_recheck_count_ << ','
                 << backend_incumbent_recheck_skipped_same_world_count_ << ','
                 << last_reference_sample_ros_s_ << ','
                 << last_reference_error_ros_s_ << ',' << safeCsvField(last_reference_error_) << ','
                 << accepted_candidates_ << ',' << failed_candidates_ << ',' << late_candidates_ << ','
                 << coalesced_replan_triggers_ << ',' << stale_state_replan_skips_ << ','
                 << stale_state_reference_skips_ << ',' << planner_worker_errors_ << ','
                 << payload_collision_enabled_ << ',' << attachment_fresh << ','
                 << object_attached << ',' << (attachment_fresh ? 1e3 * attachment_age_s : -1.0) << ','
                 << suspended_geometry_.payload_attached << ',' << static_scene_enabled_ << ','
                 << (physical_obstacle_.has_value() ? obstacle_centre_.x() : nan) << ','
                 << (physical_obstacle_.has_value() ? obstacle_centre_.y() : nan) << ','
                 << (physical_obstacle_.has_value() ? obstacle_centre_.z() : nan) << ','
                 << (physical_obstacle_.has_value() ? obstacle_half_extents_.x() : nan) << ','
                 << (physical_obstacle_.has_value() ? obstacle_half_extents_.y() : nan) << ','
                 << (physical_obstacle_.has_value() ? obstacle_half_extents_.z() : nan) << ','
                 << scene_witness_passed_ << ',' << safeCsvField(scene_witness_summary_) << ','
                 << cooperative_scene_enabled_ << ',' << cooperative_state_samples_.size() << ','
                 << cooperative_fresh_count << ','
                 << (cooperative_scene_enabled_ ? 1e3 * cooperative_max_age_s : 0.0) << ','
                 << safeCsvField(cooperative_prediction_mode_) << ','
                 << cooperative_shared_fresh_count << ','
                 << (std::isfinite(cooperative_min_future_coverage_s)
                        ? cooperative_min_future_coverage_s : nan) << ','
                 << cooperative_max_tracking_ratio << ','
                 << cooperative_shared_commitment_updates_ << ','
                 << cooperative_shared_commitment_rejections_ << ','
                 << cooperative_world_refresh_count_ << ',' << stale_cooperative_replan_skips_ << ',';

            auto append_cooperative_sample = [&](std::size_t index) {
                if (index < cooperative_state_samples_.size() &&
                    cooperative_state_samples_[index].received) {
                    const auto& sample = cooperative_state_samples_[index];
                    const double sample_age_s = now_s - sample.receive_ros_s;
                    csv_ << sample.position.x() << ',' << sample.position.y() << ',' << sample.position.z() << ','
                         << sample.velocity.x() << ',' << sample.velocity.y() << ',' << sample.velocity.z() << ','
                         << (std::isfinite(sample_age_s) ? 1e3 * std::max(0.0, sample_age_s) : nan) << ',';
                } else {
                    csv_ << nan << ',' << nan << ',' << nan << ','
                         << nan << ',' << nan << ',' << nan << ',' << nan << ',';
                }
            };
            append_cooperative_sample(0U);
            append_cooperative_sample(1U);
            const double execution_error_norm = execution_tracking_reference_valid_
                ? execution_position_error_.norm() : nan;
            const double execution_velocity_error_norm = execution_tracking_reference_valid_
                ? execution_velocity_error_.norm() : nan;
            csv_ << execution_tracking_safety_enabled_ << ','
                 << ego_tracking_error_half_extents_.x() << ','
                 << ego_tracking_error_half_extents_.y() << ','
                 << ego_tracking_error_half_extents_.z() << ','
                 << execution_tracking_reference_valid_ << ','
                 << (execution_tracking_reference_valid_ ? execution_position_error_.x() : nan) << ','
                 << (execution_tracking_reference_valid_ ? execution_position_error_.y() : nan) << ','
                 << (execution_tracking_reference_valid_ ? execution_position_error_.z() : nan) << ','
                 << execution_error_norm << ',' << execution_velocity_error_norm << ','
                 << execution_time_lag_s_ << ',' << execution_tracking_consecutive_violations_ << ','
                 << execution_fallback_active_ << ',' << execution_fallback_certified_ << ','
                 << execution_fallback_count_ << ',' << execution_fallback_recovery_count_ << ','
                 << safeCsvField(execution_fallback_mode_) << ','
                 << safeCsvField(execution_fallback_reason_) << ','
                 << execution_fallback_restart_blocked_ << ','
                 << execution_fallback_restart_z_m_ << ','
                 << world_->currentVersion() << '\n';
            csv_.flush();
        }
    }

    KinematicsFilter kinematics_;
    dynamic_planner::ReferenceWindowConfig reference_config_;

    std::string state_topic_;
    std::string target_topic_;
    std::string target_velocity_topic_;
    std::string object_attached_topic_;
    std::string enable_topic_;
    std::string authority_topic_;
    std::string authority_ack_topic_;
    std::string shadow_reference_topic_;
    std::string diagnostics_topic_;
    std::string marker_topic_;
    std::string frame_id_;
    std::string vehicle_id_;
    std::string csv_path_;
    std::string replan_csv_path_;
    std::string obstacle_name_;
    std::vector<std::string> cooperative_drone_state_topics_;
    std::string cooperative_prediction_mode_ = "constant_velocity";
    std::vector<std::string> cooperative_committed_trajectory_topics_;
    std::vector<std::string> cooperative_vehicle_ids_;
    std::vector<std::string> cooperative_attached_plate_topics_;
    std::string cooperative_ring_pose_topic_ = "/model/payload_model/pose";

    double state_timeout_s_ = 0.25;
    double object_attached_timeout_s_ = 0.25;
    double planner_rate_hz_ = 5.0;
    double marker_rate_hz_ = 5.0;
    double planner_period_s_ = 0.20;
    double minimum_search_z_m_ = 0.20;
    double initial_splice_timing_s_ = 0.16;
    double fixed_splice_lookahead_s_ = 0.0;
    double octopus_max_runtime_s_ = 2.0;
    double octopus_pre_authority_max_runtime_s_ = 2.0;
    bool reuse_prepared_commit_on_authority_ = false;
    double spline_time_factor_ = 3.0;
    double factor_alloc_ = 1.0;
    double factor_alloc_close_ = 2.5;
    double close_to_goal_m_ = 0.20;
    double smoother_jerk_weight_ = 1.0;
    double smoother_goal_weight_ = 10.0;
    double incumbent_failure_recheck_horizon_s_ = 2.0;
    double terminal_hold_guard_s_ = 0.0;
    double terminal_hold_guard_interval_s_ = 0.50;
    std::string planning_policy_profile_ = "conservative";
    bool require_terminal_hold_for_nominal_ = true;
    bool require_rendezvous_backup_for_nominal_ = true;
    bool defer_incumbent_conflict_until_reaction_horizon_ = false;
    double emergency_panic_horizon_s_ = 0.50;
    bool enable_octopus_useful_deadline_ = true;
    double octopus_post_search_reserve_s_ = 1.0 / kControlReferenceHz;
    bool require_stationary_activation_ = true;
    double activation_speed_tolerance_mps_ = 0.10;
    double activation_accel_tolerance_mps2_ = 0.50;
    double fixed_target_change_tolerance_m_ = 1e-4;
    bool allow_live_target_updates_ = false;
    double live_target_replan_threshold_m_ = 0.01;
    bool require_target_velocity_ = false;
    bool target_lead_enabled_ = false;
    bool arrival_time_target_lead_enabled_ = false;
    int target_lead_fixed_point_iterations_ = 3;
    bool terminal_time_target_prediction_enabled_ = false;
    int target_time_fixed_point_iterations_ = 6;
    double target_time_fixed_point_tolerance_s_ = 0.01;
    bool moving_rendezvous_enabled_ = false;
    double live_target_velocity_replan_threshold_mps_ = 0.01;
    double target_lead_nominal_speed_mps_ = 0.35;
    double target_lead_min_s_ = 0.0;
    double target_lead_max_s_ = 2.0;
    double target_lead_high_confidence_max_s_ = 10.0;
    double target_lead_max_distance_m_ = 0.0;
    std::string target_predictor_type_ = "constant_velocity";
    Vec3 scripted_target_center_ = Vec3::Zero();
    double scripted_target_radius_m_ = 0.5;
    double scripted_target_omega_rad_s_ = 0.25;
    double target_lead_initial_solve_runtime_s_ = 0.50;
    double target_lead_solve_runtime_scale_ = 1.0;
    double target_lead_solve_runtime_ema_alpha_ = 0.35;
    double estimated_solve_runtime_s_ = 0.50;
    bool seed_stationary_on_activation_ = true;
    double max_swing_angle_rad_ = 15.0 * kPi / 180.0;
    bool payload_collision_enabled_ = false;
    bool require_payload_attached_for_enable_ = false;
    Vec3 payload_half_extents_ = Vec3(0.05, 0.01, 0.003);
    Vec3 payload_center_from_magnet_center_ = Vec3(0.0, 0.0, -0.028);
    bool static_scene_enabled_ = true;
    bool require_scene_witness_ = true;
    double obstacle_yaw_rad_ = 20.0 * kPi / 180.0;
    Vec3 body_half_extents_ = Vec3(0.105, 0.105, 0.060);
    Vec3 obstacle_centre_ = Vec3(0.50, 0.06, 0.68);
    Vec3 obstacle_half_extents_ = Vec3(0.12, 0.25, 0.10);
    bool cooperative_scene_enabled_ = false;
    double cooperative_state_timeout_s_ = 0.30;
    double cooperative_prediction_horizon_s_ = 20.0;
    Vec3 cooperative_physical_half_extents_ = Vec3(0.105, 0.105, 0.060);
    Vec3 cooperative_tracking_error_half_extents_ = Vec3::Constant(0.10);
    bool cooperative_attached_tether_enabled_ = false;
    int cooperative_ring_pose_index_ = 1;
    double cooperative_ring_pose_timeout_s_ = 0.50;
    int cooperative_ring_plate_count_ = 12;
    double cooperative_ring_plate_pitch_diameter_m_ = 0.50;
    double cooperative_ring_angle_zero_rad_ = 0.0;
    double cooperative_attached_tether_radius_m_ = 0.015;
    Vec3 cooperative_tether_anchor_from_body_ = Vec3(0.0, 0.0, -0.04);
    bool moving_basket_scene_enabled_ = false;
    std::string moving_basket_committed_trajectory_topic_ = "/fake_payload/committed_trajectory";
    std::string moving_basket_collision_mode_ = "solid_box";
    MovingBasketGeometry moving_basket_geometry_;
    Vec3 moving_basket_physical_half_extents_ = moving_basket_geometry_.halfExtents();
    Vec3 moving_basket_tracking_error_half_extents_ = Vec3::Constant(0.05);
    SegmentedRingGeometry segmented_ring_geometry_;
    bool execution_tracking_safety_enabled_ = false;
    Vec3 ego_tracking_error_half_extents_ = Vec3::Zero();
    int execution_tracking_violation_samples_ = 3;
    double execution_velocity_error_limit_mps_ = 0.0;
    Vec3 execution_brake_accel_limit_ = Vec3(1.0, 1.0, 1.5);
    double execution_brake_min_duration_s_ = 0.25;
    SuspendedGeometry suspended_geometry_;
    std::optional<dynamic_planner::StaticConvexObstacle> physical_obstacle_;

    rclcpp::Subscription<interfaces::msg::MotionCaptureState>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr object_attached_sub_;
    std::vector<rclcpp::Subscription<interfaces::msg::MotionCaptureState>::SharedPtr>
        cooperative_state_subs_;
    std::vector<rclcpp::Subscription<interfaces::msg::CommittedTrajectory>::SharedPtr>
        cooperative_trajectory_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr>
        cooperative_attached_plate_subs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cooperative_ring_pose_sub_;
    rclcpp::Subscription<interfaces::msg::CommittedTrajectory>::SharedPtr
        moving_basket_trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr authority_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr shadow_reference_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr authority_ack_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr reference_timer_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::TimerBase::SharedPtr marker_timer_;

    bool enable_requested_ = false;
    bool enabled_ = false;
    bool prepared_candidate_ready_ = false;
    bool authority_granted_ = false;
    bool authority_fresh_plan_pending_ = false;
    std::size_t authority_rebase_count_ = 0U;
    std::size_t authority_fresh_commit_count_ = 0U;
    double last_authority_grant_ros_s_ = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t epoch_ = 0U;
    std::uint64_t replan_sequence_ = 0U;
    std::optional<State> latest_state_;
    std::optional<double> last_state_receive_ros_s_;
    std::optional<bool> latest_object_attached_;
    std::optional<double> last_object_attached_receive_ros_s_;
    std::vector<CooperativeStateSample> cooperative_state_samples_;
    std::vector<CooperativeTrajectorySample> cooperative_trajectory_samples_;
    std::vector<CooperativeAttachedPlateSample> cooperative_attached_plate_samples_;
    RingPoseSample cooperative_ring_pose_sample_;
    CooperativeTrajectorySample moving_basket_trajectory_sample_;
    std::optional<std::uint64_t> moving_basket_collision_sequence_;
    std::optional<Vec3> latest_target_;
    std::optional<Vec3> latest_target_velocity_;
    std::optional<Vec3> active_source_target_;
    std::optional<Vec3> active_target_;
    std::optional<Vec3> active_target_velocity_;
    std::optional<CommittedTrajectory> shadow_commit_;
    std::vector<Vec3> latest_reference_positions_;
    std::shared_ptr<VersionedWorld> world_;
    std::unique_ptr<RecedingHorizonPlanner> planner_;
    std::future<PlannerOutcome> planner_future_;
    ReplanResult latest_replan_result_;

    std::string latest_status_ = "DISABLED_WAITING_FOR_TARGET";
    bool scene_witness_passed_ = false;
    std::string scene_witness_summary_ = "PENDING";
    std::optional<double> last_reference_ros_s_;
    double last_reference_period_ms_ = 0.0;
    double last_reference_callback_ms_ = 0.0;
    double max_reference_callback_ms_ = 0.0;
    std::size_t reference_callback_overrun_count_ = 0U;
    double last_planner_result_processing_ms_ = 0.0;
    double max_planner_result_processing_ms_ = 0.0;
    std::size_t backend_incumbent_recheck_count_ = 0U;
    std::size_t backend_incumbent_recheck_skipped_same_world_count_ = 0U;
    double last_reference_sample_ros_s_ = std::numeric_limits<double>::quiet_NaN();
    double last_reference_error_ros_s_ = std::numeric_limits<double>::quiet_NaN();
    std::string last_reference_error_;
    double last_target_lead_s_ = 0.0;
    double last_target_lead_distance_m_ = 0.0;
    double last_target_rendezvous_time_unclamped_s_ = 0.0;
    double last_target_travel_lead_s_ = 0.0;
    double last_target_solve_lead_s_ = 0.0;
    bool last_target_prediction_valid_ = false;
    double last_target_prediction_horizon_s_ = 0.0;
    Vec3 last_target_prediction_velocity_ = Vec3::Zero();

    std::size_t reference_publish_count_ = 0U;
    std::size_t reference_errors_ = 0U;
    std::size_t terminal_hold_reference_recoveries_ = 0U;
    std::size_t accepted_candidates_ = 0U;
    std::size_t failed_candidates_ = 0U;
    std::size_t late_candidates_ = 0U;
    std::size_t coalesced_replan_triggers_ = 0U;
    std::size_t stale_state_replan_skips_ = 0U;
    std::size_t stale_state_reference_skips_ = 0U;
    std::size_t stale_cooperative_replan_skips_ = 0U;
    std::size_t cooperative_world_refresh_count_ = 0U;
    std::size_t cooperative_shared_commitment_updates_ = 0U;
    std::size_t cooperative_shared_commitment_rejections_ = 0U;
    std::size_t moving_basket_commitment_updates_ = 0U;
    std::size_t moving_basket_commitment_rejections_ = 0U;
    std::size_t planner_worker_errors_ = 0U;
    std::size_t live_target_rebuild_count_ = 0U;

    bool execution_tracking_reference_valid_ = false;
    Vec3 execution_position_error_ = Vec3::Zero();
    Vec3 execution_velocity_error_ = Vec3::Zero();
    double execution_time_lag_s_ = std::numeric_limits<double>::quiet_NaN();
    int execution_tracking_consecutive_violations_ = 0;
    bool execution_fallback_active_ = false;
    bool execution_fallback_certified_ = true;
    std::string execution_fallback_reason_;
    std::string execution_fallback_mode_;
    std::string execution_fallback_pending_reason_;
    bool execution_fallback_restart_blocked_ = false;
    double execution_fallback_restart_z_m_ = std::numeric_limits<double>::quiet_NaN();
    std::size_t execution_fallback_count_ = 0U;
    std::size_t execution_fallback_recovery_count_ = 0U;

    std::ofstream csv_;
    std::ofstream replan_csv_;
};

}  // namespace tejen_dynamic_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<tejen_dynamic_planner::TransferBackendNode>();
        rclcpp::spin(node);
    } catch (const std::exception& exc) {
        std::cerr << "dynamic_planner_transfer_backend: FAIL: " << exc.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
