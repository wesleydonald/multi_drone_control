#include "tejen_dynamic_planner/c1e_scene.hpp"
#include "tejen_dynamic_planner/commissioning_reference.hpp"
#include "tejen_dynamic_planner/kinematics_filter.hpp"
#include "tejen_dynamic_planner/reference_message.hpp"

#include "dynamic_planner/ego_collision_model.hpp"
#include "dynamic_planner/receding_horizon_planner.hpp"
#include "dynamic_planner/reference_window.hpp"
#include "dynamic_planner/world_snapshot.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <interfaces/msg/motion_capture_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <deque>
#include <future>
#include <fstream>
#include <functional>
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

namespace tejen_dynamic_planner {
namespace {

using dynamic_planner::CommittedTrajectory;
using dynamic_planner::LocalPlannerConfig;
using dynamic_planner::RecedingHorizonConfig;
using dynamic_planner::RecedingHorizonPlanner;
using dynamic_planner::ReplanResult;
using dynamic_planner::State;
using dynamic_planner::SuspendedGeometry;
using dynamic_planner::TrajectoryTimeSource;
using dynamic_planner::Vec3;
using dynamic_planner::VersionedWorld;
using dynamic_planner::WorldSnapshot;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kControlReferenceHz = 30.0;
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kAuthorityTopic = "/join_planner/reference";

struct StatsSummary {
    double mean = 0.0;
    double p95 = 0.0;
    double maximum = 0.0;
    std::size_t count = 0U;
};

class RollingStats {
public:
    explicit RollingStats(std::size_t capacity = 10000U) : capacity_(capacity) {}

    void add(double value) {
        if (!std::isfinite(value)) return;
        values_.push_back(value);
        if (values_.size() > capacity_) values_.pop_front();
    }

    void clear() { values_.clear(); }

    StatsSummary summary() const {
        StatsSummary out;
        if (values_.empty()) return out;
        std::vector<double> sorted(values_.begin(), values_.end());
        std::sort(sorted.begin(), sorted.end());
        double total = 0.0;
        for (double value : sorted) total += value;
        out.count = sorted.size();
        out.mean = total / static_cast<double>(sorted.size());
        const double index = 0.95 * static_cast<double>(sorted.size() - 1U);
        const auto lo = static_cast<std::size_t>(std::floor(index));
        const auto hi = static_cast<std::size_t>(std::ceil(index));
        const double frac = index - static_cast<double>(lo);
        out.p95 = sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
        out.maximum = sorted.back();
        return out;
    }

private:
    std::size_t capacity_;
    std::deque<double> values_;
};

SuspendedGeometry suspendedGeometry(double max_swing_angle_rad) {
    SuspendedGeometry g;
    g.enabled = true;
    g.cable_length_m = 0.50;
    g.cable_radius_m = 0.0025;
    g.max_swing_angle_rad = max_swing_angle_rad;
    g.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    g.magnet_center_below_cable_end_m = 0.025;
    g.payload_attached = false;
    g.payload_half_extents = Vec3(0.05, 0.01, 0.003);
    g.payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.028);
    return g;
}

geometry_msgs::msg::Point markerPoint(const Vec3& point) {
    geometry_msgs::msg::Point result;
    result.x = point.x();
    result.y = point.y();
    result.z = point.z();
    return result;
}

std_msgs::msg::ColorRGBA markerColour(float r, float g, float b, float a = 1.0F) {
    std_msgs::msg::ColorRGBA colour;
    colour.r = r;
    colour.g = g;
    colour.b = b;
    colour.a = a;
    return colour;
}

Marker baseMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
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
    const rclcpp::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const std::vector<Vec3>& points,
    const std_msgs::msg::ColorRGBA& colour,
    double width_m) {
    Marker marker = baseMarker(
        frame_id, stamp, marker_namespace, id, Marker::LINE_STRIP);
    marker.scale.x = width_m;
    marker.color = colour;
    marker.points.reserve(points.size());
    for (const auto& point : points) marker.points.push_back(markerPoint(point));
    return marker;
}

Marker pointsMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const Eigen::MatrixXd& vertices,
    const std_msgs::msg::ColorRGBA& colour,
    double scale_m) {
    Marker marker = baseMarker(frame_id, stamp, marker_namespace, id, Marker::POINTS);
    marker.scale.x = scale_m;
    marker.scale.y = scale_m;
    marker.color = colour;
    marker.points.reserve(static_cast<std::size_t>(vertices.rows()));
    for (Eigen::Index row = 0; row < vertices.rows(); ++row) {
        marker.points.push_back(markerPoint(vertices.row(row).transpose()));
    }
    return marker;
}

Marker sphereMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& marker_namespace,
    int id,
    const Vec3& centre,
    const std_msgs::msg::ColorRGBA& colour,
    double diameter_m) {
    Marker marker = baseMarker(frame_id, stamp, marker_namespace, id, Marker::SPHERE);
    marker.pose.position = markerPoint(centre);
    marker.scale.x = diameter_m;
    marker.scale.y = diameter_m;
    marker.scale.z = diameter_m;
    marker.color = colour;
    return marker;
}

RecedingHorizonConfig recedingConfig(
    double initial_timing_seed_s, double spline_time_factor,
    bool enable_octopus_useful_deadline, double octopus_post_search_reserve_s) {
    RecedingHorizonConfig c;
    c.dc_s = 1.0 / kControlReferenceHz;
    c.planning_radius_m = 2.0;
    c.factor_alpha = 2.5;
    c.min_splice_lookahead_s = 0.05;
    c.max_splice_lookahead_s = 1.0;
    c.factor_alloc = 1.0;
    c.factor_alloc_close = 2.5;
    c.spline_time_factor = spline_time_factor;
    c.close_to_goal_m = 0.20;
    c.goal_tolerance_m = 0.05;
    c.continuity_tolerance = 1e-7;
    c.separator_validation_tolerance = 1e-7;
    c.initial_splice_timing_s = std::max(0.0, initial_timing_seed_s);
    c.enable_octopus_useful_deadline = enable_octopus_useful_deadline;
    c.octopus_post_search_reserve_s = octopus_post_search_reserve_s;
    c.v_max = Vec3::Ones();
    c.a_max = Vec3(1.0, 1.0, 1.5);
    return c;
}

LocalPlannerConfig localConfig(const Vec3& start, const Vec3& goal, double minimum_search_z_m) {
    LocalPlannerConfig c;
    c.num_segments = 4;
    c.octopus.samples_per_axis = {9, 9, 9};
    c.octopus.alpha_shrink = 0.9;
    c.octopus.voxel_fraction = 0.10;
    c.octopus.heuristic_bias = 1.0;
    c.octopus.max_runtime_s = 2.0;
    const Vec3 padding(1.0, 1.0, 1.0);
    c.octopus.xyz_min = start.cwiseMin(goal) - padding;
    c.octopus.xyz_max = start.cwiseMax(goal) + padding;
    c.octopus.xyz_min.z() = std::max(minimum_search_z_m, c.octopus.xyz_min.z());
    if (c.octopus.xyz_max.z() <= c.octopus.xyz_min.z() + 0.2) {
        c.octopus.xyz_max.z() = c.octopus.xyz_min.z() + 0.2;
    }
    c.octopus.random_seed = 1;
    c.refinement.xyz_min = c.octopus.xyz_min;
    c.refinement.xyz_max = c.octopus.xyz_max;
    c.refinement.j_max = Vec3::Constant(4.0);
    c.refinement.max_working_set_recalculations = 500;
    return c;
}

std::string safeCsvField(std::string text) {
    std::replace(text.begin(), text.end(), ',', ';');
    std::replace(text.begin(), text.end(), '\n', ' ');
    return text;
}

double optionalOrNan(const std::optional<double>& value) {
    return value.has_value() ? *value : std::numeric_limits<double>::quiet_NaN();
}

enum class CommissioningPhase {
    Ground,
    Ascent,
    HoverSettle,
    ActivePlanning,
    FaultHold,
};

const char* phaseName(CommissioningPhase phase) {
    switch (phase) {
        case CommissioningPhase::Ground: return "GROUND";
        case CommissioningPhase::Ascent: return "ASCENT_C3";
        case CommissioningPhase::HoverSettle: return "HOVER_SETTLE";
        case CommissioningPhase::ActivePlanning: return "ACTIVE_PLANNING";
        case CommissioningPhase::FaultHold: return "FAULT_HOLD";
    }
    return "UNKNOWN";
}

struct PlannerOutcome {
    std::uint64_t sequence = 0U;
    std::uint64_t epoch = 0U;
    double request_ros_time_s = 0.0;
    State request_state;
    std::unique_ptr<RecedingHorizonPlanner> planner;
    ReplanResult result;
    bool worker_exception = false;
    std::string worker_error;
};

}  // namespace

class ActiveCommissioningNode final : public rclcpp::Node {
public:
    ActiveCommissioningNode()
        : Node("dynamic_planner_active_commissioning"),
          kinematics_(declare_parameter<double>("acceleration_filter_tau_s", 0.15)) {
        state_topic_ = declare_parameter<std::string>("state_topic", "/motion_capture_state");
        pendulum_topic_ = declare_parameter<std::string>("pendulum_topic", "/pendulum_swing_state");
        magnet_tip_topic_ = declare_parameter<std::string>("magnet_tip_topic", "/magnet_tip_pose");
        command_topic_ = declare_parameter<std::string>("command_topic", "/drone_command");
        reference_topic_ = declare_parameter<std::string>("reference_topic", kAuthorityTopic);
        diagnostics_topic_ = declare_parameter<std::string>(
            "diagnostics_topic", "/dynamic_planner/c1d_status");
        marker_topic_ = declare_parameter<std::string>(
            "marker_topic", "/dynamic_planner/markers");
        csv_path_ = declare_parameter<std::string>(
            "csv_path", "/tmp/r6_3c1d_active.csv");
        trace_csv_path_ = declare_parameter<std::string>("trace_csv_path", "");
        replan_csv_path_ = declare_parameter<std::string>("replan_csv_path", "");
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        vehicle_id_ = declare_parameter<std::string>("vehicle_id", "drone_0");
        commissioning_label_ = declare_parameter<std::string>(
            "commissioning_label", "R6.3C.1d");

        state_timeout_s_ = declare_parameter<double>("state_timeout_s", 0.25);
        pendulum_timeout_s_ = declare_parameter<double>("pendulum_timeout_s", 0.25);
        planner_rate_hz_ = declare_parameter<double>("planner_rate_hz", 5.0);
        marker_rate_hz_ = declare_parameter<double>("marker_rate_hz", 5.0);
        minimum_search_z_m_ = declare_parameter<double>("minimum_search_z_m", 0.20);
        initial_splice_timing_s_ = declare_parameter<double>("initial_splice_timing_s", 0.16);
        spline_time_factor_ = declare_parameter<double>("spline_time_factor", 2.5);
        enable_octopus_useful_deadline_ = declare_parameter<bool>(
            "enable_octopus_useful_deadline", false);
        octopus_post_search_reserve_s_ = declare_parameter<double>(
            "octopus_post_search_reserve_s", 1.0 / kControlReferenceHz);

        hover_z_m_ = declare_parameter<double>("hover_z_m", 1.20);
        takeoff_duration_s_ = declare_parameter<double>("takeoff_duration_s", 4.0);
        settle_dwell_s_ = declare_parameter<double>("settle_dwell_s", 1.0);
        settle_position_tolerance_m_ = declare_parameter<double>(
            "settle_position_tolerance_m", 0.10);
        settle_speed_tolerance_mps_ = declare_parameter<double>(
            "settle_speed_tolerance_mps", 0.10);
        settle_accel_tolerance_mps2_ = declare_parameter<double>(
            "settle_accel_tolerance_mps2", 0.50);
        settle_swing_tolerance_rad_ = declare_parameter<double>(
            "settle_swing_tolerance_rad", 5.0 * kPi / 180.0);
        settle_swing_rate_tolerance_radps_ = declare_parameter<double>(
            "settle_swing_rate_tolerance_radps", 0.20);

        goal_dx_m_ = declare_parameter<double>("goal_dx_m", 1.0);
        goal_dy_m_ = declare_parameter<double>("goal_dy_m", 0.0);
        goal_dz_m_ = declare_parameter<double>("goal_dz_m", 0.0);

        max_swing_angle_rad_ = declare_parameter<double>(
            "max_swing_angle_rad", 10.0 * kPi / 180.0);
        swing_warning_rad_ = declare_parameter<double>(
            "swing_warning_rad", 10.0 * kPi / 180.0);
        swing_claim_invalid_rad_ = declare_parameter<double>(
            "swing_claim_invalid_rad", max_swing_angle_rad_);

        static_scene_enabled_ = declare_parameter<bool>("static_scene_enabled", false);
        require_scene_witness_ = declare_parameter<bool>("require_scene_witness", false);
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
        const auto measured_trace_capacity_parameter = declare_parameter<std::int64_t>(
            "measured_trace_capacity", 900);

        if (reference_topic_ != kAuthorityTopic) {
            throw std::invalid_argument(
                "C.1d commissioning must publish the inspected MPC authority topic /join_planner/reference");
        }
        if (!(state_timeout_s_ > 0.0) || !(pendulum_timeout_s_ > 0.0) ||
            !(planner_rate_hz_ > 0.0) || planner_rate_hz_ > kControlReferenceHz ||
            !(marker_rate_hz_ > 0.0) || marker_rate_hz_ > kControlReferenceHz ||
            !std::isfinite(minimum_search_z_m_) || !(initial_splice_timing_s_ >= 0.0) ||
            !std::isfinite(spline_time_factor_) || !(spline_time_factor_ > 0.0) ||
            !std::isfinite(octopus_post_search_reserve_s_) ||
            !(octopus_post_search_reserve_s_ >= 0.0) ||
            (enable_octopus_useful_deadline_ &&
             (!(octopus_post_search_reserve_s_ > 0.0) ||
              !(octopus_post_search_reserve_s_ < 0.05))) ||
            !std::isfinite(hover_z_m_) || !(takeoff_duration_s_ > 0.0) ||
            !(settle_dwell_s_ >= 0.0) || !(settle_position_tolerance_m_ >= 0.0) ||
            !(settle_speed_tolerance_mps_ >= 0.0) ||
            !(settle_accel_tolerance_mps2_ >= 0.0) ||
            !(settle_swing_tolerance_rad_ >= 0.0) ||
            !(settle_swing_rate_tolerance_radps_ >= 0.0) ||
            !std::isfinite(goal_dx_m_) || !std::isfinite(goal_dy_m_) ||
            !std::isfinite(goal_dz_m_) || !std::isfinite(max_swing_angle_rad_) ||
            !(max_swing_angle_rad_ > 0.0) || max_swing_angle_rad_ >= 0.5 * kPi ||
            !std::isfinite(swing_warning_rad_) || swing_warning_rad_ < 0.0 ||
            !std::isfinite(swing_claim_invalid_rad_) ||
            swing_claim_invalid_rad_ < swing_warning_rad_ ||
            swing_claim_invalid_rad_ > max_swing_angle_rad_ + 1e-12 ||
            measured_trace_capacity_parameter <= 0 ||
            commissioning_label_.empty() || frame_id_.empty() || vehicle_id_.empty()) {
            throw std::invalid_argument("invalid active commissioning parameters");
        }
        if (static_scene_enabled_ &&
            (obstacle_name_.empty() || !obstacle_centre_.allFinite() ||
             !obstacle_half_extents_.allFinite() ||
             (obstacle_half_extents_.array() <= 0.0).any() ||
             !std::isfinite(obstacle_yaw_rad_))) {
            throw std::invalid_argument("invalid C.1e static-scene parameters");
        }
        if (require_scene_witness_ && !static_scene_enabled_) {
            throw std::invalid_argument(
                "require_scene_witness needs static_scene_enabled");
        }
        measured_trace_capacity_ = static_cast<std::size_t>(
            measured_trace_capacity_parameter);
        planner_period_s_ = 1.0 / planner_rate_hz_;

        reference_config_.reference_rate_hz = kControlReferenceHz;
        reference_config_.mpc_horizon_stages = 20U;
        reference_config_.mpc_skip_steps = 3U;
        reference_config_.validate();
        if (reference_config_.requiredSampleCount() != 61U) {
            throw std::logic_error("C.1d expected the inspected 61-sample MPC contract");
        }

        suspended_geometry_ = suspendedGeometry(max_swing_angle_rad_);
        WorldSnapshot initial_world;
        initial_world.captured_at_s = 0.0;
        initial_world.ego_half_extents = body_half_extents_;
        initial_world.ego_suspended_geometry = suspended_geometry_;
        if (static_scene_enabled_) {
            physical_obstacle_ = makeYawedCuboidObstacle(
                obstacle_name_, obstacle_centre_, obstacle_half_extents_,
                obstacle_yaw_rad_);
            initial_world.physical_static_obstacles.push_back(*physical_obstacle_);
        }
        world_ = std::make_shared<VersionedWorld>(std::move(initial_world));

        state_sub_ = create_subscription<interfaces::msg::MotionCaptureState>(
            state_topic_, rclcpp::QoS(10),
            std::bind(&ActiveCommissioningNode::stateCallback, this, std::placeholders::_1));
        pendulum_sub_ = create_subscription<interfaces::msg::MotionCaptureState>(
            pendulum_topic_, rclcpp::QoS(10),
            std::bind(&ActiveCommissioningNode::pendulumCallback, this, std::placeholders::_1));
        magnet_tip_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            magnet_tip_topic_, rclcpp::QoS(10),
            std::bind(&ActiveCommissioningNode::magnetTipCallback, this, std::placeholders::_1));
        command_sub_ = create_subscription<std_msgs::msg::String>(
            command_topic_, rclcpp::QoS(10),
            std::bind(&ActiveCommissioningNode::commandCallback, this, std::placeholders::_1));
        reference_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            reference_topic_, rclcpp::QoS(10));
        diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
            diagnostics_topic_, rclcpp::QoS(10));
        marker_pub_ = create_publisher<MarkerArray>(
            marker_topic_, rclcpp::QoS(1).transient_local());

        reference_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(1.0 / kControlReferenceHz),
            std::bind(&ActiveCommissioningNode::referenceTimer, this));
        replan_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(planner_period_s_),
            std::bind(&ActiveCommissioningNode::replanTimer, this));
        diagnostics_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(1.0),
            std::bind(&ActiveCommissioningNode::diagnosticsTimer, this));
        marker_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(1.0 / marker_rate_hz_),
            [this]() { markerTimer(); });

        if (!csv_path_.empty()) {
            csv_.open(csv_path_);
            if (!csv_) {
                RCLCPP_WARN(get_logger(), "Could not open diagnostics CSV: %s", csv_path_.c_str());
            } else {
                csv_ << "ros_time,phase,reference_source,state_fresh,state_age_ms,speed_mps,"
                        "accel_mps2,pendulum_fresh,pendulum_age_ms,swing_deg,swing_rate_degps,"
                        "settle_dwell_s,reference_publish_count,reference_errors,reference_period_mean_ms,"
                        "reference_period_p95_ms,reference_period_max_ms,accepted,failed,late,coalesced,"
                        "stale_state_replan_skips,planner_worker_errors,planner_period_overruns,"
                        "runtime_mean_ms,runtime_p95_ms,runtime_max_ms,octopus_mean_ms,octopus_p95_ms,"
                        "octopus_max_ms,qp_mean_ms,qp_p95_ms,qp_max_ms,latest_status,fault_reason,"
                        "first_c2_verified,first_c2_margin_ms,scene_witness_passed,"
                        "airborne_swing_max_deg,active_swing_max_deg,active_swing_rate_max_degps,"
                        "swing_warning_exceeded,geometric_claim_invalid,world_version\n";
            }
        }

        if (!trace_csv_path_.empty()) {
            trace_csv_.open(trace_csv_path_);
            if (!trace_csv_) {
                RCLCPP_WARN(get_logger(), "Could not open 30 Hz trace CSV: %s",
                            trace_csv_path_.c_str());
            } else {
                trace_csv_ << "ros_time,phase,reference_source,state_fresh,ref_x,ref_y,ref_z,"
                              "ref_vx,ref_vy,ref_vz,ref_ax,ref_ay,ref_az,measured_x,measured_y,"
                              "measured_z,measured_vx,measured_vy,measured_vz,measured_ax,"
                              "measured_ay,measured_az,position_error_m,swing_deg,swing_rate_degps,"
                              "world_version\n";
            }
        }

        if (!replan_csv_path_.empty()) {
            replan_csv_.open(replan_csv_path_);
            if (!replan_csv_) {
                RCLCPP_WARN(get_logger(), "Could not open per-replan CSV: %s",
                            replan_csv_path_.c_str());
            } else {
                replan_csv_
                    << "sequence,epoch,discarded_epoch,request_ros_time_s,harvest_ros_time_s,"
                       "request_x,request_y,request_z,request_vx,request_vy,request_vz,"
                       "request_ax,request_ay,request_az,attempted,accepted,candidate_late,"
                       "goal_seen,goal_reached,status,splice_time_s,splice_lookahead_s,"
                       "octopus_useful_deadline_enabled,octopus_post_search_reserve_s,"
                       "octopus_useful_deadline_time_s,octopus_useful_budget_s,"
                       "octopus_deadline_clock_invalid,"
                       "splice_x,splice_y,splice_z,splice_vx,splice_vy,splice_vz,"
                       "splice_ax,splice_ay,splice_az,local_goal_x,local_goal_y,local_goal_z,"
                       "local_duration_s,minimum_time_s,time_allocation_factor,"
                       "global_distance_from_splice_m,committed_endpoint_distance_m,"
                       "replan_runtime_ms,trajectory_elapsed_ms,candidate_finish_time_s,"
                       "authority_decision_time_s,finish_to_splice_margin_ms,c2_accepted,"
                       "c2_position_error,c2_velocity_error,c2_acceleration_error,"
                       "planning_world_version,recheck_world_version,final_recheck_world_version,"
                       "recheck_retry_performed,incumbent_prefix_unsafe,prefix_safety_present,"
                       "prefix_safe,prefix_status,candidate_safety_present,candidate_safe,"
                       "candidate_safety_status,local_plan_present,local_status,local_message,"
                       "local_total_ms,octopus_status,octopus_success,octopus_reached_goal,"
                       "octopus_expanded_nodes,octopus_popped_nodes,octopus_separator_lp_calls,"
                       "octopus_search_ms,octopus_voxel_size_m,octopus_goal_distance_m,"
                       "octopus_termination_reason,octopus_complete_available_at_termination,"
                       "octopus_first_complete_ms,octopus_first_complete_goal_distance_m,"
                       "octopus_first_complete_expanded_nodes,octopus_first_complete_popped_nodes,"
                       "octopus_complete_improvements,"
                       "refinement_present,refinement_status,refinement_message,"
                       "qp_solve_ms,qp_working_set_recalculations,qp_raw_goal_distance_m,"
                       "qp_refined_goal_distance_m,qp_goal_accepted,qp_max_equality_residual,"
                       "qp_max_inequality_violation,qp_min_fixed_separator_slack,worker_exception,"
                       "worker_error\n";
                replan_csv_.flush();
            }
        }

        RCLCPP_WARN(get_logger(),
            "%s ACTIVE COMMISSIONING owns %s. Do not run online_join_planner or any other reference publisher.",
            commissioning_label_.c_str(), reference_topic_.c_str());
        RCLCPP_INFO(get_logger(),
            "%s phases: GROUND -> %.1f s seventh-order C3 ascent to z=%.2f -> settle -> persistent %.1f Hz planner -> [%.2f %.2f %.2f] m relative goal.",
            commissioning_label_.c_str(), takeoff_duration_s_, hover_z_m_,
            planner_rate_hz_, goal_dx_m_, goal_dy_m_, goal_dz_m_);
        if (static_scene_enabled_) {
            RCLCPP_WARN(get_logger(),
                "%s static scene: %s centre=[%.3f %.3f %.3f], half=[%.3f %.3f %.3f], yaw=%.1f deg, swing envelope=%.1f deg.",
                commissioning_label_.c_str(), obstacle_name_.c_str(),
                obstacle_centre_.x(), obstacle_centre_.y(), obstacle_centre_.z(),
                obstacle_half_extents_.x(), obstacle_half_extents_.y(),
                obstacle_half_extents_.z(), obstacle_yaw_rad_ * 180.0 / kPi,
                max_swing_angle_rad_ * 180.0 / kPi);
        }
        RCLCPP_WARN(get_logger(),
            "%s ground inspection: ARM ONLY and DO NOT TAKEOFF. Flight is only after ground-log review.",
            commissioning_label_.c_str());
    }

    ~ActiveCommissioningNode() override {
        if (planner_future_.valid()) {
            planner_future_.wait();
            try {
                PlannerOutcome outcome = planner_future_.get();
                writeReplanCsvRow(outcome, outcome.epoch != epoch_);
            } catch (...) {
                // Best-effort shutdown diagnostics only. Runtime worker exceptions
                // are already converted into PlannerOutcome by the async body.
            }
        }
        if (csv_) csv_.flush();
        if (trace_csv_) trace_csv_.flush();
        if (replan_csv_) replan_csv_.flush();
    }

private:
    bool updateSceneWitness(const Vec3& intended_hover) {
        if (!require_scene_witness_) {
            scene_witness_passed_ = true;
            scene_witness_summary_ = "NOT_REQUIRED";
            scene_witness_blocking_component_.clear();
            return true;
        }
        if (!physical_obstacle_.has_value()) {
            scene_witness_passed_ = false;
            scene_witness_summary_ = "MISSING_PHYSICAL_OBSTACLE";
            scene_witness_blocking_component_.clear();
            return false;
        }
        try {
            const Vec3 intended_goal = intended_hover +
                Vec3(goal_dx_m_, goal_dy_m_, goal_dz_m_);
            const C1eSceneWitness witness = evaluateC1eSceneWitness(
                intended_hover, intended_goal, body_half_extents_, suspended_geometry_,
                *physical_obstacle_);
            scene_witness_passed_ = witness.passed();
            scene_witness_summary_ = witness.summary();
            scene_witness_blocking_component_ = witness.blocking_component;
        } catch (const std::exception& exc) {
            scene_witness_passed_ = false;
            scene_witness_summary_ = std::string("EXCEPTION:") + exc.what();
            scene_witness_blocking_component_.clear();
        }
        return scene_witness_passed_;
    }

    void stateCallback(const interfaces::msg::MotionCaptureState::SharedPtr msg) {
        const double now_s = get_clock()->now().seconds();
        const Vec3 position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        const Vec3 velocity(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

        if (last_state_receive_ros_s_.has_value() &&
            now_s < *last_state_receive_ros_s_ - 1e-6) {
            enterFaultHold(position, "ROS_TIME_BACKJUMP_STATE");
            kinematics_.reset();
        }
        try {
            latest_state_ = kinematics_.update(position, velocity, now_s);
            last_state_receive_ros_s_ = now_s;
            if (!ground_anchor_.has_value() && phase_ == CommissioningPhase::Ground) {
                ground_anchor_ = position;
            }
        } catch (const std::exception& exc) {
            RCLCPP_WARN(get_logger(), "Rejected commissioning kinematics sample: %s", exc.what());
        }
    }

    void magnetTipCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_magnet_tip_ = Vec3(
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void pendulumCallback(const interfaces::msg::MotionCaptureState::SharedPtr msg) {
        const double now_s = get_clock()->now().seconds();
        pendulum_phi_rad_ = msg->pose.position.x;
        pendulum_theta_rad_ = msg->pose.position.y;
        pendulum_phi_dot_radps_ = msg->twist.angular.x;
        pendulum_theta_dot_radps_ = msg->twist.angular.y;
        last_pendulum_receive_ros_s_ = now_s;
    }

    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        std::transform(command.begin(), command.end(), command.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        if (command == "TAKEOFF") {
            if (phase_ != CommissioningPhase::Ground) return;
            const double now_s = get_clock()->now().seconds();
            State measured;
            double age_s = 0.0;
            if (!currentState(now_s, &measured, &age_s)) {
                RCLCPP_ERROR(get_logger(),
                    "Ignoring TAKEOFF because /motion_capture_state is not fresh.");
                return;
            }
            Vec3 proposed_hover = measured.position;
            proposed_hover.z() = hover_z_m_;
            if (!updateSceneWitness(proposed_hover)) {
                RCLCPP_ERROR(get_logger(),
                    "Ignoring TAKEOFF because the C.1e scene witness failed: %s",
                    scene_witness_summary_.c_str());
                return;
            }
            takeoff_start_position_ = measured.position;
            hover_target_ = proposed_hover;
            takeoff_start_ros_s_ = now_s;
            settle_started_ros_s_.reset();
            settle_progress_s_ = 0.0;
            completed_settle_dwell_s_ = 0.0;
            airborne_swing_max_rad_ = 0.0;
            active_swing_max_rad_ = 0.0;
            active_swing_rate_max_radps_ = 0.0;
            swing_warning_exceeded_ = false;
            geometric_claim_invalid_ = false;
            measured_trace_.clear();
            phase_ = CommissioningPhase::Ascent;
            RCLCPP_WARN(get_logger(),
                "%s TAKEOFF accepted: seventh-order C3 ascent starts from [%.3f %.3f %.3f] to z=%.3f over %.2f s.",
                commissioning_label_.c_str(),
                measured.position.x(), measured.position.y(), measured.position.z(),
                hover_z_m_, takeoff_duration_s_);
        } else if (command == "DISARM") {
            resetToGround();
        }
    }

    bool currentState(double now_s, State* state, double* age_s) const {
        if (!latest_state_.has_value() || !last_state_receive_ros_s_.has_value()) return false;
        const double age = now_s - *last_state_receive_ros_s_;
        if (!std::isfinite(age) || age < -0.02 || age > state_timeout_s_) return false;
        *state = *latest_state_;
        *age_s = std::max(0.0, age);
        return true;
    }

    bool pendulumFresh(double now_s, double* age_s) const {
        if (!last_pendulum_receive_ros_s_.has_value()) return false;
        const double age = now_s - *last_pendulum_receive_ros_s_;
        if (!std::isfinite(age) || age < -0.02 || age > pendulum_timeout_s_) return false;
        *age_s = std::max(0.0, age);
        return true;
    }

    void resetToGround() {
        ++epoch_;
        if (planner_future_.valid()) {
            // Do not block callbacks waiting for a flight planner. Its epoch will
            // make the completed result non-authoritative.
        }
        planner_.reset();
        authoritative_commit_.reset();
        takeoff_start_position_.reset();
        takeoff_start_ros_s_.reset();
        hover_target_.reset();
        global_goal_.reset();
        settle_started_ros_s_.reset();
        settle_progress_s_ = 0.0;
        completed_settle_dwell_s_ = 0.0;
        phase_ = CommissioningPhase::Ground;
        fault_reason_.clear();
        fault_hold_position_.reset();
        ground_anchor_.reset();
        measured_trace_.clear();
        latest_reference_window_.reset();
        scene_witness_passed_ = !require_scene_witness_;
        scene_witness_summary_ = require_scene_witness_ ? "PENDING" : "NOT_REQUIRED";
        RCLCPP_INFO(get_logger(), "%s authority state reset to GROUND after DISARM.",
                    commissioning_label_.c_str());
    }

    void enterFaultHold(const Vec3& hold_position, const std::string& reason) {
        ++epoch_;
        planner_.reset();
        authoritative_commit_.reset();
        fault_hold_position_ = hold_position;
        fault_reason_ = reason;
        phase_ = CommissioningPhase::FaultHold;
        settle_started_ros_s_.reset();
        settle_progress_s_ = 0.0;
        RCLCPP_ERROR(get_logger(),
            "%s entered FAULT_HOLD (%s). This is controlled-stop/hold fault containment, not an obstacle-safety certificate.",
            commissioning_label_.c_str(), reason.c_str());
    }

    bool settleSatisfied(double now_s, const State& measured) {
        if (!hover_target_.has_value()) return false;
        double pendulum_age_s = 0.0;
        if (!pendulumFresh(now_s, &pendulum_age_s)) return false;

        const double position_error_m = (measured.position - *hover_target_).norm();
        const double speed_mps = measured.velocity.norm();
        const double accel_mps2 = measured.acceleration.norm();
        const double swing_rad = std::hypot(pendulum_phi_rad_, pendulum_theta_rad_);
        const double swing_rate_radps = std::hypot(
            pendulum_phi_dot_radps_, pendulum_theta_dot_radps_);
        return position_error_m <= settle_position_tolerance_m_ &&
               speed_mps <= settle_speed_tolerance_mps_ &&
               accel_mps2 <= settle_accel_tolerance_mps2_ &&
               swing_rad <= settle_swing_tolerance_rad_ &&
               swing_rate_radps <= settle_swing_rate_tolerance_radps_;
    }

    bool initialisePersistentPlanner(double now_s, const State& measured) {
        if (!hover_target_.has_value()) {
            throw std::logic_error("cannot initialise planner without hover target");
        }
        global_goal_ = *hover_target_ + Vec3(goal_dx_m_, goal_dy_m_, goal_dz_m_);

        if (require_scene_witness_) {
            if (!updateSceneWitness(*hover_target_)) {
                enterFaultHold(measured.position, "C1E_SCENE_WITNESS_FAILED");
                RCLCPP_ERROR(get_logger(), "C.1e scene witness failed: %s",
                             scene_witness_summary_.c_str());
                return false;
            }
            RCLCPP_WARN(get_logger(), "C.1e scene witness PASS: %s",
                        scene_witness_summary_.c_str());
        }

        planner_ = std::make_unique<RecedingHorizonPlanner>(
            *global_goal_, recedingConfig(
                initial_splice_timing_s_, spline_time_factor_,
                enable_octopus_useful_deadline_, octopus_post_search_reserve_s_),
            localConfig(*hover_target_, *global_goal_, minimum_search_z_m_));

        CommittedTrajectory hover = dynamic_planner::makeStationaryHoverTrajectory(
            *hover_target_, now_s, 0.10);
        planner_->initializeCommittedTrajectory(hover);
        authoritative_commit_ = std::move(hover);
        phase_ = CommissioningPhase::ActivePlanning;
        RCLCPP_WARN(get_logger(),
            "%s persistent planner enabled after hover settle. Goal=[%.3f %.3f %.3f], initial timing seed=%.1f ms, spline time factor=%.2f, useful Octopus deadline=%s, reserve=%.1f ms.",
            commissioning_label_.c_str(),
            global_goal_->x(), global_goal_->y(), global_goal_->z(),
            1e3 * initial_splice_timing_s_, spline_time_factor_,
            enable_octopus_useful_deadline_ ? "ON" : "OFF",
            1e3 * octopus_post_search_reserve_s_);
        return true;
    }

    void writeReplanCsvRow(const PlannerOutcome& outcome, bool discarded_epoch) {
        if (!replan_csv_) return;

        const ReplanResult& result = outcome.result;
        // replans.csv is deliberately one row per actual planner attempt. Mission
        // execution states such as GOAL_SEEN_EXECUTE_COMMITTED remain available in
        // active.csv/reference_trace.csv instead of producing empty pseudo-replans.
        if (!result.attempted && !outcome.worker_exception) return;
        const auto* local = result.local_plan.has_value() ? &*result.local_plan : nullptr;
        const auto* refinement =
            local != nullptr && local->refinement.has_value() ? &*local->refinement : nullptr;
        const auto* prefix =
            result.prefix_safety.has_value() ? &*result.prefix_safety : nullptr;
        const auto* candidate =
            result.candidate_safety.has_value() ? &*result.candidate_safety : nullptr;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double relevant_finish_time_s =
            std::isfinite(result.authority_decision_time_s) && result.authority_decision_time_s > 0.0
                ? result.authority_decision_time_s
                : result.candidate_finish_time_s;
        const double finish_to_splice_margin_ms =
            std::isfinite(relevant_finish_time_s) && relevant_finish_time_s > 0.0
                ? 1e3 * (result.splice_time_s - relevant_finish_time_s)
                : nan;
        const std::string effective_status = outcome.worker_exception
            ? std::string("WORKER_EXCEPTION:") + outcome.worker_error
            : result.status;

        replan_csv_ << std::setprecision(15)
            << outcome.sequence << ',' << outcome.epoch << ',' << discarded_epoch << ','
            << outcome.request_ros_time_s << ',' << get_clock()->now().seconds() << ','
            << outcome.request_state.position.x() << ',' << outcome.request_state.position.y() << ','
            << outcome.request_state.position.z() << ',' << outcome.request_state.velocity.x() << ','
            << outcome.request_state.velocity.y() << ',' << outcome.request_state.velocity.z() << ','
            << outcome.request_state.acceleration.x() << ',' << outcome.request_state.acceleration.y() << ','
            << outcome.request_state.acceleration.z() << ',' << result.attempted << ',' << result.accepted << ','
            << result.candidate_late << ',' << result.goal_seen << ',' << result.goal_reached << ','
            << safeCsvField(effective_status) << ',' << result.splice_time_s << ','
            << result.splice_lookahead_s << ',' << result.octopus_useful_deadline_enabled << ','
            << result.octopus_post_search_reserve_s << ','
            << result.octopus_useful_deadline_time_s << ',' << result.octopus_useful_budget_s << ','
            << result.octopus_deadline_clock_invalid << ',' << result.splice_state.position.x() << ','
            << result.splice_state.position.y() << ',' << result.splice_state.position.z() << ','
            << result.splice_state.velocity.x() << ',' << result.splice_state.velocity.y() << ','
            << result.splice_state.velocity.z() << ',' << result.splice_state.acceleration.x() << ','
            << result.splice_state.acceleration.y() << ',' << result.splice_state.acceleration.z() << ','
            << result.local_goal.x() << ',' << result.local_goal.y() << ',' << result.local_goal.z() << ','
            << result.local_duration_s << ',' << result.minimum_time_s << ','
            << result.time_allocation_factor << ',' << result.global_distance_from_splice_m << ','
            << result.committed_endpoint_distance_m << ',' << 1e3 * result.replan_runtime_s << ','
            << 1e3 * result.trajectory_elapsed_s << ',' << result.candidate_finish_time_s << ','
            << result.authority_decision_time_s << ',' << finish_to_splice_margin_ms << ','
            << result.splice_diagnostics.accepted << ',' << result.splice_diagnostics.position_error << ','
            << result.splice_diagnostics.velocity_error << ',' << result.splice_diagnostics.acceleration_error << ','
            << result.planning_world_version << ',' << result.recheck_world_version << ','
            << result.final_recheck_world_version << ',' << result.recheck_retry_performed << ','
            << result.incumbent_prefix_unsafe << ',' << (prefix != nullptr) << ','
            << (prefix != nullptr ? prefix->safe : false) << ','
            << safeCsvField(prefix != nullptr ? prefix->status : std::string()) << ','
            << (candidate != nullptr) << ',' << (candidate != nullptr ? candidate->safe : false) << ','
            << safeCsvField(candidate != nullptr ? candidate->status : std::string()) << ','
            << (local != nullptr) << ','
            << safeCsvField(local != nullptr ? local->status : std::string()) << ','
            << safeCsvField(local != nullptr ? local->message : std::string()) << ','
            << (local != nullptr ? 1e3 * local->total_solve_time_s : nan) << ','
            << safeCsvField(local != nullptr ? local->search.status : std::string()) << ','
            << (local != nullptr ? local->search.success : false) << ','
            << (local != nullptr ? local->search.reached_goal : false) << ','
            << (local != nullptr ? local->search.expanded_nodes : 0U) << ','
            << (local != nullptr ? local->search.popped_nodes : 0U) << ','
            << (local != nullptr ? local->search.separator_lp_calls : 0U) << ','
            << (local != nullptr ? 1e3 * local->search.search_time_s : nan) << ','
            << (local != nullptr ? local->search.voxel_size_m : nan) << ','
            << (local != nullptr ? optionalOrNan(local->search.goal_distance_m) : nan) << ','
            << safeCsvField(local != nullptr ? local->search.termination_reason : std::string()) << ','
            << (local != nullptr ? local->search.complete_available_at_termination : false) << ','
            << (local != nullptr ? 1e3 * optionalOrNan(local->search.first_complete_time_s) : nan) << ','
            << (local != nullptr ? optionalOrNan(local->search.first_complete_goal_distance_m) : nan) << ','
            << (local != nullptr ? local->search.first_complete_expanded_nodes : 0U) << ','
            << (local != nullptr ? local->search.first_complete_popped_nodes : 0U) << ','
            << (local != nullptr ? local->search.closest_complete_improvements : 0U) << ','
            << (refinement != nullptr) << ','
            << safeCsvField(refinement != nullptr ? refinement->status : std::string()) << ','
            << safeCsvField(refinement != nullptr ? refinement->message : std::string()) << ','
            << (refinement != nullptr ? 1e3 * refinement->solve_time_s : nan) << ','
            << (refinement != nullptr ? refinement->working_set_recalculations : 0) << ','
            << (refinement != nullptr ? refinement->raw_goal_distance_m : nan) << ','
            << (refinement != nullptr ? optionalOrNan(refinement->refined_goal_distance_m) : nan) << ','
            << (refinement != nullptr ? refinement->goal_accepted : false) << ','
            << (refinement != nullptr ? optionalOrNan(refinement->max_equality_residual) : nan) << ','
            << (refinement != nullptr ? optionalOrNan(refinement->max_inequality_violation) : nan) << ','
            << (refinement != nullptr ? optionalOrNan(refinement->min_fixed_separator_slack) : nan) << ','
            << outcome.worker_exception << ',' << safeCsvField(outcome.worker_error) << '\n';
        replan_csv_.flush();
    }

    void pollPlannerFuture() {
        if (!planner_future_.valid()) return;
        if (planner_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;

        PlannerOutcome outcome = planner_future_.get();
        const bool discarded_epoch = outcome.epoch != epoch_;
        writeReplanCsvRow(outcome, discarded_epoch);
        if (discarded_epoch) {
            ++discarded_epoch_results_;
            return;
        }
        planner_ = std::move(outcome.planner);
        if (outcome.worker_exception) {
            ++planner_worker_errors_;
            latest_status_ = "WORKER_EXCEPTION:" + outcome.worker_error;
            return;
        }

        latest_replan_result_ = outcome.result;
        latest_status_ = outcome.result.status;
        if (outcome.result.attempted) {
            const double runtime_ms = 1e3 * outcome.result.replan_runtime_s;
            replan_runtime_stats_.add(runtime_ms);
            if (outcome.result.replan_runtime_s > planner_period_s_) {
                ++planner_period_overruns_;
            }
            if (outcome.result.local_plan.has_value()) {
                octopus_runtime_stats_.add(1e3 * outcome.result.local_plan->search.search_time_s);
                if (outcome.result.local_plan->refinement.has_value()) {
                    qp_runtime_stats_.add(1e3 * outcome.result.local_plan->refinement->solve_time_s);
                }
            }
        }
        if (outcome.result.candidate_late) ++late_candidates_;
        if (outcome.result.accepted) {
            ++accepted_candidates_;
            if (planner_ && !planner_->committedTrajectory().empty()) {
                authoritative_commit_ = planner_->committedTrajectory();
            }
            const double margin = outcome.result.splice_time_s -
                outcome.result.authority_decision_time_s;
            if (!first_accepted_future_splice_verified_) {
                first_accepted_future_splice_verified_ =
                    margin > 0.0 &&
                    outcome.result.splice_diagnostics.position_error <= 1e-7 &&
                    outcome.result.splice_diagnostics.velocity_error <= 1e-7 &&
                    outcome.result.splice_diagnostics.acceleration_error <= 1e-7;
                first_accepted_finish_to_splice_margin_s_ = margin;
            }
        } else if (outcome.result.attempted) {
            ++failed_candidates_;
        }
    }

    void replanTimer() {
        pollPlannerFuture();
        if (phase_ != CommissioningPhase::ActivePlanning) return;
        if (planner_future_.valid()) {
            ++coalesced_replan_triggers_;
            return;
        }
        if (!planner_) {
            ++planner_worker_errors_;
            latest_status_ = "ACTIVE_WITHOUT_PLANNER";
            return;
        }

        const double now_s = get_clock()->now().seconds();
        State measured;
        double state_age_s = 0.0;
        if (!currentState(now_s, &measured, &state_age_s)) {
            ++stale_state_replan_skips_;
            return;
        }

        auto planner = std::move(planner_);
        const std::uint64_t epoch = epoch_;
        const std::uint64_t sequence = ++replan_sequence_;
        auto world = world_;
        auto ros_clock = get_clock();
        planner_future_ = std::async(std::launch::async,
            [sequence, epoch, planner = std::move(planner), world, ros_clock, measured, now_s]() mutable {
                PlannerOutcome outcome;
                outcome.sequence = sequence;
                outcome.epoch = epoch;
                outcome.request_ros_time_s = now_s;
                outcome.request_state = measured;
                outcome.planner = std::move(planner);
                try {
                    const TrajectoryTimeSource trajectory_clock = [ros_clock]() {
                        return ros_clock->now().seconds();
                    };
                    outcome.result = outcome.planner->replan(
                        now_s, measured, *world, trajectory_clock);
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
        pollPlannerFuture();
        const double now_s = get_clock()->now().seconds();
        if (last_ros_now_s_.has_value() && now_s < *last_ros_now_s_ - 1e-6) {
            State measured;
            double age_s = 0.0;
            if (currentState(now_s, &measured, &age_s)) {
                enterFaultHold(measured.position, "ROS_TIME_BACKJUMP_REFERENCE");
            }
            reference_period_stats_.clear();
            last_reference_ros_s_.reset();
        }
        last_ros_now_s_ = now_s;
        if (last_reference_ros_s_.has_value()) {
            reference_period_stats_.add(1e3 * (now_s - *last_reference_ros_s_));
        }
        last_reference_ros_s_ = now_s;

        State measured;
        double state_age_s = 0.0;
        const bool state_fresh = currentState(now_s, &measured, &state_age_s);
        latest_state_fresh_ = state_fresh;
        latest_state_age_s_ = state_fresh ? state_age_s : -1.0;
        if (state_fresh) {
            latest_speed_mps_ = measured.velocity.norm();
            latest_accel_mps2_ = measured.acceleration.norm();
        }
        double pendulum_age_s = 0.0;
        latest_pendulum_fresh_ = pendulumFresh(now_s, &pendulum_age_s);
        latest_pendulum_age_s_ = latest_pendulum_fresh_ ? pendulum_age_s : -1.0;
        latest_swing_rad_ = std::hypot(pendulum_phi_rad_, pendulum_theta_rad_);
        latest_swing_rate_radps_ = std::hypot(
            pendulum_phi_dot_radps_, pendulum_theta_dot_radps_);
        if (latest_pendulum_fresh_ && phase_ != CommissioningPhase::Ground) {
            airborne_swing_max_rad_ = std::max(
                airborne_swing_max_rad_, latest_swing_rad_);
            if (phase_ == CommissioningPhase::ActivePlanning) {
                active_swing_max_rad_ = std::max(
                    active_swing_max_rad_, latest_swing_rad_);
                active_swing_rate_max_radps_ = std::max(
                    active_swing_rate_max_radps_, latest_swing_rate_radps_);
            }
            if (latest_swing_rad_ > swing_warning_rad_) {
                swing_warning_exceeded_ = true;
            }
            if (latest_swing_rad_ > swing_claim_invalid_rad_) {
                geometric_claim_invalid_ = true;
            }
        }
        if (state_fresh && phase_ != CommissioningPhase::Ground) {
            measured_trace_.push_back(measured.position);
            if (measured_trace_.size() > measured_trace_capacity_) {
                measured_trace_.pop_front();
            }
        }

        try {
            dynamic_planner::ReferenceWindow window;
            switch (phase_) {
                case CommissioningPhase::Ground: {
                    if (!state_fresh) {
                        ++reference_skips_no_state_;
                        return;
                    }
                    ground_anchor_ = measured.position;
                    window = makeStationaryReferenceWindow(
                        *ground_anchor_, now_s, reference_config_);
                    reference_source_ = "GROUND_MEASURED_HOLD";
                    break;
                }
                case CommissioningPhase::Ascent: {
                    if (!takeoff_start_position_.has_value() ||
                        !takeoff_start_ros_s_.has_value() || !hover_target_.has_value()) {
                        if (state_fresh) enterFaultHold(measured.position, "ASCENT_STATE_INVALID");
                        ++reference_errors_;
                        return;
                    }
                    window = makeVerticalTakeoffReferenceWindow(
                        *takeoff_start_position_, hover_target_->z(),
                        *takeoff_start_ros_s_, now_s, takeoff_duration_s_, reference_config_);
                    reference_source_ = "C3_TAKEOFF";
                    if (now_s >= *takeoff_start_ros_s_ + takeoff_duration_s_) {
                        phase_ = CommissioningPhase::HoverSettle;
                        settle_started_ros_s_.reset();
                        RCLCPP_INFO(get_logger(),
                            "%s takeoff profile complete; entering vehicle+payload settle gate.",
                            commissioning_label_.c_str());
                    }
                    break;
                }
                case CommissioningPhase::HoverSettle: {
                    if (!hover_target_.has_value()) {
                        ++reference_errors_;
                        return;
                    }
                    window = makeStationaryReferenceWindow(
                        *hover_target_, now_s, reference_config_);
                    reference_source_ = "HOVER_SETTLE_HOLD";
                    if (state_fresh && settleSatisfied(now_s, measured)) {
                        if (!settle_started_ros_s_.has_value()) {
                            settle_started_ros_s_ = now_s;
                        }
                        settle_progress_s_ = std::max(
                            0.0, now_s - *settle_started_ros_s_);
                        if (now_s - *settle_started_ros_s_ >= settle_dwell_s_) {
                            completed_settle_dwell_s_ = now_s - *settle_started_ros_s_;
                            settle_progress_s_ = completed_settle_dwell_s_;
                            if (!initialisePersistentPlanner(now_s, measured) &&
                                fault_hold_position_.has_value()) {
                                window = makeStationaryReferenceWindow(
                                    *fault_hold_position_, now_s, reference_config_);
                                reference_source_ = "FAULT_CONTROLLED_HOLD";
                            }
                        }
                    } else {
                        settle_started_ros_s_.reset();
                        settle_progress_s_ = 0.0;
                    }
                    break;
                }
                case CommissioningPhase::ActivePlanning: {
                    if (!authoritative_commit_.has_value()) {
                        if (state_fresh) enterFaultHold(measured.position, "MISSING_COMMITTED_AUTHORITY");
                        ++reference_errors_;
                        return;
                    }
                    window = dynamic_planner::sampleCommittedReferenceWindow(
                        *authoritative_commit_, now_s, reference_config_);
                    reference_source_ = "COMMITTED_PLANNER";
                    break;
                }
                case CommissioningPhase::FaultHold: {
                    if (!fault_hold_position_.has_value()) {
                        if (!state_fresh) {
                            ++reference_skips_no_state_;
                            return;
                        }
                        fault_hold_position_ = measured.position;
                    }
                    window = makeStationaryReferenceWindow(
                        *fault_hold_position_, now_s, reference_config_);
                    reference_source_ = "FAULT_CONTROLLED_HOLD";
                    break;
                }
            }

            const rclcpp::Time publish_time = get_clock()->now();
            const builtin_interfaces::msg::Time publish_stamp = publish_time;
            const auto msg = makeReferenceMessage(
                window, publish_stamp, frame_id_, vehicle_id_);
            if (msg.points.size() != 61U) {
                throw std::logic_error("commissioning generated a non-61-sample reference");
            }
            reference_pub_->publish(msg);
            latest_reference_window_ = window;
            if (trace_csv_) {
                const State& reference_now = window.samples.front().state;
                const State measured_for_trace = state_fresh ? measured : State{};
                const double position_error_m = state_fresh
                    ? (measured.position - reference_now.position).norm()
                    : std::numeric_limits<double>::quiet_NaN();
                const double missing = std::numeric_limits<double>::quiet_NaN();
                trace_csv_ << std::setprecision(15)
                           << now_s << ',' << phaseName(phase_) << ','
                           << safeCsvField(reference_source_) << ',' << state_fresh << ','
                           << reference_now.position.x() << ',' << reference_now.position.y() << ','
                           << reference_now.position.z() << ',' << reference_now.velocity.x() << ','
                           << reference_now.velocity.y() << ',' << reference_now.velocity.z() << ','
                           << reference_now.acceleration.x() << ','
                           << reference_now.acceleration.y() << ','
                           << reference_now.acceleration.z() << ','
                           << (state_fresh ? measured_for_trace.position.x() : missing) << ','
                           << (state_fresh ? measured_for_trace.position.y() : missing) << ','
                           << (state_fresh ? measured_for_trace.position.z() : missing) << ','
                           << (state_fresh ? measured_for_trace.velocity.x() : missing) << ','
                           << (state_fresh ? measured_for_trace.velocity.y() : missing) << ','
                           << (state_fresh ? measured_for_trace.velocity.z() : missing) << ','
                           << (state_fresh ? measured_for_trace.acceleration.x() : missing) << ','
                           << (state_fresh ? measured_for_trace.acceleration.y() : missing) << ','
                           << (state_fresh ? measured_for_trace.acceleration.z() : missing) << ','
                           << position_error_m << ',' << latest_swing_rad_ * 180.0 / kPi << ','
                           << latest_swing_rate_radps_ * 180.0 / kPi << ','
                           << world_->currentVersion() << '\n';
                if (++trace_rows_since_flush_ >= 30U) {
                    trace_csv_.flush();
                    trace_rows_since_flush_ = 0U;
                }
            }
            ++reference_publish_count_;
        } catch (const std::exception& exc) {
            ++reference_errors_;
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "%s authoritative reference generation failed: %s",
                commissioning_label_.c_str(), exc.what());
            if (state_fresh && phase_ != CommissioningPhase::FaultHold) {
                enterFaultHold(measured.position, "REFERENCE_GENERATION_EXCEPTION");
            }
        }
    }

    void markerTimer() noexcept {
        try {
            const rclcpp::Time stamp = get_clock()->now();
            MarkerArray array;
            Marker clear;
            clear.action = Marker::DELETEALL;
            array.markers.push_back(clear);

        if (physical_obstacle_.has_value()) {
            Marker obstacle = baseMarker(
                frame_id_, stamp, "physical_obstacle", 0, Marker::CUBE);
            obstacle.pose.position = markerPoint(obstacle_centre_);
            obstacle.pose.orientation.z = std::sin(0.5 * obstacle_yaw_rad_);
            obstacle.pose.orientation.w = std::cos(0.5 * obstacle_yaw_rad_);
            obstacle.scale.x = 2.0 * obstacle_half_extents_.x();
            obstacle.scale.y = 2.0 * obstacle_half_extents_.y();
            obstacle.scale.z = 2.0 * obstacle_half_extents_.z();
            obstacle.color = markerColour(0.95F, 0.35F, 0.05F, 0.65F);
            array.markers.push_back(std::move(obstacle));

            const auto components = dynamic_planner::assemblyComponents(
                body_half_extents_, suspended_geometry_);
            int component_id = 0;
            for (const auto& component : components) {
                const Eigen::MatrixXd cspace =
                    dynamic_planner::physicalObstacleToConfigurationSpace(
                        physical_obstacle_->vertices, component);
                std_msgs::msg::ColorRGBA colour = markerColour(0.2F, 0.8F, 1.0F, 0.75F);
                if (component.name == "cable") colour = markerColour(1.0F, 0.85F, 0.1F, 0.75F);
                if (component.name == "magnet") colour = markerColour(1.0F, 0.1F, 0.8F, 0.75F);
                if (component.name == "payload") colour = markerColour(0.7F, 0.2F, 1.0F, 0.75F);
                array.markers.push_back(pointsMarker(
                    frame_id_, stamp, "cspace_" + component.name,
                    component_id++, cspace, colour, 0.025));
            }
        }

        if (latest_state_.has_value()) {
            array.markers.push_back(sphereMarker(
                frame_id_, stamp, "current_drone_position", 0, latest_state_->position,
                markerColour(0.05F, 0.85F, 1.0F, 1.0F), 0.11));
            if (latest_pendulum_fresh_ && latest_magnet_tip_.has_value()) {
                array.markers.push_back(sphereMarker(
                    frame_id_, stamp, "current_magnet_tip", 0, *latest_magnet_tip_,
                    markerColour(1.0F, 0.1F, 0.85F, 1.0F), 0.09));
            }

            const auto components = dynamic_planner::assemblyComponents(
                body_half_extents_, suspended_geometry_);
            int component_id = 0;
            for (const auto& component : components) {
                Eigen::MatrixXd occupied = component.vertices;
                occupied.rowwise() += latest_state_->position.transpose();
                std_msgs::msg::ColorRGBA colour = markerColour(0.3F, 0.85F, 1.0F, 0.9F);
                if (component.name == "cable") colour = markerColour(1.0F, 0.9F, 0.1F, 0.9F);
                if (component.name == "magnet") colour = markerColour(1.0F, 0.2F, 0.8F, 0.9F);
                if (component.name == "payload") colour = markerColour(0.7F, 0.2F, 1.0F, 0.9F);
                array.markers.push_back(pointsMarker(
                    frame_id_, stamp, "assembly_" + component.name,
                    component_id++, occupied, colour, 0.018));
            }
        }

        std::optional<Vec3> intended_start = hover_target_;
        if (!intended_start.has_value()) {
            if (latest_state_.has_value()) {
                intended_start = latest_state_->position;
                intended_start->z() = hover_z_m_;
            } else if (ground_anchor_.has_value()) {
                intended_start = *ground_anchor_;
                intended_start->z() = hover_z_m_;
            }
        }
        if (intended_start.has_value()) {
            const Vec3 intended_goal = *intended_start +
                Vec3(goal_dx_m_, goal_dy_m_, goal_dz_m_);
            array.markers.push_back(lineStripMarker(
                frame_id_, stamp, "body_direct_witness", 0,
                {*intended_start, intended_goal},
                markerColour(0.25F, 1.0F, 0.25F, 0.8F), 0.012));
            array.markers.push_back(sphereMarker(
                frame_id_, stamp, "start_goal", 0, *intended_start,
                markerColour(0.25F, 0.9F, 1.0F), 0.07));
            array.markers.push_back(sphereMarker(
                frame_id_, stamp, "start_goal", 1, intended_goal,
                markerColour(0.25F, 1.0F, 0.25F), 0.07));
        }

        if (latest_reference_window_.has_value()) {
            std::vector<Vec3> reference_points;
            reference_points.reserve(latest_reference_window_->samples.size());
            for (const auto& sample : latest_reference_window_->samples) {
                reference_points.push_back(sample.state.position);
            }
            array.markers.push_back(lineStripMarker(
                frame_id_, stamp, "authoritative_reference", 0,
                reference_points, markerColour(0.1F, 1.0F, 0.95F), 0.025));
        }

        if (authoritative_commit_.has_value() && !authoritative_commit_->empty()) {
            std::vector<Vec3> committed_points;
            try {
                const double now_s = stamp.seconds();
                const double t0 = std::max(now_s, authoritative_commit_->startTime());
                const double t1 = std::max(t0, std::min(
                    authoritative_commit_->endTime(), now_s + 5.0));
                if (t1 <= t0 + 1e-9) {
                    committed_points.push_back(authoritative_commit_->evaluate(t0).position);
                } else {
                    for (double t = t0; t < t1 - 0.025; t += 0.05) {
                        committed_points.push_back(authoritative_commit_->evaluate(t).position);
                    }
                    committed_points.push_back(authoritative_commit_->evaluate(t1).position);
                }
            } catch (const std::exception&) {
                committed_points.clear();
            }
            if (!committed_points.empty()) {
                array.markers.push_back(lineStripMarker(
                    frame_id_, stamp, "committed_route", 0, committed_points,
                    markerColour(0.1F, 0.45F, 1.0F), 0.035));
            }
        }

        if (latest_replan_result_.local_plan.has_value()) {
            const auto add_control_polygon = [&](
                const std::optional<dynamic_planner::ControlPoints>& control_points,
                const std::string& marker_namespace,
                const std_msgs::msg::ColorRGBA& colour) {
                if (!control_points.has_value()) return;
                std::vector<Vec3> points;
                points.reserve(static_cast<std::size_t>(control_points->rows()));
                for (Eigen::Index row = 0; row < control_points->rows(); ++row) {
                    points.push_back(control_points->row(row).transpose());
                }
                array.markers.push_back(lineStripMarker(
                    frame_id_, stamp, marker_namespace, 0, points, colour, 0.012));
            };
            add_control_polygon(
                latest_replan_result_.local_plan->search.control_points,
                "octopus_control_polygon", markerColour(1.0F, 0.55F, 0.05F));
            add_control_polygon(
                latest_replan_result_.local_plan->control_points,
                "selected_control_polygon", markerColour(0.75F, 0.2F, 1.0F));
        }

        if (!measured_trace_.empty()) {
            const std::vector<Vec3> measured_points(
                measured_trace_.begin(), measured_trace_.end());
            array.markers.push_back(lineStripMarker(
                frame_id_, stamp, "measured_trace", 0, measured_points,
                markerColour(1.0F, 0.15F, 0.15F), 0.025));
        }

        if (latest_replan_result_.attempted) {
            array.markers.push_back(sphereMarker(
                frame_id_, stamp, "splice_point", 0,
                latest_replan_result_.splice_state.position,
                markerColour(1.0F, 1.0F, 0.1F), 0.08));
            array.markers.push_back(sphereMarker(
                frame_id_, stamp, "local_goal", 0,
                latest_replan_result_.local_goal,
                markerColour(1.0F, 0.35F, 0.85F), 0.09));
        }
        if (global_goal_.has_value()) {
            array.markers.push_back(sphereMarker(
                frame_id_, stamp, "global_goal", 0, *global_goal_,
                markerColour(0.2F, 1.0F, 0.2F), 0.11));
        }

        Marker status = baseMarker(
            frame_id_, stamp, "commissioning_status", 0, Marker::TEXT_VIEW_FACING);
        status.pose.position = markerPoint(
            physical_obstacle_.has_value()
                ? obstacle_centre_ + Vec3(0.0, 0.0, obstacle_half_extents_.z() + 0.18)
                : Vec3(0.0, 0.0, hover_z_m_ + 0.25));
        status.scale.z = 0.065;
        status.color = geometric_claim_invalid_
            ? markerColour(1.0F, 0.1F, 0.1F)
            : markerColour(1.0F, 1.0F, 1.0F);
        std::ostringstream status_text;
        status_text << commissioning_label_ << " - " << phaseName(phase_)
                    << "\nscene witness: "
                    << (scene_witness_passed_ ? "PASS" : "FAIL");
        if (!scene_witness_blocking_component_.empty()) {
            status_text << "\ndirect-path blocker: "
                        << scene_witness_blocking_component_;
        }
        status_text << "\nswing: " << std::fixed << std::setprecision(1)
                    << latest_swing_rad_ * 180.0 / kPi << " deg";
        status.text = status_text.str();
        array.markers.push_back(std::move(status));

            marker_pub_->publish(array);
        } catch (const std::exception& exc) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "C.1e visualization update skipped: %s", exc.what());
        } catch (...) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "C.1e visualization update skipped: unknown exception");
        }
    }

    void diagnosticsTimer() {
        pollPlannerFuture();
        const double now_s = get_clock()->now().seconds();
        State measured;
        double state_age_s = 0.0;
        latest_state_fresh_ = currentState(now_s, &measured, &state_age_s);
        if (latest_state_fresh_) {
            latest_state_age_s_ = state_age_s;
            latest_speed_mps_ = measured.velocity.norm();
            latest_accel_mps2_ = measured.acceleration.norm();
            if (phase_ == CommissioningPhase::Ground && require_scene_witness_) {
                Vec3 proposed_hover = measured.position;
                proposed_hover.z() = hover_z_m_;
                (void)updateSceneWitness(proposed_hover);
            }
        }
        double pendulum_age_s = 0.0;
        latest_pendulum_fresh_ = pendulumFresh(now_s, &pendulum_age_s);
        if (latest_pendulum_fresh_) latest_pendulum_age_s_ = pendulum_age_s;

        const StatsSummary ref = reference_period_stats_.summary();
        const StatsSummary runtime = replan_runtime_stats_.summary();
        const StatsSummary octopus = octopus_runtime_stats_.summary();
        const StatsSummary qp = qp_runtime_stats_.summary();
        const double settle_elapsed_s = phase_ == CommissioningPhase::HoverSettle
            ? settle_progress_s_ : completed_settle_dwell_s_;

        std::ostringstream text;
        text << std::fixed << std::setprecision(3)
             << commissioning_label_ << " ACTIVE COMMISSIONING\n"
             << "phase: " << phaseName(phase_) << " reference_source: " << reference_source_ << "\n"
             << "state_fresh: " << std::boolalpha << latest_state_fresh_
             << " age_ms: " << 1e3 * latest_state_age_s_
             << " speed_mps: " << latest_speed_mps_
             << " accel_mps2: " << latest_accel_mps2_ << "\n"
             << "pendulum_fresh: " << latest_pendulum_fresh_
             << " age_ms: " << 1e3 * latest_pendulum_age_s_
             << " swing_deg: " << latest_swing_rad_ * 180.0 / kPi
             << " swing_rate_degps: " << latest_swing_rate_radps_ * 180.0 / kPi << "\n"
             << "settle gate dwell/required s: " << settle_elapsed_s << " / " << settle_dwell_s_ << "\n"
             << "scene enabled/witness passed: " << static_scene_enabled_ << " / "
             << scene_witness_passed_ << " (" << scene_witness_summary_ << ")"
             << " world_version: " << world_->currentVersion() << "\n"
             << "swing maxima airborne/ACTIVE deg: "
             << airborne_swing_max_rad_ * 180.0 / kPi << " / "
             << active_swing_max_rad_ * 180.0 / kPi
             << " active max rate deg/s: "
             << active_swing_rate_max_radps_ * 180.0 / kPi
             << " warning>" << swing_warning_rad_ * 180.0 / kPi << ": "
             << swing_warning_exceeded_ << " claim-invalid>"
             << swing_claim_invalid_rad_ * 180.0 / kPi << ": "
             << geometric_claim_invalid_ << "\n"
             << "authority_topic: " << reference_topic_
             << " published: " << reference_publish_count_
             << " reference_errors/no-state-skips: " << reference_errors_ << " / "
             << reference_skips_no_state_ << "\n"
             << "reference period mean/p95/max ms: " << ref.mean << " / " << ref.p95
             << " / " << ref.maximum << "\n"
             << "planner target/period/overruns: " << planner_rate_hz_ << " Hz / "
             << 1e3 * planner_period_s_ << " ms / " << planner_period_overruns_ << "\n"
             << "replans accepted/failed/late: " << accepted_candidates_ << " / "
             << failed_candidates_ << " / " << late_candidates_ << "\n"
             << "coalesced/stale-state/worker-errors: " << coalesced_replan_triggers_ << " / "
             << stale_state_replan_skips_ << " / " << planner_worker_errors_ << "\n"
             << "runtime mean/p95/max ms: " << runtime.mean << " / " << runtime.p95
             << " / " << runtime.maximum << "\n"
             << "Octopus mean/p95/max ms: " << octopus.mean << " / " << octopus.p95
             << " / " << octopus.maximum << "\n"
             << "qpOASES mean/p95/max ms: " << qp.mean << " / " << qp.p95
             << " / " << qp.maximum << "\n"
             << "latest planner status: " << latest_status_ << "\n"
             << "first accepted future C2 splice verified: "
             << first_accepted_future_splice_verified_;
        if (first_accepted_future_splice_verified_) {
            text << " margin_ms: " << 1e3 * first_accepted_finish_to_splice_margin_s_;
        }
        text << "\ncontrolled fault-hold active: "
             << (phase_ == CommissioningPhase::FaultHold)
             << " reason: " << (fault_reason_.empty() ? "none" : fault_reason_)
             << "\nNOTE: FAULT_HOLD is controller fault containment, not an obstacle-safety certificate.";

        std_msgs::msg::String msg;
        msg.data = text.str();
        diagnostics_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "%s", msg.data.c_str());

        if (csv_) {
            csv_ << std::setprecision(15)
                 << now_s << ','
                 << phaseName(phase_) << ','
                 << safeCsvField(reference_source_) << ','
                 << latest_state_fresh_ << ',' << 1e3 * latest_state_age_s_ << ','
                 << latest_speed_mps_ << ',' << latest_accel_mps2_ << ','
                 << latest_pendulum_fresh_ << ',' << 1e3 * latest_pendulum_age_s_ << ','
                 << latest_swing_rad_ * 180.0 / kPi << ','
                 << latest_swing_rate_radps_ * 180.0 / kPi << ','
                 << settle_elapsed_s << ',' << reference_publish_count_ << ','
                 << reference_errors_ << ',' << ref.mean << ',' << ref.p95 << ',' << ref.maximum << ','
                 << accepted_candidates_ << ',' << failed_candidates_ << ',' << late_candidates_ << ','
                 << coalesced_replan_triggers_ << ',' << stale_state_replan_skips_ << ','
                 << planner_worker_errors_ << ',' << planner_period_overruns_ << ','
                 << runtime.mean << ',' << runtime.p95 << ',' << runtime.maximum << ','
                 << octopus.mean << ',' << octopus.p95 << ',' << octopus.maximum << ','
                 << qp.mean << ',' << qp.p95 << ',' << qp.maximum << ','
                 << safeCsvField(latest_status_) << ',' << safeCsvField(fault_reason_) << ','
                 << first_accepted_future_splice_verified_ << ','
                 << (first_accepted_future_splice_verified_
                     ? 1e3 * first_accepted_finish_to_splice_margin_s_ : 0.0) << ','
                 << scene_witness_passed_ << ','
                 << airborne_swing_max_rad_ * 180.0 / kPi << ','
                 << active_swing_max_rad_ * 180.0 / kPi << ','
                 << active_swing_rate_max_radps_ * 180.0 / kPi << ','
                 << swing_warning_exceeded_ << ',' << geometric_claim_invalid_ << ','
                 << world_->currentVersion() << '\n';
            csv_.flush();
        }
        if (trace_csv_) trace_csv_.flush();
    }

    KinematicsFilter kinematics_;
    dynamic_planner::ReferenceWindowConfig reference_config_;

    std::string state_topic_;
    std::string pendulum_topic_;
    std::string magnet_tip_topic_;
    std::string command_topic_;
    std::string reference_topic_;
    std::string diagnostics_topic_;
    std::string marker_topic_;
    std::string csv_path_;
    std::string trace_csv_path_;
    std::string replan_csv_path_;
    std::string frame_id_;
    std::string vehicle_id_;
    std::string commissioning_label_;
    std::string obstacle_name_;

    double state_timeout_s_ = 0.25;
    double pendulum_timeout_s_ = 0.25;
    double planner_rate_hz_ = 5.0;
    double marker_rate_hz_ = 5.0;
    double planner_period_s_ = 0.20;
    double minimum_search_z_m_ = 0.20;
    double initial_splice_timing_s_ = 0.16;
    double spline_time_factor_ = 2.5;
    bool enable_octopus_useful_deadline_ = false;
    double octopus_post_search_reserve_s_ = 1.0 / kControlReferenceHz;
    double hover_z_m_ = 1.20;
    double takeoff_duration_s_ = 4.0;
    double settle_dwell_s_ = 1.0;
    double settle_position_tolerance_m_ = 0.10;
    double settle_speed_tolerance_mps_ = 0.10;
    double settle_accel_tolerance_mps2_ = 0.50;
    double settle_swing_tolerance_rad_ = 5.0 * kPi / 180.0;
    double settle_swing_rate_tolerance_radps_ = 0.20;
    double goal_dx_m_ = 1.0;
    double goal_dy_m_ = 0.0;
    double goal_dz_m_ = 0.0;
    double max_swing_angle_rad_ = 10.0 * kPi / 180.0;
    double swing_warning_rad_ = 10.0 * kPi / 180.0;
    double swing_claim_invalid_rad_ = 10.0 * kPi / 180.0;
    double obstacle_yaw_rad_ = 0.0;
    double settle_progress_s_ = 0.0;
    double completed_settle_dwell_s_ = 0.0;
    double airborne_swing_max_rad_ = 0.0;
    double active_swing_max_rad_ = 0.0;
    double active_swing_rate_max_radps_ = 0.0;
    bool static_scene_enabled_ = false;
    bool require_scene_witness_ = false;
    bool scene_witness_passed_ = true;
    bool swing_warning_exceeded_ = false;
    bool geometric_claim_invalid_ = false;
    Vec3 body_half_extents_ = Vec3(0.105, 0.105, 0.060);
    Vec3 obstacle_centre_ = Vec3(0.50, 0.06, 0.68);
    Vec3 obstacle_half_extents_ = Vec3(0.12, 0.25, 0.10);
    SuspendedGeometry suspended_geometry_;
    std::optional<dynamic_planner::StaticConvexObstacle> physical_obstacle_;
    std::string scene_witness_summary_ = "NOT_REQUIRED";
    std::string scene_witness_blocking_component_;
    std::size_t measured_trace_capacity_ = 900U;

    rclcpp::Subscription<interfaces::msg::MotionCaptureState>::SharedPtr state_sub_;
    rclcpp::Subscription<interfaces::msg::MotionCaptureState>::SharedPtr pendulum_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr magnet_tip_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr reference_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr reference_timer_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::TimerBase::SharedPtr marker_timer_;

    CommissioningPhase phase_ = CommissioningPhase::Ground;
    std::uint64_t epoch_ = 0U;
    std::optional<State> latest_state_;
    std::optional<double> last_state_receive_ros_s_;
    std::optional<double> last_pendulum_receive_ros_s_;
    std::optional<double> last_ros_now_s_;
    std::optional<double> last_reference_ros_s_;
    std::optional<Vec3> ground_anchor_;
    std::optional<Vec3> latest_magnet_tip_;
    std::optional<Vec3> takeoff_start_position_;
    std::optional<double> takeoff_start_ros_s_;
    std::optional<Vec3> hover_target_;
    std::optional<Vec3> global_goal_;
    std::optional<double> settle_started_ros_s_;
    std::optional<Vec3> fault_hold_position_;
    std::optional<dynamic_planner::ReferenceWindow> latest_reference_window_;
    std::deque<Vec3> measured_trace_;

    double pendulum_phi_rad_ = 0.0;
    double pendulum_theta_rad_ = 0.0;
    double pendulum_phi_dot_radps_ = 0.0;
    double pendulum_theta_dot_radps_ = 0.0;

    std::shared_ptr<VersionedWorld> world_;
    std::unique_ptr<RecedingHorizonPlanner> planner_;
    std::future<PlannerOutcome> planner_future_;
    std::optional<CommittedTrajectory> authoritative_commit_;
    ReplanResult latest_replan_result_;

    std::string reference_source_ = "WAITING_FOR_STATE";
    std::string latest_status_ = "NOT_STARTED";
    std::string fault_reason_;
    bool latest_state_fresh_ = false;
    bool latest_pendulum_fresh_ = false;
    double latest_state_age_s_ = -1.0;
    double latest_pendulum_age_s_ = -1.0;
    double latest_speed_mps_ = 0.0;
    double latest_accel_mps2_ = 0.0;
    double latest_swing_rad_ = 0.0;
    double latest_swing_rate_radps_ = 0.0;

    RollingStats reference_period_stats_;
    RollingStats replan_runtime_stats_;
    RollingStats octopus_runtime_stats_;
    RollingStats qp_runtime_stats_;

    std::size_t reference_publish_count_ = 0U;
    std::size_t reference_errors_ = 0U;
    std::size_t reference_skips_no_state_ = 0U;
    std::size_t accepted_candidates_ = 0U;
    std::size_t failed_candidates_ = 0U;
    std::size_t late_candidates_ = 0U;
    std::size_t coalesced_replan_triggers_ = 0U;
    std::size_t stale_state_replan_skips_ = 0U;
    std::size_t planner_worker_errors_ = 0U;
    std::size_t planner_period_overruns_ = 0U;
    std::size_t discarded_epoch_results_ = 0U;
    std::uint64_t replan_sequence_ = 0U;
    std::ofstream csv_;
    std::ofstream trace_csv_;
    std::ofstream replan_csv_;
    std::size_t trace_rows_since_flush_ = 0U;
    bool first_accepted_future_splice_verified_ = false;
    double first_accepted_finish_to_splice_margin_s_ = 0.0;
};

}  // namespace tejen_dynamic_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<tejen_dynamic_planner::ActiveCommissioningNode>();
        rclcpp::spin(node);
    } catch (const std::exception& exc) {
        std::cerr << "dynamic_planner_active_commissioning: FAIL: " << exc.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
