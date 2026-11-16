#include "tejen_dynamic_planner/kinematics_filter.hpp"
#include "tejen_dynamic_planner/reference_message.hpp"

#include "dynamic_planner/ego_collision_model.hpp"
#include "dynamic_planner/receding_horizon_planner.hpp"
#include "dynamic_planner/reference_window.hpp"
#include "dynamic_planner/world_snapshot.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <interfaces/msg/motion_capture_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <future>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
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
using dynamic_planner::WorldSnapshotSource;

constexpr double kControlReferenceHz = 30.0;
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kMpcAuthorityTopic = "/join_planner/reference";

struct StatsSummary {
    double mean = 0.0;
    double median = 0.0;
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
        const auto quantile = [&](double p) {
            const double index = p * static_cast<double>(sorted.size() - 1U);
            const auto lo = static_cast<std::size_t>(std::floor(index));
            const auto hi = static_cast<std::size_t>(std::ceil(index));
            const double frac = index - static_cast<double>(lo);
            return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
        };
        out.median = quantile(0.50);
        out.p95 = quantile(0.95);
        out.maximum = sorted.back();
        return out;
    }

private:
    std::size_t capacity_ = 2000U;
    std::deque<double> values_;
};

SuspendedGeometry suspendedGeometry() {
    SuspendedGeometry g;
    g.enabled = true;
    g.cable_length_m = 0.50;
    g.cable_radius_m = 0.0025;
    g.max_swing_angle_rad = 10.0 * kPi / 180.0;
    g.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    g.magnet_center_below_cable_end_m = 0.025;
    g.payload_attached = false;
    g.payload_half_extents = Vec3(0.05, 0.01, 0.003);
    g.payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.028);
    return g;
}

RecedingHorizonConfig recedingConfig(double splice_timing_seed_s) {
    RecedingHorizonConfig c;
    c.dc_s = 1.0 / kControlReferenceHz;
    c.planning_radius_m = 2.0;
    c.factor_alpha = 2.5;
    c.min_splice_lookahead_s = 0.05;
    c.max_splice_lookahead_s = 1.0;
    c.factor_alloc = 1.0;
    c.factor_alloc_close = 2.5;
    c.spline_time_factor = 2.5;
    c.close_to_goal_m = 0.20;
    c.goal_tolerance_m = 0.05;
    c.continuity_tolerance = 1e-7;
    c.separator_validation_tolerance = 1e-7;
    c.initial_splice_timing_s = std::max(0.0, splice_timing_seed_s);
    c.v_max = Vec3::Ones();
    c.a_max = Vec3(1.0, 1.0, 1.5);
    return c;
}

LocalPlannerConfig localConfig(
    const Vec3& start,
    const Vec3& goal,
    double minimum_search_z_m) {
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
    // C.1b/C.1c v3 runs stationary and disarmed at the actual Gazebo spawn
    // position. The older airborne-only 0.20 m floor made a perfectly valid
    // ground/spawn state fail immediately with INVALID_INITIAL_Q2. This bound
    // is commissioning-only and remains passive; it is not a flight-floor or
    // obstacle-clearance rule.
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

class DelayedWorldSource final : public WorldSnapshotSource {
public:
    DelayedWorldSource(WorldSnapshot initial, double delay_s)
        : world_(std::move(initial)), delay_s_(std::max(0.0, delay_s)) {}

    WorldSnapshot snapshot(double captured_at_s) const override {
        bool should_delay = false;
        {
            std::lock_guard<std::mutex> lock(delay_mutex_);
            if (!delay_consumed_ && delay_s_ > 0.0) {
                delay_consumed_ = true;
                should_delay = true;
            }
        }
        if (should_delay) {
            std::this_thread::sleep_for(std::chrono::duration<double>(delay_s_));
        }
        return world_.snapshot(captured_at_s);
    }

    std::uint64_t currentVersion() const override {
        return world_.currentVersion();
    }

    bool runIfVersionCurrent(
        std::uint64_t expected_version,
        const std::function<void()>& action) const override {
        return world_.runIfVersionCurrent(expected_version, action);
    }

private:
    VersionedWorld world_;
    double delay_s_ = 0.0;
    mutable std::mutex delay_mutex_;
    mutable bool delay_consumed_ = false;
};

struct ShadowOutcome {
    std::uint64_t epoch = 0U;
    double request_time_ros_s = 0.0;
    State measured_state;
    ReplanResult result;
    CommittedTrajectory committed;
    bool has_committed = false;
    bool late_preserved_hover = false;
};

std::string safeCsvField(std::string text) {
    std::replace(text.begin(), text.end(), ',', ';');
    return text;
}

}  // namespace

class PassiveCommissioningNode final : public rclcpp::Node {
public:
    PassiveCommissioningNode()
        : Node("dynamic_planner_passive_commissioning"),
          kinematics_(declare_parameter<double>("acceleration_filter_tau_s", 0.15)) {
        state_topic_ = declare_parameter<std::string>("state_topic", "/motion_capture_state");
        shadow_reference_topic_ = declare_parameter<std::string>(
            "shadow_reference_topic", "/dynamic_planner/shadow_reference");
        diagnostics_topic_ = declare_parameter<std::string>(
            "diagnostics_topic", "/dynamic_planner/c1bc_status");
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        vehicle_id_ = declare_parameter<std::string>("vehicle_id", "drone_0");
        goal_ = Vec3(
            declare_parameter<double>("goal_x", 3.0),
            declare_parameter<double>("goal_y", 0.0),
            declare_parameter<double>("goal_z", 1.5));
        state_timeout_s_ = declare_parameter<double>("state_timeout_s", 0.25);
        hover_speed_tolerance_mps_ = declare_parameter<double>(
            "hover_speed_tolerance_mps", 0.10);
        hover_accel_tolerance_mps2_ = declare_parameter<double>(
            "hover_accel_tolerance_mps2", 0.50);
        artificial_planning_delay_s_ = 1e-3 * declare_parameter<double>(
            "artificial_planning_delay_ms", 0.0);
        planner_rate_hz_ = declare_parameter<double>("planner_rate_hz", 5.0);
        stationary_shadow_state_ = declare_parameter<bool>("stationary_shadow_state", true);
        minimum_search_z_m_ = declare_parameter<double>("minimum_search_z_m", -0.05);
        csv_path_ = declare_parameter<std::string>(
            "csv_path", "/tmp/r6_3c1bc_passive.csv");

        if (shadow_reference_topic_ == kMpcAuthorityTopic) {
            throw std::invalid_argument(
                "C.1b/C.1c is PASSIVE ONLY: shadow_reference_topic must not be /join_planner/reference");
        }
        if (!goal_.allFinite() || !(state_timeout_s_ > 0.0) ||
            !(hover_speed_tolerance_mps_ >= 0.0) ||
            !(hover_accel_tolerance_mps2_ >= 0.0) ||
            !(artificial_planning_delay_s_ >= 0.0) ||
            !(planner_rate_hz_ > 0.0) ||
            planner_rate_hz_ > kControlReferenceHz ||
            !std::isfinite(minimum_search_z_m_)) {
            throw std::invalid_argument("invalid C.1b/C.1c commissioning parameters");
        }
        planner_period_s_ = 1.0 / planner_rate_hz_;

        reference_config_.reference_rate_hz = kControlReferenceHz;
        reference_config_.mpc_horizon_stages = 20U;
        reference_config_.mpc_skip_steps = 3U;
        reference_config_.validate();

        state_sub_ = create_subscription<interfaces::msg::MotionCaptureState>(
            state_topic_, rclcpp::QoS(10),
            std::bind(&PassiveCommissioningNode::stateCallback, this, std::placeholders::_1));
        shadow_reference_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            shadow_reference_topic_, rclcpp::QoS(10));
        // Self-subscription measures DDS/executor message age using the exact same
        // trajectory type that the MPC will later receive. It has no authority.
        shadow_reference_echo_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            shadow_reference_topic_, rclcpp::QoS(10),
            std::bind(&PassiveCommissioningNode::shadowReferenceEchoCallback,
                      this, std::placeholders::_1));
        diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
            diagnostics_topic_, rclcpp::QoS(10));

        // These timers use the node's ROS clock. The existing PayloadSimClean
        // stack does not provide a /clock stream, so C.1b/C.1c v3 defaults
        // use_sim_time=false and therefore follows the same live clock domain as
        // the rest of that workflow. Do not silently enable simulated time
        // unless the actual current stack has been inspected and /clock verified.
        reference_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(1.0 / kControlReferenceHz),
            std::bind(&PassiveCommissioningNode::referenceTimer, this));
        replan_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(planner_period_s_),
            std::bind(&PassiveCommissioningNode::replanTimer, this));
        diagnostics_timer_ = rclcpp::create_timer(
            get_node_base_interface(), get_node_timers_interface(), get_clock(),
            rclcpp::Duration::from_seconds(1.0),
            std::bind(&PassiveCommissioningNode::diagnosticsTimer, this));

        if (!csv_path_.empty()) {
            csv_.open(csv_path_);
            if (!csv_) {
                RCLCPP_WARN(get_logger(), "Could not open C.1b/C.1c CSV: %s", csv_path_.c_str());
            } else {
                csv_ << "ros_time,state_age_ms,speed_mps,accel_mps2,replan_status,"
                        "replan_runtime_ms,trajectory_elapsed_ms,octopus_ms,qp_ms,accepted,late,splice_lookahead_ms,"
                        "finish_to_splice_margin_ms,coalesced_triggers,reference_period_mean_ms,"
                        "reference_period_p95_ms,reference_period_max_ms,message_age_mean_ms,"
                        "message_age_p95_ms,message_age_max_ms,artificial_delay_ms,"
                        "late_preserved_hover,first_accepted_future_splice_verified,"
                        "planner_rate_hz,planner_period_overruns,stationary_shadow_state\n";
            }
        }

        RCLCPP_WARN(get_logger(),
            "R6.3C.1b/C.1c PASSIVE ONLY. Publishing shadow reference to %s; MPC authority topic %s is untouched.",
            shadow_reference_topic_.c_str(), kMpcAuthorityTopic);
        RCLCPP_INFO(get_logger(),
            "state=%s goal=[%.3f %.3f %.3f] reference=30 Hz planner=%.1f Hz samples=61 "
            "Octopus=9^3 stationary_shadow_state=%s minimum_search_z=%.3f m artificial_delay=%.1f ms",
            state_topic_.c_str(), goal_.x(), goal_.y(), goal_.z(), planner_rate_hz_,
            stationary_shadow_state_ ? "true" : "false", minimum_search_z_m_,
            1e3 * artificial_planning_delay_s_);
        RCLCPP_WARN(get_logger(),
            "C.1b/C.1c v3 commissioning is designed to run STATIONARY AND DISARMED. "
            "No hover-reference helper or MPC flight is required.");
    }

    ~PassiveCommissioningNode() override {
        if (shadow_future_.valid()) {
            shadow_future_.wait();
        }
        if (csv_) csv_.flush();
    }

private:
    void stateCallback(const interfaces::msg::MotionCaptureState::SharedPtr msg) {
        const double ros_now_s = get_clock()->now().seconds();
        const double header_stamp_s = rclcpp::Time(msg->header.stamp).seconds();

        const Vec3 position(
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        const Vec3 velocity(
            msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

        // Use this node's ROS-time receive epoch for planner kinematics. The
        // existing Gazebo/mocap publishers are not required to share the same
        // clock configuration as this commissioning node, so interpreting an
        // arbitrary producer header stamp as local simulated time could make a
        // perfectly fresh state appear billions of seconds old. The producer
        // stamp is still diagnosed below whenever it is comparable.
        const double sample_time_s = ros_now_s;
        if (std::isfinite(header_stamp_s) && header_stamp_s > 0.0 && ros_now_s > 0.0) {
            const double header_age_ms = 1e3 * (ros_now_s - header_stamp_s);
            if (std::abs(header_age_ms) <= 10000.0) {
                state_header_age_stats_.add(header_age_ms);
            } else {
                ++state_header_clock_mismatch_count_;
            }
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (last_state_receive_ros_s_.has_value() &&
            sample_time_s < *last_state_receive_ros_s_ - 1e-6) {
            ++time_epoch_;
            kinematics_.reset();
            latest_shadow_commit_.reset();
            splice_timing_seed_s_ = 0.0;
            RCLCPP_WARN(get_logger(),
                "Local ROS time moved backwards while receiving state; invalidated passive temporal state and restarted hover shadow.");
        }
        try {
            latest_state_ = kinematics_.update(position, velocity, sample_time_s);
            last_state_receive_ros_s_ = sample_time_s;
        } catch (const std::exception& exc) {
            RCLCPP_WARN(get_logger(), "Rejected kinematics sample: %s", exc.what());
        }
    }

    bool currentState(double ros_now_s, State* state, double* state_age_s) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_state_.has_value() || !last_state_receive_ros_s_.has_value()) return false;
        const double age = ros_now_s - *last_state_receive_ros_s_;
        if (!std::isfinite(age) || age < -0.02 || age > state_timeout_s_) return false;
        *state = *latest_state_;
        *state_age_s = std::max(0.0, age);
        return true;
    }

    void checkRosTimeJump(double ros_now_s) {
        if (last_ros_now_s_.has_value() && ros_now_s < *last_ros_now_s_ - 1e-6) {
            ++time_epoch_;
            latest_shadow_commit_.reset();
            splice_timing_seed_s_ = 0.0;
            reference_wall_period_stats_.clear();
            reference_ros_period_stats_.clear();
            message_age_stats_.clear();
            state_header_age_stats_.clear();
            last_reference_ros_s_.reset();
            RCLCPP_WARN(get_logger(),
                "ROS time moved backwards; discarded passive committed/cadence state. No MPC authority was affected.");
        }
        last_ros_now_s_ = ros_now_s;
    }

    void pollShadowFuture() {
        if (!shadow_future_.valid()) return;
        if (shadow_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;

        ShadowOutcome outcome = shadow_future_.get();
        if (outcome.epoch != time_epoch_) {
            ++discarded_epoch_results_;
            return;
        }

        latest_replan_result_ = outcome.result;
        latest_late_preserved_hover_ = outcome.late_preserved_hover;
        if (outcome.result.attempted) {
            splice_timing_seed_s_ = std::max(0.0, outcome.result.trajectory_elapsed_s);
            const double runtime_ms = 1e3 * outcome.result.replan_runtime_s;
            replan_runtime_stats_.add(runtime_ms);
            if (outcome.result.replan_runtime_s > planner_period_s_) {
                ++planner_period_overruns_;
            }
            if (outcome.result.local_plan.has_value()) {
                octopus_runtime_stats_.add(1e3 * outcome.result.local_plan->search.search_time_s);
                if (outcome.result.local_plan->refinement.has_value()) {
                    qp_runtime_stats_.add(
                        1e3 * outcome.result.local_plan->refinement->solve_time_s);
                }
            }
            if (outcome.result.accepted) {
                accepted_replan_runtime_stats_.add(runtime_ms);
                if (outcome.result.local_plan.has_value()) {
                    accepted_octopus_runtime_stats_.add(
                        1e3 * outcome.result.local_plan->search.search_time_s);
                }
            }
        }
        if (outcome.result.candidate_late) ++late_candidates_;
        if (outcome.result.accepted && outcome.has_committed) {
            ++accepted_candidates_;
            latest_shadow_commit_ = std::move(outcome.committed);
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
        pollShadowFuture();
        if (shadow_future_.valid()) {
            ++coalesced_replan_triggers_;
            return;
        }

        const double ros_now_s = get_clock()->now().seconds();
        checkRosTimeJump(ros_now_s);
        State measured;
        double age_s = 0.0;
        if (!currentState(ros_now_s, &measured, &age_s)) {
            ++stale_state_replan_skips_;
            return;
        }
        latest_state_age_s_ = age_s;
        latest_speed_mps_ = measured.velocity.norm();
        latest_accel_mps2_ = measured.acceleration.norm();
        if (latest_speed_mps_ > hover_speed_tolerance_mps_ ||
            latest_accel_mps2_ > hover_accel_tolerance_mps2_) {
            ++not_hovering_replan_skips_;
            return;
        }

        const std::uint64_t epoch = time_epoch_;
        const Vec3 goal = goal_;
        const double splice_timing_seed = splice_timing_seed_s_;
        const double delay_s = artificial_planning_delay_s_;
        auto ros_clock = get_clock();
        State state_snapshot = measured;
        if (stationary_shadow_state_) {
            // C.1b/C.1c v3 is a stationary/disarmed integration test. Use the
            // measured position, but deliberately seed zero velocity/acceleration
            // so mocap derivative noise does not turn a ground commissioning test
            // into a flight-dynamics test. C.1d must return to the true measured
            // state after the current controller plumbing is re-inspected.
            state_snapshot.velocity.setZero();
            state_snapshot.acceleration.setZero();
        }
        const double minimum_search_z_m = minimum_search_z_m_;

        shadow_future_ = std::async(std::launch::async,
            [epoch, goal, splice_timing_seed, delay_s, ros_clock, state_snapshot,
             ros_now_s, minimum_search_z_m]() mutable {
                ShadowOutcome outcome;
                outcome.epoch = epoch;
                outcome.request_time_ros_s = ros_now_s;
                outcome.measured_state = state_snapshot;

                WorldSnapshot world_snapshot;
                world_snapshot.captured_at_s = ros_now_s;
                world_snapshot.ego_half_extents = Vec3(0.105, 0.105, 0.060);
                world_snapshot.ego_suspended_geometry = suspendedGeometry();
                DelayedWorldSource world(std::move(world_snapshot), delay_s);

                RecedingHorizonPlanner planner(
                    goal, recedingConfig(splice_timing_seed),
                    localConfig(state_snapshot.position, goal, minimum_search_z_m));
                planner.initializeCommittedTrajectory(
                    dynamic_planner::makeStationaryHoverTrajectory(
                        state_snapshot.position, ros_now_s, 0.10));

                const TrajectoryTimeSource trajectory_clock = [ros_clock]() {
                    return ros_clock->now().seconds();
                };
                outcome.result = planner.replan(
                    ros_now_s, state_snapshot, world, trajectory_clock);

                if (!planner.committedTrajectory().empty()) {
                    outcome.committed = planner.committedTrajectory();
                    outcome.has_committed = true;
                }
                if (outcome.result.candidate_late && outcome.has_committed) {
                    const State at_finish = outcome.committed.evaluate(
                        outcome.result.candidate_finish_time_s);
                    outcome.late_preserved_hover =
                        (at_finish.position - state_snapshot.position).norm() <= 1e-9 &&
                        at_finish.velocity.norm() <= 1e-9 &&
                        at_finish.acceleration.norm() <= 1e-9;
                }
                return outcome;
            });
    }

    void referenceTimer() {
        pollShadowFuture();
        const double ros_now_s = get_clock()->now().seconds();
        checkRosTimeJump(ros_now_s);

        const auto steady_now = std::chrono::steady_clock::now();
        if (last_reference_steady_.has_value()) {
            const double period_ms = 1e3 * std::chrono::duration<double>(
                steady_now - *last_reference_steady_).count();
            reference_wall_period_stats_.add(period_ms);
        }
        last_reference_steady_ = steady_now;
        if (last_reference_ros_s_.has_value()) {
            reference_ros_period_stats_.add(1e3 * (ros_now_s - *last_reference_ros_s_));
        }
        last_reference_ros_s_ = ros_now_s;

        State measured;
        double age_s = 0.0;
        if (!currentState(ros_now_s, &measured, &age_s)) {
            ++stale_state_reference_skips_;
            return;
        }
        latest_state_age_s_ = age_s;
        latest_speed_mps_ = measured.velocity.norm();
        latest_accel_mps2_ = measured.acceleration.norm();

        try {
            CommittedTrajectory source;
            if (latest_shadow_commit_.has_value() &&
                ros_now_s >= latest_shadow_commit_->startTime() - 1e-9) {
                source = *latest_shadow_commit_;
                shadow_reference_source_ = "one_shot_candidate";
            } else {
                source = dynamic_planner::makeStationaryHoverTrajectory(
                    measured.position, ros_now_s, 0.10);
                shadow_reference_source_ = "measured_stationary";
            }
            const auto window = dynamic_planner::sampleCommittedReferenceWindow(
                source, ros_now_s, reference_config_);
            // ROS 2 Humble rclcpp::Time does not provide to_msg().  It does
            // provide an implicit conversion to builtin_interfaces::msg::Time.
            // Keep the publish timestamp in the same ROS clock domain as the
            // trajectory sampling epoch.
            const rclcpp::Time publish_time = get_clock()->now();
            const builtin_interfaces::msg::Time publish_stamp = publish_time;
            const auto msg = makeReferenceMessage(
                window, publish_stamp, frame_id_, vehicle_id_);
            shadow_reference_pub_->publish(msg);
            ++reference_publish_count_;
        } catch (const std::exception& exc) {
            ++reference_errors_;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Passive reference sampling failed: %s", exc.what());
        }
    }

    void shadowReferenceEchoCallback(
        const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg) {
        const double now_s = get_clock()->now().seconds();
        const double stamp_s = rclcpp::Time(msg->header.stamp).seconds();
        const double age_ms = 1e3 * (now_s - stamp_s);
        if (std::isfinite(age_ms) && age_ms >= -1.0) {
            message_age_stats_.add(std::max(0.0, age_ms));
        }
    }

    void diagnosticsTimer() {
        pollShadowFuture();
        const double now_s = get_clock()->now().seconds();
        State measured;
        double age_s = std::numeric_limits<double>::quiet_NaN();
        const bool state_fresh = currentState(now_s, &measured, &age_s);
        if (state_fresh) {
            latest_state_age_s_ = age_s;
            latest_speed_mps_ = measured.velocity.norm();
            latest_accel_mps2_ = measured.acceleration.norm();
        }

        const StatsSummary ref_ros = reference_ros_period_stats_.summary();
        const StatsSummary ref_wall = reference_wall_period_stats_.summary();
        const StatsSummary age = message_age_stats_.summary();
        const StatsSummary state_header_age = state_header_age_stats_.summary();
        const StatsSummary replans = replan_runtime_stats_.summary();
        const StatsSummary accepted_replans = accepted_replan_runtime_stats_.summary();
        const StatsSummary octopus = octopus_runtime_stats_.summary();
        const StatsSummary accepted_octopus = accepted_octopus_runtime_stats_.summary();
        const StatsSummary qp = qp_runtime_stats_.summary();

        std::ostringstream text;
        text << std::fixed << std::setprecision(3)
             << "R6.3C.1b/C.1c PASSIVE ONLY\n"
             << "state_fresh: " << std::boolalpha << state_fresh
             << " age_ms: " << (state_fresh ? 1e3 * age_s : -1.0)
             << " speed_mps: " << latest_speed_mps_
             << " accel_mps2: " << latest_accel_mps2_ << "\n"
             << "shadow_reference_topic: " << shadow_reference_topic_
             << " source: " << shadow_reference_source_
             << " published: " << reference_publish_count_ << "\n"
             << "reference ROS-time period mean/p95/max ms: "
             << ref_ros.mean << " / " << ref_ros.p95 << " / " << ref_ros.maximum << "\n"
             << "reference wall-time period mean/p95/max ms: "
             << ref_wall.mean << " / " << ref_wall.p95 << " / " << ref_wall.maximum << "\n"
             << "ROS shadow-message age mean/p95/max ms: "
             << age.mean << " / " << age.p95 << " / " << age.maximum << "\n"
             << "state header age mean/p95/max ms (when same clock): "
             << state_header_age.mean << " / " << state_header_age.p95 << " / "
             << state_header_age.maximum << " mismatched_clock_samples: "
             << state_header_clock_mismatch_count_ << "\n"
             << "replans accepted/failed/late: " << accepted_candidates_ << " / "
             << failed_candidates_ << " / " << late_candidates_ << "\n"
             << "coalesced replan triggers: " << coalesced_replan_triggers_ << "\n"
             << "replan skips stale/not-hovering: " << stale_state_replan_skips_ << " / "
             << not_hovering_replan_skips_ << " reference stale/errors: "
             << stale_state_reference_skips_ << " / " << reference_errors_ << "\n"
             << "runtime all-attempts mean/p95/max ms: " << replans.mean << " / "
             << replans.p95 << " / " << replans.maximum << "\n"
             << "runtime ACCEPTED mean/p95/max ms: " << accepted_replans.mean << " / "
             << accepted_replans.p95 << " / " << accepted_replans.maximum << "\n"
             << "planner target/period/overruns: " << planner_rate_hz_ << " Hz / "
             << 1e3 * planner_period_s_ << " ms / " << planner_period_overruns_ << "\n"
             << "latest trajectory-clock elapsed ms: "
             << 1e3 * latest_replan_result_.trajectory_elapsed_s << "\n"
             << "Octopus all-attempts mean/p95/max ms: " << octopus.mean << " / "
             << octopus.p95 << " / " << octopus.maximum << "\n"
             << "Octopus ACCEPTED mean/p95/max ms: " << accepted_octopus.mean << " / "
             << accepted_octopus.p95 << " / " << accepted_octopus.maximum << "\n"
             << "qpOASES mean/p95/max ms: " << qp.mean << " / "
             << qp.p95 << " / " << qp.maximum << "\n"
             << "latest status: " << latest_replan_result_.status
             << " splice_timing_seed_ms: " << 1e3 * splice_timing_seed_s_ << "\n"
             << "artificial planning delay ms: " << 1e3 * artificial_planning_delay_s_ << "\n"
             << "latest late preserved stationary incumbent: " << latest_late_preserved_hover_ << "\n"
             << "stationary/disarmed commissioning mode: " << stationary_shadow_state_ << "\n"
             << "first accepted future C2 splice verified: "
             << first_accepted_future_splice_verified_;
        if (first_accepted_future_splice_verified_) {
            text << " margin_ms: " << 1e3 * first_accepted_finish_to_splice_margin_s_;
        }
        text << "\nMPC authority changed: false";

        std_msgs::msg::String msg;
        msg.data = text.str();
        diagnostics_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "%s", msg.data.c_str());

        if (csv_) {
            double octopus_ms = 0.0;
            double qp_ms = 0.0;
            if (latest_replan_result_.local_plan.has_value()) {
                octopus_ms = 1e3 * latest_replan_result_.local_plan->search.search_time_s;
                if (latest_replan_result_.local_plan->refinement.has_value()) {
                    qp_ms = 1e3 * latest_replan_result_.local_plan->refinement->solve_time_s;
                }
            }
            const double margin_ms = latest_replan_result_.attempted
                ? 1e3 * (latest_replan_result_.splice_time_s -
                          latest_replan_result_.authority_decision_time_s)
                : 0.0;
            csv_ << std::setprecision(15)
                 << now_s << ','
                 << (state_fresh ? 1e3 * age_s : -1.0) << ','
                 << latest_speed_mps_ << ',' << latest_accel_mps2_ << ','
                 << safeCsvField(latest_replan_result_.status) << ','
                 << 1e3 * latest_replan_result_.replan_runtime_s << ','
                 << 1e3 * latest_replan_result_.trajectory_elapsed_s << ','
                 << octopus_ms << ',' << qp_ms << ','
                 << latest_replan_result_.accepted << ','
                 << latest_replan_result_.candidate_late << ','
                 << 1e3 * latest_replan_result_.splice_lookahead_s << ','
                 << margin_ms << ',' << coalesced_replan_triggers_ << ','
                 << ref_ros.mean << ',' << ref_ros.p95 << ',' << ref_ros.maximum << ','
                 << age.mean << ',' << age.p95 << ',' << age.maximum << ','
                 << 1e3 * artificial_planning_delay_s_ << ','
                 << latest_late_preserved_hover_ << ','
                 << first_accepted_future_splice_verified_ << ','
                 << planner_rate_hz_ << ','
                 << planner_period_overruns_ << ','
                 << stationary_shadow_state_ << '\n';
            csv_.flush();
        }
    }

    KinematicsFilter kinematics_;
    std::string state_topic_;
    std::string shadow_reference_topic_;
    std::string diagnostics_topic_;
    std::string frame_id_;
    std::string vehicle_id_;
    std::string csv_path_;
    Vec3 goal_ = Vec3::Zero();
    double state_timeout_s_ = 0.25;
    double hover_speed_tolerance_mps_ = 0.10;
    double hover_accel_tolerance_mps2_ = 0.50;
    double artificial_planning_delay_s_ = 0.0;
    double planner_rate_hz_ = 5.0;
    double planner_period_s_ = 0.20;
    bool stationary_shadow_state_ = true;
    double minimum_search_z_m_ = -0.05;
    dynamic_planner::ReferenceWindowConfig reference_config_;

    rclcpp::Subscription<interfaces::msg::MotionCaptureState>::SharedPtr state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr shadow_reference_pub_;
    rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr shadow_reference_echo_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr reference_timer_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    std::mutex state_mutex_;
    std::optional<State> latest_state_;
    std::optional<double> last_state_receive_ros_s_;
    std::optional<double> last_ros_now_s_;
    std::uint64_t time_epoch_ = 0U;

    std::future<ShadowOutcome> shadow_future_;
    std::optional<CommittedTrajectory> latest_shadow_commit_;
    ReplanResult latest_replan_result_;
    double splice_timing_seed_s_ = 0.0;
    bool latest_late_preserved_hover_ = false;
    bool first_accepted_future_splice_verified_ = false;
    double first_accepted_finish_to_splice_margin_s_ = 0.0;
    std::string shadow_reference_source_ = "measured_stationary";
    double latest_state_age_s_ = 0.0;
    double latest_speed_mps_ = 0.0;
    double latest_accel_mps2_ = 0.0;

    std::optional<std::chrono::steady_clock::time_point> last_reference_steady_;
    std::optional<double> last_reference_ros_s_;
    RollingStats reference_wall_period_stats_;
    RollingStats reference_ros_period_stats_;
    RollingStats message_age_stats_;
    RollingStats state_header_age_stats_;
    RollingStats replan_runtime_stats_;
    RollingStats accepted_replan_runtime_stats_;
    RollingStats octopus_runtime_stats_;
    RollingStats accepted_octopus_runtime_stats_;
    RollingStats qp_runtime_stats_;

    std::size_t reference_publish_count_ = 0U;
    std::size_t reference_errors_ = 0U;
    std::size_t coalesced_replan_triggers_ = 0U;
    std::size_t stale_state_replan_skips_ = 0U;
    std::size_t stale_state_reference_skips_ = 0U;
    std::size_t not_hovering_replan_skips_ = 0U;
    std::size_t discarded_epoch_results_ = 0U;
    std::size_t state_header_clock_mismatch_count_ = 0U;
    std::size_t accepted_candidates_ = 0U;
    std::size_t failed_candidates_ = 0U;
    std::size_t late_candidates_ = 0U;
    std::size_t planner_period_overruns_ = 0U;

    std::ofstream csv_;
};

}  // namespace tejen_dynamic_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<tejen_dynamic_planner::PassiveCommissioningNode>();
        rclcpp::spin(node);
    } catch (const std::exception& exc) {
        std::cerr << "dynamic_planner_passive_commissioning: FAIL: " << exc.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
