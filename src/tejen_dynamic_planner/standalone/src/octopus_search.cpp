#include "dynamic_planner/octopus_search.hpp"

#include "dynamic_planner/minvo.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <functional>
#include <memory>
#include <queue>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace dynamic_planner {

struct OctopusSearch::Node {
    Vec3 qi = Vec3::Zero();
    int index = 0;
    Node* previous = nullptr;
    double g = 0.0;
    double h = 0.0;
};

struct OctopusSearch::VoxelKey {
    long long x = 0;
    long long y = 0;
    long long z = 0;

    bool operator==(const VoxelKey& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct OctopusSearch::VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const noexcept {
        std::size_t seed = 0;
        const auto mix = [&seed](long long value) {
            const std::size_t h = std::hash<long long>{}(value);
            seed ^= h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
        };
        mix(key.x);
        mix(key.y);
        mix(key.z);
        return seed;
    }
};

namespace {

bool finitePositiveVector(const Vec3& value) {
    return value.allFinite() && (value.array() > 0.0).all();
}

bool finiteVector(const Vec3& value) {
    return value.allFinite();
}

}  // namespace

OctopusSearch::OctopusSearch(std::vector<double> knots,
                             ControlPoints initial_control_points,
                             Vec3 goal,
                             std::vector<TimeIndexedObstacle> obstacles,
                             OctopusConfig config,
                             OctopusStopPredicate stop_requested,
                             std::string stop_reason)
    : OctopusSearch(
          std::move(knots), std::move(initial_control_points), std::move(goal),
          std::move(obstacles), {}, std::move(config),
          std::move(stop_requested), std::move(stop_reason)) {}

OctopusSearch::OctopusSearch(std::vector<double> knots,
                             ControlPoints initial_control_points,
                             Vec3 goal,
                             std::vector<TimeIndexedObstacle> obstacles,
                             std::vector<TimeIndexedObstacle> terminal_hold_obstacles,
                             OctopusConfig config,
                             OctopusStopPredicate stop_requested,
                             std::string stop_reason)
    : OctopusSearch(std::move(knots), std::move(initial_control_points), std::move(goal),
                    std::move(obstacles), std::move(terminal_hold_obstacles),
                    TerminalBoundary{}, std::move(config),
                    std::move(stop_requested), std::move(stop_reason)) {}

OctopusSearch::OctopusSearch(std::vector<double> knots,
                             ControlPoints initial_control_points,
                             Vec3 goal,
                             std::vector<TimeIndexedObstacle> obstacles,
                             std::vector<TimeIndexedObstacle> terminal_hold_obstacles,
                             TerminalBoundary terminal_boundary,
                             OctopusConfig config,
                             OctopusStopPredicate stop_requested,
                             std::string stop_reason)
    : knots_(std::move(knots)),
      q012_(std::move(initial_control_points)),
      goal_(std::move(goal)),
      obstacles_(std::move(obstacles)),
      terminal_hold_obstacles_(std::move(terminal_hold_obstacles)),
      terminal_boundary_(std::move(terminal_boundary)),
      config_(std::move(config)),
      separator_(1e-7),
      stop_requested_(std::move(stop_requested)),
      stop_reason_(std::move(stop_reason)) {
    num_segments_ = static_cast<int>(knots_.size()) - 2 * kCubicDegree - 1;
    num_control_points_ = num_segments_ + kCubicDegree;
    // C1F.8d: STOPPED and MOVING_RENDEZVOUS have deterministic two-point
    // tails after q_(N-2). CONTINUATION fixes only q_N, so q_(N-1) is a real
    // search degree of freedom and terminal velocity remains genuinely free.
    final_search_index_ = terminal_boundary_.mode == TerminalMode::Continuation
        ? num_control_points_ - 2
        : num_control_points_ - 3;
    validateInputs();

    // C1F.5p: cache interval AABBs once.  AABB-disjoint convex sets are
    // provably separated, so both ordinary Octopus checks and terminal-hold
    // checks can bypass GLPK in those cases without changing feasibility.
    const auto cacheAabbs = [](const std::vector<TimeIndexedObstacle>& obstacles) {
        std::vector<std::vector<IntervalAabb>> result;
        result.reserve(obstacles.size());
        for (const auto& obstacle : obstacles) {
            std::vector<IntervalAabb> obstacle_aabbs;
            obstacle_aabbs.reserve(obstacle.interval_vertices.size());
            for (const auto& vertices : obstacle.interval_vertices) {
                IntervalAabb aabb;
                aabb.lower = vertices.colwise().minCoeff().transpose();
                aabb.upper = vertices.colwise().maxCoeff().transpose();
                obstacle_aabbs.push_back(aabb);
            }
            result.push_back(std::move(obstacle_aabbs));
        }
        return result;
    };
    obstacle_aabbs_ = cacheAabbs(obstacles_);
    terminal_hold_aabbs_ = cacheAabbs(terminal_hold_obstacles_);

    for (int ix = 0; ix < config_.samples_per_axis[0]; ++ix) {
        for (int iy = 0; iy < config_.samples_per_axis[1]; ++iy) {
            for (int iz = 0; iz < config_.samples_per_axis[2]; ++iz) {
                sample_combinations_.emplace_back(ix, iy, iz);
            }
        }
    }
    std::mt19937 generator(config_.random_seed);
    std::shuffle(sample_combinations_.begin(), sample_combinations_.end(), generator);

    voxel_size_ = computeVoxelSize();
    // Matches the Python R6.2 reference's source-faithful large q2-centred bbox.
    voxel_origin_ = q012_.row(2).transpose() - Vec3(1000.0, 1000.0, 1000.0);
}

void OctopusSearch::validateInputs() const {
    if (q012_.rows() != 3 || q012_.cols() != 3 || !q012_.allFinite()) {
        throw std::invalid_argument("initial_control_points must be finite q0,q1,q2 with shape (3,3)");
    }
    if (!finiteVector(goal_)) {
        throw std::invalid_argument("goal must be finite");
    }
    if (num_segments_ < 4) {
        throw std::invalid_argument("R4 Octopus requires at least four spline intervals");
    }
    if (static_cast<int>(knots_.size()) != num_control_points_ + kCubicDegree + 1) {
        throw std::invalid_argument("knot/control-point dimensions are inconsistent");
    }
    for (std::size_t i = 1; i < knots_.size(); ++i) {
        if (!std::isfinite(knots_[i]) || knots_[i] < knots_[i - 1]) {
            throw std::invalid_argument("knots must be finite and nondecreasing");
        }
    }
    if (!finitePositiveVector(config_.v_max) || !finitePositiveVector(config_.a_max)) {
        throw std::invalid_argument("v_max and a_max must be finite and positive");
    }
    if (!finiteVector(config_.xyz_min) || !finiteVector(config_.xyz_max) ||
        !(config_.xyz_max.array() > config_.xyz_min.array()).all()) {
        throw std::invalid_argument("xyz_max must exceed xyz_min componentwise");
    }
    for (int count : config_.samples_per_axis) {
        if (count < 3 || count % 2 == 0) {
            throw std::invalid_argument("samples_per_axis must contain odd integers >= 3");
        }
    }
    if (!(config_.alpha_shrink >= 0.0 && config_.alpha_shrink <= 1.0) ||
        !(config_.voxel_fraction >= 0.0 && config_.voxel_fraction <= 1.0)) {
        throw std::invalid_argument("alpha_shrink and voxel_fraction must lie in [0,1]");
    }
    if (!(config_.goal_tolerance_m >= 0.0) || !(config_.max_runtime_s > 0.0) ||
        !(config_.planning_radius_m > 0.0) || !std::isfinite(config_.heuristic_bias) ||
        (stop_requested_ && stop_reason_.empty())) {
        throw std::invalid_argument("goal/runtime/radius/bias configuration is invalid");
    }
    for (const auto& obstacle : obstacles_) {
        if (obstacle.name.empty()) {
            throw std::invalid_argument("obstacle name must be non-empty");
        }
        if (static_cast<int>(obstacle.interval_vertices.size()) != num_segments_) {
            throw std::invalid_argument("each obstacle must contain one hull per spline interval");
        }
        for (const auto& vertices : obstacle.interval_vertices) {
            if (vertices.rows() <= 0 || vertices.cols() != 3 || !vertices.allFinite()) {
                throw std::invalid_argument("obstacle hulls must have finite shape (N,3), N>=1");
            }
        }
    }
    for (const auto& obstacle : terminal_hold_obstacles_) {
        if (obstacle.name.empty() || obstacle.interval_vertices.empty()) {
            throw std::invalid_argument("terminal-hold obstacle requires a name and intervals");
        }
        for (const auto& vertices : obstacle.interval_vertices) {
            if (vertices.rows() <= 0 || vertices.cols() != 3 || !vertices.allFinite()) {
                throw std::invalid_argument("terminal-hold obstacle hulls must be finite (N,3)");
            }
        }
    }
}

Vec3 OctopusSearch::velocityBetween(const Vec3& q_prev,
                                    const Vec3& q_curr,
                                    int control_index_prev) const {
    const double denom =
        knots_.at(static_cast<std::size_t>(control_index_prev + kCubicDegree + 1)) -
        knots_.at(static_cast<std::size_t>(control_index_prev + 1));
    if (!(denom > 0.0)) {
        throw std::runtime_error("degenerate velocity knot interval");
    }
    return static_cast<double>(kCubicDegree) * (q_curr - q_prev) / denom;
}

std::optional<std::pair<Vec3, Vec3>> OctopusSearch::velocityBounds(
    int index,
    const Vec3& qi_m2,
    const Vec3& qi_m1,
    const Vec3& qi) const {
    // Source/Python parity: the previous-previous velocity is retained to mirror
    // RMADER's state semantics although the active acceleration box only needs v_{i-1}.
    const Vec3 vi_m2 = velocityBetween(qi_m2, qi_m1, index - 2);
    (void)vi_m2;
    const Vec3 vi_m1 = velocityBetween(qi_m1, qi, index - 1);

    Vec3 lower = -config_.v_max;
    Vec3 upper = config_.v_max;

    const Vec3 mean = 0.5 * (lower + upper);
    const Vec3 half = 0.5 * (upper - lower) * config_.alpha_shrink;
    lower = mean - half;
    upper = mean + half;

    const double d =
        (knots_.at(static_cast<std::size_t>(index + 3)) -
         knots_.at(static_cast<std::size_t>(index + 1))) /
        2.0;
    lower = lower.cwiseMax(vi_m1 - config_.a_max * d);
    upper = upper.cwiseMin(vi_m1 + config_.a_max * d);

    if (index == final_search_index_ - 1) {
        const double c =
            (knots_.at(static_cast<std::size_t>(final_search_index_ - 1 + 3 + 1)) -
             knots_.at(static_cast<std::size_t>(final_search_index_ - 1 + 2))) /
            2.0;
        if (terminal_boundary_.mode == TerminalMode::Stopped) {
            // Preserve the validated stopped-terminal predecessor bound exactly:
            // q_(N-1)=q_(N-2), so the next derivative control point is zero.
            lower = lower.cwiseMax(-config_.a_max * c);
            upper = upper.cwiseMin(config_.a_max * c);
        } else if (terminal_boundary_.mode == TerminalMode::Continuation) {
            // Here index=N-2 and the sampled derivative u chooses q_(N-1).
            // q_N is fixed at the local continuation waypoint, therefore the
            // terminal derivative w is affine in u. Enforce the exact final
            // velocity and acceleration boxes without fixing terminal velocity.
            const double u_span =
                knots_.at(static_cast<std::size_t>(index + kCubicDegree + 1)) -
                knots_.at(static_cast<std::size_t>(index + 1));
            const double w_span =
                knots_.at(static_cast<std::size_t>(index + kCubicDegree + 2)) -
                knots_.at(static_cast<std::size_t>(index + 2));
            if (!(u_span > 0.0) || !(w_span > 0.0) ||
                !terminal_boundary_.position.allFinite()) {
                return std::nullopt;
            }

            // q_(N-1) = qi + u_span/degree * u
            // w = degree*(q_N-q_(N-1))/w_span
            //   = affine_offset - affine_gain*u.
            const Vec3 affine_offset = static_cast<double>(kCubicDegree)
                * (terminal_boundary_.position - qi) / w_span;
            const double affine_gain = u_span / w_span;
            const double predecessor_gain = 1.0 + affine_gain;

            // |w-u| <= a_max*c and |w| <= v_max.
            lower = lower.cwiseMax(
                (affine_offset - config_.a_max * c) / predecessor_gain);
            upper = upper.cwiseMin(
                (affine_offset + config_.a_max * c) / predecessor_gain);
            lower = lower.cwiseMax(
                (affine_offset - config_.v_max) / affine_gain);
            upper = upper.cwiseMin(
                (affine_offset + config_.v_max) / affine_gain);
        } else {
            // Let u be the sampled derivative from q_(N-3) to q_(N-2).
            // The moving terminal tail fixes q_(N-1), so the following
            // derivative w is affine in u. Bound u against the exact completed
            // tail rather than against the stopped value w=0.
            const TerminalTailControlPoints tail = terminalTailControlPoints(
                qi, knots_, num_segments_, terminal_boundary_);
            const double u_span =
                knots_.at(static_cast<std::size_t>(index + kCubicDegree + 1)) -
                knots_.at(static_cast<std::size_t>(index + 1));
            const double w_span =
                knots_.at(static_cast<std::size_t>(index + kCubicDegree + 2)) -
                knots_.at(static_cast<std::size_t>(index + 2));
            const double next_acceleration_span =
                (knots_.at(static_cast<std::size_t>(index + kCubicDegree + 2)) -
                 knots_.at(static_cast<std::size_t>(index + 3))) /
                2.0;
            if (!(u_span > 0.0) || !(w_span > 0.0) ||
                !(next_acceleration_span > 0.0) ||
                (terminal_boundary_.velocity.cwiseAbs().array() >
                 config_.v_max.array() + 1e-9).any()) {
                return std::nullopt;
            }

            // w = affine_offset - affine_gain * u.
            const Vec3 affine_offset =
                static_cast<double>(kCubicDegree) * (tail.penultimate - qi) / w_span;
            const double affine_gain = u_span / w_span;
            const double predecessor_gain = 1.0 + affine_gain;

            // |w-u| <= a_max*c.
            lower = lower.cwiseMax(
                (affine_offset - config_.a_max * c) / predecessor_gain);
            upper = upper.cwiseMin(
                (affine_offset + config_.a_max * c) / predecessor_gain);

            // |w| <= v_max.
            lower = lower.cwiseMax(
                (affine_offset - config_.v_max) / affine_gain);
            upper = upper.cwiseMin(
                (affine_offset + config_.v_max) / affine_gain);

            // |v_f-w| <= a_max*c_next.
            const Vec3 next_delta = config_.a_max * next_acceleration_span;
            lower = lower.cwiseMax(
                (affine_offset - terminal_boundary_.velocity - next_delta) /
                affine_gain);
            upper = upper.cwiseMin(
                (affine_offset - terminal_boundary_.velocity + next_delta) /
                affine_gain);
        }
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (lower(axis) > upper(axis) || std::abs(upper(axis) - lower(axis)) < 1e-4) {
            return std::nullopt;
        }
    }
    return std::make_pair(lower, upper);
}

Vec3 OctopusSearch::nextQ(const Vec3& qi, int index, const Vec3& velocity) const {
    const double scale =
        (knots_.at(static_cast<std::size_t>(index + kCubicDegree + 1)) -
         knots_.at(static_cast<std::size_t>(index + 1))) /
        static_cast<double>(kCubicDegree);
    return qi + scale * velocity;
}

ControlPoints OctopusSearch::completedControlPoints(const Node* node) const {
    std::vector<Vec3> prefix = pathToNode(node);
    const bool partial = node->index < final_search_index_;

    if (partial) {
        // Any padded partial is the stopped evasive backup semantics, regardless
        // of the requested nominal terminal mode. Repeat the last actually
        // reached control point through q_N.
        while (static_cast<int>(prefix.size()) < num_control_points_) {
            prefix.push_back(prefix.back());
        }
        ControlPoints stopped(num_control_points_, 3);
        for (int i = 0; i < num_control_points_; ++i) {
            stopped.row(i) = prefix.at(static_cast<std::size_t>(i)).transpose();
        }
        return stopped;
    }

    if (terminal_boundary_.mode == TerminalMode::Continuation) {
        // q0..q_(N-1) were searched. The continuation contract fixes only q_N;
        // q_(N-1) remains free, hence so does terminal velocity.
        if (static_cast<int>(prefix.size()) != num_control_points_ - 1) {
            throw std::runtime_error("continuation completion requires q0 through q_(N-1)");
        }
        ControlPoints completed(num_control_points_, 3);
        for (int i = 0; i < num_control_points_ - 1; ++i) {
            completed.row(i) = prefix.at(static_cast<std::size_t>(i)).transpose();
        }
        completed.row(num_control_points_ - 1) = terminal_boundary_.position.transpose();
        return completed;
    }

    while (static_cast<int>(prefix.size()) < final_search_index_ + 1) {
        prefix.push_back(prefix.back());
    }
    ControlPoints prefix_array(final_search_index_ + 1, 3);
    for (int i = 0; i <= final_search_index_; ++i) {
        prefix_array.row(i) = prefix.at(static_cast<std::size_t>(i)).transpose();
    }
    return terminalCompletion(prefix_array, knots_, num_segments_, terminal_boundary_);
}

double OctopusSearch::computeVoxelSize() const {
    const auto bounds = velocityBounds(2,
                                       q012_.row(0).transpose(),
                                       q012_.row(1).transpose(),
                                       q012_.row(2).transpose());
    if (!bounds.has_value()) {
        throw std::invalid_argument("initial state has no dynamically feasible velocity-sampling interval");
    }

    const Vec3 lower = bounds->first;
    const Vec3 upper = bounds->second;
    Vec3 counts_minus_one;
    for (int axis = 0; axis < 3; ++axis) {
        counts_minus_one(axis) = static_cast<double>(config_.samples_per_axis[axis] - 1);
    }
    const Vec3 deltas = (upper - lower).cwiseQuotient(counts_minus_one);

    double min_distance = std::numeric_limits<double>::infinity();
    double max_distance = 0.0;
    bool found = false;
    for (int ix = 0; ix < config_.samples_per_axis[0]; ++ix) {
        for (int iy = 0; iy < config_.samples_per_axis[1]; ++iy) {
            for (int iz = 0; iz < config_.samples_per_axis[2]; ++iz) {
                const Vec3 velocity = lower + Vec3(static_cast<double>(ix),
                                                    static_cast<double>(iy),
                                                    static_cast<double>(iz))
                                                .cwiseProduct(deltas);
                if (velocity.norm() < 1e-6) {
                    continue;
                }
                const Vec3 q = nextQ(q012_.row(2).transpose(), 2, velocity);
                const double distance = (q - q012_.row(2).transpose()).norm();
                min_distance = std::min(min_distance, distance);
                max_distance = std::max(max_distance, distance);
                found = true;
            }
        }
    }
    if (!found) {
        throw std::invalid_argument("velocity sampling produced no non-zero initial neighbor");
    }
    return min_distance + config_.voxel_fraction * (max_distance - min_distance);
}

OctopusSearch::VoxelKey OctopusSearch::voxelKey(const Vec3& q) const {
    const Vec3 scaled = (q - voxel_origin_) / voxel_size_;
    return VoxelKey{
        std::llround(scaled.x()),
        std::llround(scaled.y()),
        std::llround(scaled.z()),
    };
}

std::vector<Vec3> OctopusSearch::pathToNode(const Node* node) const {
    std::vector<Vec3> reverse;
    for (const Node* cursor = node; cursor != nullptr; cursor = cursor->previous) {
        reverse.push_back(cursor->qi);
    }
    std::reverse(reverse.begin(), reverse.end());

    std::vector<Vec3> path;
    path.reserve(reverse.size() + 2U);
    path.push_back(q012_.row(0).transpose());
    path.push_back(q012_.row(1).transpose());
    path.insert(path.end(), reverse.begin(), reverse.end());
    return path;
}

bool OctopusSearch::controlHullSeparable(const FourPoints& control_points, int interval) {
    const FourPoints q_minvo = minvoVertices(control_points, interval, num_segments_);
    const Vec3 q_lower = q_minvo.colwise().minCoeff().transpose();
    const Vec3 q_upper = q_minvo.colwise().maxCoeff().transpose();
    for (std::size_t obstacle_index = 0; obstacle_index < obstacles_.size(); ++obstacle_index) {
        const auto& obstacle = obstacles_.at(obstacle_index);
        const auto& aabb = obstacle_aabbs_.at(obstacle_index).at(
            static_cast<std::size_t>(interval));
        bool disjoint = false;
        for (int axis = 0; axis < 3; ++axis) {
            disjoint = disjoint ||
                q_upper(axis) < aabb.lower(axis) - 1e-12 ||
                aabb.upper(axis) < q_lower(axis) - 1e-12;
        }
        if (disjoint) {
            ++aabb_separation_skips_;
            continue;
        }
        ++separator_calls_;
        const SeparationResult result = separator_.solve(
            q_minvo,
            obstacle.interval_vertices.at(static_cast<std::size_t>(interval)));
        if (!result.feasible) {
            return false;
        }
    }
    return true;
}

bool OctopusSearch::intervalSeparable(const Node* node) {
    if (node->index < 3) {
        return true;
    }

    const std::vector<Vec3> path = pathToNode(node);
    FourPoints last4;
    for (int row = 0; row < 4; ++row) {
        last4.row(row) = path.at(path.size() - 4U + static_cast<std::size_t>(row)).transpose();
    }
    const int interval = node->index - 3;
    if (!controlHullSeparable(last4, interval)) {
        return false;
    }

    if (node->index == final_search_index_) {
        // Search and final certification must reason about precisely the same
        // completion. STOPPED/MOVING_RENDEZVOUS have two unsearched terminal
        // intervals; CONTINUATION searches q_(N-1), leaving only the final one.
        const ControlPoints completed = completedControlPoints(node);
        for (int terminal_interval = interval + 1;
             terminal_interval < num_segments_; ++terminal_interval) {
            const FourPoints terminal = completed.middleRows(terminal_interval, 4);
            if (!controlHullSeparable(terminal, terminal_interval)) {
                return false;
            }
        }
    }
    return true;
}

bool OctopusSearch::terminalHoldViable(const Vec3& terminal_position) {
    if (terminal_hold_obstacles_.empty()) return true;
    FourPoints terminal;
    for (int row = 0; row < 4; ++row) {
        terminal.row(row) = terminal_position.transpose();
    }
    for (std::size_t obstacle_index = 0;
         obstacle_index < terminal_hold_obstacles_.size(); ++obstacle_index) {
        const auto& obstacle = terminal_hold_obstacles_.at(obstacle_index);
        const auto& aabbs = terminal_hold_aabbs_.at(obstacle_index);
        for (std::size_t interval_index = 0;
             interval_index < obstacle.interval_vertices.size(); ++interval_index) {
            const auto& vertices = obstacle.interval_vertices.at(interval_index);
            const auto& aabb = aabbs.at(interval_index);
            // Exact cheap prefilter: if the terminal point lies outside the
            // obstacle hull's AABB on any axis, the convex hull cannot contain
            // the point and no LP separator call is necessary. AABBs are cached
            // once in the constructor; only ambiguous overlap cases pay for the
            // exact separator.
            bool inside_aabb = true;
            for (int axis = 0; axis < 3; ++axis) {
                inside_aabb = inside_aabb &&
                    terminal_position(axis) >= aabb.lower(axis) - 1e-9 &&
                    terminal_position(axis) <= aabb.upper(axis) + 1e-9;
            }
            if (!inside_aabb) {
                ++aabb_separation_skips_;
                continue;
            }

            ++separator_calls_;
            ++terminal_hold_separator_calls_;
            const SeparationResult result = separator_.solve(terminal, vertices);
            if (!result.feasible) {
                ++terminal_hold_rejections_;
                return false;
            }
        }
    }
    return true;
}

std::pair<bool, std::vector<IntervalSeparator>> OctopusSearch::finalCheck(
    const ControlPoints& control_points) {
    std::vector<IntervalSeparator> separators;
    separators.reserve(static_cast<std::size_t>(num_segments_) * obstacles_.size());

    // Keep the exact separator result for every chosen interval/obstacle pair.
    // Refinement consumes these planes as fixed QP constraints, so unlike the
    // exploratory broad phase above we must materialize a separator here.
    for (int interval = 0; interval < num_segments_; ++interval) {
        FourPoints interval_cp;
        interval_cp = control_points.middleRows(interval, 4);
        const FourPoints q_minvo = minvoVertices(interval_cp, interval, num_segments_);
        for (const auto& obstacle : obstacles_) {
            ++separator_calls_;
            SeparationResult result = separator_.solve(
                q_minvo,
                obstacle.interval_vertices.at(static_cast<std::size_t>(interval)));
            separators.push_back(IntervalSeparator{obstacle.name, interval, result});
            if (!result.feasible) {
                return {false, std::move(separators)};
            }
        }
    }
    return {true, std::move(separators)};
}

bool OctopusSearch::finalDynamicsFeasible(const ControlPoints& control_points) const {
    const DerivativeSpline velocity =
        derivativeSpline(control_points, knots_, kCubicDegree, 1);
    const DerivativeSpline acceleration =
        derivativeSpline(control_points, knots_, kCubicDegree, 2);
    constexpr double tolerance = 1e-9;

    for (Eigen::Index row = 0; row < velocity.control_points.rows(); ++row) {
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(velocity.control_points(row, axis)) > config_.v_max(axis) + tolerance) {
                return false;
            }
        }
    }
    for (Eigen::Index row = 0; row < acceleration.control_points.rows(); ++row) {
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(acceleration.control_points(row, axis)) > config_.a_max(axis) + tolerance) {
                return false;
            }
        }
    }
    return true;
}

bool OctopusSearch::withinSearchLimits(const Vec3& q) const {
    return (q.array() >= config_.xyz_min.array()).all() &&
           (q.array() <= config_.xyz_max.array()).all() &&
           (q - q012_.row(0).transpose()).norm() < config_.planning_radius_m;
}

OctopusResult OctopusSearch::search() {
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    const auto elapsedSeconds = [&started]() {
        return std::chrono::duration<double>(Clock::now() - started).count();
    };

    separator_calls_ = 0;
    aabb_separation_skips_ = 0;
    terminal_hold_rejections_ = 0;
    terminal_hold_separator_calls_ = 0;
    separator_ = Separator(1e-7);

    OctopusResult result;
    result.voxel_size_m = voxel_size_;

    const Vec3 q0 = q012_.row(0).transpose();
    const Vec3 q1 = q012_.row(1).transpose();
    const Vec3 q2 = q012_.row(2).transpose();
    result.initial_q0 = q0;
    result.initial_q1 = q1;
    result.initial_q2 = q2;
    result.search_xyz_min = config_.xyz_min;
    result.search_xyz_max = config_.xyz_max;
    result.initial_q2_radius_m = (q2 - q0).norm();

    // The restart state can lie exactly on a configured boundary (notably the
    // minimum-search-z floor) after a certified stop. Admit only floating-point
    // residue at that boundary. Ordinary generated Octopus nodes continue to use
    // the exact withinSearchLimits() gate below.
    constexpr double kInitialQ2Tolerance = 1e-6;
    result.initial_q2_box_valid =
        (q2.array() >= (config_.xyz_min.array() - kInitialQ2Tolerance)).all() &&
        (q2.array() <= (config_.xyz_max.array() + kInitialQ2Tolerance)).all();
    result.initial_q2_radius_valid =
        result.initial_q2_radius_m <= config_.planning_radius_m + kInitialQ2Tolerance;
    if (!result.initial_q2_box_valid || !result.initial_q2_radius_valid) {
        result.status = "INVALID_INITIAL_Q2";
        if (!result.initial_q2_box_valid && !result.initial_q2_radius_valid) {
            result.termination_reason = "INVALID_INITIAL_Q2:BOX_AND_RADIUS";
        } else if (!result.initial_q2_box_valid) {
            result.termination_reason = "INVALID_INITIAL_Q2:BOX";
        } else {
            result.termination_reason = "INVALID_INITIAL_Q2:RADIUS";
        }
        result.terminal_hold_rejections = terminal_hold_rejections_;
        result.terminal_hold_separator_lp_calls = terminal_hold_separator_calls_;
        result.aabb_separation_skips = aabb_separation_skips_;
        result.search_time_s = elapsedSeconds();
        return result;
    }

    struct QueueEntry {
        double f = 0.0;
        double h = 0.0;
        std::uint64_t serial = 0;
        Node* node = nullptr;
    };
    struct CompareQueueEntry {
        bool operator()(const QueueEntry& a, const QueueEntry& b) const noexcept {
            if (a.f != b.f) {
                return a.f > b.f;
            }
            if (a.h != b.h) {
                return a.h > b.h;
            }
            return a.serial > b.serial;
        }
    };

    std::vector<std::unique_ptr<Node>> arena;
    arena.reserve(65536);
    const auto makeNode = [&arena](const Vec3& qi, int index, Node* previous,
                                   double g, double h) -> Node* {
        arena.push_back(std::make_unique<Node>());
        Node* node = arena.back().get();
        node->qi = qi;
        node->index = index;
        node->previous = previous;
        node->g = g;
        node->h = h;
        return node;
    };

    // A cubic B-spline search state's future is not determined by q_i alone.
    // The next interval after selecting q_(i+1) depends on
    // [q_(i-2), q_(i-1), q_i, q_(i+1)], and the admissible next-velocity box
    // depends on the same predecessor history.  Preserve that stage/history in
    // the closed-set identity instead of merging every node that merely lands
    // in the same current-position voxel.
    struct StateKey {
        int index = 0;
        VoxelKey qi_m2;
        VoxelKey qi_m1;
        VoxelKey qi;

        bool operator==(const StateKey& other) const noexcept {
            return index == other.index &&
                   qi_m2 == other.qi_m2 &&
                   qi_m1 == other.qi_m1 &&
                   qi == other.qi;
        }
    };
    struct StateKeyHash {
        std::size_t operator()(const StateKey& key) const noexcept {
            std::size_t seed = std::hash<int>{}(key.index);
            const VoxelKeyHash voxel_hash;
            const auto mix = [&seed](std::size_t value) {
                seed ^= value + 0x9e3779b97f4a7c15ULL +
                        (seed << 6U) + (seed >> 2U);
            };
            mix(voxel_hash(key.qi_m2));
            mix(voxel_hash(key.qi_m1));
            mix(voxel_hash(key.qi));
            return seed;
        }
    };

    const auto stateHistory = [this](const Node* node) {
        if (node == nullptr) {
            throw std::runtime_error("null Octopus node in state history");
        }

        std::array<Vec3, 3> history;
        if (node->index == 2) {
            history[0] = q012_.row(0).transpose();
            history[1] = q012_.row(1).transpose();
        } else if (node->index == 3) {
            if (node->previous == nullptr) {
                throw std::runtime_error("invalid Octopus parent chain at index 3");
            }
            history[0] = q012_.row(1).transpose();
            history[1] = node->previous->qi;
        } else {
            if (node->previous == nullptr || node->previous->previous == nullptr) {
                throw std::runtime_error("invalid Octopus parent chain");
            }
            history[0] = node->previous->previous->qi;
            history[1] = node->previous->qi;
        }
        history[2] = node->qi;
        return history;
    };

    const auto stateKey = [this, &stateHistory](const Node* node) {
        const auto history = stateHistory(node);
        return StateKey{
            node->index,
            voxelKey(history[0]),
            voxelKey(history[1]),
            voxelKey(history[2]),
        };
    };

    Node* start = makeNode(q2, 2, nullptr, 0.0, (q2 - goal_).norm());
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, CompareQueueEntry> queue;
    std::uint64_t serial = 0;
    queue.push(QueueEntry{start->g + config_.heuristic_bias * start->h,
                          start->h, serial, start});

    std::unordered_set<StateKey, StateKeyHash> visited;
    visited.reserve(8192);

    // If no complete candidate is found, keep a small bounded set of fallback
    // partial histories instead of betting the entire solve on one Euclidean-
    // closest partial. Preserve the historical immediate-stop behavior by
    // seeding this set with the valid start node q2 before queue exploration;
    // later expanded partials compete with it under the unchanged endpoint
    // distance-to-goal ranking. Certification remains unchanged and is performed
    // only after queue exploration terminates.
    constexpr std::size_t kPartialFallbackCandidateLimit = 8U;
    std::vector<std::pair<double, Node*>> partial_fallback_candidates;
    partial_fallback_candidates.reserve(kPartialFallbackCandidateLimit);
    const auto considerPartialFallback = [&](Node* node, double distance) {
        if (node == nullptr || node->index >= final_search_index_) {
            return;
        }
        const bool already_retained = std::any_of(
            partial_fallback_candidates.begin(), partial_fallback_candidates.end(),
            [node](const auto& entry) { return entry.second == node; });
        if (already_retained) {
            return;
        }
        partial_fallback_candidates.emplace_back(distance, node);
        std::stable_sort(
            partial_fallback_candidates.begin(), partial_fallback_candidates.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        if (partial_fallback_candidates.size() > kPartialFallbackCandidateLimit) {
            partial_fallback_candidates.resize(kPartialFallbackCandidateLimit);
        }
    };
    // Historical single-partial behavior initialized the fallback to q2 before
    // checking an invocation-level deadline. Keep that contract so an immediate
    // stop still returns the certified stopped start-state fallback.
    considerPartialFallback(start, start->h);

    Node* closest_complete = nullptr;
    double closest_complete_dist = std::numeric_limits<double>::infinity();
    Node* goal_node = nullptr;
    std::string status = "EMPTY_OPEN_LIST";

    while (!queue.empty()) {
        // Optional invocation-level stop hook. Receding-horizon integration uses
        // this for an execution-aware trajectory-time deadline while preserving
        // max_runtime_s as an independent steady-clock hard backstop.
        if (stop_requested_ && stop_requested_()) {
            status = stop_reason_;
            break;
        }
        if (elapsedSeconds() > config_.max_runtime_s) {
            status = "RUNTIME_REACHED";
            break;
        }

        const QueueEntry entry = queue.top();
        queue.pop();
        Node* current = entry.node;
        ++result.popped_nodes;
        const double dist = (current->qi - goal_).norm();

        const StateKey key = stateKey(current);
        if (visited.find(key) != visited.end() || !withinSearchLimits(current->qi)) {
            continue;
        }

        std::optional<std::pair<Vec3, Vec3>> bounds;
        if (current->index < final_search_index_) {
            const auto history = stateHistory(current);
            bounds = velocityBounds(
                current->index, history[0], history[1], history[2]);
            if (!bounds.has_value()) {
                continue;
            }
        }

        if (!intervalSeparable(current)) {
            continue;
        }

        visited.insert(key);
        ++result.expanded_nodes;

        const bool complete_node = current->index == final_search_index_;
        if (!complete_node) {
            considerPartialFallback(current, dist);
        }
        if (complete_node && terminal_boundary_.mode == TerminalMode::Stopped &&
            !terminalHoldViable(current->qi)) {
            continue;
        }
        if (complete_node && dist < closest_complete_dist - 1e-4) {
            if (closest_complete == nullptr) {
                result.first_complete_time_s = elapsedSeconds();
                result.first_complete_goal_distance_m = dist;
                result.first_complete_expanded_nodes = result.expanded_nodes;
                result.first_complete_popped_nodes = result.popped_nodes;
            }
            closest_complete = current;
            closest_complete_dist = dist;
            ++result.closest_complete_improvements;
        }
        if (complete_node &&
            (terminal_boundary_.mode == TerminalMode::Continuation ||
             dist < config_.goal_tolerance_m)) {
            // Continuation already fixes q_N exactly at the waypoint, so any
            // dynamically/collision-feasible q_(N-1) is a complete goal. The
            // queue heuristic softly prefers lower-cost q_(N-1) choices without
            // imposing an artificial near-zero terminal velocity tolerance.
            goal_node = current;
            status = "GOAL_REACHED";
            break;
        }
        if (complete_node) {
            continue;
        }

        const Vec3 lower = bounds->first;
        const Vec3 upper = bounds->second;
        Vec3 counts_minus_one;
        for (int axis = 0; axis < 3; ++axis) {
            counts_minus_one(axis) = static_cast<double>(config_.samples_per_axis[axis] - 1);
        }
        const Vec3 deltas = (upper - lower).cwiseQuotient(counts_minus_one);

        for (const auto& combination : sample_combinations_) {
            const auto [ix, iy, iz] = combination;
            const Vec3 velocity = lower + Vec3(static_cast<double>(ix),
                                                static_cast<double>(iy),
                                                static_cast<double>(iz))
                                            .cwiseProduct(deltas);
            if (velocity.norm() < 1e-5) {
                continue;
            }
            const Vec3 q_next = nextQ(current->qi, current->index, velocity);
            if (!withinSearchLimits(q_next)) {
                continue;
            }
            const double g = current->g + (q_next - current->qi).norm();
            const double h = (q_next - goal_).norm();
            Node* neighbor = makeNode(q_next, current->index + 1, current, g, h);
            ++serial;
            queue.push(QueueEntry{g + config_.heuristic_bias * h, h, serial, neighbor});
        }
    }

    result.termination_reason = status;
    result.complete_available_at_termination =
        goal_node != nullptr || closest_complete != nullptr;

    result.partial_fallback_candidates_retained = partial_fallback_candidates.size();

    // Complete candidates retain exact historical precedence and behavior. Only
    // the no-complete-solution fallback path is broadened to try several
    // already-explored partial histories.
    Node* chosen_complete = goal_node != nullptr ? goal_node : closest_complete;
    if (chosen_complete != nullptr) {
        const std::string selection_status =
            goal_node != nullptr ? "GOAL_REACHED" : "CLOSEST_COMPLETE";
        const ControlPoints completed = completedControlPoints(chosen_complete);
        result.diagnostic_control_points = completed;

        auto [final_feasible, separators] = finalCheck(completed);
        const bool final_dynamics_feasible =
            final_feasible && finalDynamicsFeasible(completed);
        const bool nonstopped_complete =
            terminal_boundary_.mode != TerminalMode::Stopped;
        const bool final_terminal_viable =
            final_feasible && final_dynamics_feasible &&
            (nonstopped_complete ||
             terminalHoldViable(completed.row(completed.rows() - 1).transpose()));
        const bool accepted =
            final_feasible && final_dynamics_feasible && final_terminal_viable;

        result.success = accepted;
        result.reached_goal = goal_node != nullptr;
        result.separators = std::move(separators);
        result.goal_distance_m =
            (completed.row(completed.rows() - 1).transpose()
             - terminal_boundary_.position).norm();
        if (terminal_boundary_.mode == TerminalMode::Stopped) {
            result.goal_distance_m =
                (completed.row(final_search_index_).transpose() - goal_).norm();
        }

        if (accepted) {
            result.status = selection_status;
            result.control_points = completed;
        } else if (final_feasible && !final_dynamics_feasible) {
            result.status = selection_status + "_FINAL_DYNAMICS_FAILED";
        } else if (final_feasible && final_dynamics_feasible && !final_terminal_viable) {
            result.status = selection_status + "_TERMINAL_HOLD_UNSAFE";
        } else {
            result.status = selection_status + "_FINAL_CHECK_FAILED";
        }
    } else {
        // Preserve the nearest partial's failed candidate/witness for RViz and
        // status compatibility if none of the retained alternatives certifies.
        std::optional<ControlPoints> closest_failed_completed;
        std::vector<IntervalSeparator> closest_failed_separators;
        bool closest_failed_final_feasible = false;
        bool closest_failed_dynamics_feasible = false;
        bool closest_failed_terminal_viable = false;

        for (std::size_t rank = 0; rank < partial_fallback_candidates.size(); ++rank) {
            Node* candidate = partial_fallback_candidates[rank].second;
            const ControlPoints completed = completedControlPoints(candidate);
            ++result.partial_fallback_candidates_tested;

            auto [final_feasible, separators] = finalCheck(completed);
            const bool final_dynamics_feasible =
                final_feasible && finalDynamicsFeasible(completed);
            const bool final_terminal_viable =
                final_feasible && final_dynamics_feasible &&
                terminalHoldViable(completed.row(completed.rows() - 1).transpose());
            const bool accepted =
                final_feasible && final_dynamics_feasible && final_terminal_viable;

            if (rank == 0U) {
                closest_failed_completed = completed;
                closest_failed_separators = separators;
                closest_failed_final_feasible = final_feasible;
                closest_failed_dynamics_feasible = final_dynamics_feasible;
                closest_failed_terminal_viable = final_terminal_viable;
            }

            if (!accepted) {
                continue;
            }

            result.success = true;
            result.reached_goal = false;
            result.status = "PADDED_CLOSEST_PARTIAL";
            result.control_points = completed;
            result.diagnostic_control_points = completed;
            result.separators = std::move(separators);
            result.partial_fallback_selected_rank = rank + 1U;
            result.goal_distance_m =
                (completed.row(final_search_index_).transpose() - goal_).norm();
            break;
        }

        if (!result.success) {
            if (!closest_failed_completed.has_value()) {
                result.status = status;
            } else {
                result.diagnostic_control_points = *closest_failed_completed;
                result.separators = std::move(closest_failed_separators);
                result.goal_distance_m =
                    ((*closest_failed_completed).row(final_search_index_).transpose() - goal_).norm();
                if (closest_failed_final_feasible && !closest_failed_dynamics_feasible) {
                    result.status = "PADDED_CLOSEST_PARTIAL_FINAL_DYNAMICS_FAILED";
                } else if (closest_failed_final_feasible &&
                           closest_failed_dynamics_feasible &&
                           !closest_failed_terminal_viable) {
                    result.status = "PADDED_CLOSEST_PARTIAL_TERMINAL_HOLD_UNSAFE";
                } else {
                    result.status = "PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED";
                }
            }
        }
    }

    result.separator_lp_calls = separator_calls_;
    result.aabb_separation_skips = aabb_separation_skips_;
    result.terminal_hold_rejections = terminal_hold_rejections_;
    result.terminal_hold_separator_lp_calls = terminal_hold_separator_calls_;
    result.search_time_s = elapsedSeconds();
    return result;
}

}  // namespace dynamic_planner
