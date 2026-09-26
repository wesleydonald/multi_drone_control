#include "dynamic_planner/world_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include "dynamic_planner/trajectory_geometry.hpp"

namespace dynamic_planner {
namespace {

void validateStaticObstacle(const StaticConvexObstacle& obstacle) {
    if (obstacle.name.empty() || obstacle.vertices.rows() < 1 ||
        obstacle.vertices.cols() != 3 || !obstacle.vertices.allFinite()) {
        throw std::invalid_argument("invalid static world obstacle");
    }
}

void validateHalfExtents(const Vec3& half, const char* label) {
    if (!half.allFinite() || (half.array() < 0.0).any()) {
        throw std::invalid_argument(std::string("invalid ") + label + " half extents");
    }
}

void validateUniqueNames(const std::vector<StaticConvexObstacle>& legacy,
                         const std::vector<StaticConvexObstacle>& physical,
                         const std::vector<CooperativeObstacleTrajectory>& cooperative) {
    std::unordered_set<std::string> names;
    for (const auto& obstacle : legacy) {
        validateStaticObstacle(obstacle);
        if (!names.insert(obstacle.name).second) {
            throw std::invalid_argument("duplicate obstacle name in world snapshot");
        }
    }
    for (const auto& obstacle : physical) {
        validateStaticObstacle(obstacle);
        if (!names.insert(obstacle.name).second) {
            throw std::invalid_argument("duplicate obstacle name in world snapshot");
        }
    }
    for (const auto& obstacle : cooperative) {
        obstacle.validate();
        if (!names.insert(obstacle.name).second) {
            throw std::invalid_argument("duplicate obstacle name in world snapshot");
        }
    }
}

Eigen::MatrixXd shiftedVertices(const Eigen::MatrixXd& vertices, const Vec3& shift) {
    Eigen::MatrixXd result = vertices;
    result.rowwise() += shift.transpose();
    return result;
}

}  // namespace

void CooperativeObstacleTrajectory::validate() const {
    if (name.empty() || trajectory.empty()) {
        throw std::invalid_argument("cooperative obstacle requires name and trajectory");
    }
    validateHalfExtents(physical_half_extents, "cooperative physical");
    validateHalfExtents(tracking_error_half_extents, "cooperative tracking-error");
    suspended_geometry.validate();
    attached_tether_geometry.validate();
}

void WorldSnapshot::validate() const {
    if (!std::isfinite(captured_at_s)) {
        throw std::invalid_argument("world snapshot capture time must be finite");
    }
    validateHalfExtents(ego_half_extents, "ego");
    validateHalfExtents(ego_tracking_error_half_extents, "ego tracking-error");
    ego_suspended_geometry.validate();
    validateUniqueNames(static_obstacles, physical_static_obstacles, cooperative_obstacles);
}

std::vector<TimeIndexedObstacle> WorldSnapshot::timeIndexedObstacles(
    int num_segments,
    double t_start_s,
    double t_end_s) const {
    validate();
    if (num_segments < 1 || !std::isfinite(t_start_s) || !std::isfinite(t_end_s) ||
        !(t_end_s > t_start_s)) {
        throw std::invalid_argument("invalid world time-index request");
    }

    const auto ego_components = assemblyComponents(
        ego_half_extents, ego_suspended_geometry);

    std::vector<TimeIndexedObstacle> result;
    const std::size_t rough_component_count = ego_components.size();
    result.reserve(static_obstacles.size() +
                   physical_static_obstacles.size() * rough_component_count +
                   cooperative_obstacles.size() * rough_component_count * 4U);

    // Legacy pre-inflated B.1/B.3 obstacles.
    for (const auto& obstacle : static_obstacles) {
        TimeIndexedObstacle timed;
        timed.name = obstacle.name;
        timed.interval_vertices.assign(
            static_cast<std::size_t>(num_segments), obstacle.vertices);
        result.push_back(std::move(timed));
    }

    // B.4 physical static obstacles. Keep the actual component polytope here,
    // rather than replacing the suspended system by one giant body inflation.
    for (const auto& obstacle : physical_static_obstacles) {
        for (const auto& ego_component : ego_components) {
            TimeIndexedObstacle timed;
            timed.name = obstacle.name + "::ego_" + ego_component.name;
            ConvexComponent tracked_ego = ego_component;
            tracked_ego.vertices = inflateVerticesByAabb(
                tracked_ego.vertices, ego_tracking_error_half_extents);
            const Eigen::MatrixXd cspace = physicalObstacleToConfigurationSpace(
                obstacle.vertices, tracked_ego);
            timed.interval_vertices.assign(
                static_cast<std::size_t>(num_segments), cspace);
            result.push_back(std::move(timed));
        }
    }

    const double dt = (t_end_s - t_start_s) / static_cast<double>(num_segments);
    for (const auto& obstacle : cooperative_obstacles) {
        // C1F.5p: the cooperative centre hull depends only on the advertised
        // trajectory and time interval.  It does NOT depend on which ego/other
        // suspended component pair is being checked.  The original C1F.5 code
        // rebuilt the exact same cubic hull inside every component-pair loop,
        // which multiplied spline evaluation work by the number of assembly
        // components.  Cache each interval centre hull once per cooperative
        // obstacle and reuse it below.  This is an exact computation-elimination
        // optimisation: the returned C-space vertices are unchanged.
        std::vector<Eigen::MatrixXd> center_hulls;
        center_hulls.reserve(static_cast<std::size_t>(num_segments));
        for (int i = 0; i < num_segments; ++i) {
            const double a = t_start_s + static_cast<double>(i) * dt;
            const double b = (i + 1 == num_segments)
                ? t_end_s
                : t_start_s + static_cast<double>(i + 1) * dt;
            center_hulls.push_back(convexCurveHullVertices(obstacle.trajectory, a, b));
        }

        // Preserve the validated B.3 exact body-only path byte-for-byte in
        // semantics and naming when neither assembly has suspended geometry.
        if (!ego_suspended_geometry.enabled && !obstacle.suspended_geometry.enabled &&
            !obstacle.attached_tether_geometry.enabled) {
            TimeIndexedObstacle timed;
            timed.name = obstacle.name;
            timed.interval_vertices.reserve(static_cast<std::size_t>(num_segments));
            const Vec3 inflation = ego_half_extents + ego_tracking_error_half_extents +
                                   obstacle.physical_half_extents +
                                   obstacle.tracking_error_half_extents;
            for (int i = 0; i < num_segments; ++i) {
                timed.interval_vertices.push_back(
                    inflateVerticesByAabb(
                        center_hulls.at(static_cast<std::size_t>(i)), inflation));
            }
            result.push_back(std::move(timed));
            continue;
        }

        const auto other_components = cooperativeAssemblyComponents(
            obstacle.physical_half_extents, obstacle.suspended_geometry,
            obstacle.attached_tether_geometry);
        for (const auto& other_component : other_components) {
            Vec3 other_center;
            Vec3 other_half;
            componentAabb(other_component, &other_center, &other_half);
            for (const auto& ego_component : ego_components) {
                Vec3 ego_center;
                Vec3 ego_half;
                componentAabb(ego_component, &ego_center, &ego_half);

                TimeIndexedObstacle timed;
                timed.name = obstacle.name + "::" + other_component.name +
                             "_vs_ego_" + ego_component.name;
                timed.interval_vertices.reserve(static_cast<std::size_t>(num_segments));

                // For fixed/shared other-drone trajectories, B.4 keeps the exact
                // cubic centre hull time-indexed. Component-pair geometry is an
                // efficient AABB bound around each convex relative component; this
                // avoids vertex explosion while still preserving body/cable/magnet/
                // payload as separate collision relations.
                const Vec3 relative_shift = other_center - ego_center;
                const Vec3 pair_half = other_half + ego_half +
                                       ego_tracking_error_half_extents +
                                       obstacle.tracking_error_half_extents;
                for (int i = 0; i < num_segments; ++i) {
                    timed.interval_vertices.push_back(inflateVerticesByAabb(
                        shiftedVertices(
                            center_hulls.at(static_cast<std::size_t>(i)), relative_shift),
                        pair_half));
                }
                result.push_back(std::move(timed));
            }
        }
    }
    return result;
}

VersionedWorld::VersionedWorld() = default;

VersionedWorld::VersionedWorld(WorldSnapshot initial)
    : version_(initial.version),
      static_obstacles_(std::move(initial.static_obstacles)),
      physical_static_obstacles_(std::move(initial.physical_static_obstacles)),
      cooperative_obstacles_(std::move(initial.cooperative_obstacles)),
      ego_half_extents_(initial.ego_half_extents),
      ego_tracking_error_half_extents_(initial.ego_tracking_error_half_extents),
      ego_suspended_geometry_(initial.ego_suspended_geometry) {
    initial.static_obstacles = static_obstacles_;
    initial.physical_static_obstacles = physical_static_obstacles_;
    initial.cooperative_obstacles = cooperative_obstacles_;
    initial.ego_half_extents = ego_half_extents_;
    initial.ego_tracking_error_half_extents = ego_tracking_error_half_extents_;
    initial.ego_suspended_geometry = ego_suspended_geometry_;
    initial.captured_at_s = 0.0;
    initial.validate();
}

WorldSnapshot VersionedWorld::snapshot(double captured_at_s) const {
    if (!std::isfinite(captured_at_s)) {
        throw std::invalid_argument("world snapshot capture time must be finite");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    WorldSnapshot result;
    result.version = version_;
    result.captured_at_s = captured_at_s;
    result.static_obstacles = static_obstacles_;
    result.physical_static_obstacles = physical_static_obstacles_;
    result.cooperative_obstacles = cooperative_obstacles_;
    result.ego_half_extents = ego_half_extents_;
    result.ego_tracking_error_half_extents = ego_tracking_error_half_extents_;
    result.ego_suspended_geometry = ego_suspended_geometry_;
    result.validate();
    return result;
}

std::uint64_t VersionedWorld::currentVersion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return version_;
}

bool VersionedWorld::runIfVersionCurrent(
    std::uint64_t expected_version,
    const std::function<void()>& action) const {
    if (!action) {
        throw std::invalid_argument("commit action must be callable");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (version_ != expected_version) {
        return false;
    }
    action();
    return true;
}

void VersionedWorld::setStaticObstacles(std::vector<StaticConvexObstacle> obstacles) {
    std::unordered_set<std::string> names;
    for (const auto& obstacle : obstacles) {
        validateStaticObstacle(obstacle);
        if (!names.insert(obstacle.name).second) {
            throw std::invalid_argument("duplicate static obstacle name");
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& physical : physical_static_obstacles_) {
        if (names.count(physical.name) != 0U) {
            throw std::invalid_argument("legacy/physical static obstacle name collision");
        }
    }
    for (const auto& cooperative : cooperative_obstacles_) {
        if (names.count(cooperative.name) != 0U) {
            throw std::invalid_argument("static/cooperative obstacle name collision");
        }
    }
    static_obstacles_ = std::move(obstacles);
    ++version_;
}

void VersionedWorld::setPhysicalStaticObstacles(
    std::vector<StaticConvexObstacle> obstacles) {
    std::unordered_set<std::string> names;
    for (const auto& obstacle : obstacles) {
        validateStaticObstacle(obstacle);
        if (!names.insert(obstacle.name).second) {
            throw std::invalid_argument("duplicate physical static obstacle name");
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& legacy : static_obstacles_) {
        if (names.count(legacy.name) != 0U) {
            throw std::invalid_argument("legacy/physical static obstacle name collision");
        }
    }
    for (const auto& cooperative : cooperative_obstacles_) {
        if (names.count(cooperative.name) != 0U) {
            throw std::invalid_argument("physical/cooperative obstacle name collision");
        }
    }
    physical_static_obstacles_ = std::move(obstacles);
    ++version_;
}

void VersionedWorld::setEgoHalfExtents(const Vec3& half_extents) {
    validateHalfExtents(half_extents, "ego");
    std::lock_guard<std::mutex> lock(mutex_);
    ego_half_extents_ = half_extents;
    ++version_;
}

void VersionedWorld::setEgoTrackingErrorHalfExtents(const Vec3& half_extents) {
    validateHalfExtents(half_extents, "ego tracking-error");
    std::lock_guard<std::mutex> lock(mutex_);
    ego_tracking_error_half_extents_ = half_extents;
    ++version_;
}

void VersionedWorld::setEgoSuspendedGeometry(const SuspendedGeometry& geometry) {
    geometry.validate();
    std::lock_guard<std::mutex> lock(mutex_);
    ego_suspended_geometry_ = geometry;
    ++version_;
}

void VersionedWorld::upsertCooperativeTrajectory(CooperativeObstacleTrajectory obstacle) {
    obstacle.validate();
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& fixed : static_obstacles_) {
        if (fixed.name == obstacle.name) {
            throw std::invalid_argument("static/cooperative obstacle name collision");
        }
    }
    for (const auto& fixed : physical_static_obstacles_) {
        if (fixed.name == obstacle.name) {
            throw std::invalid_argument("physical/cooperative obstacle name collision");
        }
    }
    auto it = std::find_if(
        cooperative_obstacles_.begin(), cooperative_obstacles_.end(),
        [&](const CooperativeObstacleTrajectory& existing) {
            return existing.name == obstacle.name;
        });
    if (it == cooperative_obstacles_.end()) {
        cooperative_obstacles_.push_back(std::move(obstacle));
    } else {
        *it = std::move(obstacle);
    }
    ++version_;
}

void VersionedWorld::upsertCooperativeTrajectories(
    std::vector<CooperativeObstacleTrajectory> obstacles) {
    if (obstacles.empty()) return;

    std::unordered_set<std::string> incoming_names;
    for (const auto& obstacle : obstacles) {
        obstacle.validate();
        if (!incoming_names.insert(obstacle.name).second) {
            throw std::invalid_argument("duplicate cooperative obstacle name in batch");
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& obstacle : obstacles) {
        for (const auto& fixed : static_obstacles_) {
            if (fixed.name == obstacle.name) {
                throw std::invalid_argument("static/cooperative obstacle name collision");
            }
        }
        for (const auto& fixed : physical_static_obstacles_) {
            if (fixed.name == obstacle.name) {
                throw std::invalid_argument("physical/cooperative obstacle name collision");
            }
        }
    }

    for (auto& obstacle : obstacles) {
        auto it = std::find_if(
            cooperative_obstacles_.begin(), cooperative_obstacles_.end(),
            [&](const CooperativeObstacleTrajectory& existing) {
                return existing.name == obstacle.name;
            });
        if (it == cooperative_obstacles_.end()) {
            cooperative_obstacles_.push_back(std::move(obstacle));
        } else {
            *it = std::move(obstacle);
        }
    }
    ++version_;
}

bool VersionedWorld::removeCooperativeTrajectory(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto old_size = cooperative_obstacles_.size();
    cooperative_obstacles_.erase(
        std::remove_if(
            cooperative_obstacles_.begin(), cooperative_obstacles_.end(),
            [&](const CooperativeObstacleTrajectory& obstacle) {
                return obstacle.name == name;
            }),
        cooperative_obstacles_.end());
    if (cooperative_obstacles_.size() != old_size) {
        ++version_;
        return true;
    }
    return false;
}

}  // namespace dynamic_planner
