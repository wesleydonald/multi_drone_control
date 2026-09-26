#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "dynamic_planner/committed_trajectory.hpp"
#include "dynamic_planner/frozen_world.hpp"
#include "dynamic_planner/ego_collision_model.hpp"

namespace dynamic_planner {

struct CooperativeObstacleTrajectory {
    std::string name;
    CommittedTrajectory trajectory;
    Vec3 physical_half_extents = Vec3(0.105, 0.105, 0.060);
    // B.2 commissioning value. This is a project-specific bounded tracking
    // tube, not an RMADER-derived numerical guarantee. Replace with measured
    // tracking statistics before claiming an experimental bound.
    Vec3 tracking_error_half_extents = Vec3::Constant(0.05);
    // B.4: fixed/committed other drones may carry the same suspended hardware.
    SuspendedGeometry suspended_geometry;
    AttachedTetherGeometry attached_tether_geometry;

    void validate() const;
};

struct WorldSnapshot {
    std::uint64_t version = 0;
    double captured_at_s = 0.0;
    // Legacy B.1/B.3 pre-inflated static obstacles. Kept so old regressions remain
    // meaningful. B.4 static scenes should use physical_static_obstacles instead.
    std::vector<StaticConvexObstacle> static_obstacles;
    // B.4 physical obstacles are transformed component-by-component into ego C-space.
    std::vector<StaticConvexObstacle> physical_static_obstacles;
    std::vector<CooperativeObstacleTrajectory> cooperative_obstacles;
    Vec3 ego_half_extents = Vec3(0.105, 0.105, 0.060);
    // C1F.4 execution certificate: the planned ego geometry is inflated by the
    // maximum admitted closed-loop position tracking error. A trajectory is
    // only treated as certified while the measured ego remains inside this
    // axis-wise tube around the committed reference.
    Vec3 ego_tracking_error_half_extents = Vec3::Zero();
    SuspendedGeometry ego_suspended_geometry;

    void validate() const;

    // Build interval hulls aligned with the local planner's uniform spline
    // intervals. Cooperative trajectory hulls are exact cubic convex hulls,
    // inflated by physical body + tracking tube + ego body.
    std::vector<TimeIndexedObstacle> timeIndexedObstacles(
        int num_segments,
        double t_start_s,
        double t_end_s) const;
};

class WorldSnapshotSource {
public:
    virtual ~WorldSnapshotSource() = default;
    virtual WorldSnapshot snapshot(double captured_at_s) const = 0;
    virtual std::uint64_t currentVersion() const = 0;

    // Run a very short authority-change action while guaranteeing that no world
    // update can advance the version between the version comparison and the
    // action. False means the world changed and action was not run.
    virtual bool runIfVersionCurrent(
        std::uint64_t expected_version,
        const std::function<void()>& action) const = 0;
};

class VersionedWorld final : public WorldSnapshotSource {
public:
    VersionedWorld();
    explicit VersionedWorld(WorldSnapshot initial);

    WorldSnapshot snapshot(double captured_at_s) const override;
    std::uint64_t currentVersion() const override;
    bool runIfVersionCurrent(
        std::uint64_t expected_version,
        const std::function<void()>& action) const override;

    void setStaticObstacles(std::vector<StaticConvexObstacle> obstacles);
    void setPhysicalStaticObstacles(std::vector<StaticConvexObstacle> obstacles);
    void setEgoHalfExtents(const Vec3& half_extents);
    void setEgoTrackingErrorHalfExtents(const Vec3& half_extents);
    void setEgoSuspendedGeometry(const SuspendedGeometry& geometry);
    void upsertCooperativeTrajectory(CooperativeObstacleTrajectory obstacle);
    // Atomically upsert a coherent set of moving obstacles while advancing the
    // world version once. This prevents a multi-part physical object from
    // appearing to change 24 separate times during one planner scene update.
    void upsertCooperativeTrajectories(std::vector<CooperativeObstacleTrajectory> obstacles);
    bool removeCooperativeTrajectory(const std::string& name);

private:
    mutable std::mutex mutex_;
    std::uint64_t version_ = 0;
    std::vector<StaticConvexObstacle> static_obstacles_;
    std::vector<StaticConvexObstacle> physical_static_obstacles_;
    std::vector<CooperativeObstacleTrajectory> cooperative_obstacles_;
    Vec3 ego_half_extents_ = Vec3(0.105, 0.105, 0.060);
    Vec3 ego_tracking_error_half_extents_ = Vec3::Zero();
    SuspendedGeometry ego_suspended_geometry_;
};

}  // namespace dynamic_planner
