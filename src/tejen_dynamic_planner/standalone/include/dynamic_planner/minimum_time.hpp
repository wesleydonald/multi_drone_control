#pragma once

#include "dynamic_planner/types.hpp"

namespace dynamic_planner {

// Literal C++ port of RMADER's constrained double-integrator minimum-time helper.
double minimumTimeDoubleIntegrator1D(double p0, double v0,
                                     double pf, double vf,
                                     double v_max, double a_max);

double minimumTimeDoubleIntegrator3D(const Vec3& p0, const Vec3& v0,
                                     const Vec3& pf, const Vec3& vf,
                                     const Vec3& v_max, const Vec3& a_max);

}  // namespace dynamic_planner
