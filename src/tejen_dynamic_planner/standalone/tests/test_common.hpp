#pragma once

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

inline void requireTrue(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

inline void requireNear(double actual, double expected, double tolerance,
                        const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << message << ": actual=" << actual << " expected=" << expected
                  << " tolerance=" << tolerance << '\n';
        throw std::runtime_error(message);
    }
}

template <typename DerivedA, typename DerivedB>
inline void requireMatrixNear(const Eigen::MatrixBase<DerivedA>& actual,
                              const Eigen::MatrixBase<DerivedB>& expected,
                              double tolerance,
                              const std::string& message) {
    requireTrue(actual.rows() == expected.rows() && actual.cols() == expected.cols(),
                message + " shape mismatch");
    const double max_error = (actual - expected).cwiseAbs().maxCoeff();
    if (max_error > tolerance) {
        std::cerr << message << ": max_error=" << max_error
                  << " tolerance=" << tolerance << '\n';
        std::cerr << "actual:\n" << actual << "\nexpected:\n" << expected << '\n';
        throw std::runtime_error(message);
    }
}
