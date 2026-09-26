cmake_minimum_required(VERSION 3.16)
project(standalone_dynamic_planner_cpp VERSION 0.11.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_path(GLPK_INCLUDE_DIR glpk.h)
find_library(GLPK_LIBRARY NAMES glpk)
if(NOT GLPK_INCLUDE_DIR OR NOT GLPK_LIBRARY)
    message(FATAL_ERROR
        "GLPK development files were not found. On Ubuntu run: sudo apt install -y libglpk-dev")
endif()

find_path(QPOASES_INCLUDE_DIR
    NAMES qpOASES.hpp
    PATH_SUFFIXES qpOASES
)
find_library(QPOASES_LIBRARY NAMES qpOASES)
if(NOT QPOASES_INCLUDE_DIR OR NOT QPOASES_LIBRARY)
    message(FATAL_ERROR
        "qpOASES was not found. Expected the R6.3A setup install under ~/.local. "
        "Reconfigure with -DCMAKE_PREFIX_PATH=$HOME/.local after installing qpOASES 3.2.2.")
endif()

message(STATUS "Found qpOASES include: ${QPOASES_INCLUDE_DIR}")
message(STATUS "Found qpOASES library: ${QPOASES_LIBRARY}")

add_library(dynamic_planner_math
    src/bspline.cpp
    src/minvo.cpp
)

target_include_directories(dynamic_planner_math
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dynamic_planner_math
    PUBLIC
        Eigen3::Eigen
)

target_compile_options(dynamic_planner_math PRIVATE -Wall -Wextra -Wpedantic)


# R6.3B.4 - suspended whole-body geometry.
add_library(dynamic_planner_collision
    src/ego_collision_model.cpp
)

target_include_directories(dynamic_planner_collision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dynamic_planner_collision PUBLIC Eigen3::Eigen)
target_compile_options(dynamic_planner_collision PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_separator
    src/separator.cpp
)

target_include_directories(dynamic_planner_separator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${GLPK_INCLUDE_DIR}
)

target_link_libraries(dynamic_planner_separator
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        ${GLPK_LIBRARY}
)

target_compile_options(dynamic_planner_separator PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_octopus
    src/octopus_search.cpp
)

target_include_directories(dynamic_planner_octopus
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dynamic_planner_octopus
    PUBLIC
        dynamic_planner_math
        dynamic_planner_separator
        Eigen3::Eigen
)

target_compile_options(dynamic_planner_octopus PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_refinement
    src/trajectory_refinement.cpp
)

target_include_directories(dynamic_planner_refinement
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${QPOASES_INCLUDE_DIR}
)

target_link_libraries(dynamic_planner_refinement
    PUBLIC
        dynamic_planner_math
        dynamic_planner_separator
        dynamic_planner_octopus
        Eigen3::Eigen
    PRIVATE
        ${QPOASES_LIBRARY}
)

target_compile_options(dynamic_planner_refinement PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_validation_scenarios
    src/validation_scenarios.cpp
)

target_include_directories(dynamic_planner_validation_scenarios
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dynamic_planner_validation_scenarios
    PUBLIC
        dynamic_planner_octopus
        dynamic_planner_math
        Eigen3::Eigen
)

target_compile_options(dynamic_planner_validation_scenarios PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()

add_executable(test_ego_collision_model tests/test_ego_collision_model.cpp)
target_link_libraries(test_ego_collision_model PRIVATE dynamic_planner_collision)
add_test(NAME test_ego_collision_model COMMAND test_ego_collision_model)


add_executable(test_bspline tests/test_bspline.cpp)
target_link_libraries(test_bspline PRIVATE dynamic_planner_math)
add_test(NAME test_bspline COMMAND test_bspline)

add_executable(test_minvo tests/test_minvo.cpp)
target_link_libraries(test_minvo PRIVATE dynamic_planner_math)
add_test(NAME test_minvo COMMAND test_minvo)

add_executable(test_separator tests/test_separator.cpp)
target_link_libraries(test_separator PRIVATE dynamic_planner_separator)
add_test(NAME test_separator COMMAND test_separator)

add_executable(test_octopus tests/test_octopus.cpp)
target_link_libraries(test_octopus PRIVATE dynamic_planner_octopus dynamic_planner_collision)
add_test(NAME test_octopus COMMAND test_octopus)

add_executable(test_octopus_terminal_contract tests/test_octopus_terminal_contract.cpp)
target_link_libraries(test_octopus_terminal_contract PRIVATE dynamic_planner_octopus)
add_test(NAME test_octopus_terminal_contract COMMAND test_octopus_terminal_contract)

add_executable(test_target_predictor
    tests/test_target_predictor.cpp
    src/committed_trajectory.cpp)
target_link_libraries(test_target_predictor PRIVATE dynamic_planner_math Eigen3::Eigen)
add_test(NAME test_target_predictor COMMAND test_target_predictor)

add_executable(test_refinement tests/test_refinement.cpp)
target_link_libraries(test_refinement PRIVATE dynamic_planner_refinement dynamic_planner_validation_scenarios)
add_test(NAME test_refinement COMMAND test_refinement)

add_executable(math_parity_demo apps/math_parity_demo.cpp)
target_link_libraries(math_parity_demo PRIVATE dynamic_planner_math)

add_executable(separator_demo apps/separator_demo.cpp)
target_link_libraries(separator_demo PRIVATE dynamic_planner_separator)

add_executable(octopus_demo apps/octopus_demo.cpp)
target_link_libraries(octopus_demo PRIVATE dynamic_planner_octopus)

add_executable(refinement_demo apps/refinement_demo.cpp)
target_link_libraries(refinement_demo PRIVATE dynamic_planner_refinement dynamic_planner_validation_scenarios)

# R6.3B.1 - RMADER-style future-splice / committed-trajectory layer.
add_library(dynamic_planner_receding
    src/minimum_time.cpp
    src/frozen_world.cpp
    src/trajectory_geometry.cpp
    src/world_snapshot.cpp
    src/trajectory_safety_checker.cpp
    src/local_planner.cpp
    src/committed_trajectory.cpp
    src/receding_horizon_planner.cpp
)

target_include_directories(dynamic_planner_receding
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dynamic_planner_receding
    PUBLIC
        dynamic_planner_math
        dynamic_planner_separator
        dynamic_planner_octopus
        dynamic_planner_refinement
        dynamic_planner_collision
        Eigen3::Eigen
)

target_compile_options(dynamic_planner_receding PRIVATE -Wall -Wextra -Wpedantic)

# R6.3C.1a - exact rolling committed-trajectory sampling for the existing MPC.
add_library(dynamic_planner_reference
    src/reference_window.cpp
)

target_include_directories(dynamic_planner_reference
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dynamic_planner_reference
    PUBLIC
        dynamic_planner_receding
        Eigen3::Eigen
)

target_compile_options(dynamic_planner_reference PRIVATE -Wall -Wextra -Wpedantic)

add_executable(test_minimum_time tests/test_minimum_time.cpp)
target_link_libraries(test_minimum_time PRIVATE dynamic_planner_receding)
add_test(NAME test_minimum_time COMMAND test_minimum_time)

add_executable(test_committed_trajectory tests/test_committed_trajectory.cpp)
target_link_libraries(test_committed_trajectory PRIVATE dynamic_planner_receding)
add_test(NAME test_committed_trajectory COMMAND test_committed_trajectory)

add_executable(test_receding_horizon tests/test_receding_horizon.cpp)
target_link_libraries(test_receding_horizon PRIVATE dynamic_planner_receding)
add_test(NAME test_receding_horizon COMMAND test_receding_horizon)
set_tests_properties(test_receding_horizon PROPERTIES TIMEOUT 30)

add_executable(test_reference_window tests/test_reference_window.cpp)
target_link_libraries(test_reference_window PRIVATE dynamic_planner_reference)
add_test(NAME test_reference_window COMMAND test_reference_window)

# R6.3C.1b/C.1c - external trajectory clock, stationary incumbent and
# trajectory-time-seeded fresh-shadow timing contract.
add_executable(test_integration_timing tests/test_integration_timing.cpp)
target_link_libraries(test_integration_timing PRIVATE dynamic_planner_reference)
add_test(NAME test_integration_timing COMMAND test_integration_timing)
set_tests_properties(test_integration_timing PROPERTIES TIMEOUT 60)

# R6.3B.2 - versioned world Check/Recheck and atomic commit gate.
add_executable(test_world_snapshot tests/test_world_snapshot.cpp)
target_link_libraries(test_world_snapshot PRIVATE dynamic_planner_receding)
add_test(NAME test_world_snapshot COMMAND test_world_snapshot)

add_executable(test_trajectory_safety_checker tests/test_trajectory_safety_checker.cpp)
target_link_libraries(test_trajectory_safety_checker PRIVATE dynamic_planner_receding)
add_test(NAME test_trajectory_safety_checker COMMAND test_trajectory_safety_checker)

add_executable(test_recheck_commit tests/test_recheck_commit.cpp)
target_link_libraries(test_recheck_commit PRIVATE dynamic_planner_receding)
add_test(NAME test_recheck_commit COMMAND test_recheck_commit)
set_tests_properties(test_recheck_commit PROPERTIES TIMEOUT 30)

# R6.3B.4 whole suspended-system physical/C-space regressions.
add_executable(test_suspended_world tests/test_suspended_world.cpp)
target_link_libraries(test_suspended_world PRIVATE dynamic_planner_receding)
add_test(NAME test_suspended_world COMMAND test_suspended_world)


add_executable(receding_horizon_demo apps/receding_horizon_demo.cpp)
target_link_libraries(receding_horizon_demo PRIVATE dynamic_planner_receding)

add_executable(recheck_demo apps/recheck_demo.cpp)
target_link_libraries(recheck_demo PRIVATE dynamic_planner_receding)

# R6.3B.3 - fixed shared-trajectory cooperative mission commissioning harness.
add_executable(cooperative_mission_demo apps/cooperative_mission_demo.cpp)
target_link_libraries(cooperative_mission_demo PRIVATE dynamic_planner_receding)

# End-to-end commissioning regression. The demo itself returns non-zero unless
# goal liveness, continuous safety, C2 splice integrity and fixed-world authority
# invariants all hold. Path shape and path optimality are deliberately not fixed.
add_test(
    NAME test_cooperative_mission
    COMMAND ${CMAKE_COMMAND}
        -DDEMO=$<TARGET_FILE:cooperative_mission_demo>
        -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/b3_ctest
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_cooperative_mission_tests.cmake
)
set_tests_properties(test_cooperative_mission PROPERTIES TIMEOUT 180)

# Stronger B.3 commissioning set. Each scenario first re-runs the same planner in
# an empty world and requires that nominal trajectory to collide with the
# configured other-drone tube. The obstacle-aware mission must then still reach
# the goal safely.
add_test(
    NAME test_cooperative_stress
    COMMAND ${CMAKE_COMMAND}
        -DDEMO=$<TARGET_FILE:cooperative_mission_demo>
        -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/b3_stress_ctest
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_cooperative_stress_tests.cmake
)
set_tests_properties(test_cooperative_stress PROPERTIES TIMEOUT 240)

# R6.3B.4 - suspended-system commissioning harness.
add_executable(suspended_system_demo apps/suspended_system_demo.cpp)
target_link_libraries(suspended_system_demo PRIVATE dynamic_planner_receding)

add_test(
    NAME test_suspended_system
    COMMAND ${CMAKE_COMMAND}
        -DDEMO=$<TARGET_FILE:suspended_system_demo>
        -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/b4_suspended_ctest
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_suspended_system_tests.cmake
)
set_tests_properties(test_suspended_system PROPERTIES TIMEOUT 240)
