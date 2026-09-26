cmake_minimum_required(VERSION 3.16)
project(tejen_dynamic_planner VERSION 0.5.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# C1F.8: this package contains the performance-critical standalone planner core.
# A fresh single-config CMake build otherwise defaults to an unoptimised build
# when CMAKE_BUILD_TYPE is omitted. Keep debug symbols while enabling normal
# compiler optimisation for commissioning/runtime builds. Explicit caller
# choices (for example Debug) are preserved.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
        "Build type for tejen_dynamic_planner" FORCE)
endif()
message(STATUS "tejen_dynamic_planner build type: ${CMAKE_BUILD_TYPE}")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# C.1b/C.1c compiles the exact source files from the validated standalone tree.
# Nothing is copied/reimplemented inside the ROS package.
set(DYNAMIC_PLANNER_CORE_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/standalone"
    CACHE PATH "Path to the validated standalone dynamic planner source tree")

foreach(required_file
    include/dynamic_planner/receding_horizon_planner.hpp
    include/dynamic_planner/reference_window.hpp
    src/bspline.cpp
    src/minvo.cpp
    src/separator.cpp
    src/octopus_search.cpp
    src/trajectory_refinement.cpp
    src/ego_collision_model.cpp
    src/minimum_time.cpp
    src/frozen_world.cpp
    src/trajectory_geometry.cpp
    src/world_snapshot.cpp
    src/trajectory_safety_checker.cpp
    src/local_planner.cpp
    src/committed_trajectory.cpp
    src/receding_horizon_planner.cpp
    src/reference_window.cpp)
    if(NOT EXISTS "${DYNAMIC_PLANNER_CORE_ROOT}/${required_file}")
        message(FATAL_ERROR
            "tejen_dynamic_planner requires the cumulative C.1b/C.1c standalone core at "
            "${DYNAMIC_PLANNER_CORE_ROOT}; missing ${required_file}")
    endif()
endforeach()

find_path(GLPK_INCLUDE_DIR glpk.h)
find_library(GLPK_LIBRARY NAMES glpk)
if(NOT GLPK_INCLUDE_DIR OR NOT GLPK_LIBRARY)
    message(FATAL_ERROR "GLPK development files not found. Install libglpk-dev.")
endif()

find_path(QPOASES_INCLUDE_DIR
    NAMES qpOASES.hpp
    HINTS "$ENV{HOME}/.local/include"
    PATH_SUFFIXES qpOASES)
find_library(QPOASES_LIBRARY
    NAMES qpOASES
    HINTS "$ENV{HOME}/.local/lib" "$ENV{HOME}/.local/lib64")
if(NOT QPOASES_INCLUDE_DIR OR NOT QPOASES_LIBRARY)
    message(FATAL_ERROR
        "qpOASES 3.2.2 not found. Expected the validated install under ~/.local.")
endif()

set(CORE_INCLUDE "${DYNAMIC_PLANNER_CORE_ROOT}/include")

add_library(dynamic_planner_math_ros STATIC
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/bspline.cpp"
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/minvo.cpp")
target_include_directories(dynamic_planner_math_ros PUBLIC "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_math_ros PUBLIC Eigen3::Eigen)
target_compile_options(dynamic_planner_math_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_collision_ros STATIC
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/ego_collision_model.cpp")
target_include_directories(dynamic_planner_collision_ros PUBLIC "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_collision_ros PUBLIC Eigen3::Eigen)
target_compile_options(dynamic_planner_collision_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_separator_ros STATIC
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/separator.cpp")
target_include_directories(dynamic_planner_separator_ros PUBLIC "${CORE_INCLUDE}" "${GLPK_INCLUDE_DIR}")
target_link_libraries(dynamic_planner_separator_ros PUBLIC Eigen3::Eigen PRIVATE "${GLPK_LIBRARY}")
target_compile_options(dynamic_planner_separator_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_octopus_ros STATIC
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/octopus_search.cpp")
target_include_directories(dynamic_planner_octopus_ros PUBLIC "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_octopus_ros
    PUBLIC dynamic_planner_math_ros dynamic_planner_separator_ros Eigen3::Eigen)
target_compile_options(dynamic_planner_octopus_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_refinement_ros STATIC
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/trajectory_refinement.cpp")
target_include_directories(dynamic_planner_refinement_ros
    PUBLIC "${CORE_INCLUDE}" "${QPOASES_INCLUDE_DIR}")
target_link_libraries(dynamic_planner_refinement_ros
    PUBLIC dynamic_planner_math_ros dynamic_planner_separator_ros dynamic_planner_octopus_ros Eigen3::Eigen
    PRIVATE "${QPOASES_LIBRARY}")
target_compile_options(dynamic_planner_refinement_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_receding_ros STATIC
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/minimum_time.cpp"
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/frozen_world.cpp"
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/trajectory_geometry.cpp"
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/world_snapshot.cpp"
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/trajectory_safety_checker.cpp"
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/local_planner.cpp"
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/committed_trajectory.cpp"
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/receding_horizon_planner.cpp")
target_include_directories(dynamic_planner_receding_ros PUBLIC "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_receding_ros
    PUBLIC
        dynamic_planner_math_ros
        dynamic_planner_separator_ros
        dynamic_planner_octopus_ros
        dynamic_planner_refinement_ros
        dynamic_planner_collision_ros
        Eigen3::Eigen)
target_compile_options(dynamic_planner_receding_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_reference_ros STATIC
    "${DYNAMIC_PLANNER_CORE_ROOT}/src/reference_window.cpp")
target_include_directories(dynamic_planner_reference_ros PUBLIC "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_reference_ros PUBLIC dynamic_planner_receding_ros Eigen3::Eigen)
target_compile_options(dynamic_planner_reference_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_reference_message
    src/reference_message.cpp
    src/kinematics_filter.cpp
    src/commissioning_reference.cpp)
target_include_directories(dynamic_planner_reference_message
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        "${CORE_INCLUDE}")
# Plain signature is intentional: Humble ament_target_dependencies also uses the
# plain target_link_libraries signature internally.
target_link_libraries(dynamic_planner_reference_message
    dynamic_planner_reference_ros
    Eigen3::Eigen)
ament_target_dependencies(dynamic_planner_reference_message
    builtin_interfaces geometry_msgs trajectory_msgs)
target_compile_options(dynamic_planner_reference_message PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_planner_c1e_scene STATIC src/c1e_scene.cpp)
target_include_directories(dynamic_planner_c1e_scene
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_c1e_scene
    PUBLIC dynamic_planner_separator_ros dynamic_planner_collision_ros Eigen3::Eigen)
target_compile_options(dynamic_planner_c1e_scene PRIVATE -Wall -Wextra -Wpedantic)

add_executable(dynamic_planner_passive_commissioning
    src/passive_commissioning_node.cpp)
target_include_directories(dynamic_planner_passive_commissioning PRIVATE "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_passive_commissioning
    dynamic_planner_reference_message
    dynamic_planner_reference_ros
    Eigen3::Eigen)
ament_target_dependencies(dynamic_planner_passive_commissioning
    rclcpp interfaces std_msgs builtin_interfaces geometry_msgs trajectory_msgs)
target_compile_options(dynamic_planner_passive_commissioning PRIVATE -Wall -Wextra -Wpedantic)

add_executable(c1e_octopus_budget_replay
    src/c1e_octopus_budget_replay.cpp)
target_include_directories(c1e_octopus_budget_replay PRIVATE "${CORE_INCLUDE}")
target_link_libraries(c1e_octopus_budget_replay
    dynamic_planner_c1e_scene
    dynamic_planner_receding_ros
    Eigen3::Eigen)
target_compile_options(c1e_octopus_budget_replay PRIVATE -Wall -Wextra -Wpedantic)

add_executable(dynamic_planner_active_commissioning
    src/active_commissioning_node.cpp)
target_include_directories(dynamic_planner_active_commissioning PRIVATE "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_active_commissioning
    dynamic_planner_reference_message
    dynamic_planner_reference_ros
    dynamic_planner_c1e_scene
    Eigen3::Eigen)
ament_target_dependencies(dynamic_planner_active_commissioning
    rclcpp interfaces std_msgs builtin_interfaces geometry_msgs trajectory_msgs
    visualization_msgs)
target_compile_options(dynamic_planner_active_commissioning PRIVATE -Wall -Wextra -Wpedantic)

# C1F.0 reusable fixed-target transfer backend. It is deliberately passive: the
# node publishes only /dynamic_planner/transfer_reference and rejects the MPC
# authority topic. Mission integration/authority switching is deferred to C1F.1+.
add_executable(dynamic_planner_transfer_backend
    src/transfer_backend_node.cpp)
target_include_directories(dynamic_planner_transfer_backend PRIVATE "${CORE_INCLUDE}")
target_link_libraries(dynamic_planner_transfer_backend
    dynamic_planner_reference_message
    dynamic_planner_reference_ros
    dynamic_planner_c1e_scene
    Eigen3::Eigen)
ament_target_dependencies(dynamic_planner_transfer_backend
    rclcpp interfaces std_msgs builtin_interfaces geometry_msgs trajectory_msgs visualization_msgs)
target_compile_options(dynamic_planner_transfer_backend PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS
    dynamic_planner_math_ros
    dynamic_planner_collision_ros
    dynamic_planner_separator_ros
    dynamic_planner_octopus_ros
    dynamic_planner_refinement_ros
    dynamic_planner_receding_ros
    dynamic_planner_reference_ros
    dynamic_planner_reference_message
    dynamic_planner_c1e_scene
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(TARGETS
    dynamic_planner_passive_commissioning
    dynamic_planner_active_commissioning
    dynamic_planner_transfer_backend
    c1e_octopus_budget_replay
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY "${DYNAMIC_PLANNER_CORE_ROOT}/include/dynamic_planner" DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
    scripts/generate_c1e_world.py
    scripts/generate_c1f3a_world.py
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    enable_testing()

    add_executable(test_reference_message test/test_reference_message.cpp)
    target_link_libraries(test_reference_message dynamic_planner_reference_message)
    target_compile_options(test_reference_message PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_reference_message COMMAND test_reference_message)

    add_executable(test_kinematics_filter test/test_kinematics_filter.cpp)
    target_link_libraries(test_kinematics_filter dynamic_planner_reference_message)
    target_compile_options(test_kinematics_filter PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_kinematics_filter COMMAND test_kinematics_filter)

    add_executable(test_commissioning_reference test/test_commissioning_reference.cpp)
    target_link_libraries(test_commissioning_reference dynamic_planner_reference_message)
    target_compile_options(test_commissioning_reference PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_commissioning_reference COMMAND test_commissioning_reference)

    add_executable(test_c1e_scene test/test_c1e_scene.cpp)
    target_link_libraries(test_c1e_scene
        dynamic_planner_c1e_scene
        dynamic_planner_reference_ros
        dynamic_planner_receding_ros)
    target_compile_options(test_c1e_scene PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1e_scene COMMAND test_c1e_scene)

    add_executable(test_c1f3a_scene test/test_c1f3a_scene.cpp)
    target_link_libraries(test_c1f3a_scene dynamic_planner_c1e_scene)
    target_compile_options(test_c1f3a_scene PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f3a_scene COMMAND test_c1f3a_scene)

    add_executable(test_c1f3b_scene test/test_c1f3b_scene.cpp)
    target_include_directories(test_c1f3b_scene PRIVATE "${CORE_INCLUDE}")
    target_link_libraries(test_c1f3b_scene dynamic_planner_receding_ros)
    target_compile_options(test_c1f3b_scene PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f3b_scene COMMAND test_c1f3b_scene)

    add_executable(test_c1f4_execution_contract test/test_c1f4_execution_contract.cpp)
    target_include_directories(test_c1f4_execution_contract PRIVATE "${CORE_INCLUDE}")
    target_link_libraries(test_c1f4_execution_contract
        dynamic_planner_reference_ros
        dynamic_planner_receding_ros)
    target_compile_options(test_c1f4_execution_contract PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f4_execution_contract COMMAND test_c1f4_execution_contract)

    # Run the standalone newest-world recheck regression through the ROS package
    # too, so C1F.4's failed-replan incumbent revalidation is part of Terminal 0.
    add_executable(test_c1f4_incumbent_recheck
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests/test_recheck_commit.cpp")
    target_include_directories(test_c1f4_incumbent_recheck PRIVATE
        "${CORE_INCLUDE}"
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests")
    target_link_libraries(test_c1f4_incumbent_recheck dynamic_planner_receding_ros)
    target_compile_options(test_c1f4_incumbent_recheck PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f4_incumbent_recheck COMMAND test_c1f4_incumbent_recheck)

    add_executable(test_c1f5_terminal_viability test/test_c1f5_terminal_viability.cpp)
    target_include_directories(test_c1f5_terminal_viability PRIVATE "${CORE_INCLUDE}")
    target_link_libraries(test_c1f5_terminal_viability dynamic_planner_receding_ros)
    target_compile_options(test_c1f5_terminal_viability PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f5_terminal_viability COMMAND test_c1f5_terminal_viability)

    # C1F.5p2: keep the deadline-budget and initial-q2 restart regressions in
    # the normal ROS Terminal-0 gate instead of relying on a separate standalone build.
    add_executable(test_c1f5p2_octopus_restart_contract
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests/test_octopus.cpp")
    target_include_directories(test_c1f5p2_octopus_restart_contract PRIVATE
        "${CORE_INCLUDE}"
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests")
    target_link_libraries(test_c1f5p2_octopus_restart_contract dynamic_planner_octopus_ros dynamic_planner_collision_ros)
    target_compile_options(test_c1f5p2_octopus_restart_contract PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f5p2_octopus_restart_contract COMMAND test_c1f5p2_octopus_restart_contract)

    # C1F.6a: Octopus incremental terminal checks and final certification must
    # use the same stopped or moving-rendezvous control-point tail.
    add_executable(test_c1f6a_octopus_terminal_contract
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests/test_octopus_terminal_contract.cpp")
    target_include_directories(test_c1f6a_octopus_terminal_contract PRIVATE
        "${CORE_INCLUDE}"
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests")
    target_link_libraries(test_c1f6a_octopus_terminal_contract dynamic_planner_octopus_ros)
    target_compile_options(test_c1f6a_octopus_terminal_contract PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f6a_octopus_terminal_contract COMMAND test_c1f6a_octopus_terminal_contract)

    add_executable(test_c1f7_target_predictor
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests/test_target_predictor.cpp"
        "${DYNAMIC_PLANNER_CORE_ROOT}/src/committed_trajectory.cpp")
    target_include_directories(test_c1f7_target_predictor PRIVATE
        "${CORE_INCLUDE}"
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests")
    target_link_libraries(test_c1f7_target_predictor
        dynamic_planner_math_ros
        Eigen3::Eigen)
    target_compile_options(test_c1f7_target_predictor PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f7_target_predictor COMMAND test_c1f7_target_predictor)

    add_executable(test_c1f5p2_integration_timing
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests/test_integration_timing.cpp")
    target_include_directories(test_c1f5p2_integration_timing PRIVATE
        "${CORE_INCLUDE}"
        "${DYNAMIC_PLANNER_CORE_ROOT}/tests")
    target_link_libraries(test_c1f5p2_integration_timing
        dynamic_planner_reference_ros
        dynamic_planner_receding_ros)
    target_compile_options(test_c1f5p2_integration_timing PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f5p2_integration_timing COMMAND test_c1f5p2_integration_timing)

    # C1F.6: moving-rendezvous p/v terminal contract plus explicit stopped
    # backup must remain in the normal Terminal-0 regression gate.
    add_executable(test_c1f6_moving_rendezvous
        test/test_c1f6_moving_rendezvous.cpp)
    target_include_directories(test_c1f6_moving_rendezvous PRIVATE "${CORE_INCLUDE}")
    target_link_libraries(test_c1f6_moving_rendezvous
        dynamic_planner_reference_ros
        dynamic_planner_receding_ros)
    target_compile_options(test_c1f6_moving_rendezvous PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f6_moving_rendezvous COMMAND test_c1f6_moving_rendezvous)

    # M1 ring/net TDD gate. This test is intentionally added before the production
    # helper it names. Step 2 should therefore be RED until Step 3 implements the
    # agreed collision-envelope translation contract.
    add_executable(test_c1f6_ring_basket_contract
        test/test_c1f6_ring_basket_contract.cpp
        "${DYNAMIC_PLANNER_CORE_ROOT}/src/committed_trajectory.cpp")
    target_include_directories(test_c1f6_ring_basket_contract PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        "${CORE_INCLUDE}")
    target_link_libraries(test_c1f6_ring_basket_contract
        dynamic_planner_math_ros
        Eigen3::Eigen)
    target_compile_options(test_c1f6_ring_basket_contract PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_c1f6_ring_basket_contract COMMAND test_c1f6_ring_basket_contract)
endif()

ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp interfaces std_msgs builtin_interfaces geometry_msgs trajectory_msgs
    visualization_msgs)
ament_package()
