if(NOT DEFINED DEMO OR DEMO STREQUAL "")
    message(FATAL_ERROR "DEMO executable path was not provided")
endif()
if(NOT EXISTS "${DEMO}")
    message(FATAL_ERROR "cooperative_mission_demo does not exist: ${DEMO}")
endif()
if(NOT DEFINED OUT_DIR OR OUT_DIR STREQUAL "")
    set(OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/b3_stress_ctest")
endif()
file(MAKE_DIRECTORY "${OUT_DIR}")

set(SCENARIOS crossing_stress same_direction_stress two_drones_stress)
foreach(SCENARIO IN LISTS SCENARIOS)
    set(PREFIX "${OUT_DIR}/${SCENARIO}")
    execute_process(
        COMMAND "${DEMO}"
            --scenario "${SCENARIO}"
            --samples 7
            --csv-prefix "${PREFIX}"
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE STDOUT_TEXT
        ERROR_VARIABLE STDERR_TEXT
    )
    if(NOT RESULT EQUAL 0)
        message(STATUS "---- cooperative_mission_demo stdout (${SCENARIO}) ----\n${STDOUT_TEXT}")
        message(STATUS "---- cooperative_mission_demo stderr (${SCENARIO}) ----\n${STDERR_TEXT}")
        message(FATAL_ERROR
            "B.3 stress mission scenario '${SCENARIO}' failed with exit code ${RESULT}")
    endif()

    string(FIND "${STDOUT_TEXT}" "stress unobstructed-conflict witness: true" WITNESS_POS)
    if(WITNESS_POS EQUAL -1)
        message(STATUS "---- cooperative_mission_demo stdout (${SCENARIO}) ----\n${STDOUT_TEXT}")
        message(FATAL_ERROR
            "B.3 stress scenario '${SCENARIO}' did not certify an unobstructed collision witness")
    endif()
endforeach()
