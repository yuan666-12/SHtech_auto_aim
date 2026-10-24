cmake_minimum_required(VERSION 3.15)
project(hero_offline_checks LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(HERO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_compile_definitions(PROJECT_DIR="${HERO_ROOT}")
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
    "${HERO_ROOT}/common" "${HERO_ROOT}/mathutils" "${HERO_ROOT}/predict"
    "${HERO_ROOT}/planner" "${HERO_ROOT}/timedserial"
    "${HERO_ROOT}/timedserial/UartIMU" "${HERO_ROOT}/sensor"
    "${HERO_ROOT}/sensor/hikcam")
include_directories("${HERO_ROOT}/detect")
add_subdirectory("${HERO_ROOT}/planner/tinympc" tinympc)
add_library(hero_check_core STATIC
    "${HERO_ROOT}/common/log/log.cpp"
    "${HERO_ROOT}/common/timer/timer.cpp"
    "${HERO_ROOT}/mathutils/CoordTransformer.cpp"
    "${HERO_ROOT}/predict/Tracker.cpp"
    "${HERO_ROOT}/planner/Planner.cpp"
    "${HERO_ROOT}/planner/planner_submodule.cpp"
    "${HERO_ROOT}/timedserial/timed_serial.cpp"
    "${HERO_ROOT}/timedserial/UartIMU/xuc_driver.cpp")
target_link_libraries(hero_check_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads tinympcstatic)
add_library(hero_vision_check OBJECT
    "${HERO_ROOT}/predict/MultiPolicyPredictor_submodule.cpp"
    "${HERO_ROOT}/sensor/sensor_submodule.cpp"
    "${HERO_ROOT}/sensor/video/video_wrapper.cpp"
    "${HERO_ROOT}/detect/preprocess_submodule.cpp"
    "${HERO_ROOT}/detect/corner_refine_submodule.cpp"
    "${HERO_ROOT}/detect/armor_corner_optimizer.cpp"
    "${HERO_ROOT}/entrystage/entryStage_submodule.cpp")
target_link_libraries(hero_vision_check PRIVATE hero_check_core)
add_library(hero_runtime_syntax OBJECT "${HERO_ROOT}/main.cpp")
target_link_libraries(hero_runtime_syntax PRIVATE hero_check_core)
add_executable(hero_refactor_check "${HERO_ROOT}/tools/hero_refactor_check.cpp"
    $<TARGET_OBJECTS:hero_vision_check>)
target_link_libraries(hero_refactor_check PRIVATE hero_check_core)
add_test(NAME refactor COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/run_case.py"
    "${CMAKE_CURRENT_BINARY_DIR}/refactor-xuc.log" $<TARGET_FILE:hero_refactor_check> "${HERO_ROOT}")
add_executable(hero_dropout_check "${HERO_ROOT}/tools/hero_dropout_check.cpp"
    $<TARGET_OBJECTS:hero_vision_check>)
target_link_libraries(hero_dropout_check PRIVATE hero_check_core)
add_test(NAME dropout COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/run_case.py"
    "${CMAKE_CURRENT_BINARY_DIR}/dropout-xuc.log" $<TARGET_FILE:hero_dropout_check> "${HERO_ROOT}")
set_tests_properties(dropout PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
foreach(name algorithm tracking center_window console capture vehicle_model tx_diagnostics)
    add_executable(hero_${name}_check "${HERO_ROOT}/tools/hero_${name}_check.cpp")
    target_link_libraries(hero_${name}_check PRIVATE hero_check_core)
    add_test(NAME ${name} COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/run_case.py"
        "${CMAKE_CURRENT_BINARY_DIR}/${name}-xuc.log" $<TARGET_FILE:hero_${name}_check> "${HERO_ROOT}"
        "${HERO_ROOT}/tools/fixtures/planner_geometry_only.yml")
    set_tests_properties(${name} PROPERTIES
        ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_XUC_DRY_RUN=1;AUTOAIM_XUC_LOG_FILE=${CMAKE_CURRENT_BINARY_DIR}/${name}-xuc.log"
        TIMEOUT 30 RUN_SERIAL TRUE)
endforeach()
add_executable(hero_startup_check "${HERO_ROOT}/tools/hero_startup_check.cpp"
    "${HERO_ROOT}/sensor/hikcam/hikcam_wrapper.cpp")
target_link_libraries(hero_startup_check PRIVATE ${OpenCV_LIBS})
add_test(NAME camera_lifecycle COMMAND hero_startup_check)
foreach(backend mock real)
    add_executable(hero_headless_${backend} "${HERO_ROOT}/tools/hero_headless_check.cpp")
    target_link_libraries(hero_headless_${backend} PRIVATE ${OpenCV_LIBS})
endforeach()
target_compile_definitions(hero_headless_real PRIVATE HEADLESS_REAL_BACKEND)
add_test(NAME headless_mock COMMAND hero_headless_mock 1)
add_test(NAME headless_real COMMAND hero_headless_real 1)
set_tests_properties(headless_mock headless_real PROPERTIES ENVIRONMENT "AUTOAIM_HEADLESS=1;DISPLAY=;WAYLAND_DISPLAY=")
add_test(NAME gui_delegation COMMAND hero_headless_mock 0)
set_tests_properties(gui_delegation PROPERTIES ENVIRONMENT "AUTOAIM_HEADLESS=0")
add_test(NAME python_serial COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/check_serial_tool.py")

add_executable(hero_fixed_target_check "${HERO_ROOT}/tools/hero_fixed_target_check.cpp")
target_link_libraries(hero_fixed_target_check PRIVATE hero_check_core)
foreach(profile free province_fixed)
    add_test(NAME target_model_${profile} COMMAND hero_fixed_target_check)
    set_tests_properties(target_model_${profile} PROPERTIES
        ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_TARGET_MODEL=${profile};AUTOAIM_TARGET_RADIUS_M=0.265;AUTOAIM_OMEGA_PRIOR=0"
        TIMEOUT 30 RUN_SERIAL TRUE)
endforeach()
add_test(NAME target_model_radius_override COMMAND hero_fixed_target_check)
set_tests_properties(target_model_radius_override PROPERTIES
    ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_TARGET_MODEL=province_fixed;AUTOAIM_TARGET_RADIUS_M=0.3;AUTOAIM_OMEGA_PRIOR=0"
    TIMEOUT 30 RUN_SERIAL TRUE)

foreach(profile free province_fixed)
    add_test(NAME vehicle_pipeline_${profile}
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/run_case.py"
        "${CMAKE_CURRENT_BINARY_DIR}/pipeline-${profile}-xuc.log"
        $<TARGET_FILE:hero_vehicle_model_check> "${HERO_ROOT}" --pipeline-only)
    set_tests_properties(vehicle_pipeline_${profile} PROPERTIES
        ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_XUC_DRY_RUN=1;AUTOAIM_TARGET_MODEL=${profile};AUTOAIM_TARGET_RADIUS_M=0.265"
        TIMEOUT 30 RUN_SERIAL TRUE)
endforeach()

add_executable(hero_first_shot_check "${HERO_ROOT}/tools/hero_first_shot_check.cpp")
target_link_libraries(hero_first_shot_check PRIVATE hero_check_core)
add_test(NAME first_shot COMMAND hero_first_shot_check)
add_test(NAME first_shot_trace COMMAND hero_first_shot_check "${HERO_ROOT}/tools/fixtures/first_shot_motion_20261024.csv")
add_test(NAME first_shot_tx COMMAND hero_first_shot_check --tx)
set_tests_properties(first_shot first_shot_trace first_shot_tx PROPERTIES
    ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_XUC_DRY_RUN=1"
    TIMEOUT 30 RUN_SERIAL TRUE)

add_executable(hero_rotation_prior_math_check "${HERO_ROOT}/tools/hero_rotation_prior_math_check.cpp")
add_test(NAME rotation_prior_math COMMAND hero_rotation_prior_math_check)
add_executable(hero_rotation_speed_check "${HERO_ROOT}/tools/hero_rotation_speed_check.cpp")
target_link_libraries(hero_rotation_speed_check PRIVATE hero_check_core)
add_test(NAME rotation_speed COMMAND hero_rotation_speed_check)
set_tests_properties(rotation_speed PROPERTIES ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_TARGET_MODEL=province_fixed" TIMEOUT 30)
add_test(NAME rotation_speed_free COMMAND hero_rotation_speed_check --free)
set_tests_properties(rotation_speed_free PROPERTIES ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_TARGET_MODEL=free" TIMEOUT 30)
add_test(NAME rotation_speed_trace COMMAND hero_rotation_speed_check --replay
    "${HERO_ROOT}/tools/fixtures/rotation_observations_20261024.csv" "${CMAKE_CURRENT_BINARY_DIR}/rotation-speed-replay.csv")
set_tests_properties(rotation_speed_trace PROPERTIES ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_TARGET_MODEL=province_fixed" TIMEOUT 30)

add_test(NAME rotation_prior_invalid COMMAND hero_rotation_prior_math_check --invalid)
set_tests_properties(rotation_prior_invalid PROPERTIES ENVIRONMENT "AUTOAIM_TARGET_PERIOD_S=nan" TIMEOUT 10)

add_executable(hero_aim_session_check "${HERO_ROOT}/tools/hero_aim_session_check.cpp"
    $<TARGET_OBJECTS:hero_vision_check>)
target_link_libraries(hero_aim_session_check PRIVATE hero_check_core)
add_test(NAME aim_session_province_fixed COMMAND hero_aim_session_check "${HERO_ROOT}")
add_test(NAME aim_session_free COMMAND hero_aim_session_check "${HERO_ROOT}" --free)
foreach(profile province_fixed free)
    set_tests_properties(aim_session_${profile} PROPERTIES
        ENVIRONMENT "AUTOAIM_HEADLESS=1;AUTOAIM_TARGET_MODEL=${profile};AUTOAIM_TARGET_RADIUS_M=0.265;AUTOAIM_TRACK_ONLY=0;AUTOAIM_XUC_DRY_RUN=0;AUTOAIM_XUC_LOG_FILE=${CMAKE_CURRENT_BINARY_DIR}/aim-session-${profile}.log"
        TIMEOUT 60 RUN_SERIAL TRUE)
endforeach()
