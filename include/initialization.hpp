#ifndef INITIALIZATION_HPP
#define INITIALIZATION_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "scene_types.hpp"

/**
 * @defgroup Initialization Initialization
 * @brief Compute the startup state used to begin live filtering.
 * @{
 */

/// Truth values from the pose stream at one startup frame endpoint.
struct StartupTraceTruthState {
    Eigen::Vector2d position_xy{0.0, 0.0};  ///< Truth position in meters.
    double yaw_rad = 0.0;                   ///< Truth yaw in radians.
    double speed_mps = 0.0;                 ///< Truth speed magnitude in m/s.
};

/// One startup-prefix trace frame.
struct StartupTraceFrame {
    std::size_t fit_end_index = 0;  ///< Inclusive GPS end index for this frame.
    std::size_t gps_points_used = 0;  ///< Number of GPS points in the prefix.
    std::int64_t end_utime = 0;       ///< Timestamp at the frame endpoint.

    Eigen::Vector2d latest_gps_xy{0.0, 0.0};  ///< Latest GPS point in the prefix.
    Eigen::Vector2d fitted_endpoint_xy{
        0.0, 0.0};  ///< Fitted path endpoint position.
    Eigen::Vector2d fitted_velocity_xy{
        0.0, 0.0};  ///< Fitted path endpoint velocity.
    double fitted_speed_mps = 0.0;  ///< Speed from the fitted endpoint tangent.
    double fitted_yaw_rad = 0.0;    ///< Yaw from the fitted endpoint tangent.

    double selected_yaw_rad = 0.0;   ///< Final startup yaw choice.
    double tangent_weight = 0.0;     ///< Weight assigned to the local tangent.
    double global_yaw_rad = 0.0;     ///< Global yaw candidate.
    double pca_yaw_rad = 0.0;        ///< Principal-direction yaw.
    double projected_separation_m = 0.0;  ///< PCA projected start-to-end span.
    double required_projected_separation_m =
        0.0;  ///< Projected-separation threshold recorded in the trace.
    bool line_like = true;  ///< Whether the PCA cloud is line-like.

    double local_speed_mps = 0.0;     ///< Recent fitted-path local speed.
    double global_speed_mps = 0.0;    ///< Recent global speed candidate.
    double selected_speed_mps = 0.0;  ///< Final startup speed choice.
    double wheel_speed_mps = 0.0;  ///< Smoothed robust wheel-speed magnitude.
    double wheel_supported_travel_m =
        0.0;  ///< Wheel-supported travel from startup begin to this frame.
    double required_wheel_supported_travel_m =
        0.0;  ///< Active wheel-supported travel gate.

    std::vector<Eigen::Vector2d> path_xy;  ///< Sampled fitted path points.
    StartupTraceTruthState truth;          ///< Truth at this frame endpoint.
};

/// Full startup trace and final handoff result.
struct StartupTraceResult {
    std::optional<std::size_t> first_usable_gps_index;  ///< First GPS index startup may use.
    std::vector<StartupTraceFrame> frames;  ///< One frame per evaluated startup prefix.
    std::optional<std::size_t> ready_frame_index;  ///< Index of the first ready frame, if any.
    std::optional<StartupInitialization>
        startup_initialization;  ///< Final handoff state, if startup succeeds.
};

/**
 * @brief Compute the startup initialization from IMU and GPS samples.
 *
 * The returned object contains the initial state together with the first IMU
 * and GPS indices that still belong to the live post-startup run.
 *
 * @param imu_samples Loaded IMU samples.
 * @param gps_samples Generated GPS samples.
 * @param wheel_speed_samples Loaded wheel-speed samples.
 * @return The startup handoff state, or `std::nullopt` when startup never
 *     reaches a state suitable for handoff.
 */
std::optional<StartupInitialization> ComputeStartupInitialization(
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples,
    const std::vector<WheelSpeedSample>& wheel_speed_samples);

/**
 * @brief Trace the startup evaluation across growing GPS prefixes.
 *
 * The returned trace records the quantities the startup algorithm computes for
 * each tested GPS prefix. If startup succeeds, the trace also includes the
 * resulting handoff state.
 *
 * @param pose_samples Loaded pose samples used for truth lookup.
 * @param imu_samples Loaded IMU samples.
 * @param gps_samples Generated GPS samples.
 * @param wheel_speed_samples Loaded wheel-speed samples.
 * @return The per-prefix startup trace and optional final handoff state.
 */
StartupTraceResult TraceStartupInitialization(
    const std::vector<PoseSample>& pose_samples,
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples,
    const std::vector<WheelSpeedSample>& wheel_speed_samples);

/** @} */

#endif  // INITIALIZATION_HPP
