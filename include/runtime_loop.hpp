#ifndef RUNTIME_LOOP_HPP
#define RUNTIME_LOOP_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "scene_types.hpp"

/**
 * @defgroup RuntimeLoop Runtime Loop
 * @brief Run the post-startup ESKF loop and collect in-memory results.
 * @{
 */

/// One in-memory log row corresponding to one GPS update.
struct EskfLogRow {
    std::int64_t utime = 0;  ///< GPS update timestamp in microseconds.
    Eigen::Vector3d estimated_position_G{
        0.0, 0.0, 0.0};  ///< Estimated position after the GPS update.
    Eigen::Vector2d gps_xy{0.0, 0.0};  ///< GPS measurement used in the update.
    Eigen::Vector2d true_xy{
        0.0, 0.0};  ///< Pose-truth horizontal position at the same timestamp.
    Eigen::Vector2d innovation_xy{
        0.0, 0.0};     ///< Horizontal innovation returned by the GPS update.
    double nis = 0.0;  ///< Normalized innovation squared for the update.
    Eigen::Vector2d estimator_error_xy{
        0.0, 0.0};  ///< Horizontal estimator error relative to pose truth.
    double estimator_error_norm = 0.0;  ///< Norm of estimator_error_xy.
};

/// In-memory result from a full post-startup ESKF run.
struct EskfRunResult {
    std::vector<EskfLogRow>
        log_rows;                     ///< One row per GPS update after startup.
    std::size_t num_gps_updates = 0;  ///< Number of processed GPS updates.
    double raw_gps_rmse_xy = 0.0;     ///< Raw GPS horizontal RMSE.
    double eskf_rmse_xy = 0.0;        ///< ESKF horizontal RMSE.
};

/**
 * @brief Run the post-startup ESKF loop and collect the data needed for
 * output writing.
 *
 * The returned rows match the eventual CSV content one row per GPS update.
 *
 * @param loaded_scene Loaded pose and IMU streams.
 * @param gps_samples Generated GPS samples.
 * @param startup_initialization Startup state and handoff indices.
 * @return In-memory log rows and RMSE summary values.
 */
EskfRunResult RunEskfLoop(const LoadedScene& loaded_scene,
                          const std::vector<GpsSample>& gps_samples,
                          const StartupInitialization& startup_initialization);

/** @} */

#endif  // RUNTIME_LOOP_HPP