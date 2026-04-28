#ifndef INITIALIZATION_HPP
#define INITIALIZATION_HPP

#include <optional>
#include <vector>

#include "scene_types.hpp"

/**
 * @defgroup Initialization Initialization
 * @brief Compute the startup state used to begin live filtering.
 * @{
 */

/**
 * @brief Compute the startup initialization from IMU and GPS samples.
 *
 * The returned object contains the initial state together with the first IMU
 * and GPS indices that still belong to the live post-startup run.
 *
 * @param imu_samples Loaded IMU samples.
 * @param gps_samples Generated GPS samples.
 * @return The startup handoff state, or `std::nullopt` when startup never
 *     reaches a state suitable for handoff.
 */
std::optional<StartupInitialization> ComputeStartupInitialization(
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples);

/** @} */

#endif  // INITIALIZATION_HPP
