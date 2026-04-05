#ifndef GPS_GENERATION_HPP
#define GPS_GENERATION_HPP

#include <vector>

#include "scene_types.hpp"

/**
 * @defgroup GpsGeneration GPS Generation
 * @brief Generate synthetic GPS samples from pose samples.
 * @{
 */

/**
 * @brief Generate synthetic GPS samples from the loaded pose stream.
 *
 * @param pose_samples Loaded pose samples.
 * @return Generated GPS samples.
 */
std::vector<GpsSample> GenerateGpsSamplesFromPose(
    const std::vector<PoseSample>& pose_samples);

/** @} */

#endif  // GPS_GENERATION_HPP
