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
 * The first pose sample is always kept. Later pose samples are kept only after
 * at least `100 ms` have elapsed since the last kept sample. Each kept sample
 * becomes one noisy horizontal GPS measurement in the global frame.
 *
 * @param pose_samples Loaded pose samples.
 * @return Generated GPS samples.
 */
std::vector<GpsSample> GenerateGpsSamplesFromPose(
    const std::vector<PoseSample>& pose_samples);

/** @} */

#endif  // GPS_GENERATION_HPP
