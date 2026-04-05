#include "gps_generation.hpp"

#include <cmath>
#include <random>

#include "scene_types.hpp"

std::vector<GpsSample> GenerateGpsSamplesFromPose(
    const std::vector<PoseSample>& pose_samples) {
    // 1. Use the same fixed synthetic-GPS settings as the older app:
    //    `10 Hz`, `2 m` xy noise, and seed `1234`.
    // NOLINTNEXTLINE(readability-identifier-naming)
    constexpr double kGpsHz = 10.0;
    // NOLINTNEXTLINE(readability-identifier-naming)
    constexpr double kGpsSigmaXY = 2.0;
    // NOLINTNEXTLINE(readability-identifier-naming)
    constexpr unsigned int kGpsSeed = 1234;
    const long long desired_dt_us =
        static_cast<long long>(std::llround(1e6 / kGpsHz));

    // 2. Create the random-number machinery for independent Gaussian noise in
    //    `x` and `y`.
    std::mt19937 rng(kGpsSeed);
    std::normal_distribution<double> standard_normal(0.0, 1.0);

    // 3. Create the output vector. Also create the small variables needed to
    //    remember whether we have already kept a pose sample, and if we have,
    //    what the timestamp of that last kept sample was.
    std::vector<GpsSample> gps_samples;
    gps_samples.reserve(pose_samples.size());
    bool first_kept = false;
    long long last_kept_utime = 0;

    // 4. Go through the pose samples in time order. Keep the first pose sample
    //    no matter what, so the generated GPS stream starts at the first pose
    //    timestamp.
    for (const PoseSample& pose_sample : pose_samples) {
        const long long utime = pose_sample.utime;

        // 5. For each later pose sample, compare its timestamp to the
        //    timestamp of the last pose sample we kept. Skip it unless enough
        //    time has passed for one `10 Hz` GPS step.
        if (!first_kept) {
            first_kept = true;
            last_kept_utime = utime;
        } else if (utime - last_kept_utime < desired_dt_us) {
            continue;
        } else {
            last_kept_utime = utime;
        }

        // 6. Whenever a pose sample is kept, copy its timestamp, take its `x`
        //    and `y` position as the true horizontal position, add
        //    independent Gaussian noise to `x` and `y`, and store the result
        //    in one `GpsSample`.
        const double nx = kGpsSigmaXY * standard_normal(rng);
        const double ny = kGpsSigmaXY * standard_normal(rng);

        GpsSample gps_sample;
        gps_sample.utime = utime;
        gps_sample.xy =
            Eigen::Vector2d(pose_sample.pos.x() + nx, pose_sample.pos.y() + ny);
        gps_samples.push_back(gps_sample);
    }

    // 7. After all pose samples have been processed, return the completed
    //    vector of generated GPS samples.
    return gps_samples;
}
