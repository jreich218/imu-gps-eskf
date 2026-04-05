#include "initialization.hpp"

std::optional<StartupInitialization> ComputeStartupInitialization(
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples) {
    // 1. Find the first GPS sample whose timestamp is not earlier than the
    //    first IMU timestamp, because startup can only begin once the IMU
    //    stream has begun.
    // 2. Stop and return no result if there are not enough usable GPS samples
    //    to fit the startup path.
    // 3. Starting from the first usable GPS sample, repeatedly extend the GPS
    //    window by one sample and fit a quadratic path in time to the GPS `x`
    //    and `y` coordinates.
    // 4. For each fitted path, compute the local tangent heading and local
    //    speed at the end of the path.
    // 5. Summarize the GPS cloud with a principal-direction calculation, use
    //    it to get a global heading, and use IMU yaw accumulation to make that
    //    heading curvature-aware when the GPS cloud is not line-like.
    // 6. Measure how stable the recent tangent headings are, then use that
    //    stability to blend the local tangent heading with the global heading.
    // 7. Compute a recent local speed from the fitted path, compute a recent
    //    global speed from the recent path geometry, and blend those two
    //    speeds with the same stability weight used for heading.
    // 8. Require enough projected start-to-end separation before trusting the
    //    startup result, using the earlier threshold at first and the relaxed
    //    threshold after enough GPS points have been seen.
    // 9. As soon as one GPS window looks trustworthy, form `p0_G`, `v0_G`,
    //    and `q0_GI` from the ready position, blended speed, and blended yaw.
    // 10. Carry that ready state forward to the first IMU sample at or after
    //     the ready time, update the yaw by the IMU yaw accumulated across the
    //     carry interval, and record `next_imu_index` and `next_gps_index`.
    // 11. Return the completed `StartupInitialization`. If no GPS window ever
    //     becomes trustworthy, return no result.
    (void)imu_samples;
    (void)gps_samples;
    return std::nullopt;
}
