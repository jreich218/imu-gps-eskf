#ifndef SCENE_TYPES_HPP
#define SCENE_TYPES_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

/// Selected pose and IMU input file paths.
struct SceneInputs {
    std::filesystem::path pose_path;  ///< Pose file path.
    std::filesystem::path imu_path;   ///< IMU file path.
};

/// One sample from the reference pose stream.
struct PoseSample {
    std::int64_t utime = 0;  ///< Timestamp in microseconds as std::int64_t.
    Eigen::Vector3d pos{0.0, 0.0, 0.0};  ///< Reference global position in meters.
    Eigen::Quaterniond orientation{
        1.0, 0.0, 0.0, 0.0};  ///< Reference orientation quaternion in [w, x, y, z].
    Eigen::Vector3d vel{
        0.0, 0.0, 0.0};  ///< Reference ego-frame velocity sample in m/s.
};

/// One sample from the input IMU stream.
///
/// In Eskf::Predict, this sample is treated as the current IMU sample k. The
/// stored filter state before the Predict call is at the previous IMU sample
/// k-1, and the Predict call propagates that state from k-1 to k.
struct ImuSample {
    std::int64_t utime = 0;  ///< Current IMU timestamp t_k in microseconds.
    Eigen::Vector3d specific_force{
        0.0, 0.0, 0.0};  ///< Measured specific force in current frame I_k.
    Eigen::Vector3d rotation_rate{
        0.0, 0.0, 0.0};  ///< Measured angular velocity in current frame I_k.
    Eigen::Quaterniond q_AI{
        1.0,
        0.0,
        0.0,
        0.0};  ///< Quaternion from current frame I_k to gravity-aligned frame A.
};

/// Loaded reference-pose and IMU streams.
struct LoadedScene {
    std::vector<PoseSample> pose_samples;  ///< Loaded pose samples.
    std::vector<ImuSample> imu_samples;    ///< Loaded IMU samples.
};

/// One synthetic 2D GPS sample.
struct GpsSample {
    std::int64_t utime = 0;  ///< Timestamp in microseconds as std::int64_t.
    Eigen::Vector2d xy{0.0, 0.0};  ///< Measured horizontal position in meters.
};

/// Initialization data required to start the filter.
struct StartupInitialization {
    Eigen::Vector3d v0_G{
        0.0, 0.0, 0.0};  ///< Initial velocity in frame G at previous_imu_utime.
    Eigen::Vector3d p0_G{
        0.0, 0.0, 0.0};  ///< Initial position in frame G at previous_imu_utime.
    Eigen::Quaterniond q0_GI{
        1.0,
        0.0,
        0.0,
        0.0};  ///< Initial attitude quaternion from the startup state's IMU frame to G at previous_imu_utime.

    std::size_t first_unprocessed_imu_index =
        0;  ///< First IMU sample to pass to Eskf::Predict after initialization.
    std::size_t first_unprocessed_gps_index =
        0;  ///< First GPS sample not used by initialization.

    std::int64_t previous_imu_utime =
        0;  ///< IMU timestamp of the stored state used as t_{k-1} by the first post-initialization Predict.
};

#endif  // SCENE_TYPES_HPP
