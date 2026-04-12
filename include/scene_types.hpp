#ifndef SCENE_TYPES_HPP
#define SCENE_TYPES_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
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
struct ImuSample {
    std::int64_t utime = 0;  ///< Timestamp in microseconds as std::int64_t.
    Eigen::Vector3d specific_force{
        0.0, 0.0, 0.0};  ///< Measured specific force in frame I.
    Eigen::Vector3d rotation_rate{
        0.0, 0.0, 0.0};  ///< Measured angular velocity in frame I.
    Eigen::Quaterniond q_AI{
        1.0,
        0.0,
        0.0,
        0.0};  ///< Quaternion from frame I to gravity-aligned frame A in [w, x, y, z].
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
    Eigen::Vector3d v0_G{0.0, 0.0, 0.0};  ///< Initial velocity in frame G.
    Eigen::Vector3d p0_G{0.0, 0.0, 0.0};  ///< Initial position in frame G.
    Eigen::Quaterniond q0_GI{
        1.0, 0.0, 0.0, 0.0};  ///< Initial attitude quaternion from I to G.
    std::size_t next_imu_index =
        0;  ///< First IMU sample not used by initialization.
    std::size_t next_gps_index =
        0;  ///< First GPS sample not used by initialization.
    std::int64_t last_imu_utime =
        0;  ///< Timestamp of the last IMU sample used by initialization.
};

#endif  // SCENE_TYPES_HPP
