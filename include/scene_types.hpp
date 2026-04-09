#ifndef SCENE_TYPES_HPP
#define SCENE_TYPES_HPP

#include <cstdint>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <vector>

/// Selected pose and IMU input paths.
struct SceneInputs {
    std::filesystem::path pose_path;  ///< Pose file path.
    std::filesystem::path imu_path;   ///< IMU file path.
};

/// One pose sample from the pose stream.
struct PoseSample {
    std::int64_t utime = 0;  ///< Timestamp in microseconds as std::int64_t.
    Eigen::Vector3d pos{0.0, 0.0, 0.0};  ///< Global position in meters.
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};  ///< Pose orientation.
    Eigen::Vector3d vel{0.0, 0.0, 0.0};                  ///< Velocity sample.
};

/// One IMU sample from the IMU stream.
struct ImuSample {
    std::int64_t utime = 0;  ///< Timestamp in microseconds as std::int64_t.
    Eigen::Vector3d specific_force{
        0.0, 0.0, 0.0};  ///< Specific force in frame I.
    Eigen::Vector3d rotation_rate{
        0.0, 0.0, 0.0};  ///< Angular velocity in frame I.
    Eigen::Quaterniond q_AI{
        1.0, 0.0, 0.0, 0.0};  ///< IMU orientation in [w, x, y, z].
};

/// Loaded pose and IMU streams.
struct LoadedScene {
    std::vector<PoseSample> pose_samples;  ///< Loaded pose samples.
    std::vector<ImuSample> imu_samples;    ///< Loaded IMU samples.
};

/// One GPS-like sample.
struct GpsSample {
    std::int64_t utime = 0;  ///< Timestamp in microseconds as std::int64_t.
    Eigen::Vector2d xy{0.0, 0.0};  ///< Horizontal position in meters.
};

/// Initialization data required to start the filter.
struct StartupInitialization {
    Eigen::Vector3d v0_G{0.0, 0.0, 0.0};  ///< Initial velocity in frame G.
    Eigen::Vector3d p0_G{0.0, 0.0, 0.0};  ///< Initial position in frame G.
    Eigen::Quaterniond q0_GI{
        1.0, 0.0, 0.0, 0.0};  ///< Initial attitude from I to G.
    std::size_t next_imu_index =
        0;  ///< First IMU sample that wasn't used by initialization.
    std::size_t next_gps_index =
        0;  ///< First GPS sample that wasn't used by initialization.
};

#endif  // SCENE_TYPES_HPP
