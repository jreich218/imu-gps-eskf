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
struct ImuSample {
    std::int64_t utime = 0;  ///< Timestamp in microseconds.
    Eigen::Vector3d specific_force{
        0.0, 0.0, 0.0};  ///< Measured specific force in frame I.
    Eigen::Vector3d rotation_rate{
        0.0, 0.0, 0.0};  ///< Measured angular velocity in frame I.
    Eigen::Quaterniond q_AI{
        1.0,
        0.0,
        0.0,
        0.0};  ///< Quaternion from frame I to gravity-aligned frame A.
};

/// One wheel-speed CAN sample from `zoe_veh_info`.
struct WheelSpeedSample {
    std::int64_t utime = 0;  ///< Timestamp in microseconds.
    double fl_wheel_speed_rpm = 0.0;  ///< Front-left wheel speed in rpm.
    double fr_wheel_speed_rpm = 0.0;  ///< Front-right wheel speed in rpm.
    double rl_wheel_speed_rpm = 0.0;  ///< Rear-left wheel speed in rpm.
    double rr_wheel_speed_rpm = 0.0;  ///< Rear-right wheel speed in rpm.
};

/// Loaded reference-pose, IMU, and wheel-speed streams.
struct LoadedScene {
    std::vector<PoseSample> pose_samples;  ///< Loaded pose samples.
    std::vector<ImuSample> imu_samples;    ///< Loaded IMU samples.
    std::vector<WheelSpeedSample>
        wheel_speed_samples;  ///< Loaded wheel-speed samples.
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
