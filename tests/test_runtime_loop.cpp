#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "runtime_loop.hpp"

namespace {

constexpr double kGravityMps2 = 9.8;

PoseSample MakePoseSample(std::int64_t utime, double x, double y) {
    PoseSample pose_sample;
    pose_sample.utime = utime;
    pose_sample.pos = Eigen::Vector3d(x, y, 0.0);
    pose_sample.orientation = Eigen::Quaterniond::Identity();
    pose_sample.vel = Eigen::Vector3d::Zero();
    return pose_sample;
}

ImuSample MakeImuSample(std::int64_t utime) {
    ImuSample imu_sample;
    imu_sample.utime = utime;
    imu_sample.specific_force = Eigen::Vector3d(0.0, 0.0, kGravityMps2);
    imu_sample.rotation_rate = Eigen::Vector3d::Zero();
    imu_sample.q_AI = Eigen::Quaterniond::Identity();
    return imu_sample;
}

GpsSample MakeGpsSample(std::int64_t utime, double x, double y) {
    GpsSample gps_sample;
    gps_sample.utime = utime;
    gps_sample.xy = Eigen::Vector2d(x, y);
    return gps_sample;
}

StartupInitialization MakeStartupInitialization(
    const Eigen::Vector3d& p0_G = Eigen::Vector3d::Zero(),
    std::int64_t previous_imu_utime = 0,
    std::size_t first_unprocessed_imu_index = 0,
    std::size_t first_unprocessed_gps_index = 0) {
    StartupInitialization startup_initialization;
    startup_initialization.p0_G = p0_G;
    startup_initialization.v0_G = Eigen::Vector3d::Zero();
    startup_initialization.q0_GI = Eigen::Quaterniond::Identity();
    startup_initialization.previous_imu_utime = previous_imu_utime;
    startup_initialization.first_unprocessed_imu_index =
        first_unprocessed_imu_index;
    startup_initialization.first_unprocessed_gps_index =
        first_unprocessed_gps_index;
    return startup_initialization;
}

}  // namespace

TEST(RunEskfLoop, ReturnsOneLogRowPerProcessedGpsUpdate) {
    LoadedScene loaded_scene;
    loaded_scene.pose_samples = {
        MakePoseSample(100000, 0.0, 0.0),
        MakePoseSample(200000, 0.0, 0.0),
    };
    loaded_scene.imu_samples = {
        MakeImuSample(100000),
        MakeImuSample(200000),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 0.0, 0.0),
    };

    const StartupInitialization startup_initialization =
        MakeStartupInitialization();

    const EskfRunResult run_result =
        RunEskfLoop(loaded_scene, gps_samples, startup_initialization);

    ASSERT_EQ(run_result.log_rows.size(), 2U);
    EXPECT_EQ(run_result.num_gps_updates, 2U);
    EXPECT_DOUBLE_EQ(run_result.raw_gps_rmse_xy, 0.0);
    EXPECT_DOUBLE_EQ(run_result.eskf_rmse_xy, 0.0);

    EXPECT_EQ(run_result.log_rows[0].utime, 100000);
    EXPECT_EQ(run_result.log_rows[1].utime, 200000);

    for (const EskfLogRow& log_row : run_result.log_rows) {
        EXPECT_DOUBLE_EQ(log_row.estimated_position_G.x(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.estimated_position_G.y(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.estimated_position_G.z(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.gps_xy.x(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.gps_xy.y(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.true_xy.x(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.true_xy.y(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.innovation_xy.x(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.innovation_xy.y(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.nis, 0.0);
        EXPECT_DOUBLE_EQ(log_row.estimator_error_xy.x(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.estimator_error_xy.y(), 0.0);
        EXPECT_DOUBLE_EQ(log_row.estimator_error_norm, 0.0);
    }
}

TEST(RunEskfLoop, StartsFromStartupHandoffIndices) {
    LoadedScene loaded_scene;
    loaded_scene.pose_samples = {
        MakePoseSample(100000, 10.0, 0.0),
        MakePoseSample(200000, 10.0, 0.0),
    };
    loaded_scene.imu_samples = {
        MakeImuSample(100000),
        MakeImuSample(200000),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 1000.0, 0.0),
        MakeGpsSample(200000, 10.0, 0.0),
    };

    const StartupInitialization startup_initialization =
        MakeStartupInitialization(
            Eigen::Vector3d(10.0, 0.0, 0.0), 100000, 1, 1);

    const EskfRunResult run_result =
        RunEskfLoop(loaded_scene, gps_samples, startup_initialization);

    ASSERT_EQ(run_result.log_rows.size(), 1U);
    EXPECT_EQ(run_result.num_gps_updates, 1U);
    EXPECT_EQ(run_result.log_rows[0].utime, 200000);
    EXPECT_DOUBLE_EQ(run_result.raw_gps_rmse_xy, 0.0);
    EXPECT_DOUBLE_EQ(run_result.eskf_rmse_xy, 0.0);

    const EskfLogRow& log_row = run_result.log_rows[0];
    EXPECT_DOUBLE_EQ(log_row.estimated_position_G.x(), 10.0);
    EXPECT_DOUBLE_EQ(log_row.estimated_position_G.y(), 0.0);
    EXPECT_DOUBLE_EQ(log_row.estimated_position_G.z(), 0.0);
    EXPECT_DOUBLE_EQ(log_row.gps_xy.x(), 10.0);
    EXPECT_DOUBLE_EQ(log_row.gps_xy.y(), 0.0);
    EXPECT_DOUBLE_EQ(log_row.true_xy.x(), 10.0);
    EXPECT_DOUBLE_EQ(log_row.true_xy.y(), 0.0);
    EXPECT_DOUBLE_EQ(log_row.innovation_xy.x(), 0.0);
    EXPECT_DOUBLE_EQ(log_row.innovation_xy.y(), 0.0);
    EXPECT_DOUBLE_EQ(log_row.nis, 0.0);
    EXPECT_DOUBLE_EQ(log_row.estimator_error_xy.x(), 0.0);
    EXPECT_DOUBLE_EQ(log_row.estimator_error_xy.y(), 0.0);
    EXPECT_DOUBLE_EQ(log_row.estimator_error_norm, 0.0);
}