#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "eskf.hpp"
#include "scene_types.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravityMps2 = 9.8;

StartupInitialization MakeStartupInitialization(
    const Eigen::Vector3d& p0_G = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& v0_G = Eigen::Vector3d::Zero(),
    const Eigen::Quaterniond& q0_GI = Eigen::Quaterniond::Identity(),
    std::int64_t previous_imu_utime = 0) {
    StartupInitialization startup_initialization;
    startup_initialization.p0_G = p0_G;
    startup_initialization.v0_G = v0_G;
    startup_initialization.q0_GI = q0_GI;
    startup_initialization.previous_imu_utime = previous_imu_utime;
    return startup_initialization;
}

ImuSample MakeImuSample(
    std::int64_t utime,
    const Eigen::Vector3d& specific_force = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& rotation_rate = Eigen::Vector3d::Zero(),
    const Eigen::Quaterniond& q_AI = Eigen::Quaterniond::Identity()) {
    ImuSample imu_sample;
    imu_sample.utime = utime;
    imu_sample.specific_force = specific_force;
    imu_sample.rotation_rate = rotation_rate;
    imu_sample.q_AI = q_AI;
    return imu_sample;
}

GpsSample MakeGpsSample(std::int64_t utime, double x, double y) {
    GpsSample gps_sample;
    gps_sample.utime = utime;
    gps_sample.xy = Eigen::Vector2d(x, y);
    return gps_sample;
}

}  // namespace

TEST(EskfPredict, ThrowsWhenFilterIsNotInitialized) {
    Eskf eskf;

    EXPECT_THROW(eskf.Predict(MakeImuSample(1'000'000)), std::runtime_error);
}

TEST(EskfPredict, PropagatesConstantHorizontalAccelerationToPosition) {
    Eskf eskf;
    eskf.Initialize(MakeStartupInitialization());

    eskf.Predict(
        MakeImuSample(2'000'000,
                      Eigen::Vector3d(1.0, 0.0, kGravityMps2),
                      Eigen::Vector3d::Zero(),
                      Eigen::Quaterniond::Identity()));

    const GpsUpdateResult update_result =
        eskf.UpdateGps(MakeGpsSample(2'000'000, 2.0, 0.0));

    EXPECT_NEAR(update_result.innovation_xy.x(), 0.0, 1e-12);
    EXPECT_NEAR(update_result.innovation_xy.y(), 0.0, 1e-12);
}

TEST(EskfPredict, ConsecutiveCallsUseUpdatedTimestampAndVelocity) {
    Eskf eskf;
    eskf.Initialize(MakeStartupInitialization());

    const ImuSample imu_sample_1 =
        MakeImuSample(1'000'000,
                      Eigen::Vector3d(1.0, 0.0, kGravityMps2),
                      Eigen::Vector3d::Zero(),
                      Eigen::Quaterniond::Identity());

    const ImuSample imu_sample_2 =
        MakeImuSample(2'000'000,
                      Eigen::Vector3d(1.0, 0.0, kGravityMps2),
                      Eigen::Vector3d::Zero(),
                      Eigen::Quaterniond::Identity());

    eskf.Predict(imu_sample_1);
    eskf.Predict(imu_sample_2);

    const GpsUpdateResult update_result =
        eskf.UpdateGps(MakeGpsSample(2'000'000, 2.0, 0.0));

    EXPECT_NEAR(update_result.innovation_xy.x(), 0.0, 1e-12);
    EXPECT_NEAR(update_result.innovation_xy.y(), 0.0, 1e-12);
}

TEST(EskfPredict, RotatesBodyAccelerationIntoGlobalFrameUsingIntegratedYaw) {
    Eskf eskf;
    eskf.Initialize(MakeStartupInitialization());

    eskf.Predict(
        MakeImuSample(1'000'000,
                      Eigen::Vector3d(1.0, 0.0, kGravityMps2),
                      Eigen::Vector3d(0.0, 0.0, kPi / 2.0),
                      Eigen::Quaterniond::Identity()));

    const GpsUpdateResult update_result =
        eskf.UpdateGps(MakeGpsSample(1'000'000, 0.0, 0.5));

    EXPECT_NEAR(update_result.innovation_xy.x(), 0.0, 1e-10);
    EXPECT_NEAR(update_result.innovation_xy.y(), 0.0, 1e-10);
}

TEST(EskfPredict, UsesImuToGravityQuaternionWhenRecoveringLinearAcceleration) {
    Eskf eskf;
    eskf.Initialize(MakeStartupInitialization());

    const Eigen::Quaterniond q_AI(
        Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitY()));

    eskf.Predict(MakeImuSample(1'000'000,
                               Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(),
                               q_AI));

    const GpsUpdateResult update_result =
        eskf.UpdateGps(MakeGpsSample(1'000'000, 4.9, 0.0));

    EXPECT_NEAR(update_result.innovation_xy.x(), 0.0, 1e-10);
    EXPECT_NEAR(update_result.innovation_xy.y(), 0.0, 1e-10);
}

TEST(EskfPredict, GrowsPositionUncertaintyDuringPropagation) {
    Eskf eskf_without_predict;
    eskf_without_predict.Initialize(MakeStartupInitialization());

    const GpsUpdateResult update_without_predict =
        eskf_without_predict.UpdateGps(MakeGpsSample(1'000'000, 4.0, 0.0));

    Eskf eskf_with_predict;
    eskf_with_predict.Initialize(MakeStartupInitialization());

    eskf_with_predict.Predict(
        MakeImuSample(1'000'000,
                      Eigen::Vector3d(0.0, 0.0, kGravityMps2),
                      Eigen::Vector3d::Zero(),
                      Eigen::Quaterniond::Identity()));

    const GpsUpdateResult update_with_predict =
        eskf_with_predict.UpdateGps(MakeGpsSample(1'000'000, 4.0, 0.0));

    EXPECT_NEAR(update_with_predict.innovation_xy.x(), 4.0, 1e-12);
    EXPECT_NEAR(update_with_predict.innovation_xy.y(), 0.0, 1e-12);
    EXPECT_LT(update_with_predict.nis, update_without_predict.nis);
}

TEST(EskfUpdateGps, ThrowsWhenFilterIsNotInitialized) {
    Eskf eskf;
    const GpsSample gps_sample = MakeGpsSample(200000, 10.0, -5.0);

    EXPECT_THROW(eskf.UpdateGps(gps_sample), std::runtime_error);
}

TEST(EskfUpdateGps, ReturnsExpectedInnovationAndNisAtInitializationState) {
    Eskf eskf;
    eskf.Initialize(
        MakeStartupInitialization(Eigen::Vector3d(1.0, -2.0, 3.0)));

    const GpsSample gps_sample = MakeGpsSample(200000, 6.0, -1.0);
    const GpsUpdateResult update_result = eskf.UpdateGps(gps_sample);

    EXPECT_NEAR(update_result.innovation_xy.x(), 5.0, 1e-12);
    EXPECT_NEAR(update_result.innovation_xy.y(), 1.0, 1e-12);
    EXPECT_TRUE(std::isfinite(update_result.nis));
    EXPECT_NEAR(update_result.nis, 26.0 / 8.0, 1e-12);
}

TEST(EskfUpdateGps, RepeatedSameMeasurementProducesSmallerInnovationAndNis) {
    Eskf eskf;
    eskf.Initialize(MakeStartupInitialization());

    const GpsUpdateResult first_update =
        eskf.UpdateGps(MakeGpsSample(200000, 10.0, -5.0));
    const GpsUpdateResult second_update =
        eskf.UpdateGps(MakeGpsSample(300000, 10.0, -5.0));

    EXPECT_NEAR(first_update.innovation_xy.x(), 10.0, 1e-12);
    EXPECT_NEAR(first_update.innovation_xy.y(), -5.0, 1e-12);

    EXPECT_NEAR(second_update.innovation_xy.x(), 5.0, 1e-12);
    EXPECT_NEAR(second_update.innovation_xy.y(), -2.5, 1e-12);

    EXPECT_LT(second_update.innovation_xy.norm(),
              first_update.innovation_xy.norm());
    EXPECT_LT(second_update.nis, first_update.nis);
}

TEST(EskfUpdateGps, ZeroInnovationMeasurementReturnsZeroNis) {
    Eskf eskf;
    eskf.Initialize(
        MakeStartupInitialization(Eigen::Vector3d(4.0, -3.0, 2.0)));

    const GpsSample gps_sample = MakeGpsSample(200000, 4.0, -3.0);
    const GpsUpdateResult update_result = eskf.UpdateGps(gps_sample);

    EXPECT_NEAR(update_result.innovation_xy.x(), 0.0, 1e-12);
    EXPECT_NEAR(update_result.innovation_xy.y(), 0.0, 1e-12);
    EXPECT_NEAR(update_result.nis, 0.0, 1e-12);
}
