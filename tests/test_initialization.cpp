#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <vector>

#include "initialization.hpp"
#include "scene_types.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWheelRadiusM = 0.305;

ImuSample MakeImuSample(std::int64_t utime,
                        double yaw_rate_z,
                        double q_ai_yaw_rad = 0.0) {
    ImuSample imu_sample;
    imu_sample.utime = utime;
    imu_sample.specific_force = Eigen::Vector3d(0.0, 0.0, 9.8);
    imu_sample.rotation_rate = Eigen::Vector3d(0.0, 0.0, yaw_rate_z);
    imu_sample.q_AI = Eigen::Quaterniond(
        Eigen::AngleAxisd(q_ai_yaw_rad, Eigen::Vector3d::UnitZ()));
    return imu_sample;
}

GpsSample MakeGpsSample(std::int64_t utime, double x, double y) {
    GpsSample gps_sample;
    gps_sample.utime = utime;
    gps_sample.xy = Eigen::Vector2d(x, y);
    return gps_sample;
}

PoseSample MakePoseSample(std::int64_t utime,
                          double x,
                          double y,
                          double yaw_rad,
                          double speed_mps) {
    PoseSample pose_sample;
    pose_sample.utime = utime;
    pose_sample.pos = Eigen::Vector3d(x, y, 0.0);
    pose_sample.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    pose_sample.vel = Eigen::Vector3d(speed_mps * std::cos(yaw_rad),
                                      speed_mps * std::sin(yaw_rad),
                                      0.0);
    return pose_sample;
}

double SpeedMpsToWheelRpm(double speed_mps) {
    const double circumference_m = 2.0 * kPi * kWheelRadiusM;
    return speed_mps * 60.0 / circumference_m;
}

WheelSpeedSample MakeUniformWheelSpeedSample(std::int64_t utime,
                                             double speed_mps) {
    const double wheel_speed_rpm = SpeedMpsToWheelRpm(speed_mps);

    WheelSpeedSample wheel_speed_sample;
    wheel_speed_sample.utime = utime;
    wheel_speed_sample.fl_wheel_speed_rpm = wheel_speed_rpm;
    wheel_speed_sample.fr_wheel_speed_rpm = wheel_speed_rpm;
    wheel_speed_sample.rl_wheel_speed_rpm = wheel_speed_rpm;
    wheel_speed_sample.rr_wheel_speed_rpm = wheel_speed_rpm;
    return wheel_speed_sample;
}

std::vector<WheelSpeedSample> MakeUniformWheelSpeedSamples(
    const std::vector<std::int64_t>& utimes, double speed_mps) {
    std::vector<WheelSpeedSample> wheel_speed_samples;
    wheel_speed_samples.reserve(utimes.size());
    for (std::int64_t utime : utimes) {
        wheel_speed_samples.push_back(
            MakeUniformWheelSpeedSample(utime, speed_mps));
    }
    return wheel_speed_samples;
}

double YawFromQuaternion(const Eigen::Quaterniond& q_GI) {
    const double w = q_GI.w();
    const double x = q_GI.x();
    const double y = q_GI.y();
    const double z = q_GI.z();
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Vector2d CircleArcXy(double radius_m, double heading_rad) {
    return Eigen::Vector2d(radius_m * std::sin(heading_rad),
                           radius_m * (1.0 - std::cos(heading_rad)));
}

}  // namespace

TEST(ComputeStartupInitialization,
     IgnoresGpsBeforeFirstImuAndReadiesOnStableHeading) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0),
        MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0),
        MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0),
        MakeImuSample(600000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(0, -1.0, 0.0),
        MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 3.0, 0.0),
        MakeGpsSample(300000, 6.0, 0.0),
        MakeGpsSample(400000, 9.0, 0.0),
        MakeGpsSample(500000, 12.0, 0.0),
        MakeGpsSample(600000, 15.0, 0.0),
    };

    const std::vector<WheelSpeedSample> wheel_speed_samples =
        MakeUniformWheelSpeedSamples(
            {100000, 200000, 300000, 400000, 500000, 600000}, 30.0);

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(
            imu_samples, gps_samples, wheel_speed_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_EQ(startup_initialization->first_unprocessed_imu_index, 5U);
    EXPECT_EQ(startup_initialization->first_unprocessed_gps_index, 6U);
    EXPECT_EQ(startup_initialization->previous_imu_utime, 500000);
    EXPECT_NEAR(startup_initialization->p0_G.x(), 12.0, 1e-9);
    EXPECT_NEAR(startup_initialization->p0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->p0_G.z(), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.x(), 30.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.z(), 0.0, 1e-9);
    EXPECT_NEAR(YawFromQuaternion(startup_initialization->q0_GI), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->q0_GI.norm(), 1.0, 1e-12);
}

TEST(ComputeStartupInitialization,
     CarriesTheReadyStateToTheFirstLaterImuSample) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0),
        MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0),
        MakeImuSample(400000, 0.0),
        MakeImuSample(520000, 0.5),
        MakeImuSample(620000, 0.5),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 3.0, 0.0),
        MakeGpsSample(300000, 6.0, 0.0),
        MakeGpsSample(400000, 9.0, 0.0),
        MakeGpsSample(500000, 12.0, 0.0),
        MakeGpsSample(600000, 15.0, 0.0),
    };

    const std::vector<WheelSpeedSample> wheel_speed_samples =
        MakeUniformWheelSpeedSamples(
            {100000, 200000, 300000, 400000, 500000, 600000}, 25.0);

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(
            imu_samples, gps_samples, wheel_speed_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_EQ(startup_initialization->first_unprocessed_imu_index, 5U);
    EXPECT_EQ(startup_initialization->first_unprocessed_gps_index, 5U);
    EXPECT_EQ(startup_initialization->previous_imu_utime, 520000);
    EXPECT_NEAR(startup_initialization->p0_G.x(), 12.5, 1e-9);
    EXPECT_NEAR(startup_initialization->p0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.x(), 25.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(YawFromQuaternion(startup_initialization->q0_GI), 0.01, 1e-9);
}

TEST(ComputeStartupInitialization,
     UsesWheelSpeedMagnitudeForTheHandoffVelocity) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0),
        MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0),
        MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0),
        MakeImuSample(600000, 0.0),
        MakeImuSample(700000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 4.0, 0.0),
        MakeGpsSample(300000, 8.0, 0.0),
        MakeGpsSample(400000, 12.0, 0.0),
        MakeGpsSample(500000, 16.0, 0.0),
        MakeGpsSample(600000, 20.0, 0.0),
        MakeGpsSample(700000, 24.0, 0.0),
    };

    const std::vector<WheelSpeedSample> wheel_speed_samples =
        MakeUniformWheelSpeedSamples(
            {100000, 200000, 300000, 400000, 500000, 600000, 700000}, 26.0);

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(
            imu_samples, gps_samples, wheel_speed_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_NEAR(startup_initialization->v0_G.head<2>().norm(), 26.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.x(), 26.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.y(), 0.0, 1e-9);
}

TEST(ComputeStartupInitialization,
     ReturnsNulloptWhenWheelSupportedTravelNeverReachesThreshold) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0),
        MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0),
        MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0),
        MakeImuSample(600000, 0.0),
        MakeImuSample(700000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 4.0, 0.0),
        MakeGpsSample(300000, 8.0, 0.0),
        MakeGpsSample(400000, 12.0, 0.0),
        MakeGpsSample(500000, 16.0, 0.0),
        MakeGpsSample(600000, 20.0, 0.0),
        MakeGpsSample(700000, 24.0, 0.0),
    };

    const std::vector<WheelSpeedSample> wheel_speed_samples =
        MakeUniformWheelSpeedSamples(
            {100000, 200000, 300000, 400000, 500000, 600000, 700000}, 0.0);

    EXPECT_FALSE(
        ComputeStartupInitialization(imu_samples,
                                     gps_samples,
                                     wheel_speed_samples)
            .has_value());
}

TEST(ComputeStartupInitialization,
     AllowsReadyWhenWheelSupportedTravelReachesTenMetersEvenIfProjectedSeparationDoesNot) {
    constexpr double kRadiusM = 5.0;
    constexpr double kHeadingStepRad = 0.5;
    constexpr double kSpeedMps = 25.0;
    constexpr double kYawRateZ = kHeadingStepRad / 0.1;

    std::vector<ImuSample> imu_samples;
    std::vector<GpsSample> gps_samples;
    std::vector<PoseSample> pose_samples;
    for (std::size_t index = 0; index < 6; ++index) {
        const std::int64_t utime =
            100000 + static_cast<std::int64_t>(index) * 100000;
        const double heading_rad = static_cast<double>(index) * kHeadingStepRad;
        const Eigen::Vector2d xy = CircleArcXy(kRadiusM, heading_rad);

        imu_samples.push_back(MakeImuSample(utime, kYawRateZ));
        gps_samples.push_back(MakeGpsSample(utime, xy.x(), xy.y()));
        pose_samples.push_back(
            MakePoseSample(utime, xy.x(), xy.y(), heading_rad, kSpeedMps));
    }

    const std::vector<WheelSpeedSample> wheel_speed_samples =
        MakeUniformWheelSpeedSamples(
            {100000, 200000, 300000, 400000, 500000, 600000}, kSpeedMps);

    const StartupTraceResult trace_result = TraceStartupInitialization(
        pose_samples, imu_samples, gps_samples, wheel_speed_samples);

    ASSERT_TRUE(trace_result.ready_frame_index.has_value());
    const StartupTraceFrame& ready_frame =
        trace_result.frames[*trace_result.ready_frame_index];
    EXPECT_LT(ready_frame.projected_separation_m,
              ready_frame.required_projected_separation_m);
    EXPECT_GE(ready_frame.wheel_supported_travel_m,
              ready_frame.required_wheel_supported_travel_m);
    EXPECT_EQ(ready_frame.end_utime, 500000);
}

TEST(ComputeStartupInitialization, UsesHandoffImuOrientationToBuildQ0GI) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0, 0.3),
        MakeImuSample(200000, 0.0, 0.3),
        MakeImuSample(300000, 0.0, 0.3),
        MakeImuSample(400000, 0.0, 0.3),
        MakeImuSample(500000, 0.0, 0.3),
        MakeImuSample(600000, 0.0, 0.3),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 3.0, 0.0),
        MakeGpsSample(300000, 6.0, 0.0),
        MakeGpsSample(400000, 9.0, 0.0),
        MakeGpsSample(500000, 12.0, 0.0),
        MakeGpsSample(600000, 15.0, 0.0),
    };

    const std::vector<WheelSpeedSample> wheel_speed_samples =
        MakeUniformWheelSpeedSamples(
            {100000, 200000, 300000, 400000, 500000, 600000}, 30.0);

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(
            imu_samples, gps_samples, wheel_speed_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_EQ(startup_initialization->first_unprocessed_imu_index, 5U);
    EXPECT_NEAR(YawFromQuaternion(startup_initialization->q0_GI), 0.0, 1e-9);
}

TEST(TraceStartupInitialization,
     ReportsWheelSupportedTravelAndMatchesFinalInitialization) {
    const std::vector<PoseSample> pose_samples = {
        MakePoseSample(100000, 0.0, 0.0, 0.0, 30.0),
        MakePoseSample(200000, 3.0, 0.0, 0.0, 30.0),
        MakePoseSample(300000, 6.0, 0.0, 0.0, 30.0),
        MakePoseSample(400000, 9.0, 0.0, 0.0, 30.0),
        MakePoseSample(500000, 12.0, 0.0, 0.0, 30.0),
        MakePoseSample(600000, 15.0, 0.0, 0.0, 30.0),
    };

    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0),
        MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0),
        MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0),
        MakeImuSample(600000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(0, -1.0, 0.0),
        MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 3.0, 0.0),
        MakeGpsSample(300000, 6.0, 0.0),
        MakeGpsSample(400000, 9.0, 0.0),
        MakeGpsSample(500000, 12.0, 0.0),
        MakeGpsSample(600000, 15.0, 0.0),
    };

    const std::vector<WheelSpeedSample> wheel_speed_samples =
        MakeUniformWheelSpeedSamples(
            {100000, 200000, 300000, 400000, 500000, 600000}, 30.0);

    const StartupTraceResult trace_result = TraceStartupInitialization(
        pose_samples, imu_samples, gps_samples, wheel_speed_samples);
    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(
            imu_samples, gps_samples, wheel_speed_samples);

    ASSERT_TRUE(trace_result.first_usable_gps_index.has_value());
    EXPECT_EQ(*trace_result.first_usable_gps_index, 1U);
    ASSERT_TRUE(trace_result.ready_frame_index.has_value());
    ASSERT_TRUE(trace_result.startup_initialization.has_value());
    ASSERT_TRUE(startup_initialization.has_value());

    const StartupTraceFrame& ready_frame =
        trace_result.frames[*trace_result.ready_frame_index];
    EXPECT_EQ(ready_frame.end_utime, 500000);
    EXPECT_DOUBLE_EQ(ready_frame.projected_separation_m, 12.0);
    EXPECT_DOUBLE_EQ(ready_frame.required_projected_separation_m, 10.0);
    EXPECT_DOUBLE_EQ(ready_frame.wheel_speed_mps, 30.0);
    EXPECT_DOUBLE_EQ(ready_frame.required_wheel_supported_travel_m, 10.0);
    EXPECT_DOUBLE_EQ(ready_frame.wheel_supported_travel_m, 12.0);
    EXPECT_NEAR(ready_frame.fitted_endpoint_xy.x(), 12.0, 1e-9);
    EXPECT_NEAR(ready_frame.fitted_endpoint_xy.y(), 0.0, 1e-9);
    EXPECT_NEAR(ready_frame.selected_yaw_rad, 0.0, 1e-9);
    EXPECT_NEAR(ready_frame.selected_speed_mps, 30.0, 1e-9);
    EXPECT_NEAR(ready_frame.truth.position_xy.x(), 12.0, 1e-9);
    EXPECT_NEAR(ready_frame.truth.position_xy.y(), 0.0, 1e-9);
    EXPECT_NEAR(ready_frame.truth.yaw_rad, 0.0, 1e-9);
    EXPECT_NEAR(ready_frame.truth.speed_mps, 30.0, 1e-9);
    EXPECT_EQ(ready_frame.path_xy.size(), 100U);

    EXPECT_EQ(trace_result.startup_initialization->first_unprocessed_imu_index,
              startup_initialization->first_unprocessed_imu_index);
    EXPECT_EQ(trace_result.startup_initialization->first_unprocessed_gps_index,
              startup_initialization->first_unprocessed_gps_index);
    EXPECT_EQ(trace_result.startup_initialization->previous_imu_utime,
              startup_initialization->previous_imu_utime);
    EXPECT_TRUE(trace_result.startup_initialization->p0_G.isApprox(
        startup_initialization->p0_G));
    EXPECT_TRUE(trace_result.startup_initialization->v0_G.isApprox(
        startup_initialization->v0_G));
    EXPECT_TRUE(trace_result.startup_initialization->q0_GI.isApprox(
        startup_initialization->q0_GI));
}

TEST(TraceStartupInitialization,
     ReturnsFramesWithoutReadyWhenWheelSupportedTravelNeverReachesThreshold) {
    const std::vector<PoseSample> pose_samples = {
        MakePoseSample(100000, 0.0, 0.0, 0.0, 0.0),
        MakePoseSample(200000, 1.0, 0.0, 0.0, 0.0),
        MakePoseSample(300000, 2.0, 0.0, 0.0, 0.0),
        MakePoseSample(400000, 3.0, 0.0, 0.0, 0.0),
        MakePoseSample(500000, 4.0, 0.0, 0.0, 0.0),
        MakePoseSample(600000, 5.0, 0.0, 0.0, 0.0),
        MakePoseSample(700000, 6.0, 0.0, 0.0, 0.0),
    };

    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0),
        MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0),
        MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0),
        MakeImuSample(600000, 0.0),
        MakeImuSample(700000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 1.0, 0.0),
        MakeGpsSample(300000, 2.0, 0.0),
        MakeGpsSample(400000, 3.0, 0.0),
        MakeGpsSample(500000, 4.0, 0.0),
        MakeGpsSample(600000, 5.0, 0.0),
        MakeGpsSample(700000, 6.0, 0.0),
    };

    const std::vector<WheelSpeedSample> wheel_speed_samples =
        MakeUniformWheelSpeedSamples(
            {100000, 200000, 300000, 400000, 500000, 600000, 700000}, 0.0);

    const StartupTraceResult trace_result = TraceStartupInitialization(
        pose_samples, imu_samples, gps_samples, wheel_speed_samples);

    ASSERT_TRUE(trace_result.first_usable_gps_index.has_value());
    EXPECT_EQ(*trace_result.first_usable_gps_index, 0U);
    EXPECT_FALSE(trace_result.frames.empty());
    EXPECT_FALSE(trace_result.ready_frame_index.has_value());
    EXPECT_FALSE(trace_result.startup_initialization.has_value());
    EXPECT_DOUBLE_EQ(trace_result.frames.back().wheel_supported_travel_m, 0.0);
    EXPECT_DOUBLE_EQ(trace_result.frames.back().required_wheel_supported_travel_m,
                     10.0);
}
