#include "initialization.hpp"
#include "scene_types.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace {

ImuSample MakeImuSample(std::int64_t utime,
                        double yaw_rate_z,
                        double q_ai_yaw_rad = 0.0) {
    ImuSample imu_sample;
    imu_sample.utime = utime;
    imu_sample.rotation_rate = Eigen::Vector3d(0.0, 0.0, yaw_rate_z);
    imu_sample.q_AI =
        Eigen::Quaterniond(Eigen::AngleAxisd(q_ai_yaw_rad,
                                             Eigen::Vector3d::UnitZ()));
    return imu_sample;
}

GpsSample MakeGpsSample(std::int64_t utime, double x, double y) {
    GpsSample gps_sample;
    gps_sample.utime = utime;
    gps_sample.xy = Eigen::Vector2d(x, y);
    return gps_sample;
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
        MakeImuSample(100000, 0.0), MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0), MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0), MakeImuSample(600000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(0, -1.0, 0.0),      MakeGpsSample(100000, 0.0, 0.0),
        MakeGpsSample(200000, 3.0, 0.0),  MakeGpsSample(300000, 6.0, 0.0),
        MakeGpsSample(400000, 9.0, 0.0),  MakeGpsSample(500000, 12.0, 0.0),
        MakeGpsSample(600000, 15.0, 0.0),
    };

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(imu_samples, gps_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_EQ(startup_initialization->next_imu_index, 5U);
    EXPECT_EQ(startup_initialization->next_gps_index, 6U);
    EXPECT_EQ(startup_initialization->utime_last_imu, 500000);
    EXPECT_NEAR(startup_initialization->p0_G.x(), 12.0, 1e-9);
    EXPECT_NEAR(startup_initialization->p0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->p0_G.z(), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.x(), 30.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.z(), 0.0, 1e-9);
    EXPECT_NEAR(YawFromQuaternion(startup_initialization->q0_GI), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->q0_GI.norm(), 1.0, 1e-12);
}

TEST(ComputeStartupInitialization, CarriesTheReadyStateToTheFirstLaterImuSample) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0), MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0), MakeImuSample(400000, 0.0),
        MakeImuSample(520000, 0.5), MakeImuSample(620000, 0.5),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),  MakeGpsSample(200000, 3.0, 0.0),
        MakeGpsSample(300000, 6.0, 0.0),  MakeGpsSample(400000, 9.0, 0.0),
        MakeGpsSample(500000, 12.0, 0.0), MakeGpsSample(600000, 15.0, 0.0),
    };

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(imu_samples, gps_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_EQ(startup_initialization->next_imu_index, 5U);
    EXPECT_EQ(startup_initialization->next_gps_index, 5U);
    EXPECT_EQ(startup_initialization->utime_last_imu, 520000);
    EXPECT_NEAR(startup_initialization->p0_G.x(), 12.6, 1e-9);
    EXPECT_NEAR(startup_initialization->p0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.x(), 30.0, 1e-9);
    EXPECT_NEAR(startup_initialization->v0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(YawFromQuaternion(startup_initialization->q0_GI), 0.01, 1e-9);
}

TEST(ComputeStartupInitialization,
     ReturnsNulloptWhenProjectedSeparationStaysBelowTheRequiredThreshold) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0), MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0), MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0), MakeImuSample(600000, 0.0),
        MakeImuSample(700000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0), MakeGpsSample(200000, 1.0, 0.0),
        MakeGpsSample(300000, 2.0, 0.0), MakeGpsSample(400000, 3.0, 0.0),
        MakeGpsSample(500000, 4.0, 0.0), MakeGpsSample(600000, 5.0, 0.0),
        MakeGpsSample(700000, 6.0, 0.0),
    };

    EXPECT_FALSE(
        ComputeStartupInitialization(imu_samples, gps_samples).has_value());
}

TEST(ComputeStartupInitialization, RelaxesProjectedSeparationAfterEightyGpsPoints) {
    std::vector<ImuSample> imu_samples;
    std::vector<GpsSample> gps_samples;
    imu_samples.reserve(80);
    gps_samples.reserve(80);

    for (std::size_t index = 0; index < 80; ++index) {
        const std::int64_t utime =
            100000 + static_cast<std::int64_t>(index) * 100000;
        imu_samples.push_back(MakeImuSample(utime, 0.0));
        gps_samples.push_back(
            MakeGpsSample(utime, 0.1 * static_cast<double>(index), 0.0));
    }

    const std::optional<StartupInitialization> startup_before_eighty =
        ComputeStartupInitialization(
            imu_samples,
            std::vector<GpsSample>(gps_samples.begin(), gps_samples.end() - 1));
    EXPECT_FALSE(startup_before_eighty.has_value());

    const std::optional<StartupInitialization> startup_at_eighty =
        ComputeStartupInitialization(imu_samples, gps_samples);

    ASSERT_TRUE(startup_at_eighty.has_value());
    EXPECT_EQ(startup_at_eighty->next_imu_index, 80U);
    EXPECT_EQ(startup_at_eighty->next_gps_index, 80U);
    EXPECT_EQ(startup_at_eighty->utime_last_imu, 8000000);
    EXPECT_NEAR(startup_at_eighty->p0_G.x(), 7.9, 1e-9);
    EXPECT_NEAR(startup_at_eighty->p0_G.y(), 0.0, 1e-9);
    EXPECT_NEAR(startup_at_eighty->v0_G.norm(), 1.0, 1e-6);
    EXPECT_NEAR(YawFromQuaternion(startup_at_eighty->q0_GI), 0.0, 1e-9);
}

TEST(ComputeStartupInitialization, ReturnsNulloptWhenStartupNeverLooksTrustworthy) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0), MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0), MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0), MakeImuSample(600000, 0.0),
        MakeImuSample(700000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0), MakeGpsSample(200000, 1.0, 0.0),
        MakeGpsSample(300000, 2.0, 2.0), MakeGpsSample(400000, 3.0, -2.0),
        MakeGpsSample(500000, 4.0, 2.0), MakeGpsSample(600000, 5.0, -2.0),
        MakeGpsSample(700000, 6.0, 2.0),
    };

    EXPECT_FALSE(
        ComputeStartupInitialization(imu_samples, gps_samples).has_value());
}

TEST(ComputeStartupInitialization,
     UsesPrincipalDirectionWhenLineLikeHeadingIsStillWobbling) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0), MakeImuSample(200000, 0.0),
        MakeImuSample(300000, 0.0), MakeImuSample(400000, 0.0),
        MakeImuSample(500000, 0.0), MakeImuSample(600000, 0.0),
        MakeImuSample(700000, 0.0),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),   MakeGpsSample(200000, 3.0, 1.8),
        MakeGpsSample(300000, 6.0, -1.8),  MakeGpsSample(400000, 9.0, 1.8),
        MakeGpsSample(500000, 12.0, -1.8), MakeGpsSample(600000, 15.0, 1.8),
        MakeGpsSample(700000, 18.0, -1.8),
    };

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(imu_samples, gps_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_EQ(startup_initialization->next_imu_index, 5U);
    EXPECT_EQ(startup_initialization->next_gps_index, 5U);
    EXPECT_NEAR(YawFromQuaternion(startup_initialization->q0_GI), 0.0, 0.15);
    EXPECT_NEAR(startup_initialization->v0_G.head<2>().norm(), 30.2152, 0.05);
}

TEST(ComputeStartupInitialization,
     UsesCurvatureAwareGlobalYawWhenTheCloudIsNotLineLike) {
    constexpr double kRadiusM = 12.0;
    constexpr double kHeadingStepRad = 1.0 / 6.0;
    const double yaw_rate_z = kHeadingStepRad / 0.1;

    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, yaw_rate_z), MakeImuSample(200000, yaw_rate_z),
        MakeImuSample(300000, yaw_rate_z), MakeImuSample(400000, yaw_rate_z),
        MakeImuSample(500000, yaw_rate_z), MakeImuSample(600000, yaw_rate_z),
        MakeImuSample(700000, yaw_rate_z),
    };

    std::vector<GpsSample> gps_samples;
    gps_samples.reserve(7);
    for (std::size_t index = 0; index < 7; ++index) {
        const double heading_rad = static_cast<double>(index) * kHeadingStepRad;
        Eigen::Vector2d xy = CircleArcXy(kRadiusM, heading_rad);
        if (index == 5) {
            xy += Eigen::Vector2d(0.0, 2.0);
        } else if (index == 6) {
            xy += Eigen::Vector2d(0.0, -2.0);
        }

        gps_samples.push_back(MakeGpsSample(
            100000 + static_cast<std::int64_t>(index) * 100000, xy.x(), xy.y()));
    }

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(imu_samples, gps_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_EQ(startup_initialization->next_imu_index, 6U);
    EXPECT_EQ(startup_initialization->next_gps_index, 6U);
    EXPECT_EQ(startup_initialization->utime_last_imu, 600000);
    EXPECT_NEAR(YawFromQuaternion(startup_initialization->q0_GI), 0.56, 0.12);
    EXPECT_NEAR(startup_initialization->v0_G.head<2>().norm(), 20.8517, 0.1);
}

TEST(ComputeStartupInitialization, UsesHandoffImuOrientationToBuildQ0GI) {
    const std::vector<ImuSample> imu_samples = {
        MakeImuSample(100000, 0.0, 0.3), MakeImuSample(200000, 0.0, 0.3),
        MakeImuSample(300000, 0.0, 0.3), MakeImuSample(400000, 0.0, 0.3),
        MakeImuSample(500000, 0.0, 0.3), MakeImuSample(600000, 0.0, 0.3),
    };

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(100000, 0.0, 0.0),  MakeGpsSample(200000, 3.0, 0.0),
        MakeGpsSample(300000, 6.0, 0.0),  MakeGpsSample(400000, 9.0, 0.0),
        MakeGpsSample(500000, 12.0, 0.0), MakeGpsSample(600000, 15.0, 0.0),
    };

    const std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(imu_samples, gps_samples);

    ASSERT_TRUE(startup_initialization.has_value());
    EXPECT_EQ(startup_initialization->next_imu_index, 5U);
    EXPECT_NEAR(YawFromQuaternion(startup_initialization->q0_GI), 0.0, 1e-9);
}
