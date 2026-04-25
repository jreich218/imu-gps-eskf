#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "eskf.hpp"
#include "scene_types.hpp"

namespace {

StartupInitialization MakeStartupInitialization(double x = 0.0,
                                                double y = 0.0,
                                                double z = 0.0) {
    StartupInitialization startup_initialization;
    startup_initialization.p0_G = Eigen::Vector3d(x, y, z);
    startup_initialization.v0_G = Eigen::Vector3d(0.0, 0.0, 0.0);
    startup_initialization.q0_GI = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    startup_initialization.previous_imu_utime = 100000;
    return startup_initialization;
}

GpsSample MakeGpsSample(std::int64_t utime, double x, double y) {
    GpsSample gps_sample;
    gps_sample.utime = utime;
    gps_sample.xy = Eigen::Vector2d(x, y);
    return gps_sample;
}

}  // namespace

TEST(EskfUpdateGps, ThrowsWhenFilterIsNotInitialized) {
    Eskf eskf;
    const GpsSample gps_sample = MakeGpsSample(200000, 10.0, -5.0);

    EXPECT_THROW(eskf.UpdateGps(gps_sample), std::runtime_error);
}

TEST(EskfUpdateGps, ReturnsExpectedInnovationAndNisAtInitializationState) {
    Eskf eskf;
    eskf.Initialize(MakeStartupInitialization(1.0, -2.0, 3.0));

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
    eskf.Initialize(MakeStartupInitialization(4.0, -3.0, 2.0));

    const GpsSample gps_sample = MakeGpsSample(200000, 4.0, -3.0);
    const GpsUpdateResult update_result = eskf.UpdateGps(gps_sample);

    EXPECT_NEAR(update_result.innovation_xy.x(), 0.0, 1e-12);
    EXPECT_NEAR(update_result.innovation_xy.y(), 0.0, 1e-12);
    EXPECT_NEAR(update_result.nis, 0.0, 1e-12);
}