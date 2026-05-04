#include "gps_generation.hpp"
#include "scene_types.hpp"

#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace {

constexpr const char* kGpsSeedEnvVar = "IMU_GPS_ESKF_GPS_SEED";

PoseSample MakePoseSample(std::int64_t utime, double x, double y) {
  PoseSample pose_sample;
  pose_sample.utime = utime;
  pose_sample.pos = Eigen::Vector3d(x, y, 0.0);
  pose_sample.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  pose_sample.vel = Eigen::Vector3d(1.0, 0.0, 0.0);
  return pose_sample;
}

class ScopedGpsSeedEnvVar {
 public:
  ScopedGpsSeedEnvVar() {
    const char* previous = std::getenv(kGpsSeedEnvVar);
    if (previous != nullptr) {
      had_previous_value_ = true;
      previous_value_ = previous;
    }
  }

  ~ScopedGpsSeedEnvVar() {
    if (had_previous_value_) {
      setenv(kGpsSeedEnvVar, previous_value_.c_str(), 1);
    } else {
      unsetenv(kGpsSeedEnvVar);
    }
  }

 private:
  bool had_previous_value_ = false;
  std::string previous_value_;
};

}  // namespace

TEST(GenerateGpsSamplesFromPose, KeepsFirstPoseTimestamp) {
  const std::vector<PoseSample> pose_samples = {
      MakePoseSample(1000, 0.0, 0.0),
      MakePoseSample(21000, 1.0, 0.0),
      MakePoseSample(41000, 2.0, 0.0),
  };

  const std::vector<GpsSample> gps_samples =
      GenerateGpsSamplesFromPose(pose_samples);

  ASSERT_FALSE(gps_samples.empty());
  EXPECT_EQ(gps_samples.front().utime, pose_samples.front().utime);
}

TEST(GenerateGpsSamplesFromPose, Uses100msGatingFromLastKeptSample) {
  constexpr std::int64_t kBaseUtime = 1000000;

  const std::vector<PoseSample> pose_samples = {
      MakePoseSample(kBaseUtime + 0, 0.0, 0.0),
      MakePoseSample(kBaseUtime + 99999, 1.0, 0.0),
      MakePoseSample(kBaseUtime + 100000, 2.0, 0.0),
      MakePoseSample(kBaseUtime + 150000, 3.0, 0.0),
      MakePoseSample(kBaseUtime + 199999, 4.0, 0.0),
      MakePoseSample(kBaseUtime + 200000, 5.0, 0.0),
      MakePoseSample(kBaseUtime + 300000, 6.0, 0.0),
  };

  const std::vector<GpsSample> gps_samples =
      GenerateGpsSamplesFromPose(pose_samples);

  ASSERT_EQ(gps_samples.size(), 4U);
  EXPECT_EQ(gps_samples[0].utime, kBaseUtime + 0);
  EXPECT_EQ(gps_samples[1].utime, kBaseUtime + 100000);
  EXPECT_EQ(gps_samples[2].utime, kBaseUtime + 200000);
  EXPECT_EQ(gps_samples[3].utime, kBaseUtime + 300000);
}

TEST(GenerateGpsSamplesFromPose, IsDeterministicForSameInput) {
  const std::vector<PoseSample> pose_samples = {
      MakePoseSample(1000000, 0.0, 0.0),
      MakePoseSample(1020000, 1.0, 0.5),
      MakePoseSample(1040000, 2.0, 1.0),
      MakePoseSample(1060000, 3.0, 1.5),
      MakePoseSample(1080000, 4.0, 2.0),
      MakePoseSample(1100000, 5.0, 2.5),
      MakePoseSample(1120000, 6.0, 3.0),
      MakePoseSample(1140000, 7.0, 3.5),
      MakePoseSample(1160000, 8.0, 4.0),
      MakePoseSample(1180000, 9.0, 4.5),
      MakePoseSample(1200000, 10.0, 5.0),
  };

  const std::vector<GpsSample> gps_samples_a =
      GenerateGpsSamplesFromPose(pose_samples);
  const std::vector<GpsSample> gps_samples_b =
      GenerateGpsSamplesFromPose(pose_samples);

  ASSERT_EQ(gps_samples_a.size(), gps_samples_b.size());
  for (std::size_t i = 0; i < gps_samples_a.size(); ++i) {
    EXPECT_EQ(gps_samples_a[i].utime, gps_samples_b[i].utime);
    EXPECT_DOUBLE_EQ(gps_samples_a[i].xy.x(), gps_samples_b[i].xy.x());
    EXPECT_DOUBLE_EQ(gps_samples_a[i].xy.y(), gps_samples_b[i].xy.y());
  }
}

TEST(GenerateGpsSamplesFromPose, Uniform20msPoseStreamProducesEveryFifthSample) {
  std::vector<PoseSample> pose_samples;
  pose_samples.reserve(16);

  constexpr std::int64_t kBaseUtime = 2000000;
  constexpr std::int64_t kPoseDtUs = 20000;

  for (int i = 0; i < 16; ++i) {
    pose_samples.push_back(
        MakePoseSample(kBaseUtime + i * kPoseDtUs, static_cast<double>(i), 0.0));
  }

  const std::vector<GpsSample> gps_samples =
      GenerateGpsSamplesFromPose(pose_samples);

  ASSERT_EQ(gps_samples.size(), 4U);
  EXPECT_EQ(gps_samples[0].utime, kBaseUtime + 0 * kPoseDtUs);
  EXPECT_EQ(gps_samples[1].utime, kBaseUtime + 5 * kPoseDtUs);
  EXPECT_EQ(gps_samples[2].utime, kBaseUtime + 10 * kPoseDtUs);
  EXPECT_EQ(gps_samples[3].utime, kBaseUtime + 15 * kPoseDtUs);
}

TEST(GenerateGpsSamplesFromPose, UsesSeedOverrideWhenProvided) {
  ScopedGpsSeedEnvVar scoped_env_var;

  const std::vector<PoseSample> pose_samples = {
      MakePoseSample(1000000, 0.0, 0.0),
      MakePoseSample(1100000, 1.0, 0.5),
      MakePoseSample(1200000, 2.0, 1.0),
      MakePoseSample(1300000, 3.0, 1.5),
      MakePoseSample(1400000, 4.0, 2.0),
  };

  setenv(kGpsSeedEnvVar, "1", 1);
  const std::vector<GpsSample> gps_samples_seed_1_a =
      GenerateGpsSamplesFromPose(pose_samples);
  const std::vector<GpsSample> gps_samples_seed_1_b =
      GenerateGpsSamplesFromPose(pose_samples);

  setenv(kGpsSeedEnvVar, "2", 1);
  const std::vector<GpsSample> gps_samples_seed_2 =
      GenerateGpsSamplesFromPose(pose_samples);

  ASSERT_EQ(gps_samples_seed_1_a.size(), gps_samples_seed_1_b.size());
  ASSERT_EQ(gps_samples_seed_1_a.size(), gps_samples_seed_2.size());

  bool found_difference = false;
  for (std::size_t i = 0; i < gps_samples_seed_1_a.size(); ++i) {
    EXPECT_EQ(gps_samples_seed_1_a[i].utime, gps_samples_seed_1_b[i].utime);
    EXPECT_DOUBLE_EQ(gps_samples_seed_1_a[i].xy.x(), gps_samples_seed_1_b[i].xy.x());
    EXPECT_DOUBLE_EQ(gps_samples_seed_1_a[i].xy.y(), gps_samples_seed_1_b[i].xy.y());

    if (gps_samples_seed_1_a[i].xy.x() != gps_samples_seed_2[i].xy.x() ||
        gps_samples_seed_1_a[i].xy.y() != gps_samples_seed_2[i].xy.y()) {
      found_difference = true;
    }
  }

  EXPECT_TRUE(found_difference);
}
