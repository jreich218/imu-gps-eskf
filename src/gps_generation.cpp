#include "gps_generation.hpp"

#include <cstdlib>
#include <cstdint>
#include <random>

#include "scene_types.hpp"

namespace {

constexpr std::int64_t kMinGpsDtUs = 100000;
constexpr double kGpsSigmaXY = 2.0;
constexpr unsigned int kDefaultGpsSeed = 1234;
constexpr const char* kGpsSeedEnvVar = "IMU_GPS_ESKF_GPS_SEED";

unsigned int GpsSeed() {
    const char* env_value = std::getenv(kGpsSeedEnvVar);
    if (env_value == nullptr || env_value[0] == '\0') {
        return kDefaultGpsSeed;
    }

    char* parse_end = nullptr;
    const unsigned long parsed = std::strtoul(env_value, &parse_end, 10);
    if (parse_end == env_value || (parse_end != nullptr && parse_end[0] != '\0')) {
        return kDefaultGpsSeed;
    }

    return static_cast<unsigned int>(parsed);
}

}  // namespace

std::vector<GpsSample> GenerateGpsSamplesFromPose(
    const std::vector<PoseSample>& pose_samples) {
    std::mt19937 rng(GpsSeed());
    std::normal_distribution<double> standard_normal(0.0, 1.0);

    std::vector<GpsSample> gps_samples;
    gps_samples.reserve(pose_samples.size());
    bool have_last_kept_utime = false;
    std::int64_t last_kept_utime = 0;

    for (const PoseSample& pose_sample : pose_samples) {
        const std::int64_t utime = pose_sample.utime;

        if (!have_last_kept_utime) {
            have_last_kept_utime = true;
            last_kept_utime = utime;
        } else if (utime - last_kept_utime < kMinGpsDtUs) {
            continue;
        } else {
            last_kept_utime = utime;
        }

        const double nx = kGpsSigmaXY * standard_normal(rng);
        const double ny = kGpsSigmaXY * standard_normal(rng);

        GpsSample gps_sample;
        gps_sample.utime = utime;
        gps_sample.xy =
            Eigen::Vector2d(pose_sample.pos.x() + nx, pose_sample.pos.y() + ny);
        gps_samples.push_back(gps_sample);
    }

    return gps_samples;
}
