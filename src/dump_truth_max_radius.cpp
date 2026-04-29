#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "gps_generation.hpp"
#include "scene_io.hpp"
#include "scene_types.hpp"

namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace {

std::size_t FindFirstUsableGpsIndex(const std::vector<ImuSample>& imu_samples,
                                    const std::vector<GpsSample>& gps_samples) {
    if (imu_samples.empty()) {
        throw std::runtime_error("IMU stream is empty.");
    }

    std::size_t gps_index = 0;
    while (gps_index < gps_samples.size() &&
           gps_samples[gps_index].utime < imu_samples.front().utime) {
        ++gps_index;
    }

    if (gps_index >= gps_samples.size()) {
        throw std::runtime_error("No usable GPS sample exists.");
    }

    return gps_index;
}

double ComputeTruthMaxRadiusM(const std::vector<PoseSample>& pose_samples,
                              std::int64_t startup_begin_utime) {
    bool have_reference = false;
    Eigen::Vector2d reference_xy = Eigen::Vector2d::Zero();
    double max_radius_m = 0.0;

    for (const PoseSample& pose_sample : pose_samples) {
        if (pose_sample.utime < startup_begin_utime) {
            continue;
        }

        const Eigen::Vector2d current_xy = pose_sample.pos.head<2>();
        if (!have_reference) {
            reference_xy = current_xy;
            have_reference = true;
        }

        max_radius_m =
            std::max(max_radius_m, (current_xy - reference_xy).norm());
    }

    if (!have_reference) {
        throw std::runtime_error(
            "No loaded pose sample exists at or after startup begin time.");
    }

    return max_radius_m;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 4) {
            throw std::runtime_error(
                "usage: dump_truth_max_radius <pose_json> <imu_json> <out_json>");
        }

        SceneInputs scene_inputs;
        scene_inputs.pose_path = argv[1];
        scene_inputs.imu_path = argv[2];
        const fs::path out_json_path = argv[3];

        const LoadedScene loaded_scene = LoadScene(scene_inputs);
        const std::vector<GpsSample> gps_samples =
            GenerateGpsSamplesFromPose(loaded_scene.pose_samples);
        const std::size_t startup_begin_gps_index =
            FindFirstUsableGpsIndex(loaded_scene.imu_samples, gps_samples);
        const std::int64_t startup_begin_utime =
            gps_samples[startup_begin_gps_index].utime;
        const double max_distance_m =
            ComputeTruthMaxRadiusM(loaded_scene.pose_samples,
                                   startup_begin_utime);

        Json out_json;
        out_json["pose_path"] = scene_inputs.pose_path.string();
        out_json["imu_path"] = scene_inputs.imu_path.string();
        out_json["startup_begin_gps_index"] = startup_begin_gps_index;
        out_json["startup_begin_utime"] = startup_begin_utime;
        out_json["max_distance_m"] = max_distance_m;

        std::ofstream out_file(out_json_path);
        if (!out_file) {
            throw std::runtime_error("Could not open output JSON file.");
        }

        out_file << out_json.dump(2) << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
