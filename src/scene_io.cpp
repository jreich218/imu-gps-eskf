#include "scene_io.hpp"

#include <cctype>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "scene_types.hpp"

namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace {

std::optional<std::string> SceneNumberIfFilenameMatches(
    const std::string& name, const std::string& suffix) {
    constexpr char kPrefix[] = "scene-";
    constexpr std::size_t kPrefixLen = sizeof(kPrefix) - 1;
    constexpr std::size_t kSceneDigitsLen = 4;

    if (name.rfind(kPrefix, 0) != 0) {
        return std::nullopt;
    }

    if (name.size() != kPrefixLen + kSceneDigitsLen + suffix.size()) {
        return std::nullopt;
    }

    if (name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
        return std::nullopt;
    }

    for (std::size_t i = 0; i < kSceneDigitsLen; ++i) {
        const unsigned char c =
            static_cast<unsigned char>(name[kPrefixLen + i]);
        if (!std::isdigit(c)) {
            return std::nullopt;
        }
    }

    return name.substr(kPrefixLen, kSceneDigitsLen);
}

std::optional<std::string> PoseSceneNumber(const std::string& name) {
    return SceneNumberIfFilenameMatches(name, "_pose.json");
}

std::optional<std::string> ImuSceneNumber(const std::string& name) {
    return SceneNumberIfFilenameMatches(name, "_ms_imu.json");
}

SceneInputs BundledSceneInputs(const fs::path& scenarios_dir) {
    SceneInputs scene_inputs;
    scene_inputs.pose_path = scenarios_dir / "scene_pose.json";
    scene_inputs.imu_path = scenarios_dir / "scene_ms_imu.json";
    return scene_inputs;
}

bool HasMatchingPair(const SceneInputs& scene_inputs) {
    return !scene_inputs.pose_path.empty() && !scene_inputs.imu_path.empty();
}

Json LoadJsonFile(const fs::path& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Could not open JSON file: " + path.string());
    }

    Json json;
    file >> json;
    return json;
}

std::vector<PoseSample> LoadPoseSamples(const Json& json) {
    std::vector<PoseSample> pose_samples;
    pose_samples.reserve(json.size());

    for (const auto& entry : json) {
        PoseSample pose_sample;
        pose_sample.utime = entry.at("utime").get<long long>();

        const auto& pos = entry.at("pos");
        pose_sample.pos = Eigen::Vector3d(pos.at(0).get<double>(),
                                          pos.at(1).get<double>(),
                                          pos.at(2).get<double>());

        const auto& orientation = entry.at("orientation");
        pose_sample.orientation =
            Eigen::Quaterniond(orientation.at(0).get<double>(),
                               orientation.at(1).get<double>(),
                               orientation.at(2).get<double>(),
                               orientation.at(3).get<double>());

        const auto& vel = entry.at("vel");
        pose_sample.vel = Eigen::Vector3d(vel.at(0).get<double>(),
                                          vel.at(1).get<double>(),
                                          vel.at(2).get<double>());

        pose_samples.push_back(pose_sample);
    }

    return pose_samples;
}

std::vector<ImuSample> LoadImuSamples(const Json& json) {
    std::vector<ImuSample> imu_samples;
    imu_samples.reserve(json.size());

    for (const auto& entry : json) {
        ImuSample imu_sample;
        imu_sample.utime = entry.at("utime").get<long long>();

        const auto& linear_accel = entry.at("linear_accel");
        imu_sample.specific_force =
            Eigen::Vector3d(linear_accel.at(0).get<double>(),
                            linear_accel.at(1).get<double>(),
                            linear_accel.at(2).get<double>());

        const auto& rotation_rate = entry.at("rotation_rate");
        imu_sample.rotation_rate =
            Eigen::Vector3d(rotation_rate.at(0).get<double>(),
                            rotation_rate.at(1).get<double>(),
                            rotation_rate.at(2).get<double>());

        const auto& q = entry.at("q");
        imu_sample.q_AI =
            Eigen::Quaterniond(q.at(0).get<double>(),
                               q.at(1).get<double>(),
                               q.at(2).get<double>(),
                               q.at(3).get<double>());

        imu_samples.push_back(imu_sample);
    }

    return imu_samples;
}

void DropSingleTrailingPoseSample(std::vector<PoseSample>& pose_samples,
                                  const std::vector<ImuSample>& imu_samples) {
    if (pose_samples.empty() || imu_samples.empty()) {
        return;
    }

    const long long last_imu_utime = imu_samples.back().utime;
    std::size_t num_pose_samples_after_last_imu = 0;

    for (const PoseSample& pose_sample : pose_samples) {
        if (pose_sample.utime > last_imu_utime) {
            ++num_pose_samples_after_last_imu;
        }
    }

    if (num_pose_samples_after_last_imu == 1 &&
        pose_samples.back().utime > last_imu_utime) {
        pose_samples.pop_back();
    }
}

}  // namespace

SceneInputs ChooseSceneInputs() {
    const fs::path scenarios_dir = "scenarios";
    std::unordered_map<std::string, SceneInputs> scene_inputs_by_scene_number;
    std::size_t num_matching_pairs = 0;
    SceneInputs matching_pair;

    for (const auto& entry : fs::directory_iterator(scenarios_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const fs::path path = entry.path();
        const std::string name = path.filename().string();
        if (const auto scene_number = PoseSceneNumber(name)) {
            SceneInputs& scene_inputs =
                scene_inputs_by_scene_number[*scene_number];
            const bool had_matching_pair = HasMatchingPair(scene_inputs);
            scene_inputs.pose_path = path;

            if (!had_matching_pair && HasMatchingPair(scene_inputs)) {
                ++num_matching_pairs;
                matching_pair = scene_inputs;
            }
            continue;
        }

        if (const auto scene_number = ImuSceneNumber(name)) {
            SceneInputs& scene_inputs =
                scene_inputs_by_scene_number[*scene_number];
            const bool had_matching_pair = HasMatchingPair(scene_inputs);
            scene_inputs.imu_path = path;

            if (!had_matching_pair && HasMatchingPair(scene_inputs)) {
                ++num_matching_pairs;
                matching_pair = scene_inputs;
            }
        }
    }

    if (num_matching_pairs == 1) {
        return matching_pair;
    }

    const SceneInputs scene_inputs = BundledSceneInputs(scenarios_dir);
    if (!fs::is_regular_file(scene_inputs.pose_path) ||
        !fs::is_regular_file(scene_inputs.imu_path)) {
        throw std::runtime_error(
            "Could not find bundled input pair in scenarios.");
    }

    return scene_inputs;
}

LoadedScene LoadScene(const SceneInputs& scene_inputs) {
    LoadedScene loaded_scene;
    loaded_scene.pose_samples =
        LoadPoseSamples(LoadJsonFile(scene_inputs.pose_path));
    loaded_scene.imu_samples =
        LoadImuSamples(LoadJsonFile(scene_inputs.imu_path));
    DropSingleTrailingPoseSample(loaded_scene.pose_samples,
                                 loaded_scene.imu_samples);
    return loaded_scene;
}
