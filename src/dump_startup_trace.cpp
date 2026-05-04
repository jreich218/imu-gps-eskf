#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "gps_generation.hpp"
#include "initialization.hpp"
#include "scene_io.hpp"
#include "scene_types.hpp"

namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace {

double YawFromQuaternion(const Eigen::Quaterniond& q) {
    const Eigen::Quaterniond normalized = q.normalized();
    const double siny_cosp =
        2.0 * (normalized.w() * normalized.z() +
               normalized.x() * normalized.y());
    const double cosy_cosp =
        1.0 - 2.0 * (normalized.y() * normalized.y() +
                     normalized.z() * normalized.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

Json Vec2ToJson(const Eigen::Vector2d& v) {
    return Json::array({v.x(), v.y()});
}

Json Vec3ToJson(const Eigen::Vector3d& v) {
    return Json::array({v.x(), v.y(), v.z()});
}

Json QuaternionToJson(const Eigen::Quaterniond& q) {
    const Eigen::Quaterniond normalized = q.normalized();
    return Json::array({normalized.w(),
                        normalized.x(),
                        normalized.y(),
                        normalized.z()});
}

Json GpsSamplesToJson(const std::vector<GpsSample>& gps_samples) {
    Json gps_json = Json::array();
    for (const GpsSample& gps_sample : gps_samples) {
        gps_json.push_back(
            {{"utime", gps_sample.utime},
             {"x", gps_sample.xy.x()},
             {"y", gps_sample.xy.y()}});
    }
    return gps_json;
}

Json PathPointsToJson(const std::vector<Eigen::Vector2d>& path_xy) {
    Json path_json = Json::array();
    for (const Eigen::Vector2d& point : path_xy) {
        path_json.push_back(Vec2ToJson(point));
    }
    return path_json;
}

Json StartupTraceFramesToJson(const std::vector<StartupTraceFrame>& frames) {
    Json frames_json = Json::array();
    for (const StartupTraceFrame& frame : frames) {
        frames_json.push_back(
            {{"fit_end_index", frame.fit_end_index},
             {"gps_points_used", frame.gps_points_used},
             {"end_utime", frame.end_utime},
             {"latest_gps_xy", Vec2ToJson(frame.latest_gps_xy)},
             {"fitted_endpoint_xy", Vec2ToJson(frame.fitted_endpoint_xy)},
             {"fitted_velocity_xy", Vec2ToJson(frame.fitted_velocity_xy)},
             {"fitted_speed_mps", frame.fitted_speed_mps},
             {"fitted_yaw_rad", frame.fitted_yaw_rad},
             {"selected_yaw_rad", frame.selected_yaw_rad},
             {"global_yaw_rad", frame.global_yaw_rad},
             {"pca_yaw_rad", frame.pca_yaw_rad},
             {"projected_separation_m", frame.projected_separation_m},
             {"line_like", frame.line_like},
             {"wheel_speed_mps", frame.wheel_speed_mps},
             {"wheel_supported_travel_m", frame.wheel_supported_travel_m},
             {"required_wheel_supported_travel_m",
              frame.required_wheel_supported_travel_m},
             {"path_xy", PathPointsToJson(frame.path_xy)},
             {"truth_xy", Vec2ToJson(frame.truth.position_xy)},
             {"truth_yaw_rad", frame.truth.yaw_rad},
             {"truth_speed_mps", frame.truth.speed_mps}});
    }
    return frames_json;
}

Json StartupInitializationToJson(
    const std::optional<StartupInitialization>& startup_initialization) {
    if (!startup_initialization.has_value()) {
        return nullptr;
    }

    return {{"p0_G", Vec3ToJson(startup_initialization->p0_G)},
            {"v0_G", Vec3ToJson(startup_initialization->v0_G)},
            {"q0_GI", QuaternionToJson(startup_initialization->q0_GI)},
            {"yaw0_rad", YawFromQuaternion(startup_initialization->q0_GI)},
            {"previous_imu_utime",
             startup_initialization->previous_imu_utime},
            {"first_unprocessed_imu_index",
             startup_initialization->first_unprocessed_imu_index},
            {"first_unprocessed_gps_index",
             startup_initialization->first_unprocessed_gps_index}};
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 5) {
            throw std::runtime_error(
                "usage: dump_startup_trace <pose_json> <imu_json> <wheel_json> <out_json>");
        }

        SceneInputs scene_inputs;
        scene_inputs.pose_path = argv[1];
        scene_inputs.imu_path = argv[2];
        scene_inputs.wheel_speed_path = argv[3];
        const fs::path out_json_path = argv[4];

        const LoadedScene loaded_scene = LoadScene(scene_inputs);
        const std::vector<GpsSample> gps_samples =
            GenerateGpsSamplesFromPose(loaded_scene.pose_samples);
        const StartupTraceResult trace_result = TraceStartupInitialization(
            loaded_scene.pose_samples,
            loaded_scene.imu_samples,
            gps_samples,
            loaded_scene.wheel_speed_samples);

        const std::string pose_filename =
            scene_inputs.pose_path.filename().string();
        const std::string suffix = "_pose.json";
        const std::string scene_name =
            pose_filename.size() >= suffix.size() &&
                    pose_filename.compare(
                        pose_filename.size() - suffix.size(), suffix.size(),
                        suffix) == 0
                ? pose_filename.substr(0, pose_filename.size() - suffix.size())
                : scene_inputs.pose_path.stem().string();

        Json out_json;
        out_json["scene"] = scene_name;
        out_json["pose_path"] = scene_inputs.pose_path.string();
        out_json["imu_path"] = scene_inputs.imu_path.string();
        out_json["wheel_speed_path"] = scene_inputs.wheel_speed_path.string();
        out_json["first_usable_gps_index"] =
            trace_result.first_usable_gps_index.has_value()
                ? Json(*trace_result.first_usable_gps_index)
                : Json(nullptr);
        out_json["gps_samples"] = GpsSamplesToJson(gps_samples);
        out_json["frames"] = StartupTraceFramesToJson(trace_result.frames);
        out_json["ready_frame_index"] =
            trace_result.ready_frame_index.has_value()
                ? Json(*trace_result.ready_frame_index)
                : Json(nullptr);
        out_json["startup_initialization"] =
            StartupInitializationToJson(trace_result.startup_initialization);

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
