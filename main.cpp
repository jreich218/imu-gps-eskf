#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

#include "gps_generation.hpp"
#include "initialization.hpp"
#include "outputs.hpp"
#include "runtime_loop.hpp"
#include "scene_io.hpp"
#include "scene_types.hpp"

int main() {
    try {
        const SceneInputs scene_inputs = ChooseSceneInputs();
        const LoadedScene loaded_scene = LoadScene(scene_inputs);

        const std::vector<GpsSample> gps_samples =
            GenerateGpsSamplesFromPose(loaded_scene.pose_samples);

        const std::optional<StartupInitialization> startup_initialization =
            ComputeStartupInitialization(loaded_scene.imu_samples,
                                         gps_samples,
                                         loaded_scene.wheel_speed_samples);
        if (!startup_initialization.has_value()) {
            throw std::runtime_error(
                "Could not compute startup initialization.");
        }

        const EskfRunResult run_result =
            RunEskfLoop(loaded_scene, gps_samples, *startup_initialization);

        WriteGpsJson(gps_samples);
        const auto log_path = WriteEskfSimLogCsv(run_result);
        PrintRunSummary(run_result, log_path, std::cout);

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
