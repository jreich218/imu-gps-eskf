#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

#include "eskf.hpp"
#include "gps_generation.hpp"
#include "initialization.hpp"
#include "scene_io.hpp"
#include "scene_types.hpp"

int main() {
    try {
        // Select and load pose and IMU data
        SceneInputs scene_inputs = ChooseSceneInputs();
        LoadedScene loaded_scene = LoadScene(scene_inputs);
        // Generate synthetic GPS from pose
        std::vector<GpsSample> gps_samples =
            GenerateGpsSamplesFromPose(loaded_scene.pose_samples);
        // Initial state for the filter
        std::optional<StartupInitialization> startup_initialization =
            ComputeStartupInitialization(loaded_scene.imu_samples, gps_samples);
        if (!startup_initialization.has_value()) {
            throw std::runtime_error(
                "Could not compute startup initialization.");
        }
        // Initialize filter covariance
        Eskf eskf;
        eskf.Initialize(*startup_initialization);
        // Loop over estimator runtime here goes here.
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
