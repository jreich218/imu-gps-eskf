#include <optional>
#include <vector>

#include "gps_generation.hpp"
#include "initialization.hpp"
#include "scene_io.hpp"
#include "scene_types.hpp"

int main() {
    // Select and load pose and IMU data
    SceneInputs scene_inputs = ChooseSceneInputs();
    LoadedScene loaded_scene = LoadScene(scene_inputs);
    (void)loaded_scene;
    // Generate synthetic GPS from pose
    std::vector<GpsSample> gps_samples =
        GenerateGpsSamplesFromPose(loaded_scene.pose_samples);
    // Initial state for the filter
    std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(loaded_scene.imu_samples, gps_samples);
    // Initialize filter covariance

    return 0;
}
