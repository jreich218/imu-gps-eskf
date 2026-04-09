#include <optional>
#include <vector>

#include "gps_generation.hpp"
#include "initialization.hpp"
#include "scene_io.hpp"
#include "scene_types.hpp"

int main() {
    // #####################################
    // 1. Select and load pose and IMU data
    // #####################################
    SceneInputs scene_inputs = ChooseSceneInputs();
    LoadedScene loaded_scene = LoadScene(scene_inputs);
    (void)loaded_scene;
    // #####################################
    // 2. Generate synthetic GPS from pose
    // #####################################
    // See include/gps_generation.hpp
    std::vector<GpsSample> gps_samples =
        GenerateGpsSamplesFromPose(loaded_scene.pose_samples);
    // #####################################
    // 3. Create the initial state for the filter
    // #####################################
    std::optional<StartupInitialization> startup_initialization =
        ComputeStartupInitialization(loaded_scene.imu_samples, gps_samples);

    return 0;
}
