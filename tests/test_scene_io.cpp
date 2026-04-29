#include "scene_io.hpp"
#include "scene_types.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace {

class ScopedCurrentPath {
 public:
  explicit ScopedCurrentPath(const fs::path& path)
      : old_path_(fs::current_path()) {
    fs::current_path(path);
  }

  ~ScopedCurrentPath() {
    fs::current_path(old_path_);
  }

 private:
  fs::path old_path_;
};

class TempDir {
 public:
  TempDir() {
    static std::uint64_t counter = 0;
    const auto stamp =
        std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = fs::temp_directory_path() /
            ("imu-gps-eskf-scene-io-" + std::to_string(stamp) + "-" +
             std::to_string(counter++));
    fs::create_directories(path_);
  }

  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }

  const fs::path& path() const {
    return path_;
  }

 private:
  fs::path path_;
};

void TouchFile(const fs::path& path) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("Could not create file: " + path.string());
  }
}

void WriteJsonFile(const fs::path& path, const Json& json) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("Could not create file: " + path.string());
  }

  out << json.dump();
}

fs::path AddScenarioFile(const fs::path& root, const std::string& name) {
  const fs::path scenarios_dir = root / "scenarios";
  fs::create_directories(scenarios_dir);
  const fs::path path = scenarios_dir / name;
  TouchFile(path);
  return path;
}

SceneInputs WriteNuScenesSceneFiles(const fs::path& root,
                                    const Json& pose_json,
                                    const Json& imu_json,
                                    const Json& wheel_speed_json) {
  const fs::path scenarios_dir = root / "scenarios";
  fs::create_directories(scenarios_dir);

  SceneInputs scene_inputs;
  scene_inputs.pose_path = scenarios_dir / "scene-0123_pose.json";
  scene_inputs.imu_path = scenarios_dir / "scene-0123_ms_imu.json";
  const fs::path wheel_speed_path = scenarios_dir / "scene-0123_zoe_veh_info.json";

  WriteJsonFile(scene_inputs.pose_path, pose_json);
  WriteJsonFile(scene_inputs.imu_path, imu_json);
  WriteJsonFile(wheel_speed_path, wheel_speed_json);
  return scene_inputs;
}

}  // namespace

TEST(ChooseSceneInputs, ReturnsSingleMatchingNuScenesPair) {
  TempDir temp_dir;
  ScopedCurrentPath scoped_current_path(temp_dir.path());

  const fs::path pose_path =
      AddScenarioFile(temp_dir.path(), "scene-0364_pose.json");
  const fs::path imu_path =
      AddScenarioFile(temp_dir.path(), "scene-0364_ms_imu.json");

  const SceneInputs scene_inputs = ChooseSceneInputs();

  EXPECT_EQ(scene_inputs.pose_path, pose_path.lexically_relative(temp_dir.path()));
  EXPECT_EQ(scene_inputs.imu_path, imu_path.lexically_relative(temp_dir.path()));
}

TEST(ChooseSceneInputs, ReturnsBundledPairWhenTwoMatchingNuScenesPairsExist) {
  TempDir temp_dir;
  ScopedCurrentPath scoped_current_path(temp_dir.path());

  AddScenarioFile(temp_dir.path(), "scene-0364_pose.json");
  AddScenarioFile(temp_dir.path(), "scene-0364_ms_imu.json");
  AddScenarioFile(temp_dir.path(), "scene-0421_pose.json");
  AddScenarioFile(temp_dir.path(), "scene-0421_ms_imu.json");
  const fs::path bundled_pose = AddScenarioFile(temp_dir.path(), "scene_pose.json");
  const fs::path bundled_imu = AddScenarioFile(temp_dir.path(), "scene_ms_imu.json");

  const SceneInputs scene_inputs = ChooseSceneInputs();

  EXPECT_EQ(scene_inputs.pose_path,
            bundled_pose.lexically_relative(temp_dir.path()));
  EXPECT_EQ(scene_inputs.imu_path,
            bundled_imu.lexically_relative(temp_dir.path()));
}

TEST(ChooseSceneInputs, ReturnsBundledPairWhenNoMatchingNuScenesPairExists) {
  TempDir temp_dir;
  ScopedCurrentPath scoped_current_path(temp_dir.path());

  const fs::path bundled_pose = AddScenarioFile(temp_dir.path(), "scene_pose.json");
  const fs::path bundled_imu = AddScenarioFile(temp_dir.path(), "scene_ms_imu.json");

  const SceneInputs scene_inputs = ChooseSceneInputs();

  EXPECT_EQ(scene_inputs.pose_path,
            bundled_pose.lexically_relative(temp_dir.path()));
  EXPECT_EQ(scene_inputs.imu_path,
            bundled_imu.lexically_relative(temp_dir.path()));
}

TEST(ChooseSceneInputs, ThrowsWhenBundledPairIsMissingAndNuScenesPairCountIsNotOne) {
  TempDir temp_dir;
  ScopedCurrentPath scoped_current_path(temp_dir.path());

  AddScenarioFile(temp_dir.path(), "scene-0364_pose.json");
  AddScenarioFile(temp_dir.path(), "scene-0364_ms_imu.json");
  AddScenarioFile(temp_dir.path(), "scene-0421_pose.json");
  AddScenarioFile(temp_dir.path(), "scene-0421_ms_imu.json");

  EXPECT_THROW(ChooseSceneInputs(), std::runtime_error);
}

TEST(ChooseSceneInputs, ReturnsSingleMatchingNuScenesPairWithUnmatchedStrayFilePresent) {
  TempDir temp_dir;
  ScopedCurrentPath scoped_current_path(temp_dir.path());

  const fs::path pose_path =
      AddScenarioFile(temp_dir.path(), "scene-0364_pose.json");
  const fs::path imu_path =
      AddScenarioFile(temp_dir.path(), "scene-0364_ms_imu.json");
  AddScenarioFile(temp_dir.path(), "scene-0421_pose.json");

  const SceneInputs scene_inputs = ChooseSceneInputs();

  EXPECT_EQ(scene_inputs.pose_path, pose_path.lexically_relative(temp_dir.path()));
  EXPECT_EQ(scene_inputs.imu_path, imu_path.lexically_relative(temp_dir.path()));
}

TEST(LoadScene, LoadsPoseAndImuSamples) {
  TempDir temp_dir;

  const Json pose_json = Json::array(
      {{{"utime", 1000},
        {"pos", {1.0, 2.0, 0.0}},
        {"orientation", {1.0, 0.0, 0.0, 0.0}},
        {"vel", {3.0, 0.0, 0.0}}}});
  const Json imu_json = Json::array(
      {{{"utime", 1000},
        {"linear_accel", {4.0, 5.0, 6.0}},
        {"rotation_rate", {0.1, 0.2, 0.3}},
        {"q", {1.0, 0.0, 0.0, 0.0}}}});
  const Json wheel_speed_json = Json::array(
      {{{"utime", 1000},
        {"FL_wheel_speed", 10.0},
        {"FR_wheel_speed", 11.0},
        {"RL_wheel_speed", 12.0},
        {"RR_wheel_speed", 13.0}}});

  const SceneInputs scene_inputs =
      WriteNuScenesSceneFiles(temp_dir.path(),
                              pose_json,
                              imu_json,
                              wheel_speed_json);

  const LoadedScene loaded_scene = LoadScene(scene_inputs);

  ASSERT_EQ(loaded_scene.pose_samples.size(), 1U);
  ASSERT_EQ(loaded_scene.imu_samples.size(), 1U);
  ASSERT_EQ(loaded_scene.wheel_speed_samples.size(), 1U);
  EXPECT_EQ(loaded_scene.pose_samples[0].utime, 1000);
  EXPECT_DOUBLE_EQ(loaded_scene.pose_samples[0].pos.x(), 1.0);
  EXPECT_DOUBLE_EQ(loaded_scene.pose_samples[0].pos.y(), 2.0);
  EXPECT_DOUBLE_EQ(loaded_scene.pose_samples[0].vel.x(), 3.0);
  EXPECT_EQ(loaded_scene.imu_samples[0].utime, 1000);
  EXPECT_DOUBLE_EQ(loaded_scene.imu_samples[0].specific_force.x(), 4.0);
  EXPECT_DOUBLE_EQ(loaded_scene.imu_samples[0].specific_force.y(), 5.0);
  EXPECT_DOUBLE_EQ(loaded_scene.imu_samples[0].specific_force.z(), 6.0);
  EXPECT_DOUBLE_EQ(loaded_scene.imu_samples[0].rotation_rate.z(), 0.3);
  EXPECT_EQ(loaded_scene.wheel_speed_samples[0].utime, 1000);
  EXPECT_DOUBLE_EQ(loaded_scene.wheel_speed_samples[0].fl_wheel_speed_rpm, 10.0);
  EXPECT_DOUBLE_EQ(loaded_scene.wheel_speed_samples[0].fr_wheel_speed_rpm, 11.0);
  EXPECT_DOUBLE_EQ(loaded_scene.wheel_speed_samples[0].rl_wheel_speed_rpm, 12.0);
  EXPECT_DOUBLE_EQ(loaded_scene.wheel_speed_samples[0].rr_wheel_speed_rpm, 13.0);
}

TEST(LoadScene, DropsSingleTrailingPoseSampleAfterLastImu) {
  TempDir temp_dir;

  const Json pose_json = Json::array(
      {{{"utime", 1000},
        {"pos", {0.0, 0.0, 0.0}},
        {"orientation", {1.0, 0.0, 0.0, 0.0}},
        {"vel", {0.0, 0.0, 0.0}}},
       {{"utime", 2000},
        {"pos", {1.0, 0.0, 0.0}},
        {"orientation", {1.0, 0.0, 0.0, 0.0}},
        {"vel", {0.0, 0.0, 0.0}}},
       {{"utime", 3000},
        {"pos", {2.0, 0.0, 0.0}},
        {"orientation", {1.0, 0.0, 0.0, 0.0}},
        {"vel", {0.0, 0.0, 0.0}}}});
  const Json imu_json = Json::array(
      {{{"utime", 1000},
        {"linear_accel", {0.0, 0.0, 9.8}},
        {"rotation_rate", {0.0, 0.0, 0.0}},
        {"q", {1.0, 0.0, 0.0, 0.0}}},
       {{"utime", 2500},
        {"linear_accel", {0.0, 0.0, 9.8}},
        {"rotation_rate", {0.0, 0.0, 0.0}},
        {"q", {1.0, 0.0, 0.0, 0.0}}}});
  const Json wheel_speed_json = Json::array(
      {{{"utime", 1000},
        {"FL_wheel_speed", 0.0},
        {"FR_wheel_speed", 0.0},
        {"RL_wheel_speed", 0.0},
        {"RR_wheel_speed", 0.0}},
       {{"utime", 2500},
        {"FL_wheel_speed", 0.0},
        {"FR_wheel_speed", 0.0},
        {"RL_wheel_speed", 0.0},
        {"RR_wheel_speed", 0.0}}});

  const SceneInputs scene_inputs =
      WriteNuScenesSceneFiles(temp_dir.path(),
                              pose_json,
                              imu_json,
                              wheel_speed_json);

  const LoadedScene loaded_scene = LoadScene(scene_inputs);

  ASSERT_EQ(loaded_scene.pose_samples.size(), 2U);
  EXPECT_EQ(loaded_scene.pose_samples.back().utime, 2000);
}

TEST(LoadScene, ThrowsWhenMatchingWheelSpeedFileIsMissing) {
  TempDir temp_dir;

  const Json pose_json = Json::array(
      {{{"utime", 1000},
        {"pos", {0.0, 0.0, 0.0}},
        {"orientation", {1.0, 0.0, 0.0, 0.0}},
        {"vel", {0.0, 0.0, 0.0}}}});
  const Json imu_json = Json::array(
      {{{"utime", 1000},
        {"linear_accel", {0.0, 0.0, 9.8}},
        {"rotation_rate", {0.0, 0.0, 0.0}},
        {"q", {1.0, 0.0, 0.0, 0.0}}}});

  const fs::path scenarios_dir = temp_dir.path() / "scenarios";
  fs::create_directories(scenarios_dir);

  SceneInputs scene_inputs;
  scene_inputs.pose_path = scenarios_dir / "scene-0123_pose.json";
  scene_inputs.imu_path = scenarios_dir / "scene-0123_ms_imu.json";
  WriteJsonFile(scene_inputs.pose_path, pose_json);
  WriteJsonFile(scene_inputs.imu_path, imu_json);

  EXPECT_THROW(LoadScene(scene_inputs), std::runtime_error);
}
