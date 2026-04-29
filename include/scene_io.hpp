#ifndef SCENE_IO_HPP
#define SCENE_IO_HPP

#include "scene_types.hpp"

/**
 * @defgroup SceneIO Scene I/O
 * @brief Selecting input files and loading scene data.
 * @{
 */

/**
 * @brief Select the pose and IMU files for the run.
 *
 * If exactly one matching `scene-XXXX_pose.json` /
 * `scene-XXXX_ms_imu.json` pair is present under `scenarios`, that pair is
 * used. Otherwise the bundled `scene_pose.json`, `scene_ms_imu.json`, and
 * `scene_zoe_veh_info.json` set is used.
 *
 * @return The chosen input file pair.
 *
 * @throws std::runtime_error If the bundled set is needed but missing.
 */
SceneInputs ChooseSceneInputs();

/**
 * @brief Load the selected pose, IMU, and wheel-speed streams.
 *
 * If exactly one final pose sample occurs after the last IMU sample, it is
 * dropped during load. For a nuScenes scene pair, the sibling
 * `scene-XXXX_zoe_veh_info.json` file is loaded from the same directory. For
 * the bundled set, `scene_zoe_veh_info.json` is loaded from the same
 * directory as `scene_pose.json`.
 *
 * @param scene_inputs Selected pose and IMU file paths.
 * @return Loaded pose, IMU, and wheel-speed samples.
 *
 * @throws std::runtime_error If a required selected file cannot be opened.
 */
LoadedScene LoadScene(const SceneInputs& scene_inputs);

/** @} */

#endif  // SCENE_IO_HPP
