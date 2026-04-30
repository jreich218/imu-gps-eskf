#ifndef SCENE_IO_HPP
#define SCENE_IO_HPP

#include "scene_types.hpp"

/**
 * @defgroup SceneIO Scene I/O
 * @brief Selecting input files and loading scene data.
 * @{
 */

/**
 * @brief Select the input set for the run.
 *
 * If exactly one matching nuScenes input set is present under `scenarios`,
 * that scene is used. Otherwise the bundled `scene_pose.json`,
 * `scene_ms_imu.json`, and `scene_zoe_veh_info.json` set is used.
 *
 * @return Selected pose, IMU, and wheel-speed file paths.
 *
 * @throws std::filesystem::filesystem_error If `scenarios` cannot be
 *     iterated.
 * @throws std::runtime_error If the bundled set is needed but missing.
 */
SceneInputs ChooseSceneInputs();

/**
 * @brief Load the selected pose, IMU, and wheel-speed streams.
 *
 * If exactly one final pose sample occurs after the last IMU sample, it is
 * dropped during load.
 *
 * @param scene_inputs Selected pose, IMU, and wheel-speed file paths.
 * @return Loaded pose, IMU, and wheel-speed samples.
 *
 * @throws std::runtime_error If a required selected file cannot be opened, or
 *     if parsing a selected file fails.
 */
LoadedScene LoadScene(const SceneInputs& scene_inputs);

/** @} */

#endif  // SCENE_IO_HPP
