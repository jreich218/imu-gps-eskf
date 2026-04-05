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
 * @return The chosen input file pair.
 */
SceneInputs ChooseSceneInputs();

/**
 * @brief Load the selected pose and IMU streams.
 *
 * If exactly one final pose sample occurs after the last IMU sample, it is
 * dropped during load.
 *
 * @param scene_inputs Selected pose and IMU file paths.
 * @return Loaded pose and IMU samples.
 */
LoadedScene LoadScene(const SceneInputs& scene_inputs);

/** @} */

#endif  // SCENE_IO_HPP
