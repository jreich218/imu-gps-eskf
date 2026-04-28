#ifndef OUTPUTS_HPP
#define OUTPUTS_HPP

#include <filesystem>
#include <iosfwd>
#include <vector>

#include "runtime_loop.hpp"
#include "scene_types.hpp"

/**
 * @defgroup Outputs Outputs
 * @brief Write run artifacts and the successful-run summary.
 * @{
 */

/**
 * @brief Write the generated synthetic GPS samples to output/gps.json.
 *
 * @param gps_samples GPS samples used during the run.
 * @return Path of the written JSON file.
 *
 * @throws std::runtime_error If output/gps.json cannot be opened for writing.
 */
std::filesystem::path WriteGpsJson(const std::vector<GpsSample>& gps_samples);

/**
 * @brief Write one CSV row per GPS update to output/eskf_sim_log.csv.
 *
 * @param run_result In-memory result from the runtime loop.
 * @return Path of the written CSV file.
 *
 * @throws std::runtime_error If output/eskf_sim_log.csv cannot be opened for
 *     writing.
 */
std::filesystem::path WriteEskfSimLogCsv(const EskfRunResult& run_result);

/**
 * @brief Print the successful-run summary.
 *
 * The summary reports the number of GPS updates, the raw GPS horizontal RMSE,
 * the ESKF horizontal RMSE, and the written log path.
 *
 * @param run_result In-memory result from the runtime loop.
 * @param log_path Path of the written CSV log.
 * @param out Stream to receive the summary.
 */
void PrintRunSummary(const EskfRunResult& run_result,
                     const std::filesystem::path& log_path,
                     std::ostream& out);

/** @} */

#endif  // OUTPUTS_HPP
