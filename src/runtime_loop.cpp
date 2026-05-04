#include "runtime_loop.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "eskf.hpp"

namespace {

const PoseSample& PoseSampleAtTimestamp(
    const std::vector<PoseSample>& pose_samples,
    std::size_t& pose_index,
    std::int64_t utime) {
    while (pose_index < pose_samples.size() &&
           pose_samples[pose_index].utime < utime) {
        ++pose_index;
    }

    if (pose_index == pose_samples.size() ||
        pose_samples[pose_index].utime != utime) {
        throw std::runtime_error(
            "RunEskfLoop: missing pose sample for GPS timestamp " +
            std::to_string(utime));
    }

    return pose_samples[pose_index];
}

double RmseFromSquaredErrorSum(double squared_error_sum, std::size_t count) {
    return std::sqrt(squared_error_sum / static_cast<double>(count));
}

void RecordGpsUpdate(const LoadedScene& loaded_scene,
                     std::size_t& pose_index,
                     const GpsSample& gps_sample,
                     Eskf& eskf,
                     EskfRunResult& run_result,
                     double& raw_gps_squared_error_sum,
                     double& eskf_squared_error_sum) {
    const PoseSample& pose_sample =
        PoseSampleAtTimestamp(loaded_scene.pose_samples, pose_index,
                              gps_sample.utime);

    const GpsUpdateResult update_result = eskf.UpdateGps(gps_sample);
    const Eigen::Vector3d& estimated_position_G = eskf.PositionG();
    const Eigen::Vector2d true_xy = pose_sample.pos.head<2>();

    const Eigen::Vector2d raw_gps_error_xy = gps_sample.xy - true_xy;
    const Eigen::Vector2d estimator_error_xy =
        estimated_position_G.head<2>() - true_xy;

    raw_gps_squared_error_sum += raw_gps_error_xy.squaredNorm();
    eskf_squared_error_sum += estimator_error_xy.squaredNorm();

    EskfLogRow log_row;
    log_row.utime = gps_sample.utime;
    log_row.estimated_position_G = estimated_position_G;
    log_row.gps_xy = gps_sample.xy;
    log_row.true_xy = true_xy;
    log_row.innovation_xy = update_result.innovation_xy;
    log_row.nis = update_result.nis;
    log_row.estimator_error_xy = estimator_error_xy;
    log_row.estimator_error_norm = estimator_error_xy.norm();
    run_result.log_rows.push_back(log_row);
}

}  // namespace

EskfRunResult RunEskfLoop(const LoadedScene& loaded_scene,
                          const std::vector<GpsSample>& gps_samples,
                          const StartupInitialization& startup_initialization) {
    Eskf eskf;
    eskf.Initialize(startup_initialization);

    EskfRunResult run_result;
    if (startup_initialization.first_unprocessed_gps_index <
        gps_samples.size()) {
        run_result.log_rows.reserve(
            gps_samples.size() -
            startup_initialization.first_unprocessed_gps_index);
    }

    std::size_t gps_index = startup_initialization.first_unprocessed_gps_index;
    std::size_t pose_index = 0;

    double raw_gps_squared_error_sum = 0.0;
    double eskf_squared_error_sum = 0.0;

    for (std::size_t imu_index =
             startup_initialization.first_unprocessed_imu_index;
         imu_index < loaded_scene.imu_samples.size() &&
         gps_index < gps_samples.size();
         ++imu_index) {
        const ImuSample& current_imu_sample =
            loaded_scene.imu_samples[imu_index];
        const GpsSample& gps_sample = gps_samples[gps_index];

        // Supported inputs guarantee at most one GPS sample per IMU interval,
        // so the loop only needs the three timing cases below.
        if (gps_sample.utime > current_imu_sample.utime) {
            eskf.Predict(current_imu_sample);
            continue;
        }

        if (gps_sample.utime < current_imu_sample.utime) {
            ImuSample imu_sample_at_gps_time = current_imu_sample;
            imu_sample_at_gps_time.utime = gps_sample.utime;
            eskf.Predict(imu_sample_at_gps_time);
            RecordGpsUpdate(loaded_scene,
                            pose_index,
                            gps_sample,
                            eskf,
                            run_result,
                            raw_gps_squared_error_sum,
                            eskf_squared_error_sum);
            ++gps_index;
            eskf.Predict(current_imu_sample);
            continue;
        }

        eskf.Predict(current_imu_sample);
        RecordGpsUpdate(loaded_scene,
                        pose_index,
                        gps_sample,
                        eskf,
                        run_result,
                        raw_gps_squared_error_sum,
                        eskf_squared_error_sum);
        ++gps_index;
    }

    if (run_result.log_rows.empty()) {
        throw std::runtime_error("RunEskfLoop: no GPS updates were processed.");
    }

    if (gps_index != gps_samples.size()) {
        throw std::runtime_error(
            "RunEskfLoop: ran out of IMU samples before the last GPS update.");
    }

    run_result.num_gps_updates = run_result.log_rows.size();
    run_result.raw_gps_rmse_xy = RmseFromSquaredErrorSum(
        raw_gps_squared_error_sum, run_result.num_gps_updates);
    run_result.eskf_rmse_xy = RmseFromSquaredErrorSum(
        eskf_squared_error_sum, run_result.num_gps_updates);

    return run_result;
}
