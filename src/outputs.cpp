#include "outputs.hpp"

#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace {

fs::path OutputDirectoryPath() {
    return "output";
}

fs::path EnsureOutputDirectoryExists() {
    const fs::path output_dir = OutputDirectoryPath();
    fs::create_directories(output_dir);
    return output_dir;
}

}  // namespace

fs::path WriteGpsJson(const std::vector<GpsSample>& gps_samples) {
    const fs::path output_dir = EnsureOutputDirectoryExists();
    const fs::path gps_json_path = output_dir / "gps.json";

    Json gps_json = Json::array();
    for (const GpsSample& gps_sample : gps_samples) {
        gps_json.push_back({{"utime", gps_sample.utime},
                            {"pos", {gps_sample.xy.x(), gps_sample.xy.y()}}});
    }

    std::ofstream out(gps_json_path);
    if (!out) {
        throw std::runtime_error("Could not open " + gps_json_path.string() +
                                 " for writing.");
    }

    out << gps_json.dump(2) << "\n";
    return gps_json_path;
}

fs::path WriteEskfSimLogCsv(const EskfRunResult& run_result) {
    const fs::path output_dir = EnsureOutputDirectoryExists();
    const fs::path log_path = output_dir / "eskf_sim_log.csv";

    std::ofstream out(log_path);
    if (!out) {
        throw std::runtime_error("Could not open " + log_path.string() +
                                 " for writing.");
    }

    out << "utime,est_x,est_y,est_z,gps_x,gps_y,true_x,true_y,innov_x,innov_y,"
           "nis,err_x,err_y,err_norm\n";
    out << std::setprecision(17);

    for (const EskfLogRow& log_row : run_result.log_rows) {
        out << log_row.utime << ","
            << log_row.estimated_position_G.x() << ","
            << log_row.estimated_position_G.y() << ","
            << log_row.estimated_position_G.z() << ","
            << log_row.gps_xy.x() << ","
            << log_row.gps_xy.y() << ","
            << log_row.true_xy.x() << ","
            << log_row.true_xy.y() << ","
            << log_row.innovation_xy.x() << ","
            << log_row.innovation_xy.y() << ","
            << log_row.nis << ","
            << log_row.estimator_error_xy.x() << ","
            << log_row.estimator_error_xy.y() << ","
            << log_row.estimator_error_norm << "\n";
    }

    return log_path;
}

void PrintRunSummary(const EskfRunResult& run_result,
                     const fs::path& log_path,
                     std::ostream& out) {
    out << "GPS updates: " << run_result.num_gps_updates << "\n";
    out << "RMSE raw GPS (xy): " << run_result.raw_gps_rmse_xy << " m\n";
    out << "RMSE ESKF (xy): " << run_result.eskf_rmse_xy << " m\n";
    out << "Wrote log: " << log_path.string() << "\n";
}
