#include "outputs.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

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
                ("imu-gps-eskf-outputs-" + std::to_string(stamp) + "-" +
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

Json ReadJsonFile(const fs::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Could not open file: " + path.string());
    }

    Json json;
    in >> json;
    return json;
}

std::string ReadFileToString(const fs::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Could not open file: " + path.string());
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

GpsSample MakeGpsSample(std::int64_t utime, double x, double y) {
    GpsSample gps_sample;
    gps_sample.utime = utime;
    gps_sample.xy = Eigen::Vector2d(x, y);
    return gps_sample;
}

EskfLogRow MakeLogRow() {
    EskfLogRow log_row;
    log_row.utime = 123;
    log_row.estimated_position_G = Eigen::Vector3d(1.0, 2.0, 3.0);
    log_row.gps_xy = Eigen::Vector2d(4.0, 5.0);
    log_row.true_xy = Eigen::Vector2d(6.0, 7.0);
    log_row.innovation_xy = Eigen::Vector2d(8.0, 9.0);
    log_row.nis = 10.0;
    log_row.estimator_error_xy = Eigen::Vector2d(11.0, 12.0);
    log_row.estimator_error_norm = 13.0;
    return log_row;
}

EskfRunResult MakeRunResult() {
    EskfRunResult run_result;
    run_result.log_rows = {MakeLogRow()};
    run_result.num_gps_updates = 1;
    run_result.raw_gps_rmse_xy = 14.0;
    run_result.eskf_rmse_xy = 15.0;
    return run_result;
}

}  // namespace

TEST(WriteGpsJson, WritesExpectedSamplesToOutputDirectory) {
    TempDir temp_dir;
    ScopedCurrentPath scoped_current_path(temp_dir.path());

    const std::vector<GpsSample> gps_samples = {
        MakeGpsSample(1000, 1.25, -2.5),
        MakeGpsSample(2000, 3.0, 4.5),
    };

    const fs::path gps_json_path = WriteGpsJson(gps_samples);

    EXPECT_EQ(gps_json_path, fs::path("output") / "gps.json");
    ASSERT_TRUE(fs::is_regular_file(temp_dir.path() / gps_json_path));

    const Json gps_json = ReadJsonFile(temp_dir.path() / gps_json_path);

    ASSERT_TRUE(gps_json.is_array());
    ASSERT_EQ(gps_json.size(), 2U);

    EXPECT_EQ(gps_json[0].at("utime").get<std::int64_t>(), 1000);
    EXPECT_DOUBLE_EQ(gps_json[0].at("pos").at(0).get<double>(), 1.25);
    EXPECT_DOUBLE_EQ(gps_json[0].at("pos").at(1).get<double>(), -2.5);

    EXPECT_EQ(gps_json[1].at("utime").get<std::int64_t>(), 2000);
    EXPECT_DOUBLE_EQ(gps_json[1].at("pos").at(0).get<double>(), 3.0);
    EXPECT_DOUBLE_EQ(gps_json[1].at("pos").at(1).get<double>(), 4.5);
}

TEST(WriteEskfSimLogCsv, WritesExpectedHeaderAndRow) {
    TempDir temp_dir;
    ScopedCurrentPath scoped_current_path(temp_dir.path());

    const EskfRunResult run_result = MakeRunResult();

    const fs::path log_path = WriteEskfSimLogCsv(run_result);

    EXPECT_EQ(log_path, fs::path("output") / "eskf_sim_log.csv");
    ASSERT_TRUE(fs::is_regular_file(temp_dir.path() / log_path));

    const std::string csv_text = ReadFileToString(temp_dir.path() / log_path);
    std::istringstream in(csv_text);

    std::string header;
    std::string row;

    ASSERT_TRUE(static_cast<bool>(std::getline(in, header)));
    ASSERT_TRUE(static_cast<bool>(std::getline(in, row)));

    EXPECT_EQ(
        header,
        "utime,est_x,est_y,est_z,gps_x,gps_y,true_x,true_y,innov_x,innov_y,"
        "nis,err_x,err_y,err_norm");
    EXPECT_EQ(row, "123,1,2,3,4,5,6,7,8,9,10,11,12,13");
}

TEST(PrintRunSummary, WritesExpectedSummaryLines) {
    const EskfRunResult run_result = MakeRunResult();
    std::ostringstream out;

    PrintRunSummary(run_result, fs::path("output") / "eskf_sim_log.csv", out);

    EXPECT_EQ(out.str(),
              "GPS updates: 1\n"
              "RMSE raw GPS (xy): 14 m\n"
              "RMSE ESKF (xy): 15 m\n"
              "Wrote log: output/eskf_sim_log.csv\n");
}
