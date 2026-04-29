#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace {

class TempDir {
   public:
    TempDir() {
        static std::uint64_t counter = 0;
        const auto stamp =
            std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = fs::temp_directory_path() /
                ("imu-gps-eskf-app-integration-" + std::to_string(stamp) + "-" +
                 std::to_string(counter++));
        fs::create_directories(path_);
    }

    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }

    const fs::path& path() const { return path_; }

   private:
    fs::path path_;
};

struct RunSummary {
    std::size_t gps_updates = 0;
    double raw_gps_rmse_xy = 0.0;
    double eskf_rmse_xy = 0.0;
    std::string log_path;
};

std::string ReadFileToString(const fs::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Could not open file: " + path.string());
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

Json ReadJsonFile(const fs::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Could not open file: " + path.string());
    }

    Json json;
    in >> json;
    return json;
}

bool StartsWith(const std::string& text, const std::string& prefix) {
    return text.rfind(prefix, 0) == 0;
}

std::string ShellQuote(const std::string& text) {
    std::string quoted = "'";
    for (char c : text) {
        if (c == '\'') {
            quoted += "'\"'\"'";
        } else {
            quoted += c;
        }
    }
    quoted += "'";
    return quoted;
}

std::string ShellQuote(const fs::path& path) {
    return ShellQuote(path.string());
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kWheelRadiusM = 0.305;

double SpeedMpsToWheelRpm(double speed_mps) {
    const double circumference_m = 2.0 * kPi * kWheelRadiusM;
    return speed_mps * 60.0 / circumference_m;
}

void WriteSimpleNuScenesSceneFiles(const fs::path& working_dir) {
    const fs::path scenarios_dir = working_dir / "scenarios";
    fs::create_directories(scenarios_dir);

    const double speed_mps = 25.0;
    const double wheel_speed_rpm = SpeedMpsToWheelRpm(speed_mps);

    Json pose_json = Json::array();
    Json imu_json = Json::array();
    Json wheel_speed_json = Json::array();

    for (int index = 0; index < 10; ++index) {
        const std::int64_t utime = 100000 + static_cast<std::int64_t>(index) * 100000;
        const double x = static_cast<double>(index) * speed_mps * 0.1;

        pose_json.push_back({
            {"utime", utime},
            {"pos", {x, 0.0, 0.0}},
            {"orientation", {1.0, 0.0, 0.0, 0.0}},
            {"vel", {speed_mps, 0.0, 0.0}},
        });
        imu_json.push_back({
            {"utime", utime},
            {"linear_accel", {0.0, 0.0, 9.8}},
            {"rotation_rate", {0.0, 0.0, 0.0}},
            {"q", {1.0, 0.0, 0.0, 0.0}},
        });
        wheel_speed_json.push_back({
            {"utime", utime},
            {"FL_wheel_speed", wheel_speed_rpm},
            {"FR_wheel_speed", wheel_speed_rpm},
            {"RL_wheel_speed", wheel_speed_rpm},
            {"RR_wheel_speed", wheel_speed_rpm},
        });
    }

    std::ofstream pose_out(scenarios_dir / "scene-0123_pose.json");
    if (!pose_out) {
        throw std::runtime_error("Could not create pose file.");
    }
    pose_out << pose_json.dump();

    std::ofstream imu_out(scenarios_dir / "scene-0123_ms_imu.json");
    if (!imu_out) {
        throw std::runtime_error("Could not create IMU file.");
    }
    imu_out << imu_json.dump();

    std::ofstream wheel_out(scenarios_dir / "scene-0123_zoe_veh_info.json");
    if (!wheel_out) {
        throw std::runtime_error("Could not create wheel-speed file.");
    }
    wheel_out << wheel_speed_json.dump();
}

int RunApp(const fs::path& working_dir,
           const fs::path& stdout_path,
           const fs::path& stderr_path) {
    const std::string command = "cd " + ShellQuote(working_dir) + " && " +
                                ShellQuote(fs::path(IMU_GPS_ESKF_EXE_PATH)) +
                                " > " + ShellQuote(stdout_path) + " 2> " +
                                ShellQuote(stderr_path);

    const int status = std::system(command.c_str());
    if (status == -1) {
        throw std::runtime_error("Failed to launch imu-gps-eskf.");
    }

    if (!WIFEXITED(status)) {
        throw std::runtime_error("imu-gps-eskf did not exit normally.");
    }

    return WEXITSTATUS(status);
}

std::size_t CountCsvDataRows(const fs::path& csv_path) {
    std::ifstream in(csv_path);
    if (!in) {
        throw std::runtime_error("Could not open CSV file: " +
                                 csv_path.string());
    }

    std::string line;
    if (!std::getline(in, line)) {
        throw std::runtime_error("CSV file is empty: " + csv_path.string());
    }

    std::size_t row_count = 0;
    while (std::getline(in, line)) {
        if (!line.empty()) {
            ++row_count;
        }
    }

    return row_count;
}

RunSummary ParseRunSummary(const std::string& stdout_text) {
    const std::string kGpsUpdatesPrefix = "GPS updates: ";
    const std::string kRawRmsePrefix = "RMSE raw GPS (xy): ";
    const std::string kEskfRmsePrefix = "RMSE ESKF (xy): ";
    const std::string kLogPathPrefix = "Wrote log: ";
    const std::string kMetersSuffix = " m";

    RunSummary summary;
    bool saw_gps_updates = false;
    bool saw_raw_rmse = false;
    bool saw_eskf_rmse = false;
    bool saw_log_path = false;

    std::istringstream in(stdout_text);
    std::string line;
    while (std::getline(in, line)) {
        if (StartsWith(line, kGpsUpdatesPrefix)) {
            summary.gps_updates = static_cast<std::size_t>(
                std::stoull(line.substr(kGpsUpdatesPrefix.size())));
            saw_gps_updates = true;
            continue;
        }

        if (StartsWith(line, kRawRmsePrefix)) {
            const std::string value_text = line.substr(
                kRawRmsePrefix.size(),
                line.size() - kRawRmsePrefix.size() - kMetersSuffix.size());
            summary.raw_gps_rmse_xy = std::stod(value_text);
            saw_raw_rmse = true;
            continue;
        }

        if (StartsWith(line, kEskfRmsePrefix)) {
            const std::string value_text = line.substr(
                kEskfRmsePrefix.size(),
                line.size() - kEskfRmsePrefix.size() - kMetersSuffix.size());
            summary.eskf_rmse_xy = std::stod(value_text);
            saw_eskf_rmse = true;
            continue;
        }

        if (StartsWith(line, kLogPathPrefix)) {
            summary.log_path = line.substr(kLogPathPrefix.size());
            saw_log_path = true;
            continue;
        }
    }

    if (!saw_gps_updates || !saw_raw_rmse || !saw_eskf_rmse || !saw_log_path) {
        throw std::runtime_error("Could not parse run summary from stdout.");
    }

    return summary;
}

}  // namespace

TEST(AppIntegration, NuScenesRunWritesOutputsAndSummary) {
    TempDir temp_dir;
    WriteSimpleNuScenesSceneFiles(temp_dir.path());

    const fs::path stdout_path = temp_dir.path() / "stdout.txt";
    const fs::path stderr_path = temp_dir.path() / "stderr.txt";

    const int exit_code = RunApp(temp_dir.path(), stdout_path, stderr_path);
    ASSERT_EQ(exit_code, 0);

    const std::string stdout_text = ReadFileToString(stdout_path);
    const std::string stderr_text = ReadFileToString(stderr_path);

    EXPECT_TRUE(stderr_text.empty());

    const fs::path gps_json_path = temp_dir.path() / "output" / "gps.json";
    const fs::path csv_path = temp_dir.path() / "output" / "eskf_sim_log.csv";

    ASSERT_TRUE(fs::is_regular_file(gps_json_path));
    ASSERT_TRUE(fs::is_regular_file(csv_path));
    EXPECT_GT(fs::file_size(gps_json_path), 0U);
    EXPECT_GT(fs::file_size(csv_path), 0U);

    const Json gps_json = ReadJsonFile(gps_json_path);
    ASSERT_TRUE(gps_json.is_array());
    EXPECT_FALSE(gps_json.empty());

    const RunSummary summary = ParseRunSummary(stdout_text);

    EXPECT_GT(summary.gps_updates, 0U);
    EXPECT_EQ(summary.log_path, "output/eskf_sim_log.csv");

    const std::size_t csv_data_row_count = CountCsvDataRows(csv_path);
    EXPECT_EQ(summary.gps_updates, csv_data_row_count);

    EXPECT_GT(summary.raw_gps_rmse_xy, 0.0);
    EXPECT_GT(summary.eskf_rmse_xy, 0.0);
    EXPECT_LT(summary.eskf_rmse_xy, summary.raw_gps_rmse_xy);
}
