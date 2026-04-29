#include "initialization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace {

constexpr double kSecPerUsec = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMinFitPointCount = 3;
constexpr std::size_t kStabilityWindow = 3;
constexpr double kTangentStabilityTolRad = 10.0 * kPi / 180.0;
constexpr double kMinEndpointSpeed = 1e-6;
constexpr std::size_t kRelaxedProjectedSeparationPointCount = 80;
constexpr double kEarlyMinProjectedSeparationM = 10.0;
constexpr double kLateMinProjectedSeparationM = 7.0;
constexpr double kRequiredWheelSupportedTravelM = 10.0;
constexpr double kMaxLineLikeSecondToFirstVarianceRatio = 0.1;
constexpr double kRecentSpeedWindowSec = 1.0;
constexpr double kWheelSpeedSmoothingWindowSec = 0.3;
constexpr double kWheelRadiusM = 0.305;
constexpr int kPathLengthQuadratureSteps = 32;
constexpr std::size_t kPathTraceSampleCount = 100;

struct QuadraticPathFit {
    Eigen::Vector3d coeff_x = Eigen::Vector3d::Zero();
    Eigen::Vector3d coeff_y = Eigen::Vector3d::Zero();
    std::int64_t end_utime = 0;
};

struct PrincipalDirectionSummary {
    Eigen::Vector2d axis = Eigen::Vector2d::UnitX();
    double first_variance = 0.0;
    double second_variance = 0.0;
    double projected_start_to_end_m = 0.0;
};

struct ReadyEstimate2D {
    std::int64_t ready_utime = 0;
    Eigen::Vector2d position_xy{0.0, 0.0};
    Eigen::Vector2d velocity_xy{0.0, 0.0};
    double yaw_rad = 0.0;
    std::size_t first_unprocessed_gps_index = 0;
};

struct WheelMotionProfile {
    std::vector<std::int64_t> utimes;
    std::vector<double> smoothed_speed_mps;
    std::vector<double> cumulative_distance_m;
};

using TruthStateMap =
    std::unordered_map<std::int64_t, StartupTraceTruthState>;

double WrapAngle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double AngleDiff(double a_rad, double b_rad) {
    return WrapAngle(a_rad - b_rad);
}

Eigen::Vector2d UnitDirection(double yaw_rad) {
    return Eigen::Vector2d(std::cos(yaw_rad), std::sin(yaw_rad));
}

double YawFromQuaternion(const Eigen::Quaterniond& q) {
    const Eigen::Quaterniond normalized = q.normalized();
    const double siny_cosp =
        2.0 * (normalized.w() * normalized.z() +
               normalized.x() * normalized.y());
    const double cosy_cosp =
        1.0 - 2.0 * (normalized.y() * normalized.y() +
                     normalized.z() * normalized.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

double ArcLengthFromChordAndTurn(double chord_m, double abs_turn_rad) {
    if (!(chord_m > 0.0)) {
        return 0.0;
    }

    if (abs_turn_rad < 1e-6) {
        return chord_m;
    }

    const double denominator = 2.0 * std::sin(0.5 * abs_turn_rad);
    if (std::abs(denominator) < 1e-9) {
        return chord_m;
    }

    return chord_m * abs_turn_rad / denominator;
}

Eigen::Quaterniond QuaternionFromYaw(double yaw_rad) {
    const double half_yaw_rad = 0.5 * yaw_rad;
    return Eigen::Quaterniond(
        std::cos(half_yaw_rad), 0.0, 0.0, std::sin(half_yaw_rad));
}

Eigen::Quaterniond BuildGlobalOrientation(double yaw_rad,
                                          const ImuSample& imu_ref) {
    // Use the IMU sample's gravity-aligned frame to attach the requested
    // global yaw.
    const Eigen::Quaterniond q_AI = imu_ref.q_AI.normalized();
    const Eigen::Vector3d x_axis_A = q_AI * Eigen::Vector3d::UnitX();
    const double yaw_A = std::atan2(x_axis_A.y(), x_axis_A.x());
    const double yaw_correction_rad = yaw_rad - yaw_A;
    const Eigen::Quaterniond q_GA = QuaternionFromYaw(yaw_correction_rad);
    return (q_GA * q_AI).normalized();
}

std::optional<std::size_t> FindFirstUsableGpsIndex(
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples) {
    // Startup begins at the first GPS sample on the IMU time axis.
    if (imu_samples.empty()) {
        return std::nullopt;
    }

    std::size_t gps_index = 0;
    while (gps_index < gps_samples.size() &&
           gps_samples[gps_index].utime < imu_samples.front().utime) {
        ++gps_index;
    }

    if (gps_index >= gps_samples.size()) {
        return std::nullopt;
    }

    return gps_index;
}

bool HasMinimumPointsForFit(std::size_t gps_point_count) {
    return gps_point_count >= kMinFitPointCount;
}

double WheelSpeedRpmToMps(double wheel_speed_rpm) {
    const double circumference_m = 2.0 * kPi * kWheelRadiusM;
    return std::abs(wheel_speed_rpm) * circumference_m / 60.0;
}

double RobustForwardWheelSpeedMps(const WheelSpeedSample& wheel_speed_sample) {
    std::array<double, 4> wheel_speeds_mps = {
        WheelSpeedRpmToMps(wheel_speed_sample.fl_wheel_speed_rpm),
        WheelSpeedRpmToMps(wheel_speed_sample.fr_wheel_speed_rpm),
        WheelSpeedRpmToMps(wheel_speed_sample.rl_wheel_speed_rpm),
        WheelSpeedRpmToMps(wheel_speed_sample.rr_wheel_speed_rpm),
    };
    std::sort(wheel_speeds_mps.begin(), wheel_speeds_mps.end());
    return 0.5 * (wheel_speeds_mps[1] + wheel_speeds_mps[2]);
}

WheelMotionProfile BuildWheelMotionProfile(
    const std::vector<WheelSpeedSample>& wheel_speed_samples) {
    if (wheel_speed_samples.empty()) {
        throw std::runtime_error("Wheel-motion profile needs wheel-speed samples.");
    }

    WheelMotionProfile profile;
    profile.utimes.reserve(wheel_speed_samples.size());
    profile.smoothed_speed_mps.reserve(wheel_speed_samples.size());
    profile.cumulative_distance_m.reserve(wheel_speed_samples.size());

    std::vector<double> robust_speeds_mps;
    robust_speeds_mps.reserve(wheel_speed_samples.size());
    for (const WheelSpeedSample& wheel_speed_sample : wheel_speed_samples) {
        profile.utimes.push_back(wheel_speed_sample.utime);
        robust_speeds_mps.push_back(
            RobustForwardWheelSpeedMps(wheel_speed_sample));
    }

    const std::int64_t smoothing_window_us = static_cast<std::int64_t>(
        std::llround(kWheelSpeedSmoothingWindowSec / kSecPerUsec));
    double window_sum = 0.0;
    std::size_t window_start_index = 0;
    for (std::size_t index = 0; index < wheel_speed_samples.size(); ++index) {
        window_sum += robust_speeds_mps[index];
        while (profile.utimes[index] - profile.utimes[window_start_index] >
               smoothing_window_us) {
            window_sum -= robust_speeds_mps[window_start_index];
            ++window_start_index;
        }

        const std::size_t window_count = index - window_start_index + 1;
        profile.smoothed_speed_mps.push_back(
            window_sum / static_cast<double>(window_count));
    }

    profile.cumulative_distance_m.push_back(0.0);
    for (std::size_t index = 1; index < wheel_speed_samples.size(); ++index) {
        const double dt_s = static_cast<double>(profile.utimes[index] -
                                                profile.utimes[index - 1]) *
                            kSecPerUsec;
        const double segment_distance_m =
            0.5 * (profile.smoothed_speed_mps[index - 1] +
                   profile.smoothed_speed_mps[index]) *
            dt_s;
        profile.cumulative_distance_m.push_back(
            profile.cumulative_distance_m.back() + segment_distance_m);
    }

    return profile;
}

double InterpolateWheelProfileValue(const WheelMotionProfile& profile,
                                    const std::vector<double>& values,
                                    std::int64_t utime) {
    if (profile.utimes.empty() || values.size() != profile.utimes.size()) {
        throw std::runtime_error("Wheel-motion profile interpolation is invalid.");
    }

    if (utime <= profile.utimes.front()) {
        return values.front();
    }
    if (utime >= profile.utimes.back()) {
        return values.back();
    }

    const auto upper_it =
        std::lower_bound(profile.utimes.begin(), profile.utimes.end(), utime);
    const std::size_t upper_index =
        static_cast<std::size_t>(upper_it - profile.utimes.begin());
    if (*upper_it == utime) {
        return values[upper_index];
    }

    const std::size_t lower_index = upper_index - 1;
    const double alpha =
        static_cast<double>(utime - profile.utimes[lower_index]) /
        static_cast<double>(profile.utimes[upper_index] -
                            profile.utimes[lower_index]);
    return (1.0 - alpha) * values[lower_index] + alpha * values[upper_index];
}

double WheelProfileDistanceAtUtime(const WheelMotionProfile& profile,
                                   std::int64_t utime) {
    if (profile.utimes.empty()) {
        throw std::runtime_error("Wheel-motion profile is empty.");
    }

    if (utime <= profile.utimes.front()) {
        return 0.0;
    }
    if (utime >= profile.utimes.back()) {
        const double dt_s =
            static_cast<double>(utime - profile.utimes.back()) * kSecPerUsec;
        return profile.cumulative_distance_m.back() +
               profile.smoothed_speed_mps.back() * dt_s;
    }

    const auto upper_it =
        std::lower_bound(profile.utimes.begin(), profile.utimes.end(), utime);
    const std::size_t upper_index =
        static_cast<std::size_t>(upper_it - profile.utimes.begin());
    if (*upper_it == utime) {
        return profile.cumulative_distance_m[upper_index];
    }

    const std::size_t lower_index = upper_index - 1;
    const double speed_at_utime = InterpolateWheelProfileValue(
        profile, profile.smoothed_speed_mps, utime);
    const double dt_s =
        static_cast<double>(utime - profile.utimes[lower_index]) * kSecPerUsec;
    return profile.cumulative_distance_m[lower_index] +
           0.5 * (profile.smoothed_speed_mps[lower_index] + speed_at_utime) *
               dt_s;
}

double WheelSupportedTravelBetween(const WheelMotionProfile& profile,
                                   std::int64_t start_utime,
                                   std::int64_t end_utime) {
    if (end_utime <= start_utime) {
        return 0.0;
    }

    return WheelProfileDistanceAtUtime(profile, end_utime) -
           WheelProfileDistanceAtUtime(profile, start_utime);
}

QuadraticPathFit FitQuadraticPath(const std::vector<GpsSample>& gps_samples,
                                  std::size_t first_index,
                                  std::size_t last_index) {
    // Fit quadratic x(t) and y(t) over the inclusive GPS window, with tau=0
    // at the window end.
    if (last_index < first_index + kMinFitPointCount - 1) {
        throw std::runtime_error(
            "Startup path fit needs at least three GPS points.");
    }

    const std::size_t point_count = last_index - first_index + 1;
    Eigen::MatrixXd A(point_count, 3);
    Eigen::VectorXd x(point_count);
    Eigen::VectorXd y(point_count);

    const std::int64_t end_utime = gps_samples[last_index].utime;
    for (std::size_t row = 0; row < point_count; ++row) {
        const std::size_t sample_index = first_index + row;
        const double tau_s =
            static_cast<double>(gps_samples[sample_index].utime - end_utime) *
            kSecPerUsec;

        A(static_cast<Eigen::Index>(row), 0) = 1.0;
        A(static_cast<Eigen::Index>(row), 1) = tau_s;
        A(static_cast<Eigen::Index>(row), 2) = tau_s * tau_s;
        x(static_cast<Eigen::Index>(row)) = gps_samples[sample_index].xy.x();
        y(static_cast<Eigen::Index>(row)) = gps_samples[sample_index].xy.y();
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    if (qr.rank() < 3) {
        throw std::runtime_error(
            "Startup path fit needs distinct GPS timestamps.");
    }

    QuadraticPathFit path_fit;
    path_fit.coeff_x = qr.solve(x);
    path_fit.coeff_y = qr.solve(y);
    path_fit.end_utime = end_utime;
    return path_fit;
}

Eigen::Vector2d PositionAt(const QuadraticPathFit& path_fit, double tau_s) {
    return Eigen::Vector2d(path_fit.coeff_x.x() + path_fit.coeff_x.y() * tau_s +
                               path_fit.coeff_x.z() * tau_s * tau_s,
                           path_fit.coeff_y.x() + path_fit.coeff_y.y() * tau_s +
                               path_fit.coeff_y.z() * tau_s * tau_s);
}

Eigen::Vector2d VelocityAt(const QuadraticPathFit& path_fit, double tau_s) {
    return Eigen::Vector2d(
        path_fit.coeff_x.y() + 2.0 * path_fit.coeff_x.z() * tau_s,
        path_fit.coeff_y.y() + 2.0 * path_fit.coeff_y.z() * tau_s);
}

std::vector<Eigen::Vector2d> SamplePathPoints(
    const QuadraticPathFit& path_fit,
    const std::vector<GpsSample>& gps_samples,
    std::size_t first_index,
    std::size_t last_index) {
    const double tau_start_s =
        static_cast<double>(gps_samples[first_index].utime -
                            gps_samples[last_index].utime) *
        kSecPerUsec;

    std::vector<Eigen::Vector2d> path_points;
    path_points.reserve(kPathTraceSampleCount);
    for (std::size_t sample_index = 0; sample_index < kPathTraceSampleCount;
         ++sample_index) {
        const double alpha =
            kPathTraceSampleCount == 1
                ? 0.0
                : static_cast<double>(sample_index) /
                      static_cast<double>(kPathTraceSampleCount - 1);
        const double tau_s = (1.0 - alpha) * tau_start_s;
        path_points.push_back(PositionAt(path_fit, tau_s));
    }

    return path_points;
}

double PathLengthBetween(const QuadraticPathFit& path_fit,
                         double tau_start_s,
                         double tau_end_s) {
    // Compute fitted path arc length between two local times.
    if (tau_end_s < tau_start_s) {
        throw std::runtime_error(
            "Startup path length interval must be nondecreasing.");
    }

    if (tau_end_s == tau_start_s) {
        return 0.0;
    }

    const double h = (tau_end_s - tau_start_s) /
                     static_cast<double>(kPathLengthQuadratureSteps);
    double weighted_sum = 0.0;
    for (int step = 0; step <= kPathLengthQuadratureSteps; ++step) {
        const double tau_s = tau_start_s + static_cast<double>(step) * h;
        const double speed = VelocityAt(path_fit, tau_s).norm();
        if (step == 0 || step == kPathLengthQuadratureSteps) {
            weighted_sum += speed;
        } else if (step % 2 == 0) {
            weighted_sum += 2.0 * speed;
        } else {
            weighted_sum += 4.0 * speed;
        }
    }

    return weighted_sum * h / 3.0;
}

double DeltaYawBetween(const std::vector<ImuSample>& imu_samples,
                       std::int64_t start_utime,
                       std::int64_t end_utime) {
    // Integrate yaw change over the requested interval.
    if (end_utime < start_utime) {
        throw std::runtime_error("Startup IMU interval must be nondecreasing.");
    }

    double delta_yaw_rad = 0.0;
    for (std::size_t index = 1; index < imu_samples.size(); ++index) {
        const std::int64_t seg_start_utime = imu_samples[index - 1].utime;
        const std::int64_t seg_end_utime = imu_samples[index].utime;
        const std::int64_t overlap_start_utime =
            std::max(seg_start_utime, start_utime);
        const std::int64_t overlap_end_utime =
            std::min(seg_end_utime, end_utime);
        if (overlap_end_utime <= overlap_start_utime) {
            continue;
        }

        const double dt_s =
            static_cast<double>(overlap_end_utime - overlap_start_utime) *
            kSecPerUsec;
        const Eigen::Quaterniond q_AI = imu_samples[index].q_AI.normalized();
        const Eigen::Vector3d omega_A = q_AI * imu_samples[index].rotation_rate;
        delta_yaw_rad += omega_A.z() * dt_s;
    }

    return delta_yaw_rad;
}

PrincipalDirectionSummary SummarizePrincipalDirection(
    const std::vector<GpsSample>& gps_samples,
    std::size_t first_index,
    std::size_t last_index) {
    // Compute the dominant GPS-cloud axis and its projected start-to-end span.
    const std::size_t point_count = last_index - first_index + 1;
    if (point_count < 2) {
        throw std::runtime_error("Startup PCA needs at least two GPS points.");
    }

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (std::size_t index = first_index; index <= last_index; ++index) {
        mean += gps_samples[index].xy;
    }
    mean /= static_cast<double>(point_count);

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (std::size_t index = first_index; index <= last_index; ++index) {
        const Eigen::Vector2d centered = gps_samples[index].xy - mean;
        covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(point_count);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Startup PCA failed.");
    }

    PrincipalDirectionSummary principal_direction;
    principal_direction.axis = solver.eigenvectors().col(1).normalized();
    principal_direction.first_variance = solver.eigenvalues()(1);
    principal_direction.second_variance = solver.eigenvalues()(0);

    const Eigen::Vector2d start_to_end =
        gps_samples[last_index].xy - gps_samples[first_index].xy;
    if (principal_direction.axis.dot(start_to_end) < 0.0) {
        principal_direction.axis = -principal_direction.axis;
    }

    principal_direction.projected_start_to_end_m =
        std::abs(principal_direction.axis.dot(start_to_end));
    return principal_direction;
}

TruthStateMap BuildTruthStateMap(const std::vector<PoseSample>& pose_samples) {
    TruthStateMap truth_by_utime;
    truth_by_utime.reserve(pose_samples.size());

    for (const PoseSample& pose_sample : pose_samples) {
        StartupTraceTruthState truth;
        truth.position_xy = pose_sample.pos.head<2>();
        truth.yaw_rad = YawFromQuaternion(pose_sample.orientation);
        truth.speed_mps = std::abs(pose_sample.vel.x());
        truth_by_utime.emplace(pose_sample.utime, truth);
    }

    return truth_by_utime;
}

bool IsLineLike(const PrincipalDirectionSummary& principal_direction) {
    if (!(principal_direction.first_variance > 0.0)) {
        return true;
    }

    return (principal_direction.second_variance /
            principal_direction.first_variance) <=
           kMaxLineLikeSecondToFirstVarianceRatio;
}

double ComputeGlobalYaw(const PrincipalDirectionSummary& principal_direction,
                        const std::vector<ImuSample>& imu_samples,
                        std::int64_t startup_begin_utime,
                        std::int64_t ready_utime) {
    // Use the principal direction directly for line-like motion. Otherwise
    // correct it with accumulated IMU yaw.
    const double principal_component_yaw_rad =
        std::atan2(principal_direction.axis.y(), principal_direction.axis.x());
    if (IsLineLike(principal_direction)) {
        return principal_component_yaw_rad;
    }

    const double total_delta_yaw_rad =
        DeltaYawBetween(imu_samples, startup_begin_utime, ready_utime);
    return WrapAngle(principal_component_yaw_rad + 0.5 * total_delta_yaw_rad);
}

double ComputeTangentWeight(const std::vector<double>& heading_yaws_rad) {
    // Downweight the tangent heading when recent tangent estimates disagree.
    if (heading_yaws_rad.size() < kStabilityWindow) {
        return 0.0;
    }

    const double latest_yaw_rad = heading_yaws_rad.back();
    const std::size_t first_recent_index =
        heading_yaws_rad.size() - kStabilityWindow;
    double max_error_rad = 0.0;
    for (std::size_t index = first_recent_index; index < heading_yaws_rad.size();
         ++index) {
        max_error_rad =
            std::max(max_error_rad,
                     std::abs(AngleDiff(heading_yaws_rad[index],
                                        latest_yaw_rad)));
    }

    const double normalized_error =
        std::clamp(max_error_rad / kTangentStabilityTolRad, 0.0, 1.0);
    return 1.0 - normalized_error;
}

double BlendYaw(double tangent_yaw_rad,
                double global_yaw_rad,
                double tangent_weight) {
    // Blend the tangent and global headings on the unit circle.
    const double clamped_tangent_weight = std::clamp(tangent_weight, 0.0, 1.0);
    const double global_weight = 1.0 - clamped_tangent_weight;
    const Eigen::Vector2d blended =
        clamped_tangent_weight * UnitDirection(tangent_yaw_rad) +
        global_weight * UnitDirection(global_yaw_rad);

    if (blended.squaredNorm() <= 1e-12) {
        return clamped_tangent_weight >= 0.5 ? tangent_yaw_rad : global_yaw_rad;
    }

    return std::atan2(blended.y(), blended.x());
}

std::int64_t RecentSpeedWindowStartUtime(std::int64_t startup_begin_utime,
                                         std::int64_t ready_utime) {
    const std::int64_t requested_window_us = static_cast<std::int64_t>(
        std::llround(kRecentSpeedWindowSec / kSecPerUsec));
    return std::max(startup_begin_utime, ready_utime - requested_window_us);
}

double ComputeLocalSpeed(const QuadraticPathFit& path_fit,
                         std::int64_t recent_start_utime) {
    // Compute recent speed from fitted path length over the recent window.
    const double recent_start_tau_s =
        static_cast<double>(recent_start_utime - path_fit.end_utime) *
        kSecPerUsec;
    const double duration_s = -recent_start_tau_s;
    if (!(duration_s > 0.0)) {
        return VelocityAt(path_fit, 0.0).norm();
    }

    const double path_length_m =
        PathLengthBetween(path_fit, recent_start_tau_s, 0.0);
    return path_length_m / duration_s;
}

double ComputeGlobalSpeed(const QuadraticPathFit& path_fit,
                          const PrincipalDirectionSummary& principal_direction,
                          const std::vector<ImuSample>& imu_samples,
                          std::int64_t recent_start_utime,
                          double global_yaw_rad) {
    // Estimate recent global speed from projected chord or curved-path arc
    // length.
    const double recent_start_tau_s =
        static_cast<double>(recent_start_utime - path_fit.end_utime) *
        kSecPerUsec;
    const double duration_s = -recent_start_tau_s;
    if (!(duration_s > 0.0)) {
        return VelocityAt(path_fit, 0.0).norm();
    }

    const Eigen::Vector2d recent_start_xy =
        PositionAt(path_fit, recent_start_tau_s);
    const Eigen::Vector2d recent_end_xy = PositionAt(path_fit, 0.0);
    const Eigen::Vector2d delta_xy = recent_end_xy - recent_start_xy;

    double path_length_m = 0.0;
    if (IsLineLike(principal_direction)) {
        path_length_m = std::abs(UnitDirection(global_yaw_rad).dot(delta_xy));
    } else {
        const double recent_abs_turn_rad = std::abs(DeltaYawBetween(
            imu_samples, recent_start_utime, path_fit.end_utime));
        path_length_m =
            ArcLengthFromChordAndTurn(delta_xy.norm(), recent_abs_turn_rad);
    }

    return path_length_m / duration_s;
}

double RequiredProjectedSeparation(std::size_t gps_points_used) {
    return gps_points_used >= kRelaxedProjectedSeparationPointCount
               ? kLateMinProjectedSeparationM
               : kEarlyMinProjectedSeparationM;
}

ReadyEstimate2D FormReadyEstimate(const QuadraticPathFit& path_fit,
                                  double selected_speed_mps,
                                  double blended_yaw_rad,
                                  std::size_t first_unprocessed_gps_index) {
    // Package the ready 2D state before IMU handoff.
    ReadyEstimate2D ready_estimate;
    ready_estimate.ready_utime = path_fit.end_utime;
    ready_estimate.position_xy = PositionAt(path_fit, 0.0);
    ready_estimate.velocity_xy =
        selected_speed_mps * UnitDirection(blended_yaw_rad);
    ready_estimate.yaw_rad = blended_yaw_rad;
    ready_estimate.first_unprocessed_gps_index = first_unprocessed_gps_index;
    return ready_estimate;
}

std::optional<StartupInitialization> FinalizeAtImuHandoff(
    const ReadyEstimate2D& ready_estimate,
    const std::vector<ImuSample>& imu_samples) {
    // Carry the ready 2D state to the first IMU sample at or after ready
    // time.
    std::size_t handoff_imu_index = 0;
    while (handoff_imu_index < imu_samples.size() &&
           imu_samples[handoff_imu_index].utime < ready_estimate.ready_utime) {
        ++handoff_imu_index;
    }
    if (handoff_imu_index >= imu_samples.size()) {
        return std::nullopt;
    }

    const ImuSample& handoff_imu_sample = imu_samples[handoff_imu_index];
    const double dt_gap_s = static_cast<double>(handoff_imu_sample.utime -
                                                ready_estimate.ready_utime) *
                            kSecPerUsec;
    const double carried_yaw_rad = WrapAngle(
        ready_estimate.yaw_rad + DeltaYawBetween(imu_samples,
                                                 ready_estimate.ready_utime,
                                                 handoff_imu_sample.utime));

    StartupInitialization startup_initialization;
    startup_initialization.p0_G =
        Eigen::Vector3d(ready_estimate.position_xy.x() +
                            ready_estimate.velocity_xy.x() * dt_gap_s,
                        ready_estimate.position_xy.y() +
                            ready_estimate.velocity_xy.y() * dt_gap_s,
                        0.0);
    startup_initialization.v0_G = Eigen::Vector3d(
        ready_estimate.velocity_xy.x(), ready_estimate.velocity_xy.y(), 0.0);
    startup_initialization.q0_GI =
        BuildGlobalOrientation(carried_yaw_rad, handoff_imu_sample);
    startup_initialization.previous_imu_utime = handoff_imu_sample.utime;
    startup_initialization.first_unprocessed_imu_index =
        handoff_imu_index + 1;
    startup_initialization.first_unprocessed_gps_index =
        ready_estimate.first_unprocessed_gps_index;
    return startup_initialization;
}

}  // namespace

StartupTraceResult TraceStartupInitialization(
    const std::vector<PoseSample>& pose_samples,
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples,
    const std::vector<WheelSpeedSample>& wheel_speed_samples) {
    StartupTraceResult trace_result;

    trace_result.first_usable_gps_index =
        FindFirstUsableGpsIndex(imu_samples, gps_samples);
    if (!trace_result.first_usable_gps_index.has_value()) {
        return trace_result;
    }

    const std::size_t startup_begin_gps_index =
        *trace_result.first_usable_gps_index;
    const std::size_t gps_points_available =
        gps_samples.size() - startup_begin_gps_index;
    if (!HasMinimumPointsForFit(gps_points_available)) {
        return trace_result;
    }

    const bool have_truth = !pose_samples.empty();
    const TruthStateMap truth_by_utime =
        have_truth ? BuildTruthStateMap(pose_samples) : TruthStateMap{};
    if (wheel_speed_samples.empty()) {
        return trace_result;
    }

    const WheelMotionProfile wheel_motion_profile =
        BuildWheelMotionProfile(wheel_speed_samples);
    const std::int64_t startup_begin_utime =
        gps_samples[startup_begin_gps_index].utime;

    std::vector<double> recent_heading_yaws_rad;
    for (std::size_t gps_end_index =
             startup_begin_gps_index + (kMinFitPointCount - 1);
         gps_end_index < gps_samples.size();
         ++gps_end_index) {
        const QuadraticPathFit path_fit = FitQuadraticPath(
            gps_samples, startup_begin_gps_index, gps_end_index);

        const Eigen::Vector2d endpoint_velocity = VelocityAt(path_fit, 0.0);
        const double endpoint_speed_mps = endpoint_velocity.norm();
        if (!(endpoint_speed_mps > kMinEndpointSpeed)) {
            continue;
        }
        const double tangent_yaw_rad =
            std::atan2(endpoint_velocity.y(), endpoint_velocity.x());

        recent_heading_yaws_rad.push_back(tangent_yaw_rad);

        const PrincipalDirectionSummary principal_direction =
            SummarizePrincipalDirection(
                gps_samples, startup_begin_gps_index, gps_end_index);

        const std::size_t gps_points_used =
            gps_end_index - startup_begin_gps_index + 1;
        const double required_projected_separation_m =
            RequiredProjectedSeparation(gps_points_used);
        const std::int64_t ready_utime = path_fit.end_utime;
        const double global_yaw_rad = ComputeGlobalYaw(
            principal_direction, imu_samples, startup_begin_utime, ready_utime);

        const double tangent_weight =
            ComputeTangentWeight(recent_heading_yaws_rad);
        const double blended_yaw_rad =
            BlendYaw(tangent_yaw_rad, global_yaw_rad, tangent_weight);

        const std::int64_t recent_start_utime =
            RecentSpeedWindowStartUtime(startup_begin_utime, ready_utime);
        const double local_speed_mps =
            ComputeLocalSpeed(path_fit, recent_start_utime);
        const double global_speed_mps = ComputeGlobalSpeed(path_fit,
                                                           principal_direction,
                                                           imu_samples,
                                                           recent_start_utime,
                                                           global_yaw_rad);
        const double wheel_speed_mps = InterpolateWheelProfileValue(
            wheel_motion_profile, wheel_motion_profile.smoothed_speed_mps,
            ready_utime);
        const double selected_speed_mps = wheel_speed_mps;
        const double wheel_supported_travel_m = WheelSupportedTravelBetween(
            wheel_motion_profile, startup_begin_utime, ready_utime);

        StartupTraceFrame frame;
        frame.fit_end_index = gps_end_index;
        frame.gps_points_used = gps_points_used;
        frame.end_utime = ready_utime;
        frame.latest_gps_xy = gps_samples[gps_end_index].xy;
        frame.fitted_endpoint_xy = PositionAt(path_fit, 0.0);
        frame.fitted_velocity_xy = endpoint_velocity;
        frame.fitted_speed_mps = endpoint_speed_mps;
        frame.fitted_yaw_rad = tangent_yaw_rad;
        frame.selected_yaw_rad = blended_yaw_rad;
        frame.tangent_weight = tangent_weight;
        frame.global_yaw_rad = global_yaw_rad;
        frame.pca_yaw_rad = std::atan2(principal_direction.axis.y(),
                                       principal_direction.axis.x());
        frame.projected_separation_m =
            principal_direction.projected_start_to_end_m;
        frame.required_projected_separation_m =
            required_projected_separation_m;
        frame.line_like = IsLineLike(principal_direction);
        frame.local_speed_mps = local_speed_mps;
        frame.global_speed_mps = global_speed_mps;
        frame.selected_speed_mps = selected_speed_mps;
        frame.wheel_speed_mps = wheel_speed_mps;
        frame.wheel_supported_travel_m = wheel_supported_travel_m;
        frame.required_wheel_supported_travel_m =
            kRequiredWheelSupportedTravelM;
        frame.path_xy = SamplePathPoints(
            path_fit, gps_samples, startup_begin_gps_index, gps_end_index);
        if (have_truth) {
            const auto truth_it = truth_by_utime.find(ready_utime);
            if (truth_it == truth_by_utime.end()) {
                throw std::runtime_error(
                    "Startup trace missing truth pose at GPS timestamp.");
            }
            frame.truth = truth_it->second;
        }
        trace_result.frames.push_back(frame);

        if (wheel_supported_travel_m < kRequiredWheelSupportedTravelM) {
            continue;
        }

        const ReadyEstimate2D ready_estimate = FormReadyEstimate(
            path_fit, selected_speed_mps, blended_yaw_rad, gps_end_index + 1);

        trace_result.ready_frame_index = trace_result.frames.size() - 1;
        trace_result.startup_initialization =
            FinalizeAtImuHandoff(ready_estimate, imu_samples);
        return trace_result;
    }

    return trace_result;
}

std::optional<StartupInitialization> ComputeStartupInitialization(
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples,
    const std::vector<WheelSpeedSample>& wheel_speed_samples) {
    const std::optional<std::size_t> first_gps_index =
        FindFirstUsableGpsIndex(imu_samples, gps_samples);
    if (!first_gps_index.has_value()) {
        return std::nullopt;
    }

    const std::size_t startup_begin_gps_index = *first_gps_index;
    const std::size_t gps_points_available =
        gps_samples.size() - startup_begin_gps_index;
    if (!HasMinimumPointsForFit(gps_points_available)) {
        return std::nullopt;
    }
    if (wheel_speed_samples.empty()) {
        return std::nullopt;
    }

    const WheelMotionProfile wheel_motion_profile =
        BuildWheelMotionProfile(wheel_speed_samples);
    const std::int64_t startup_begin_utime =
        gps_samples[startup_begin_gps_index].utime;
    std::vector<double> recent_heading_yaws_rad;
    for (std::size_t gps_end_index =
             startup_begin_gps_index + (kMinFitPointCount - 1);
         gps_end_index < gps_samples.size();
         ++gps_end_index) {
        const QuadraticPathFit path_fit = FitQuadraticPath(
            gps_samples, startup_begin_gps_index, gps_end_index);

        const Eigen::Vector2d endpoint_velocity = VelocityAt(path_fit, 0.0);
        const double endpoint_speed_mps = endpoint_velocity.norm();
        if (!(endpoint_speed_mps > kMinEndpointSpeed)) {
            continue;
        }
        const double tangent_yaw_rad =
            std::atan2(endpoint_velocity.y(), endpoint_velocity.x());

        recent_heading_yaws_rad.push_back(tangent_yaw_rad);

        const PrincipalDirectionSummary principal_direction =
            SummarizePrincipalDirection(
                gps_samples, startup_begin_gps_index, gps_end_index);

        const std::int64_t ready_utime = path_fit.end_utime;
        const double wheel_supported_travel_m = WheelSupportedTravelBetween(
            wheel_motion_profile, startup_begin_utime, ready_utime);
        if (wheel_supported_travel_m < kRequiredWheelSupportedTravelM) {
            continue;
        }
        const double global_yaw_rad = ComputeGlobalYaw(
            principal_direction, imu_samples, startup_begin_utime, ready_utime);

        const double tangent_weight =
            ComputeTangentWeight(recent_heading_yaws_rad);
        const double blended_yaw_rad =
            BlendYaw(tangent_yaw_rad, global_yaw_rad, tangent_weight);

        const double wheel_speed_mps = InterpolateWheelProfileValue(
            wheel_motion_profile, wheel_motion_profile.smoothed_speed_mps,
            ready_utime);

        const ReadyEstimate2D ready_estimate = FormReadyEstimate(
            path_fit, wheel_speed_mps, blended_yaw_rad, gps_end_index + 1);

        return FinalizeAtImuHandoff(ready_estimate, imu_samples);
    }

    return std::nullopt;
}
