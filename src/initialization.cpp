#include "initialization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace {

constexpr double kSecPerUsec = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMinFitPointCount = 3;
constexpr double kMinEndpointSpeed = 1e-6;
constexpr double kRequiredWheelSupportedTravelM = 10.0;
constexpr double kMaxLineLikeSecondToFirstVarianceRatio = 0.1;
constexpr double kClusterYawWeight = 0.75;
constexpr double kWheelSpeedSmoothingWindowSec = 0.3;
constexpr double kWheelRadiusM = 0.305;

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

double WrapAngle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

Eigen::Vector2d UnitDirection(double yaw_rad) {
    return Eigen::Vector2d(std::cos(yaw_rad), std::sin(yaw_rad));
}

Eigen::Quaterniond QuaternionFromYaw(double yaw_rad) {
    const double half_yaw_rad = 0.5 * yaw_rad;
    return Eigen::Quaterniond(
        std::cos(half_yaw_rad), 0.0, 0.0, std::sin(half_yaw_rad));
}

Eigen::Quaterniond BuildGlobalOrientation(double yaw_rad,
                                          const ImuSample& imu_ref) {
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

double DeltaYawBetween(const std::vector<ImuSample>& imu_samples,
                       std::int64_t start_utime,
                       std::int64_t end_utime) {
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
    const double principal_component_yaw_rad =
        std::atan2(principal_direction.axis.y(), principal_direction.axis.x());
    if (IsLineLike(principal_direction)) {
        return principal_component_yaw_rad;
    }

    const double total_delta_yaw_rad =
        DeltaYawBetween(imu_samples, startup_begin_utime, ready_utime);
    return WrapAngle(principal_component_yaw_rad + 0.5 * total_delta_yaw_rad);
}

std::size_t ClusterMeanPointCount(std::size_t gps_points_used) {
    if (gps_points_used < 2) {
        throw std::runtime_error("Cluster heading needs at least two GPS points.");
    }

    const std::size_t preferred_count =
        std::clamp(gps_points_used / 4, std::size_t{2}, std::size_t{6});
    return std::min(preferred_count,
                    std::max<std::size_t>(1, gps_points_used / 2));
}

Eigen::Vector2d MeanGpsPosition(const std::vector<GpsSample>& gps_samples,
                                std::size_t first_index,
                                std::size_t last_index) {
    Eigen::Vector2d mean_xy = Eigen::Vector2d::Zero();
    for (std::size_t index = first_index; index <= last_index; ++index) {
        mean_xy += gps_samples[index].xy;
    }

    return mean_xy / static_cast<double>(last_index - first_index + 1);
}

double ComputeClusterYaw(const std::vector<GpsSample>& gps_samples,
                         std::size_t first_index,
                         std::size_t last_index,
                         double fallback_yaw_rad) {
    const std::size_t gps_points_used = last_index - first_index + 1;
    const std::size_t cluster_point_count =
        ClusterMeanPointCount(gps_points_used);
    const Eigen::Vector2d early_mean_xy = MeanGpsPosition(
        gps_samples, first_index, first_index + cluster_point_count - 1);
    const Eigen::Vector2d late_mean_xy = MeanGpsPosition(
        gps_samples, last_index - cluster_point_count + 1, last_index);
    const Eigen::Vector2d delta_xy = late_mean_xy - early_mean_xy;
    if (delta_xy.squaredNorm() <= 1e-12) {
        return fallback_yaw_rad;
    }

    return std::atan2(delta_xy.y(), delta_xy.x());
}

double BlendYaw(double primary_yaw_rad,
                double secondary_yaw_rad,
                double primary_weight) {
    const double clamped_primary_weight = std::clamp(primary_weight, 0.0, 1.0);
    const double secondary_weight = 1.0 - clamped_primary_weight;
    const Eigen::Vector2d blended =
        clamped_primary_weight * UnitDirection(primary_yaw_rad) +
        secondary_weight * UnitDirection(secondary_yaw_rad);

    if (blended.squaredNorm() <= 1e-12) {
        return clamped_primary_weight >= 0.5 ? primary_yaw_rad : secondary_yaw_rad;
    }

    return std::atan2(blended.y(), blended.x());
}

ReadyEstimate2D FormReadyEstimate(const QuadraticPathFit& path_fit,
                                  double selected_speed_mps,
                                  double selected_yaw_rad,
                                  std::size_t first_unprocessed_gps_index) {
    ReadyEstimate2D ready_estimate;
    ready_estimate.ready_utime = path_fit.end_utime;
    ready_estimate.position_xy = PositionAt(path_fit, 0.0);
    ready_estimate.velocity_xy =
        selected_speed_mps * UnitDirection(selected_yaw_rad);
    ready_estimate.yaw_rad = selected_yaw_rad;
    ready_estimate.first_unprocessed_gps_index = first_unprocessed_gps_index;
    return ready_estimate;
}

std::optional<StartupInitialization> FinalizeAtImuHandoff(
    const ReadyEstimate2D& ready_estimate,
    const std::vector<ImuSample>& imu_samples) {
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

bool HasProcessablePostStartupGpsUpdate(
    const StartupInitialization& startup_initialization,
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples) {
    if (startup_initialization.first_unprocessed_gps_index >=
        gps_samples.size()) {
        return false;
    }

    if (startup_initialization.first_unprocessed_imu_index >=
        imu_samples.size()) {
        return false;
    }

    return gps_samples[startup_initialization.first_unprocessed_gps_index]
               .utime <= imu_samples.back().utime;
}

}  // namespace

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
        const double cluster_yaw_rad = ComputeClusterYaw(
            gps_samples, startup_begin_gps_index, gps_end_index, global_yaw_rad);
        const double selected_yaw_rad =
            BlendYaw(cluster_yaw_rad, global_yaw_rad, kClusterYawWeight);

        const double wheel_speed_mps = InterpolateWheelProfileValue(
            wheel_motion_profile, wheel_motion_profile.smoothed_speed_mps,
            ready_utime);

        const ReadyEstimate2D ready_estimate = FormReadyEstimate(
            path_fit, wheel_speed_mps, selected_yaw_rad, gps_end_index + 1);

        const std::optional<StartupInitialization> startup_initialization =
            FinalizeAtImuHandoff(ready_estimate, imu_samples);
        if (!startup_initialization.has_value()) {
            continue;
        }

        if (!HasProcessablePostStartupGpsUpdate(
                *startup_initialization, imu_samples, gps_samples)) {
            continue;
        }

        return startup_initialization;
    }

    return std::nullopt;
}
