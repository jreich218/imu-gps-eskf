#include "initialization.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

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
constexpr double kMaxLineLikeSecondToFirstVarianceRatio = 0.1;
constexpr double kRecentSpeedWindowSec = 1.0;
constexpr int kPathLengthQuadratureSteps = 32;

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

struct HeadingSample {
    std::size_t gps_end_index = 0;
    std::int64_t end_utime = 0;
    double yaw_rad = 0.0;
};

struct ReadyEstimate2D {
    std::int64_t ready_utime = 0;
    Eigen::Vector2d position_xy{0.0, 0.0};
    Eigen::Vector2d velocity_xy{0.0, 0.0};
    double yaw_rad = 0.0;
    std::size_t next_gps_index = 0;
};

double WrapAngle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double AngleDiff(double a_rad, double b_rad) {
    return WrapAngle(a_rad - b_rad);
}

Eigen::Vector2d UnitDirection(double yaw_rad) {
    return Eigen::Vector2d(std::cos(yaw_rad), std::sin(yaw_rad));
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
    return Eigen::Quaterniond(std::cos(half_yaw_rad), 0.0, 0.0,
                              std::sin(half_yaw_rad));
}

Eigen::Quaterniond BuildGlobalOrientation(double yaw_rad,
                                          const ImuSample& imu_ref) {
    // `q_AI` maps IMU coordinates into the gravity-aligned frame `A`, whose
    // yaw is arbitrary. To get the desired global yaw, first measure the yaw
    // of the IMU x-axis in `A`, then add the yaw correction that takes that
    // axis direction to the requested global yaw.
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
    // Find the first GPS sample whose timestamp is not earlier than the first
    // IMU timestamp.
    //
    // Startup must begin on the IMU time axis, so earlier GPS samples cannot
    // seed the startup estimate.
    //
    // Return:
    // - `std::nullopt` when startup cannot even begin from the available
    //   streams
    // - otherwise the first GPS index that startup is allowed to use
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
    // Answer the small policy question: do we have enough GPS points to fit
    // the first quadratic startup path?
    //
    // The startup path fit begins with exactly `kMinFitPointCount` points and
    // then grows one point at a time as the window is extended.
    return gps_point_count >= kMinFitPointCount;
}

QuadraticPathFit FitQuadraticPath(const std::vector<GpsSample>& gps_samples,
                                  std::size_t first_index,
                                  std::size_t last_index) {
    // Fit one quadratic path in time to the GPS `x` and `y` coordinates over
    // the inclusive GPS window `[first_index, last_index]`.
    //
    // The fit will be expressed in a local time variable whose zero is the end
    // of the window. That keeps the endpoint evaluation simple:
    // - position at the ready time comes from evaluating the fit at `tau = 0`
    // - tangent velocity at the ready time comes from differentiating the fit
    //   and evaluating that derivative at `tau = 0`
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
    // Evaluate the fitted startup path position at a time offset `tau_s`
    // relative to the fit end time.
    //
    // It is used both at the ready time itself (`tau = 0`) and, later, at the
    // start of the recent-speed window.
    return Eigen::Vector2d(
        path_fit.coeff_x.x() + path_fit.coeff_x.y() * tau_s +
            path_fit.coeff_x.z() * tau_s * tau_s,
        path_fit.coeff_y.x() + path_fit.coeff_y.y() * tau_s +
            path_fit.coeff_y.z() * tau_s * tau_s);
}

Eigen::Vector2d VelocityAt(const QuadraticPathFit& path_fit, double tau_s) {
    // Evaluate the derivative of the fitted startup path at the requested time
    // offset.
    //
    // The endpoint derivative gives the local tangent velocity. From that one
    // vector we get both:
    // - the local tangent heading
    // - the local endpoint speed
    return Eigen::Vector2d(path_fit.coeff_x.y() + 2.0 * path_fit.coeff_x.z() * tau_s,
                           path_fit.coeff_y.y() + 2.0 * path_fit.coeff_y.z() * tau_s);
}

double PathLengthBetween(const QuadraticPathFit& path_fit,
                         double tau_start_s,
                         double tau_end_s) {
    // Compute the arc length of the fitted quadratic path between two local
    // times.
    //
    // The startup code uses that length to form a recent local speed from the
    // fitted path geometry instead of relying on a single derivative sample
    // alone.
    if (tau_end_s < tau_start_s) {
        throw std::runtime_error(
            "Startup path length interval must be nondecreasing.");
    }

    if (tau_end_s == tau_start_s) {
        return 0.0;
    }

    const double h =
        (tau_end_s - tau_start_s) / static_cast<double>(kPathLengthQuadratureSteps);
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
    // Accumulate IMU yaw change across the requested time interval.
    //
    // The accumulated yaw serves two startup purposes:
    // - making the global PCA heading curvature-aware when the GPS cloud is
    //   not line-like
    // - carrying the ready yaw forward to the first IMU sample at or after the
    //   ready time
    if (end_utime < start_utime) {
        throw std::runtime_error(
            "Startup IMU interval must be nondecreasing.");
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
    // Summarize the current GPS cloud with a principal-direction calculation.
    //
    // The summary will provide the dominant axis of the cloud, the first and
    // second variances, and the start-to-end separation projected onto that
    // dominant axis.
    //
    // Those outputs are the ingredients for:
    // - a global heading estimate
    // - the line-like vs curved decision
    // - the trust gate based on projected separation
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
    // Decide whether the current GPS cloud is line-like enough that its
    // principal direction can be used directly as the global heading.
    //
    // When the cloud is not line-like, the startup code will treat the
    // principal direction as a coarse heading and then use IMU yaw accumulation
    // to make that heading curvature-aware.
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
    // Produce the global heading estimate for the current GPS window.
    //
    // The intended behavior is:
    // - if the GPS cloud is line-like, use the principal direction directly
    // - otherwise, blend in curvature information by accumulating IMU yaw
    //   across the startup interval
    const double principal_component_yaw_rad =
        std::atan2(principal_direction.axis.y(), principal_direction.axis.x());
    if (IsLineLike(principal_direction)) {
        return principal_component_yaw_rad;
    }

    const double total_delta_yaw_rad =
        DeltaYawBetween(imu_samples, startup_begin_utime, ready_utime);
    return WrapAngle(principal_component_yaw_rad + 0.5 * total_delta_yaw_rad);
}

double ComputeTangentWeight(const std::vector<HeadingSample>& heading_samples) {
    // Measure how stable the most recent tangent headings have been.
    //
    // The result will be a weight in `[0, 1]`:
    // - near `1` when the recent tangent headings look stable enough to trust
    // - near `0` when they are still wobbling and the global heading should
    //   dominate
    if (heading_samples.size() < kStabilityWindow) {
        return 0.0;
    }

    const double latest_yaw_rad = heading_samples.back().yaw_rad;
    const std::size_t first_recent_index =
        heading_samples.size() - kStabilityWindow;
    double max_error_rad = 0.0;
    for (std::size_t index = first_recent_index;
         index < heading_samples.size();
         ++index) {
        max_error_rad = std::max(
            max_error_rad,
            std::abs(AngleDiff(heading_samples[index].yaw_rad, latest_yaw_rad)));
    }

    const double normalized_error =
        std::clamp(max_error_rad / kTangentStabilityTolRad, 0.0, 1.0);
    return 1.0 - normalized_error;
}

double BlendYaw(double tangent_yaw_rad,
                double global_yaw_rad,
                double tangent_weight) {
    // Blend the local tangent heading with the global heading using the
    // tangent-stability weight.
    //
    // Blend on the unit circle so wraparound at `+-pi` is handled correctly.
    const double clamped_tangent_weight =
        std::clamp(tangent_weight, 0.0, 1.0);
    const double global_weight = 1.0 - clamped_tangent_weight;
    const Eigen::Vector2d blended =
        clamped_tangent_weight * UnitDirection(tangent_yaw_rad) +
        global_weight * UnitDirection(global_yaw_rad);

    if (blended.squaredNorm() <= 1e-12) {
        return clamped_tangent_weight >= 0.5 ? tangent_yaw_rad
                                             : global_yaw_rad;
    }

    return std::atan2(blended.y(), blended.x());
}

std::int64_t RecentSpeedWindowStartUtime(std::int64_t startup_begin_utime,
                                         std::int64_t ready_utime) {
    // Choose the start time of the recent-speed window used for the local-
    // speed and global-speed calculations.
    //
    // Use a fixed recent duration, clipped so the recent-speed window never
    // begins before startup itself began.
    const std::int64_t requested_window_us = static_cast<std::int64_t>(
        std::llround(kRecentSpeedWindowSec / kSecPerUsec));
    return std::max(startup_begin_utime, ready_utime - requested_window_us);
}

double ComputeLocalSpeed(const QuadraticPathFit& path_fit,
                         std::int64_t recent_start_utime) {
    // Compute a recent local speed from the fitted quadratic path.
    //
    // It will do that by measuring the fitted path length across the recent
    // window and dividing by that recent-window duration.
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
    // Compute a recent global speed from the recent path geometry.
    //
    // The intended behavior is:
    // - for line-like motion, use the recent motion projected onto the global
    //   heading direction
    // - for curved motion, use the recent chord together with recent IMU yaw
    //   accumulation to estimate traveled arc length
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

double BlendSpeed(double local_speed_mps,
                  double global_speed_mps,
                  double tangent_weight) {
    // Blend the recent local speed and recent global speed with the same
    // weight already used for the heading blend.
    //
    // Reusing the same weight keeps the heading and speed confidence decisions
    // aligned.
    const double clamped_tangent_weight =
        std::clamp(tangent_weight, 0.0, 1.0);
    return clamped_tangent_weight * local_speed_mps +
           (1.0 - clamped_tangent_weight) * global_speed_mps;
}

double RequiredProjectedSeparation(std::size_t gps_points_used) {
    // Return the projected start-to-end separation threshold for the current
    // GPS window.
    //
    // Use the earlier, stricter threshold at the beginning of startup and
    // relax that threshold after enough GPS points have been seen.
    return gps_points_used >= kRelaxedProjectedSeparationPointCount
               ? kLateMinProjectedSeparationM
               : kEarlyMinProjectedSeparationM;
}

std::optional<ReadyEstimate2D> FormReadyEstimate(
    const QuadraticPathFit& path_fit,
    double blended_speed_mps,
    double blended_yaw_rad,
    std::size_t next_gps_index) {
    // Turn one trustworthy GPS window into the 2D ready state that startup has
    // been seeking.
    //
    // Once the trust gate passes, the startup code no longer needs to keep
    // extending the GPS window. It can package:
    // - the ready time
    // - the ready position
    // - the ready velocity
    // - the ready yaw
    // - the first GPS sample not consumed by initialization
    //
    // and hand that result to the final IMU-handoff step.
    ReadyEstimate2D ready_estimate;
    ready_estimate.ready_utime = path_fit.end_utime;
    ready_estimate.position_xy = PositionAt(path_fit, 0.0);
    ready_estimate.velocity_xy =
        blended_speed_mps * UnitDirection(blended_yaw_rad);
    ready_estimate.yaw_rad = blended_yaw_rad;
    ready_estimate.next_gps_index = next_gps_index;
    return ready_estimate;
}

std::optional<StartupInitialization> FinalizeAtImuHandoff(
    const ReadyEstimate2D& ready_estimate,
    const std::vector<ImuSample>& imu_samples) {
    // Carry the ready 2D state forward to the first IMU sample at or after
    // the ready time.
    //
    // This step:
    // - find that handoff IMU sample
    // - accumulate IMU yaw across the carry interval
    // - carry position forward across the gap using the ready velocity
    // - build `q0_GI` from the carried yaw and the handoff IMU orientation
    // - populate `p0_G`, `v0_G`, `q0_GI`, `next_imu_index`, and
    //   `next_gps_index`
    std::size_t handoff_imu_index = 0;
    while (handoff_imu_index < imu_samples.size() &&
           imu_samples[handoff_imu_index].utime < ready_estimate.ready_utime) {
        ++handoff_imu_index;
    }
    if (handoff_imu_index >= imu_samples.size()) {
        return std::nullopt;
    }

    const ImuSample& handoff_imu_sample = imu_samples[handoff_imu_index];
    const double dt_gap_s =
        static_cast<double>(handoff_imu_sample.utime - ready_estimate.ready_utime) *
        kSecPerUsec;
    const double carried_yaw_rad = WrapAngle(
        ready_estimate.yaw_rad + DeltaYawBetween(imu_samples,
                                                 ready_estimate.ready_utime,
                                                 handoff_imu_sample.utime));

    StartupInitialization startup_initialization;
    startup_initialization.p0_G = Eigen::Vector3d(
        ready_estimate.position_xy.x() + ready_estimate.velocity_xy.x() * dt_gap_s,
        ready_estimate.position_xy.y() + ready_estimate.velocity_xy.y() * dt_gap_s,
        0.0);
    startup_initialization.v0_G = Eigen::Vector3d(
        ready_estimate.velocity_xy.x(), ready_estimate.velocity_xy.y(), 0.0);
    startup_initialization.q0_GI =
        BuildGlobalOrientation(carried_yaw_rad, handoff_imu_sample);
    startup_initialization.utime_last_imu = handoff_imu_sample.utime;
    startup_initialization.next_imu_index = handoff_imu_index + 1;
    startup_initialization.next_gps_index = ready_estimate.next_gps_index;
    return startup_initialization;
}

}  // namespace

std::optional<StartupInitialization> ComputeStartupInitialization(
    const std::vector<ImuSample>& imu_samples,
    const std::vector<GpsSample>& gps_samples) {
    // 1. Find the first GPS sample that startup is allowed to use. Startup
    //    only begins once the IMU stream has begun, so any earlier GPS samples
    //    are outside the startup time interval.
    const std::optional<std::size_t> first_gps_index =
        FindFirstUsableGpsIndex(imu_samples, gps_samples);
    if (!first_gps_index.has_value()) {
        return std::nullopt;
    }

    // 2. Count how many GPS points remain from that first usable sample
    //    onward. Startup cannot proceed unless there are enough points to fit
    //    the first quadratic path.
    const std::size_t startup_begin_gps_index = *first_gps_index;
    const std::size_t gps_points_available =
        gps_samples.size() - startup_begin_gps_index;
    if (!HasMinimumPointsForFit(gps_points_available)) {
        return std::nullopt;
    }

    // 3. Sweep the GPS window from the minimum three-point fit upward. Each
    //    loop iteration represents one candidate startup window ending at
    //    `gps_end_index`.
    std::vector<HeadingSample> recent_heading_samples;
    for (std::size_t gps_end_index =
             startup_begin_gps_index + (kMinFitPointCount - 1);
         gps_end_index < gps_samples.size();
         ++gps_end_index) {
        // 4. Fit the current quadratic path in time over the inclusive GPS
        //    window `[startup_begin_gps_index, gps_end_index]`.
        const QuadraticPathFit path_fit = FitQuadraticPath(
            gps_samples, startup_begin_gps_index, gps_end_index);

        // 5. Evaluate the endpoint tangent velocity of that fit. The later
        //    startup logic uses that one vector to get the local tangent
        //    heading and the local endpoint speed.
        const Eigen::Vector2d endpoint_velocity = VelocityAt(path_fit, 0.0);
        const double endpoint_speed_mps = endpoint_velocity.norm();
        if (!(endpoint_speed_mps > kMinEndpointSpeed)) {
            continue;
        }
        const double tangent_yaw_rad =
            std::atan2(endpoint_velocity.y(), endpoint_velocity.x());

        // 6. Record the current tangent heading in the recent-heading history.
        //    The recent-heading history measures whether the tangent direction
        //    has stabilized yet.
        recent_heading_samples.push_back(
            {gps_end_index, path_fit.end_utime, tangent_yaw_rad});

        // 7. Summarize the current GPS cloud with a principal-direction
        //    calculation. That summary supplies the global heading candidate
        //    and the projected-separation trust signal.
        const PrincipalDirectionSummary principal_direction =
            SummarizePrincipalDirection(
                gps_samples, startup_begin_gps_index, gps_end_index);

        // 8. Require enough projected start-to-end separation before trusting
        //    this startup window. The threshold starts stricter and then
        //    relaxes after enough GPS points have been seen.
        const std::size_t gps_points_used =
            gps_end_index - startup_begin_gps_index + 1;
        const double required_projected_separation_m =
            RequiredProjectedSeparation(gps_points_used);
        if (principal_direction.projected_start_to_end_m <
            required_projected_separation_m) {
            continue;
        }

        // 9. Build the global heading estimate. When the cloud is line-like,
        //    the principal direction itself is enough. When it is not line-
        //    like, IMU yaw accumulation makes that heading curvature-aware.
        const std::int64_t startup_begin_utime =
            gps_samples[startup_begin_gps_index].utime;
        const std::int64_t ready_utime = path_fit.end_utime;
        const double global_yaw_rad = ComputeGlobalYaw(
            principal_direction, imu_samples, startup_begin_utime, ready_utime);

        // 10. Measure recent tangent-heading stability and use that stability
        //     as the weight for blending the local tangent heading with the
        //     global heading.
        const double tangent_weight =
            ComputeTangentWeight(recent_heading_samples);
        const double blended_yaw_rad =
            BlendYaw(tangent_yaw_rad, global_yaw_rad, tangent_weight);

        // 11. Choose the recent-speed window and compute the recent local and
        //     recent global speeds from the fitted path and recent geometry.
        const std::int64_t recent_start_utime =
            RecentSpeedWindowStartUtime(startup_begin_utime, ready_utime);
        const double local_speed_mps =
            ComputeLocalSpeed(path_fit, recent_start_utime);
        const double global_speed_mps =
            ComputeGlobalSpeed(path_fit, principal_direction, imu_samples,
                               recent_start_utime, global_yaw_rad);

        // 12. Blend the recent local and global speed estimates with the same
        //     weight already used for the heading blend.
        const double blended_speed_mps =
            BlendSpeed(local_speed_mps, global_speed_mps, tangent_weight);

        // 13. As soon as one GPS window looks trustworthy, package the ready
        //     2D startup state and stop extending the window any further.
        const std::optional<ReadyEstimate2D> ready_estimate =
            FormReadyEstimate(path_fit, blended_speed_mps, blended_yaw_rad,
                              gps_end_index + 1);
        if (!ready_estimate.has_value()) {
            continue;
        }

        // 14. Carry that ready state forward to the first IMU sample at or
        //     after the ready time, then return the completed startup result.
        return FinalizeAtImuHandoff(*ready_estimate, imu_samples);
    }

    // 15. If no GPS window ever becomes trustworthy, startup has no result.
    return std::nullopt;
}
