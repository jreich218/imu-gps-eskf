#include "eskf.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <sstream>

namespace {

constexpr double kCovarianceDiagonalEpsilon = 1e-12;
constexpr double kCovarianceDiagonalNegativeTolerance = 1e-9;

Eigen::Quaterniond RotVecToQuat(const Eigen::Vector3d& phi) {
    double theta = phi.norm();
    Eigen::Quaterniond dq;

    if (theta < 1e-8) {
        dq.w() = 1.0;
        dq.vec() = 0.5 * phi;
        dq.normalize();
    } else {
        double half = 0.5 * theta;
        dq.w() = std::cos(half);
        dq.vec() = (std::sin(half) / theta) * phi;
    }

    return dq;
}

constexpr double DegToRad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

void SymmetrizeAndClampCovariance(
    Eigen::Matrix<double, 9, 9>& covariance_matrix) {
    covariance_matrix =
        0.5 * (covariance_matrix + covariance_matrix.transpose());

    for (int i = 0; i < covariance_matrix.rows(); ++i) {
        const double covariance_diagonal = covariance_matrix(i, i);
        if (covariance_diagonal < 0.0) {
            if (covariance_diagonal >=
                -kCovarianceDiagonalNegativeTolerance) {
                covariance_matrix(i, i) = kCovarianceDiagonalEpsilon;
            } else {
                std::ostringstream error_message;
                error_message
                    << "Eskf: covariance diagonal became significantly "
                       "negative at i="
                    << i << " (value=" << covariance_diagonal << ")";
                throw std::runtime_error(error_message.str());
            }
        }
    }
}
}  // namespace

void Eskf::Initialize(const StartupInitialization& startup_initialization) {
    nominal_state_.p_G = startup_initialization.p0_G;
    nominal_state_.v_G = startup_initialization.v0_G;
    nominal_state_.q_GI = startup_initialization.q0_GI.normalized();
    previous_imu_utime_ = startup_initialization.previous_imu_utime;

    P_.setZero();

    P_(0, 0) = 4.0;
    P_(1, 1) = 4.0;
    P_(2, 2) = 0.01;

    P_(3, 3) = 4.0;
    P_(4, 4) = 4.0;
    P_(5, 5) = 0.01;

    P_(6, 6) = DegToRad(2.0) * DegToRad(2.0);
    P_(7, 7) = DegToRad(2.0) * DegToRad(2.0);
    P_(8, 8) = DegToRad(10.0) * DegToRad(10.0);

    initialized_ = true;
}

void Eskf::Predict(const ImuSample& current_imu_sample) {
    if (!initialized_) {
        throw std::runtime_error("Eskf::Predict: filter is not initialized.");
    }

    // The stored nominal state is currently at the previous IMU sample time
    // t_{k-1}. The incoming IMU sample is the current sample at time t_k.
    const std::int64_t current_imu_utime = current_imu_sample.utime;
    const std::int64_t dt_previous_to_current_us =
        current_imu_utime - previous_imu_utime_;
    const double dt_previous_to_current_s =
        static_cast<double>(dt_previous_to_current_us) * 1e-6;

    // Integrate the measured angular velocity to advance the nominal attitude
    // from the previous IMU frame I_{k-1} to the current IMU frame I_k.
    //
    // Before this assignment:
    //
    //     q_GI = ^G q_{I_{k-1}}
    //
    // The incremental quaternion is:
    //
    //     delta_q_I_prev_I_curr = ^{I_{k-1}} q_{I_k}
    //
    // After composition:
    //
    //     q_GI = ^G q_{I_k}
    const Eigen::Vector3d omega_I_curr = current_imu_sample.rotation_rate;
    const Eigen::Vector3d delta_theta_I_prev_to_curr =
        omega_I_curr * dt_previous_to_current_s;
    const Eigen::Quaterniond delta_q_I_prev_I_curr =
        RotVecToQuat(delta_theta_I_prev_to_curr);
    nominal_state_.q_GI =
        (nominal_state_.q_GI * delta_q_I_prev_I_curr).normalized();

    // Use the inverse of the current IMU sample's q_AI to express gravity in
    // the current IMU frame I_k, then combine that with the measured specific
    // force to recover linear acceleration in I_k.
    const Eigen::Vector3d g_A(0.0, 0.0, -9.80665);
    const Eigen::Quaterniond q_I_curr_A =
        current_imu_sample.q_AI.normalized().conjugate();
    const Eigen::Vector3d g_I_curr = q_I_curr_A * g_A;
    const Eigen::Vector3d linear_acc_I_curr =
        g_I_curr + current_imu_sample.specific_force;

    // Rotate the current-frame acceleration into frame G, then propagate the
    // nominal position and velocity from t_{k-1} to t_k.
    const Eigen::Vector3d linear_acc_G_curr =
        nominal_state_.q_GI * linear_acc_I_curr;
    nominal_state_.p_G =
        nominal_state_.p_G +
        nominal_state_.v_G * dt_previous_to_current_s +
        0.5 * linear_acc_G_curr * dt_previous_to_current_s *
            dt_previous_to_current_s;
    nominal_state_.v_G =
        nominal_state_.v_G +
        linear_acc_G_curr * dt_previous_to_current_s;

    // Covariance update. F maps the previous error state at t_{k-1} to the
    // current error state at t_k. G maps the current IMU noise over the same
    // previous-to-current interval into the current error state.
    const auto skew_symmetric = [](const Eigen::Vector3d& v) {
        Eigen::Matrix3d skew;
        skew << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
        return skew;
    };

    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_GI_curr = nominal_state_.q_GI.toRotationMatrix();
    const Eigen::Matrix3d linear_acc_I_curr_cross =
        skew_symmetric(linear_acc_I_curr);
    const Eigen::Matrix3d omega_I_curr_cross =
        skew_symmetric(omega_I_curr);

    Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Zero();
    F.block<3, 3>(0, 0) = I3;
    F.block<3, 3>(0, 3) = I3 * dt_previous_to_current_s;
    F.block<3, 3>(0, 6) =
        -0.5 * R_GI_curr * linear_acc_I_curr_cross *
        dt_previous_to_current_s * dt_previous_to_current_s;
    F.block<3, 3>(3, 3) = I3;
    F.block<3, 3>(3, 6) =
        -R_GI_curr * linear_acc_I_curr_cross * dt_previous_to_current_s;
    F.block<3, 3>(6, 6) =
        I3 - omega_I_curr_cross * dt_previous_to_current_s;

    Eigen::Matrix<double, 9, 6> G = Eigen::Matrix<double, 9, 6>::Zero();
    G.block<3, 3>(0, 0) =
        0.5 * R_GI_curr * dt_previous_to_current_s *
        dt_previous_to_current_s;
    G.block<3, 3>(3, 0) = R_GI_curr * dt_previous_to_current_s;
    G.block<3, 3>(6, 3) = I3 * dt_previous_to_current_s;

    constexpr double kAccelNoiseStd = 0.5;
    constexpr double kGyroNoiseStd = 0.05;

    Eigen::Matrix<double, 6, 6> imu_noise_covariance =
        Eigen::Matrix<double, 6, 6>::Zero();
    imu_noise_covariance.block<3, 3>(0, 0) =
        kAccelNoiseStd * kAccelNoiseStd * I3;
    imu_noise_covariance.block<3, 3>(3, 3) =
        kGyroNoiseStd * kGyroNoiseStd * I3;

    const Eigen::Matrix<double, 9, 9> process_noise_covariance =
        G * imu_noise_covariance * G.transpose();

    P_ = F * P_ * F.transpose() + process_noise_covariance;
    SymmetrizeAndClampCovariance(P_);

    // The current IMU sample is now the previous IMU sample for the next
    // Predict call.
    previous_imu_utime_ = current_imu_utime;
}
