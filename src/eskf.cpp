#include "eskf.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace {

constexpr double kCovarianceDiagonalEpsilon = 1e-12;
constexpr double kCovarianceDiagonalNegativeTolerance = 1e-9;
constexpr double kSecPerUsec = 1e-6;
constexpr double kGravityMps2 = 9.8;

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
    // Use a temporary here to avoid Eigen aliasing on the transpose expression.
    const Eigen::Matrix<double, 9, 9> symmetrized =
        0.5 * (covariance_matrix + covariance_matrix.transpose());
    covariance_matrix = symmetrized;

    for (int i = 0; i < covariance_matrix.rows(); ++i) {
        const double covariance_diagonal = covariance_matrix(i, i);
        if (covariance_diagonal < 0.0) {
            if (covariance_diagonal >= -kCovarianceDiagonalNegativeTolerance) {
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

    // The stored nominal state is at t_{k-1}, and the incoming IMU sample
    // advances that state to t_k.
    const std::int64_t current_imu_utime = current_imu_sample.utime;
    const std::int64_t dt_us = current_imu_utime - previous_imu_utime_;
    const double dt_s = static_cast<double>(dt_us) * kSecPerUsec;
    const Eigen::Vector3d omega_I = current_imu_sample.rotation_rate;
    const Eigen::Vector3d dtheta_I = omega_I * dt_s;
    const Eigen::Quaterniond dq_Ikm1_Ik = RotVecToQuat(dtheta_I);
    nominal_state_.q_GI = (nominal_state_.q_GI * dq_Ikm1_Ik).normalized();

    // Use the inverse of the current IMU sample's q_AI to express gravity in
    // the current IMU frame I_k, then combine that with the measured specific
    // force to recover linear acceleration in I_k.
    const Eigen::Vector3d g_A(0.0, 0.0, -kGravityMps2);
    const Eigen::Quaterniond q_IA =
        current_imu_sample.q_AI.normalized().conjugate();
    const Eigen::Vector3d g_I = q_IA * g_A;
    const Eigen::Vector3d linear_acc_I =
        g_I + current_imu_sample.specific_force;

    // Rotate the current-frame acceleration into frame G, then propagate the
    // nominal position and velocity from t_{k-1} to t_k.
    const Eigen::Vector3d linear_acc_G = nominal_state_.q_GI * linear_acc_I;
    nominal_state_.p_G = nominal_state_.p_G + nominal_state_.v_G * dt_s +
                         0.5 * linear_acc_G * dt_s * dt_s;
    nominal_state_.v_G = nominal_state_.v_G + linear_acc_G * dt_s;

    const auto skew_symmetric = [](const Eigen::Vector3d& v) {
        Eigen::Matrix3d skew;
        skew << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
        return skew;
    };

    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_GI = nominal_state_.q_GI.toRotationMatrix();
    const Eigen::Matrix3d linear_acc_I_cross = skew_symmetric(linear_acc_I);
    const Eigen::Matrix3d omega_I_cross = skew_symmetric(omega_I);

    Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Zero();
    F.block<3, 3>(0, 0) = I3;
    F.block<3, 3>(0, 3) = I3 * dt_s;
    F.block<3, 3>(0, 6) = -0.5 * R_GI * linear_acc_I_cross * dt_s * dt_s;
    F.block<3, 3>(3, 3) = I3;
    F.block<3, 3>(3, 6) = -R_GI * linear_acc_I_cross * dt_s;
    F.block<3, 3>(6, 6) = I3 - omega_I_cross * dt_s;

    Eigen::Matrix<double, 9, 6> G = Eigen::Matrix<double, 9, 6>::Zero();
    G.block<3, 3>(0, 0) = 0.5 * R_GI * dt_s * dt_s;
    G.block<3, 3>(3, 0) = R_GI * dt_s;
    G.block<3, 3>(6, 3) = I3 * dt_s;

    constexpr double kAccelNoiseStd = 0.5;
    constexpr double kGyroNoiseStd = 0.05;

    Eigen::Matrix<double, 6, 6> imu_noise_covariance =
        Eigen::Matrix<double, 6, 6>::Zero();
    imu_noise_covariance.block<3, 3>(0, 0) =
        kAccelNoiseStd * kAccelNoiseStd * I3;
    imu_noise_covariance.block<3, 3>(3, 3) = kGyroNoiseStd * kGyroNoiseStd * I3;

    const Eigen::Matrix<double, 9, 9> process_noise_covariance =
        G * imu_noise_covariance * G.transpose();

    P_ = F * P_ * F.transpose() + process_noise_covariance;
    SymmetrizeAndClampCovariance(P_);

    // The current IMU sample is now the previous IMU sample for the next
    // Predict call.
    previous_imu_utime_ = current_imu_utime;
}
