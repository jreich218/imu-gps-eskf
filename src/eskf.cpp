#include "eskf.hpp"

#include <Eigen/Geometry>
#include <cmath>

#include "scene_types.hpp"

namespace {

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
}  // namespace

void Eskf::Initialize(const StartupInitialization& startup_initialization) {
    nominal_state_.p_G = startup_initialization.p0_G;
    nominal_state_.v_G = startup_initialization.v0_G;
    nominal_state_.q_GI = startup_initialization.q0_GI.normalized();
    last_imu_utime_ = startup_initialization.last_imu_utime;

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

void Eskf::Predict(const ImuSample& imu_sample) {
    if (!initialized_) {
        throw std::runtime_error("Eskf::Predict: filter is not initialized.");
    }
    std::int64_t dt_us = imu_sample.utime - last_imu_utime_;
    const double dt_s = static_cast<double>(dt_us) * 1e-6;
    last_imu_utime_ = imu_sample.utime;

    // Integrate the measured angular velocity to advance the nominal attitude
    // `q_GI`.
    const Eigen::Vector3d omega_I = imu_sample.rotation_rate;
    const Eigen::Vector3d dtheta_I = omega_I * dt_s;
    Eigen::Quaterniond dq_IkIkp1 = RotVecToQuat(dtheta_I);
    nominal_state_.q_GI = (nominal_state_.q_GI * dq_IkIkp1).normalized();
    // Use the inverse of the IMU's `q_AI` to express gravity in frame `I`, then
    // combine that with the measured specific force to recover linear
    // acceleration in `I`.
    const Eigen::Vector3d g_A(0.0, 0.0, -9.80665);
    const Eigen::Quaterniond q_IA = imu_sample.q_AI.normalized().conjugate();
    const Eigen::Vector3d g_I = q_IA * g_A;
    const Eigen::Vector3d linear_acc_I = g_I + imu_sample.specific_force;
    // Rotate that acceleration into frame `G`, then propagate nominal position
    // `p_G` and velocity `v_G`.
    const Eigen::Vector3d linear_acc_G = nominal_state_.q_GI * linear_acc_I;
    nominal_state_.p_G = nominal_state_.p_G + nominal_state_.v_G * dt_s +
                         0.5 * linear_acc_G * dt_s * dt_s;
    nominal_state_.v_G = nominal_state_.v_G + linear_acc_G * dt_s;

    // clang-format off

    // 1. Build the linearized state-transition matrix `F` for the
    // position-velocity-attitude error state over this IMU step.

    // 2. Build the noise mapping `G` from accelerometer/gyro noise into the
    // 9D error state, and build the raw IMU noise covariance `Q_covariance`:

    /***

    G   = mapping from [accel_noise, gyro_noise] to [pos_error, vel_error, attitude_error]
    Q_covariance  = covariance of raw IMU noise

     */

    // 3. Propagate the covariance with `P_ = F * P_ * F.transpose() + G * Q_covariance * G.transpose()`.

    // Re-symmetrize the covariance and clamp tiny negative diagonal terms
    // caused by numerical roundoff.

    // clang-format on
}
