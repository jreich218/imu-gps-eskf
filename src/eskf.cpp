#include "eskf.hpp"

#include <cmath>

#include "scene_types.hpp"

namespace {
constexpr double DegToRad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}
}  // namespace

void Eskf::Initialize(const StartupInitialization& startup_initialization) {
    x_.p_G = startup_initialization.p0_G;
    x_.v_G = startup_initialization.v0_G;
    x_.q_GI = startup_initialization.q0_GI.normalized();
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

    // 4. Use the IMU's `q_AI` to express gravity in frame `I`, then combine
    //    that with the measured specific force to recover linear
    //    acceleration in `I`.

    // 5. Rotate that acceleration into frame `G`, then propagate nominal
    //    position `p_G` and velocity `v_G`.

    // 6. Build the linearized state-transition matrix `F` for the
    //    position-velocity-attitude error state over this IMU step.

    // 7. Build the noise mapping `G` and process-noise covariance `Q` from
    //    the accelerometer and gyro noise assumptions.

    // 8. Propagate the covariance with `P_ = F * P_ * F.transpose() + Q`.

    // 9. Re-symmetrize the covariance and clamp tiny negative diagonal terms
    //    caused by numerical roundoff.

    (void)imu_sample;
}
