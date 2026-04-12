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
