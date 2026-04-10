#include "eskf.hpp"

#include "scene_types.hpp"

void Eskf::Initialize(const StartupInitialization& startup_initialization) {
    x_.p_G = startup_initialization.p0_G;
    x_.v_G = startup_initialization.v0_G;
    x_.q_GI = startup_initialization.q0_GI.normalized();
    last_imu_utime_ = startup_initialization.last_imu_utime;

    // set the initial covariance `P_`
    // TODO initial covariance

    initialized_ = true;
}
