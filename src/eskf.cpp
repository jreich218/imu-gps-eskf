#include "eskf.hpp"

void Eskf::Initialize(const StartupInitialization& startup_initialization) {
    // 1. copy `p0_G`, `v0_G`, and `q0_GI` into the nominal state
    // 2. set the initial covariance `P_`
    // 3. set the filter time from `startup_initialization.last_imu_utime`
    // 4. mark the filter initialized
}
