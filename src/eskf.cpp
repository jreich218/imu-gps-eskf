#include "eskf.hpp"

#include <cmath>

#include "scene_types.hpp"

void Eskf::Initialize(const StartupInitialization& startup_initialization) {
    x_.p_G = startup_initialization.p0_G;
    x_.v_G = startup_initialization.v0_G;
    x_.q_GI = startup_initialization.q0_GI.normalized();
    last_imu_utime_ = startup_initialization.last_imu_utime;

    // Old `src/libeskf/Eskf.cpp` uses the same `kPi` value for its angle
    // variances. Keep that here so the benchmarked angle numbers match.
    constexpr double kPi = 3.14159265358979323846;
    // Old `src/libeskf/Eskf.cpp` writes `/ 180.0` inline in each attitude
    // variance. Pulling the conversion out keeps the three lines easier to
    // read without changing the values.
    constexpr double kDegToRad = kPi / 180.0;

    // Old `src/libeskf/Eskf.cpp` starts by zeroing `P_` and then writing a
    // diagonal prior. Keep that same diagonal-only structure here.
    P_.setZero();

    // Old `src/libeskf/Eskf.cpp` uses `25.0` here. I changed it to `4.0`
    // because the startup path fit already gives a much better x position than
    // a loose `5 m` sigma prior suggests.
    P_(0, 0) = 4.0;
    // Old `src/libeskf/Eskf.cpp` uses `25.0` here too. I changed it to `4.0`
    // for the same reason in y: trust the startup handoff more than the old
    // isotropic position prior did.
    P_(1, 1) = 4.0;
    // Old `src/libeskf/Eskf.cpp` uses `25.0` here. I changed it to `0.01`
    // because this project's supported inputs are planar, so z should start
    // tight instead of pretending we have `5 m` vertical uncertainty.
    P_(2, 2) = 0.01;

    // Old `src/libeskf/Eskf.cpp` uses `4.0` here already. I kept it because
    // the benchmark improved without needing a different x-velocity scale.
    P_(3, 3) = 4.0;
    // Old `src/libeskf/Eskf.cpp` uses `4.0` here already. I kept it for the
    // same reason in y velocity.
    P_(4, 4) = 4.0;
    // Old `src/libeskf/Eskf.cpp` uses `4.0` here. I changed it to `0.01`
    // because the supported runs stay on the plane, so z velocity should not
    // start with a `2 m/s` sigma.
    P_(5, 5) = 0.01;

    // Old `src/libeskf/Eskf.cpp` uses `(10 deg)^2` here. I changed it to
    // `(2 deg)^2` because roll comes mostly from the IMU gravity alignment,
    // and the split prior tested better with a tighter roll start.
    P_(6, 6) = std::pow(2.0 * kDegToRad, 2);
    // Old `src/libeskf/Eskf.cpp` uses `(10 deg)^2` here too. I changed it to
    // `(2 deg)^2` for the same reason as roll: pitch also comes mostly from
    // the IMU gravity alignment.
    P_(7, 7) = std::pow(2.0 * kDegToRad, 2);
    // Old `src/libeskf/Eskf.cpp` uses `(10 deg)^2` here. I kept it because yaw
    // is still the weaker part of startup, and the sweep did not justify
    // tightening yaw the way roll and pitch were tightened.
    P_(8, 8) = std::pow(10.0 * kDegToRad, 2);

    /*
    Old-version sweep summary for this split initial `P_` versus old `main`:

    - data: all `979` scene pairs in `/data/sets/nuscenes/can_bus`
    - workers: `32`
    - completed: `973 / 979` on both variants
    - failed scenes on both variants:
      `scene-0374`, `scene-0493`, `scene-0780`, `scene-0811`,
      `scene-0875`, `scene-1082`
    - raw GPS RMSE matched exactly between variants

    Old `main`:
    - median ESKF RMSE: `0.977226 m`
    - mean ESKF RMSE: `1.016013 m`
    - median early ESKF RMSE over the first `10` GPS updates: `1.698192 m`
    - mean early ESKF RMSE over the first `10` GPS updates: `1.670807 m`

    Old version with this split initial `P_`:
    - median ESKF RMSE: `0.974355 m`
    - mean ESKF RMSE: `1.000399 m`
    - median early ESKF RMSE over the first `10` GPS updates: `1.673759 m`
    - mean early ESKF RMSE over the first `10` GPS updates: `1.624530 m`

    Net effect:
    - overall mean ESKF RMSE improved by `0.015614 m` (`1.54%`)
    - early mean ESKF RMSE improved by `0.046277 m` (`2.77%`)
    - this split prior beat old `main` on `562` of the `973` completed scenes
    - both variants still lose to raw GPS on `scene-0703`
    */

    initialized_ = true;
}
