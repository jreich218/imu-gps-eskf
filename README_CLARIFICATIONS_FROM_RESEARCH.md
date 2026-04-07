The following is based on a close read of [nuscenes_paper_1903.11027.pdf](/home/dfw/dev/imu-gps-eskf/reading/nuscenes_references/nuscenes_paper_1903.11027.pdf), the high-signal devkit files that actually govern these questions, the CAN-bus tutorial/schema/API/code, all six cited GitHub issues, and a repo-wide keyword sweep of the rest of [nuscenes-devkit](/home/dfw/dev/imu-gps-eskf/reading/nuscenes_references/nuscenes-devkit).

- `scene-XXXX_ms_imu.json.q` is the raw IMU attitude output. `ego_pose.rotation` and `scene-XXXX_pose.orientation` are localization outputs, not raw IMU. That is stated directly in [can_bus README](/home/dfw/dev/imu-gps-eskf/reading/nuscenes_references/nuscenes-devkit/python-sdk/nuscenes/can_bus/README.md), [schema_nuscenes.md#L54](/home/dfw/dev/imu-gps-eskf/reading/nuscenes_references/nuscenes-devkit/docs/schema_nuscenes.md#L54), the [paper](/home/dfw/dev/imu-gps-eskf/reading/nuscenes_references/nuscenes_paper_1903.11027.pdf), and [issue #1111](https://github.com/nutonomy/nuscenes-devkit/issues/1111).
- `scene-XXXX_pose.json.pos` is not some second-rate estimate. It is the higher-rate localization pose stream; the CAN-bus README says it is identical to `ego_pose` position but sampled higher rate. The paper says that localization is lidar-map-based Monte Carlo localization with odometry, not raw GPS/IMU localization, and reports `<= 10 cm` error.
- The `q` line is factual, not merely a project-side “interpretation.” The CAN-bus README literally says `q` transforms IMU coordinates into a fixed reference frame whose yaw is arbitrary but whose `xy` plane is perpendicular to gravity and whose `z` points up. So your instinct there was right.
- `ms_imu.linear_accel` and `ms_imu.rotation_rate` are in the IMU frame. `pose.vel`, `pose.accel`, and `pose.rotation_rate` are in the ego/body frame. `pose.pos` is in the global frame. That split is explicit in [can_bus README](/home/dfw/dev/imu-gps-eskf/reading/nuscenes_references/nuscenes-devkit/python-sdk/nuscenes/can_bus/README.md) and reinforced by [issue #1066](https://github.com/nutonomy/nuscenes-devkit/issues/1066).
- Treating ego/body and IMU as the same FLU frame for this project is well supported. The maintainers say the reference point is roughly the IMU at the center of the rear axle, and the axes are `x` forward, `y` left, `z` up for both. See [issue #108](https://github.com/nutonomy/nuscenes-devkit/issues/108), [issue #699](https://github.com/nutonomy/nuscenes-devkit/issues/699), and [issue #1066](https://github.com/nutonomy/nuscenes-devkit/issues/1066).
- “2D localization” does not mean the pose quaternion is necessarily pure yaw. [schema_nuscenes.md#L54](/home/dfw/dev/imu-gps-eskf/reading/nuscenes_references/nuscenes-devkit/docs/schema_nuscenes.md#L54) says localization is 2D in `x-y` and `z` is always `0`, but [issue #122](https://github.com/nutonomy/nuscenes-devkit/issues/122) makes clear that ego/IMU `z` is not exactly global vertical because road slope can tilt the local vehicle frame. So translation is planar; attitude can still reflect slope.

The following is based on an empirical sweep of the local CAN-bus corpus in `/data/sets/nuscenes/can_bus` using `/home/dfw/dev/python_laboratory/.venv/bin/python`.

- `scene-XXXX_pose.json.orientation` behaves like a real `w, x, y, z` quaternion, not a rotation vector.
  - Across all `952,103` pose records, `orientation` always has length `4`.
  - Its norm is unit to machine precision: min `0.9999999999999999`, max `1.0`.
  - `x` and `y` are exactly `0.0` for every pose record, so every pose quaternion is of the form `[w, 0, 0, z]`.
  - The decisive check was behavioral: rotate ego-frame `pose.vel` by `orientation`, then compare the resulting horizontal velocity to the finite-difference velocity from adjacent `pose.pos` samples.
  - Using `orientation` directly gave median direction error `0.397 deg` and median speed error `0.136 m/s`.
  - Using the inverse quaternion instead gave median direction error `80.61 deg` and median speed error `6.96 m/s`.
  - So `orientation` is not just a four-number placeholder. It is acting like a quaternion that maps ego-frame motion into the global frame.

- `scene-XXXX_ms_imu.json.q` behaves like the IMU leveling quaternion described in the nuScenes material.
  - Across all `1,906,954` IMU records, `q` always has length `4`.
  - Its norm is unit to tight tolerance: min `0.9999999276`, max `1.0000001357`.
  - It is not pure yaw: `|x|` reaches `0.07116` and `|y|` reaches `0.07012`, so it carries real tilt information.
  - Raw `linear_accel` already has a strong `+g` bias: raw `z` mean `9.784` and raw vector norm mean `9.823`.
  - On a near-stationary subset of `346,395` IMU records, chosen by matching each IMU sample to the nearest pose sample and requiring `|pose.vel[0]| < 0.5 m/s` and `||pose.rotation_rate|| < 0.05 rad/s`, the gravity-aligned behavior is clear.
  - Raw `linear_accel.z` has median `9.7896`.
  - Rotating `linear_accel` by `q` gives median `z = 9.7956` and median horizontal norm `0.0392`.
  - Rotating by the inverse instead gives median `z = 9.7859` and median horizontal norm `0.3603`.
  - So `q` is doing the expected IMU-to-gravity-aligned mapping with arbitrary yaw.

- `scene-XXXX_ms_imu.json.linear_accel` is specific force, not gravity-removed acceleration.
  - If it were already gravity-removed, the raw norm and the raw `z` component would not cluster near `9.8` during quiet motion.
  - Instead, both the all-sample statistics and the near-stationary subset show the opposite: the signal naturally sits near `+9.8` after leveling.
  - For this project that means `linear_accel` is not yet the acceleration term the filter ultimately wants. Gravity handling still has to be done by the filter logic.

To reproduce those conclusions from the local corpus:

1. Use `/home/dfw/dev/python_laboratory/.venv/bin/python`.
2. Load every `scene-????_pose.json` and `scene-????_ms_imu.json` under `/data/sets/nuscenes/can_bus`.
3. Implement quaternion vector rotation for `w, x, y, z` quaternions with:

```python
def quat_rotate_wxyz(q, v):
    w, x, y, z = q
    qv = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)
```

4. For the pose check, compute finite-difference horizontal velocity from adjacent `pose.pos` and `pose.utime`, rotate `pose.vel` by `orientation`, and compare the two. Also compare against the inverse quaternion as a control.
5. For the IMU check, rotate `linear_accel` by `q` and by the inverse quaternion, then compare which one produces a quieter horizontal component and a `z` component near `+9.8`.
6. For the near-stationary subset, match each IMU sample to the nearest pose sample in time and use `|pose.vel[0]| < 0.5` and `||pose.rotation_rate|| < 0.05` as the stationary filter.
