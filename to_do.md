corpus --> nuScenes dataset

WE CAN FIND APPROPRIATE PLACES FOR WHAT FOLLOWS, BUT ONLY FOR PARTS OF THE FOLLOWING THAT AREN'T IN THE DOCS ALREADY... AND ONLY AFTER MAJOR REWORING:

- `schema_nuscenes.md` says `ego_pose` localization is 2D in the `xy` plane and `ego_pose.translation[2]` is always `0`; `schema_nuimages.md` says ego-pose acceleration `z` is close to gravitational acceleration $g = 9.81\ \mathrm{m}/\mathrm{s}^2$. [@caesar2020nuscenes; @nuscenes-canbus-readme; @nuscenes-issue-108; @nuscenes-issue-56; @nuscenes-issue-699; @nuscenes-issue-1066; @nuscenes-issue-122; @nuscenes-schema; @nuimages-schema]

- The CAN-bus README does not state an order explicitly for `q` or `pose.orientation`, so `w, x, y, z` is inferred from the broader schema convention. The README describes pose `orientation` as a 4-element "rotation vector in the ego vehicle frame"; given the `ego_pose` identity statement, the broader schema convention, and the downloaded unit-norm data, treat it as a quaternion-like rotation field, not a literal 3D rotation vector. [@nuscenes-schema; @nuimages-schema; @nuscenes-canbus-readme; @nuscenes-canbus-local-2026-03-26]

- Nominal rates are `100 Hz` for `ms_imu` and `50 Hz` for `pose`. For the streams this app uses, per-scene counts stay in tight bands, and `len(ms_imu) / len(pose)` ranges from about `1.982` to `2.025` with median about `2.002`: `pose` min `780`, median `973`, mean `972.526`, max `996`; `ms_imu` min `1565`, median `1947`, mean `1947.859`, max `1994`. The local corpus does not live on a perfect integer-clock grid: median per-file sample intervals are `20008 us` for `pose` and `9993 us` for `ms_imu`. There is real timestamp jitter and occasional larger gaps: observed global $dt$ ranges are `3280 us` to `80617 us` for `pose` and `1006 us` to `50001 us` for `ms_imu`. [@nuscenes-canbus-readme; @nuscenes-canbus-local-2026-03-26]

Next. Let's reflect on timestamp facts for the pose/IMU file pairs. For a given scene, we...

1. use IMU data with timestamps before the first GPS timesamp (it is important to always note when discussing this case, the max delta t for all scenes between the first IMU samples and the first GPS sample).
2. do *not* use GPS data with timestamps before the first IMU timestamp
3. ignore GPS data with a timestampt

How many scenes have a first IMU sample with a timestamp less than the first GPS timestamp and what is the max delta t for all such scenes between the first IMU sample and the first GPS sample.

1. do *not* use GPS data with timestamps before the first IMU timestamp
2. ignore GPS data with a timestampt
