# ros2_grandtour_publishers

ROS 2 replay node for a downloaded [ETH GrandTour](https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset)
mission (as fetched by `tools/download_grandtour.py`). GrandTour ships each
mission as independent per-stream **zarr v2** groups, not a rosbag, so this
node opens the streams directly and republishes them as standard ROS 2
messages and TF.

## Data model (reverse-engineered from the downloaded SPX-2 mission)

- Every stream lives at `<mission_dir>/data/<name>/<name>` as a zarr v2
  directory store (`blosc`/`lz4` compressed), with a `timestamp` array
  (float64 Unix epoch seconds, one per sample) plus per-sample fields.
- **Point clouds** (`hesai_points_undistorted`, ...) store a fixed-size
  padded buffer per scan (e.g. `points: (num_scans, 69000, 3)`) plus a
  `valid: (num_scans, 1)` field giving the actual point count for that scan;
  entries beyond `valid` are zero-filled. Confirmed empirically against the
  downloaded data (`tools/` was used to open the arrays with `zarr` and
  check that `points[i, :valid[i]]` are the only non-zero rows). The
  per-point `time` field is an **absolute** epoch timestamp (not a
  scan-relative offset like most lidar drivers use). The published
  `PointCloud2` carries it both ways: `t` (float32, scan-relative offset -
  the usual lidar-driver convention for generic PCL/rviz consumers) and
  `timestamp` (float64, absolute epoch seconds, unmodified - float32 can't
  hold epoch time with useful precision). The latter is what
  `scan_deskewer` (dotX-Automation/scan_deskewer) expects: it fuses point
  times directly against an absolute-epoch IMU time buffer, with no
  dtype or relative/absolute check of its own.
- **IMU-shaped streams** (`stim320_imu`, `cpt7_imu`, `anymal_imu`) carry
  `orien`/`orien_cov` (3x3), `ang_vel`/`ang_vel_cov` (3x3),
  `lin_acc`/`lin_acc_cov` (3x3) — a 1:1 match for `sensor_msgs/Imu`. When
  `orien` is the all-zero placeholder (observed on `stim320_imu`), this node
  follows REP-145 and publishes identity orientation with
  `orientation_covariance[0] = -1` instead of a bogus quaternion.
- **Odometry-shaped streams** (`anymal_state_odometry`,
  `anymal_state_state_estimator`, `cpt7_ie_tc_odometry`) carry
  `pose_pos`/`pose_orien`/`pose_cov` and `twist_lin`/`twist_ang`/`twist_cov`.
  `anymal_state_state_estimator` does **not** carry `pose_cov`/`twist_cov`
  (verified against the actual downloaded arrays); this node leaves the
  corresponding `nav_msgs/Odometry` covariance at its all-zero default in
  that case, the same "unknown" convention already used in
  `ros2_openloris_publishers/odometry_tf_node.py`.
- **Leica prism positions** (`prism_position`) carry a timestamped `point`
  in the stationary MS60 total-station frame. They are published unchanged
  as `geometry_msgs/PointStamped` on `/grandtour/gt/prism_position`.
- `anymal_state_state_estimator` additionally carries `joint_positions`,
  `joint_velocities`, `joint_efforts` `(num_samples, 12)`, published as
  `sensor_msgs/JointState`. Joint names and order are taken verbatim from
  that stream's own zarr `.zattrs` description field.
- **TF** is split in two:
  - a fully **static** extrinsics tree in `metadata/tf.yaml` (identical to
    the `tf` zarr group's `.zattrs`), broadcast once via
    `tf2_ros.StaticTransformBroadcaster`;
  - a **dynamic** stream, `cpt7_ie_tc_tf` (`translation`/`rotation` arrays),
    which is the tightly-coupled GNSS/INS ground-truth trajectory, broadcast
    per-sample via `tf2_ros.TransformBroadcaster`. **Disabled by default**
    (`ground_truth_tf.enable: false`): `cpt7_imu` already has a static
    parent (`box_base`, via `metadata/tf.yaml`), so also broadcasting
    `enu_origin -> cpt7_imu` here would give `cpt7_imu` two TF parents
    (`TF_MULTIPLE_PARENT`). Consume ground truth as the plain
    `/grandtour/gt/odometry` topic instead; only enable this stream if you
    also rewire the static tree so the frame keeps a single parent.
  - `/grandtour/gt/odometry`'s frame (`enu_origin`, a georeferenced ENU
    frame) is otherwise completely disconnected from the SLAM/robot tree
    (`odom`/`base`/...) - a consumer like an RViz `Odometry` display can
    never transform it into a fixed frame, and just silently drops every
    message ("discarding message because the queue is full"). To fix this,
    `ground_truth_alignment` (on by default) broadcasts **one** additional
    static transform, `odom -> enu_origin`, computed at startup: it matches
    `anymal_state_odometry` and `cpt7_ie_tc_odometry` at their closest
    common sample and composes through the known static
    `base -> box_base -> cpt7_imu` chain. This is a best-effort
    visualization aid (accuracy bounded by the two streams' sample-rate
    mismatch at that instant), not a calibrated registration - don't use it
    for quantitative trajectory error metrics; use `evo`/TUM alignment
    (`evo_ape --align`) against the raw ground-truth pose for that instead.
  - `prism_position` lives in `leica_total_station`, another disconnected
    reference frame. With `prism_alignment.enable` (default), the publisher
    timestamp-matches the Leica points to the NovAtel IE-TC positions and
    fits a no-scale rigid transform `enu_origin -> leica_total_station` for
    RViz. ETH-1 uses all 4,547 prism samples and the fit is approximately
    0.28 m RMSE. The residual includes the unpublished, rotating prism/body
    lever arm, so this transform is a visualization aid rather than a
    calibrated extrinsic. The published PointStamped values and frame remain
    raw; disable `prism_alignment` when inspecting the native Leica frame.
- Per-foot contact/wrench/friction data in `anymal_state_state_estimator`
  and camera images (jpeg/png) are present in the dataset but **not**
  handled by this node yet (the download script doesn't fetch images at
  all). Both are straightforward to add following the same stream-registry
  pattern in `grandtour_publisher_node.py` if needed.

### Unverified assumptions (flag before trusting downstream results)

- **Quaternion component order.** All 4-element orientation arrays
  (`pose_orien`, `orien`, `rotation`) are assumed to be `(x, y, z, w)`,
  matching `geometry_msgs/Quaternion`'s field order. This matches the
  magnitude pattern observed in the actual arrays (exactly one dominant,
  near-unit component consistent with a normalized quaternion) but was
  **not** cross-checked against the GrandTour paper or dataset schema code.
  If a consumer shows an unexpected ~90 deg/180 deg frame twist, check this
  first.
- **6x6 covariance layout.** `pose_cov`/`twist_cov` are flattened row-major
  and assigned directly to `nav_msgs/Odometry`'s `float64[36]` covariance,
  assuming the same `[x, y, z, rot_x, rot_y, rot_z]` ordering ROS uses.
  Reasonable given the streams are described as ROS-derived solutions, but
  not independently confirmed.
- **`cpt7_ie_tc_tf` parent frame.** The zarr `.zattrs` only documents the
  frame_id of the *child* (`cpt7_imu`); the parent used here,
  `enu_origin`, is inferred from the sibling `cpt7_ie_tc_odometry` stream's
  `frame_id`. Configurable via `ground_truth_tf.parent_frame_id`.

## Requirements

`zarr` and `numcodecs` have no rosdep key; install them with pip (developed
and tested against `zarr==3.2.1`):

```bash
pip install zarr numcodecs
```

## Performance note

Zarr chunks the large arrays (e.g. lidar `points`) along the scan axis
(chunk size 256 scans in the SPX-2 mission). A cold read of one row
decompresses its whole chunk (~130-190 ms measured on this machine); this
node caches the last decompressed chunk per field, so sequential playback
amortizes to well under 1 ms/row. At `playback_rate` close to 1.0 the
resulting ~150 ms hiccup every 256 scans (~every 25 s at 10 Hz) is
negligible; at large speed-ups it becomes a visible burst/catch-up pattern
in the publish rate. This is expected and not a bug.

## Usage

```bash
colcon build --packages-select ros2_grandtour_publishers
source install/setup.bash
ros2 launch ros2_grandtour_publishers grandtour_publishers.launch.py \
  mission_dir:=/home/neo/workspace/logs/grandtour/SPX-2
```

Or run the node directly with parameter overrides:

```bash
ros2 run ros2_grandtour_publishers grandtour_publisher --ros-args \
  -p mission_dir:=/home/neo/workspace/logs/grandtour/SPX-2 \
  -p playback_rate:=2.0 \
  -p loop:=true
```

Published topics (defaults, see `config/grandtour_publishers.yaml`):

| Topic                                | Type                    | Source stream                    |
|---------------------------------------|-------------------------|-----------------------------------|
| `/grandtour/lidar/points`             | `sensor_msgs/PointCloud2` | `hesai_points_undistorted`       |
| `/grandtour/imu`                      | `sensor_msgs/Imu`        | `cpt7_imu` / `stim320_imu` / ...  |
| `/grandtour/anymal/odometry`          | `nav_msgs/Odometry`      | `anymal_state_odometry`          |
| `/grandtour/anymal/state_odometry`    | `nav_msgs/Odometry`      | `anymal_state_state_estimator`   |
| `/grandtour/anymal/joint_states`      | `sensor_msgs/JointState` | `anymal_state_state_estimator`   |
| `/grandtour/gt/odometry`              | `nav_msgs/Odometry`      | `cpt7_ie_tc_odometry`            |
| `/grandtour/gt/prism_position`        | `geometry_msgs/PointStamped` | `prism_position`             |
| `/tf`                                 | dynamic ground truth     | `cpt7_ie_tc_tf`                  |
| `/tf_static`                          | full extrinsics tree     | `metadata/tf.yaml`                |
| `/clock`                              | `rosgraph_msgs/Clock`    | merged timeline                  |

Every stream is independently enable/disable-able and its candidate names,
topic, and (for TF-publishing streams) parent/child frame are configurable —
see `config/grandtour_publishers.yaml`. Candidate fallback chains mirror
`tools/download_grandtour.py`, since availability and naming vary per
mission. The complete `hesai_points_undistorted` stream is preferred:
ETH-1's smaller `*_filtered` archive contains only `points` and `valid` (no
timestamps or per-point attributes), so it cannot be replayed independently.
ETH-1 uses `cpt7_imu`, while SPX-2 falls back to `stim320_imu`.

Set `use_sim_time: true` on any downstream node consuming this replay, and
launch with `publish_clock: true` (the default).

For visualization, `anymal_odometry.normalize_position` is enabled in the
default config. The publisher subtracts ANYmal's XYZ at the first lidar scan
from `/grandtour/anymal/odometry` and its `odom -> base` TF, while preserving
the measured orientation. The ground-truth alignment uses the same translated
frame, so SLAM, odometry, and transformed GT remain co-located near the RViz
origin. `tools/grandtour_ground_truth_tum.py` applies the same translation by
default; EVO with SE(3) alignment (`evo_ape ... -a`) is invariant to it.

The `slam_evaluator` GrandTour RViz configuration displays the NovAtel IE-TC
odometry in green and the Leica MS60 prism history in magenta. Launch it with
`rviz:=true`, for example:

```bash
ros2 launch slam_evaluator lio_sam_grandtour.launch.py \
  sequence:=ETH-1 rviz:=true
```
