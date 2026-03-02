# SLAM Online ROS2 TF and PointCloud2 Publish Design

## Background

`run_slam_online` currently runs front-end SLAM and optional UI, but it does not publish standard ROS2 TF and point cloud topics for downstream visualization and integration.

The localization path already publishes TF (`map -> base_link`) in online mode. For SLAM online, we need equivalent standard outputs based on real-time updated state and scan, with frame convention confirmed as `map -> lidar`.

## Goals

- Publish real-time TF in online SLAM: `map -> lidar`.
- Publish current scan as standard `sensor_msgs/msg/PointCloud2`.
- Publish map as standard `sensor_msgs/msg/PointCloud2`.
- Use an async publishing thread similar to UI threading style, so publishing and visualization do not block lidar/imu listeners.

## Non-Goals

- No change to LIO math or optimization logic.
- No queueing/replay guarantee for every scan frame.
- No new external node graph complexity unless required by future scaling.

## Chosen Approach

Use **Approach A**: keep current `SlamSystem` node and add a dedicated publisher thread.

Why this approach:

- Isolates publish cost from callback execution path.
- Matches current project style (UI and other async workers use dedicated threads).
- Minimizes architectural churn compared to introducing a new ROS node + executor.

## Architecture Changes

### 1) `SlamSystem` publishing bridge

In `core/system/slam.h` and `core/system/slam.cc`, add online publish members:

- `std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_`
- `rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_`
- `rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_`
- `std::thread publish_thread_`
- `std::atomic_bool publish_exit_`
- publish-side shared snapshot fields (`latest_state_`, `latest_scan_`, map dirty flags)
- mutex for snapshot synchronization

### 2) Thread-safe read access in `LaserMapping`

Add minimal locking for data read/write consistency:

- protect state updates and reads (`GetState`)
- protect scan updates and reads (`GetScanUndist`)
- protect keyframe list reads used by map generation (`GetGlobalMap` / `GetAllKeyframes`)

The intent is correctness first, with short lock scopes and heavy work outside shared locks where possible.

## Data Flow

### Main SLAM processing path (existing callbacks)

After successful `lio_->Run()`:

1. get latest state (`NavState`) and undistorted scan
2. deep-copy into publish snapshots (`latest_state_`, `latest_scan_`)
3. if keyframe count changed, set `map_dirty_ = true`

No map construction is done inside lidar callback.

### Publish thread path (new)

Loop at configured `pub_rate_hz`:

1. publish TF from latest state (`map -> lidar`)
2. publish current scan point cloud (`frame_id = lidar`)
3. if `map_dirty_` is set and keyframe-gap condition met, build global map and publish (`frame_id = map`)

Map generation runs in this thread, not in subscription callback.

## Message and Frame Conventions

- TF: parent `map`, child `lidar`
- `/lightning/current_scan`: `sensor_msgs/msg/PointCloud2`, `header.frame_id = lidar`
- `/lightning/global_map`: `sensor_msgs/msg/PointCloud2`, `header.frame_id = map`
- Timestamps use latest state timestamp for TF/scan consistency.

## Queue vs Latest-Only Decision

`latest_scan_` remains **latest-only single-slot** (no queue).

Rationale:

- target is real-time visualization/integration, not lossless replay
- bounded latency and memory
- simpler lock model and fewer deadlock hazards

If strict frame retention is needed in future, add a bounded queue (size 2-3) behind a clear requirement.

## Threading and Deadlock Prevention

- fixed lock order for publish snapshots
- do not hold mutex while running heavy map operations
- publish thread catches and logs exceptions, continues running if safe
- clean shutdown order in destructor:
  1) set `publish_exit_`
  2) join `publish_thread_`
  3) tear down publishers/broadcaster

## Configuration Additions

Under `system` yaml:

- `pub_tf` (already exists)
- `pub_scan` (new)
- `pub_map` (new)
- `pub_rate_hz` (new)
- `map_pub_kf_gap` (new)

Default values should preserve current behavior if disabled.

## Validation Plan

1. Build check: `colcon build --packages-select lightning`
2. Runtime topics:
   - `ros2 topic echo /tf`
   - `ros2 topic hz /lightning/current_scan`
   - `ros2 topic hz /lightning/global_map`
3. RViz2 check:
   - fixed frame `map`
   - verify TF and clouds align visually
4. Stress check:
   - high-rate data + keyframe growth
   - confirm callback latency remains stable and no deadlocks

## Risks and Mitigations

- **Risk:** map generation cost spikes CPU.
  - **Mitigation:** trigger by keyframe gap and only when dirty.
- **Risk:** shared data races between processing and publishing.
  - **Mitigation:** explicit mutex ownership and deep-copy snapshots.
- **Risk:** frame convention mismatch in downstream stack.
  - **Mitigation:** fixed and documented `map -> lidar` contract.
