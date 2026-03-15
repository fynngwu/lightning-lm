# Lightning-LM

## 安装依赖

### 1. 基础依赖

在项目根目录执行：

```bash
./scripts/install_dep.sh
```

### 2. Livox 驱动（可选）

如果需要使用 Livox 激光雷达且尚未安装 `livox_driver2`，可通过以下仓库一键安装：

```bash
git clone https://github.com/fynngwu/livox-ros2-setup.git
cd livox-ros2-setup
./install_livox_driver.sh
```

安装完成后，需要根据实际情况修改 Livox 配置中的雷达 IP 和连接网络 IP，具体配置文件位于 Livox 驱动目录下。

## 编译

在 ROS2 工作空间根目录执行：

```bash
colcon build --packages-select lightning --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## 运行

### 1. 离线建图

先用 `run_slam_offline` 建图，并确保打开地图保存。

```bash
ros2 run lightning run_slam_offline \
  --config /home/wufy/slam_ws/src/lightning-lm/config/default_livox.yaml \
  --input_bag /path/to/your.bag
```

建议配置：

```yaml
system:
  map_path: ./data/new_map/
```

离线建图完成后，地图会保存到 `system.map_path` 对应目录。后续在线初始化直接复用这个目录。

### 2. 在线运行

```bash
ros2 run lightning run_slam_online --config /home/wufy/slam_ws/src/lightning-lm/config/default_livox.yaml
```

然后播放 bag：

```bash
ros2 bag play /path/to/your_bag
```

如果使用仓库里的测试包：

```bash
ros2 bag play ./court1-ros2bag
```

## YAML 常用参数

在线初始化和可视化相关的常用参数如下：

```yaml
fasterlio:
  enable_map_incremental: true

system:
  map_path: ./data/new_map/
  enable_online_init: true
  online_init_map_path: ./data/new_map/
  pub_tf: true
  pub_scan: true
  pub_map: true
  pub_map_voxel_leaf_size: 0.3
```

- `fasterlio.enable_map_incremental`: 是否持续增量更新 ivox 局部地图。关闭后不会继续往 ivox 里加新点。
- `system.map_path`: 离线建图输出目录。
- `system.enable_online_init`: 是否启用 `/initialpose` 在线初始化。
- `system.online_init_map_path`: 在线初始化加载的地图目录，通常就是离线阶段保存出来的 `map_path`。
- `system.pub_tf`: 是否发布 `map -> lidar` TF。
- `system.pub_scan`: 是否发布 `/lightning/current_scan`。
- `system.pub_map`: 是否发布 `/lightning/global_map`。
- `system.pub_map_voxel_leaf_size`: 发布全局地图时的体素降采样大小。

## 在线初始化测试

推荐测试顺序：

1. 先跑一次离线建图，确认地图成功保存到 `system.map_path`。
2. 修改配置，打开 `enable_online_init`，并把 `online_init_map_path` 指向离线地图目录。
3. 启动在线程序：

```bash
ros2 run lightning run_slam_online --config /home/wufy/slam_ws/src/lightning-lm/config/default_livox.yaml
```

4. 播放 bag：

```bash
ros2 bag play ./court1-ros2bag
```

5. 在 RViz 中发送 `/initialpose`，或让程序在第一帧自动触发零位姿初始化。

## 手动触发 ivox 可视化

当前程序会在 `ApplyOnlineInitResult` 后自动发布一次 `/lightning/ivox_map`。

如果想在运行过程中手动提取当前 LIO 的 ivox 全量点，可以发送：

```bash
ros2 topic pub --once /lightning/publish_ivox_map std_msgs/msg/Bool "{data: true}"
```

## 相关话题

- TF: `map -> lidar`
- TF: `map -> online_init_request`
- TF: `map -> online_init_result`
- TF: `map -> online_init_output_result`
- `/lightning/current_scan`
- `/lightning/global_map`
- `/lightning/ivox_map`

## 当前限制

当前已经支持：

- 直接加载离线地图
- 使用 `/initialpose` 输入初始化位姿
- 关闭增量建图后不再向 ivox 新增点云

当前已知问题：

- `initialpose` 触发的位姿估计仍然不够准确
- 在线初始化效果还需要继续提高，当前结果不能认为已经足够稳定
