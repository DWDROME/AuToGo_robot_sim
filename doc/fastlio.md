---

# Fast-LIO 仿真说明

本说明介绍如何在 **Gazebo 仿真环境**下运行 **Fast-LIO**，并使用 **Livox Mid-360** 模拟点云进行建图。

这是本项目的**核心目标**：为 **Fast-LIO** 提供可复现的仿真环境。

---

## 1. 安装教程

建议在 **单独的工作空间** 中安装 Fast-LIO，以避免污染仿真环境。

### Step 1. 安装 Livox SDK

```bash
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

---

### Step 2. 安装 livox\_ros\_driver2

```bash
cd ~/fastlio_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ..
catkin_make
```

---

### Step 3. 安装 Fast-LIO

```bash
cd ~/fastlio_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd ..
catkin_make
```

⚠️ Mid-360 默认依赖 **livox\_ros\_driver1**，需要适配为 **livox\_ros\_driver2**。

---

### Step 4. 适配 livox\_ros\_driver2

在 Fast-LIO 源码中，将所有 `livox_ros_driver` 替换为 `livox_ros_driver2`：

```bash
cd ~/fastlio_ws/src/FAST_LIO
grep -rl "livox_ros_driver" ./ | xargs sed -i 's/livox_ros_driver/livox_ros_driver2/g'
```

然后重新编译：

```bash
cd ~/fastlio_ws
catkin_make
```

---

## 2. 启动流程

### Step 1. 启动 Gazebo 仿真环境

选择任意带传感器的世界：

```bash
# Playpen 场景（推荐）
roslaunch scout_gazebo_sim scout_mini_playpen.launch

# 其他可选
# roslaunch scout_gazebo_sim scout_mini_empty_world.launch
# roslaunch scout_gazebo_sim scout_mini_house.launch
```

### Step 2. 启动 Fast-LIO

```bash
roslaunch fast_lio mapping_mid360.launch
```

此时 Gazebo 中的 Mid-360 插件会自动启动并发布点云与 IMU 数据，无需额外手动启动。

---

## 3. 配置要点

1. **点云话题**

   * `/livox/lidar` → Fast-LIO 默认订阅点云
   * 确保 Mid-360 插件输出格式符合 livox\_ros\_driver2

2. **IMU 话题**

   * 本项目默认输出：`/imu`
   * 若 Fast-LIO 配置文件中仍写 `/livox/imu`，需修改为 `/imu`

3. **TF 配置**

   * `lidar_link` ↔ `base_link` 转换必须存在
   * 可使用 `ros2 run tf view_frames` 或 `rqt_tf_tree` 验证

4. **时间同步**

   * 仿真下建议关闭：`time_sync_en := false`

---

## 4. 典型策略

### 4.1 单独运行 Fast-LIO

* 最简方案：Gazebo 环境 + Fast-LIO
* 用于验证点云里程计与建图功能

### 4.2 与 EKF/Navigation 并行（实验性）

* Fast-LIO 输出 `/odom`，可作为导航输入
* 建议避免与 GMapping 同时启用，否则存在冲突

### 4.3 rosbag 离线回放

```bash
rosbag record /livox/lidar /imu
```

可保存仿真点云与 IMU，用于离线测试与参数调优。

---

## 5. 常见问题

1. **点云无数据**

   * 检查 `scout_mini_xxx.launch` 是否包含 Mid-360 插件
   * `rostopic echo /livox/lidar` 验证

2. **IMU 与点云不同步**

   * 在仿真模式下关闭时间同步：`time_sync_en := false`

3. **TF 错误**

   * 确认 `lidar_link → base_link` 存在
   * 用 `rosrun tf view_frames` 检查

---

## 6. 后续工作

* **导航部分待完善**：Fast-LIO 输出 `/odom` 可与 Navigation Stack 融合，但参数仍需测试
* **配置文件细化**：Mid-360 插件参数需确保输出点云格式与 Fast-LIO 匹配，后续将补充典型配置

---

## 7. 参考资料

* [FAST-LIO 官方仓库](https://github.com/hku-mars/FAST_LIO)
* [Mid360 仿真插件](https://github.com/DWDROME/Mid360_simulation_plugin)
* [livox\_ros\_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
* [Livox SDK](https://github.com/Livox-SDK/Livox-SDK)

---
