---

# GMapping SLAM 使用说明

本说明介绍如何在 **Gazebo 仿真环境**下使用 **GMapping** 完成 2D 建图任务。
这是本项目中继承自上一代的 **传统功能模块**，与 Fast-LIO 仿真互为补充。

---

## 1. 安装依赖

确保已安装以下依赖包：

```bash
sudo apt install ros-${ROS_DISTRO}-slam-gmapping \
                 ros-${ROS_DISTRO}-openslam-gmapping \
                 ros-${ROS_DISTRO}-navigation \
                 ros-${ROS_DISTRO}-robot-localization
```

⚠️ 注意：`tf2_sensor_msgs` 等依赖会由 `rosdep` 自动安装。

---

## 2. 启动流程

### Step 1. 启动 Gazebo 仿真环境

选择任意 **带传感器的世界**（推荐 Playpen）：

```bash
roslaunch scout_gazebo_sim scout_mini_playpen.launch
# 或
# roslaunch scout_gazebo_sim scout_mini_empty_world.launch
# roslaunch scout_gazebo_sim scout_mini_house.launch
```

### Step 2. 启动底盘 + EKF

```bash
# 发布里程计
roslaunch scout_base scout_mini_base.launch

# EKF 融合（里程计 + IMU）
roslaunch scout_filter ekf_filter_cmd.launch
```

### Step 3. 启动 GMapping

```bash
roslaunch scout_slam scout_slam.launch
```

此时 RViz 中会显示实时构建的二维栅格地图。

### Step 4. 控制机器人运动

```bash
roslaunch scout_teleop scout_teleop_key.launch
```

使用 `WASD` 控制机器人探索环境。

### Step 5. 保存地图

```bash
roslaunch scout_slam gmapping_save.launch map_file:=map1
```

默认保存目录：`scout_slam/maps/`，保存文件包含 `.pgm + .yaml`。

---

## 3. 配置要点

1. **话题配置**

   * Laser 输入：`/scan`
   * 里程计输入：`/odom`（由 EKF 输出）
   * 地图输出：`/map`

2. **TF 配置**

   * 保证 `map → odom → base_link → lidar_link` 连通
   * 使用 `rqt_tf_tree` 检查 TF 树完整性

3. **参数调整**（在 `scout_slam/launch/scout_slam.launch` 内）

   * `linearUpdate`：机器人移动多少米更新一次激光（默认 1.0）
   * `angularUpdate`：旋转多少角度更新一次激光（默认 0.5 rad）
   * `particles`：粒子数（越大越精确但越耗资源，默认 80）

---

## 4. 典型策略

### 4.1 单独运行建图

* 推荐流程：Gazebo + EKF + GMapping + Teleop
* 用于快速构建并保存二维地图

### 4.2 与 Navigation 并行

* 在地图生成完成后，可直接加载并启动 Navigation：

  ```bash
  roslaunch scout_navigation scout_navigation.launch map_file:=map1.yaml
  ```
* 机器人可在已建图的环境中进行自主导航

---

## 5. 常见问题

1. **地图出现拖影 / 重叠**

   * 检查 EKF 配置，确保 `/odom` 稳定
   * 适当降低 `linearUpdate` 和 `angularUpdate`

2. **机器人不动**

   * 确认 `scout_teleop` 是否正常发布 `/cmd_vel`
   * 使用 `rostopic echo /cmd_vel` 验证

3. **地图保存失败**

   * 确认保存目录存在写权限
   * 手动执行：

     ```bash
     rosrun map_server map_saver -f ~/scout_ws/src/scout_slam/maps/map1
     ```

---

## 6. 参考资料

* [slam\_gmapping](http://wiki.ros.org/gmapping)
* [navigation stack](http://wiki.ros.org/navigation)
* [robot\_localization](http://wiki.ros.org/robot_localization)

---
