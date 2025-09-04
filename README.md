---

# AuToGo\_robot\_sim

基于 **ROS1 Noetic / Gazebo** 的 **多功能移动机器人仿真平台**。
本项目提供从 **SLAM、EKF 融合、导航** 到 **Fast-LIO 高精度点云建图** 的完整实验环境，并集成了 **Livox Mid-360 仿真插件**，可用于多场景、多模式下的自主导航与定位研究。

---

## 1. 功能概览

* **多世界场景支持**

  * `scout_mini_empty_world.launch`：空旷场景（含传感器）
  * `scout_mini_playpen.launch`：Playpen 测试场景（含传感器，可选 3D 配置）
  * `scout_mini_house.launch`：House 场景（含传感器）
  * `scout_empty_world.launch`：空世界（无传感器）

* **传感器仿真**

  * 2D 激光雷达
  * 3D 激光雷达（VLP-16 / HDL-32E / Mid-360）
  * IMU、GPS
  * RGB / 深度相机

* **算法支持**

  * EKF 融合定位
  * 2D SLAM（GMapping）
  * Navigation Stack 自主导航
  * Fast-LIO（Livox Mid-360 点云建图，**核心目标**）

---

## 2. 系统环境与依赖

* **系统**：Ubuntu 20.04 + ROS Noetic + Gazebo 11
* **依赖**：

  * `slam_gmapping`
  * `navigation`
  * `robot_localization`
  * `tf2_sensor_msgs`
  * 其他依赖由 `rosdep` 自动安装

---

## 3. 安装

### 仿真环境（本仓库）

```bash
# 1) 创建工作空间
mkdir -p ~/autogo_ws/src && cd ~/autogo_ws/src

# 2) 克隆本仓库与 Mid360 仿真插件
git clone https://github.com/<your-username>/AuToGo_robot_sim.git
git clone https://github.com/DWDROME/Mid360_simulation_plugin.git

# 3) 安装常用依赖
chmod +x AuToGo_robot_sim/install_packages.sh
./AuToGo_robot_sim/install_packages.sh

# 4) 用 rosdep 安装剩余依赖
cd ~/autogo_ws
rosdep install --from-paths src --ignore-src -r -y

# 5) 构建
sudo apt install -y python3-catkin-tools
catkin build

# 6) 环境变量
source devel/setup.bash
```

### Fast-LIO（推荐单独工作空间）

```bash
mkdir -p ~/fastlio_ws/src && cd ~/fastlio_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ..
catkin_make
```

⚠️ Fast-LIO 默认依赖 `livox_ros_driver`，需要替换为 `livox_ros_driver2`。
具体说明见 `docs/fastlio.md`。

---

## 4. 启动 Gazebo 仿真

项目提供 **三种带传感器的世界** 与 **一种空世界**：

```bash
# 空旷场景（含传感器）
roslaunch scout_gazebo_sim scout_mini_empty_world.launch

# Playpen 场景（含传感器，可选 3D）
roslaunch scout_gazebo_sim scout_mini_playpen.launch

# House 场景（含传感器）
roslaunch scout_gazebo_sim scout_mini_house.launch

# 空世界（无传感器）
roslaunch scout_gazebo_sim scout_empty_world.launch
```

---

## 5. 算法运行

### 5.1 Fast-LIO 仿真（核心目标）

```bash
# 启动带传感器的 Gazebo 世界（例：Playpen）
roslaunch scout_gazebo_sim scout_mini_playpen.launch

# 启动 Fast-LIO
roslaunch fast_lio mapping_mid360.launch
```

* IMU 默认话题：`/imu`
* 点云话题：`/livox/lidar`
* 需要根据话题名 / TF / 频率进行少量调整，详见 `docs/fastlio.md`

---

### 5.2 2D SLAM + Navigation

```bash
# 启动 Gazebo 世界（例：Playpen）
roslaunch scout_gazebo_sim scout_mini_playpen.launch

# EKF 融合
roslaunch scout_filter ekf_filter_cmd.launch

# GMapping 建图
roslaunch scout_slam scout_slam.launch

# Navigation 导航
roslaunch scout_navigation scout_navigation.launch
# 在 RViz 使用 "2D Nav Goal" 发送目标点
```

* 地图保存/加载与参数调整见 `docs/slam.md`、`docs/navigation.md`

---

## 6. 文档导航

* `docs/simulation.md`：世界选择与启动参数
* `docs/sensors.md`：传感器接口说明（含 Mid-360）
* `docs/fastlio.md`：Fast-LIO 仿真与配置要点
* `docs/slam.md`：GMapping 建图与地图保存/加载
* `docs/navigation.md`：Navigation Stack 配置与常见问题

---

## 7. 目录结构

```
AuToGo_robot_sim/
├── launch/                  # 启动文件
│   ├── scout_mini_empty_world.launch
│   ├── scout_mini_playpen.launch
│   ├── scout_mini_house.launch
│   └── scout_empty_world.launch
├── scout_description/        # 机器人模型与传感器
├── scout_gazebo_sim/         # 仿真环境与插件
├── scout_base/               # 底盘里程计
├── scout_filter/             # EKF 融合
├── scout_slam/               # GMapping 建图
├── scout_navigation/         # Navigation Stack
├── scout_teleop/             # 键盘控制
└── docs/                     # 说明文档
```

---

## 8. 贡献与许可

* 欢迎提交 Issue / PR 来改进仿真环境与配置模板。
* 本项目采用 **Apache License 2.0**，详见 [LICENSE](./LICENSE)。

---

## 9. 学术与专利声明

本项目旨在为 **科研实验、学术论文与专利研发** 提供基础仿真平台：

* 可在学术论文中引用本仓库，保持署名与链接。
* 部分创新性内容可能在专利申请与论文中使用，具体实现将以学术成果为准。
* 使用本仓库即视为接受 [LICENSE](./LICENSE) 中的相关条款。

---

This project is licensed under the Apache License 2.0 - see the [LICENSE](./LICENSE) file for details.


## Citation

If you use **AuToGo_robot_sim** in your research, please cite it as:

```bibtex
@misc{Chen2025_AuToGoRobotSim,
  author       = {Ziyang Chen},
  title        = {AuToGo\_robot\_sim: A Multi-Functional Mobile Robot Simulation Platform for ROS1 Noetic and Gazebo},
  year         = {2025},
  publisher    = {GitHub},
  howpublished = {\url{https://github.com/<your-username>/AuToGo_robot_sim}},
  note         = {Open-source simulation platform with SLAM, EKF, Navigation, and Fast-LIO support}
}
