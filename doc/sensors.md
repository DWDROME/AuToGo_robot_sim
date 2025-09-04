---

# Sensor List & Configuration

本项目支持多类型传感器的仿真与配置，用户可根据实验目的选择和定制。
所有传感器均通过 **URDF + xacro + Gazebo 插件** 集成，支持在 **launch 文件** 中开启/关闭或修改参数。

---

## 0. 传感器清单

* **2D LiDAR** 
  - Hokuyo 
* **3D LiDAR**

  * Velodyne VLP-16
  * Velodyne HDL-32E
  * Livox Mid-360（Fast-LIO 仿真新增重点）
* **IMU**
* **Camera 系列**

  * RGB Camera
  * Openni-Kinect
  * Intel RealSense D435
* **GPS**

---

## 1. 配置方法

* 每个传感器的定义写在 `scout_description/urdf/*.xacro`或者是`scout_description/urdf/sensors/*.xacro` 中

  * 机器人平台总配置：`mini.xacro`
  * 单独传感器定义：`imu.xacro`、`lidar_2d.xacro`、`livox_mid360_lib.xacro` 等
* 可修改以下内容：

  * **开关**：是否加载该传感器
  * **位置**：通过 `xacro` 参数或 launch 文件调整安装位置
  * **参数**：量程、频率、分辨率等

示例：

```xml
<!-- 在 launch 文件中通过 arg 控制 -->
<arg name="use_imu" default="true"/>
<arg name="use_lidar" default="true"/>
```

参考：

* `scout_bringup/scout_test/scout_mini_sensor_test.launch`

---

## 2. 2D LiDAR

默认配置为 Hokuyo 型激光雷达。

* 支持距离范围、分辨率等参数调整
* 常用于 GMapping / Navigation

🔗 Reference:

* [Gazebo GPU Laser 插件](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#GPULaser)
* [Hokuyo 模型资源](https://github.com/osrf/gazebo_models/tree/master/hokuyo/meshes)
* [添加 LiDAR 教程](https://level-asphalt-6c6.notion.site/Gazebo-Scout-mini-add-lidar-086c23578e904467864f284ad02c8564)

---

## 3. 3D LiDAR

### 3.1 Velodyne 系列

支持 VLP-16 / HDL-32E。

* 插件来源：`velodyne_simulator`

🔗 Reference:

* [Velodyne Simulator](https://github.com/lmark1/velodyne_simulator)

### 3.2 Livox Mid-360（新增重点）

* 用于 **Fast-LIO 仿真**
* 集成仓库：[Mid360\_simulation\_plugin](https://github.com/DWDROME/Mid360_simulation_plugin)
* 提供高频点云流（可配置为 Dense 模式）
* 典型启动方式：

  ```bash
  roslaunch mid360_plugin mid360_gazebo.launch
  ```
* ⚠️ 需与 Fast-LIO 配置文件保持一致（话题名/TF），详见 `docs/fastlio.md`

---

## 4. 相机

### 4.1 RGB Camera / Kinect

* 可用于基础视觉或 RGB-D 实验

### 4.2 Intel RealSense D435

* 插件来源：`realsense_gazebo_plugin`
* 支持 RGB + Depth
* 如未发布压缩图像，需要安装以下依赖：

  ```bash
  sudo apt install ros-${ROS_DISTRO}-image-transport-plugins \
                   ros-${ROS_DISTRO}-compressed-image-transport \
                   ros-${ROS_DISTRO}-theora-image-transport \
                   ros-${ROS_DISTRO}-compressed-depth-image-transport
  ```

🔗 Reference:

* [realsense\_gazebo\_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
* [Intel RealSense ROS](https://github.com/IntelRealSense/realsense-ros)

---

## 5. IMU

* 基于 `GazeboRosImu` 插件
* 支持角速度/加速度仿真噪声配置
* 常用于 EKF 融合、SLAM

🔗 Reference:

* [Gazebo IMU 插件](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#IMU%28GazeboRosImu%29)
* [ROS Q\&A: IMU 仿真](https://answers.ros.org/question/12430/modelling-sensorsimu-in-gazebo/)

---

## 6. GPS

* 插件来源：`hector_gazebo_plugins`
* 常用于室外场景定位
* 与 EKF 融合可提供全局坐标约束

🔗 Reference:

* [hector\_gazebo\_plugins](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/tree/melodic-devel/hector_gazebo_plugins/include/hector_gazebo_plugins)

---

## 7. 总结

* 所有传感器均可 **自由组合与配置**，通过 launch 文件参数控制开关与位置
* 本项目特别强调 **Livox Mid-360 集成**，为 **Fast-LIO 仿真** 提供完整支持
* 建议根据实验目的在 `docs/simulation.md` 中选择合适的启动世界，并结合 `docs/fastlio.md` 或 `docs/slam.md` 使用

---

