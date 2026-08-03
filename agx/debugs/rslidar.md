# RoboSense Helios 32 线激光雷达使用指南

本文整理速腾 Helios 32 机械激光雷达从驱动安装到建图调用的完整流程。

---

## 1. 硬件连接与网络配置1

Helios 32 雷达出厂默认网络参数：

| 参数 | 值 |
|------|-----|
| 雷达 IP | `192.168.1.200` |
| 电脑端 IP | `192.168.1.102` |
| MSOP 端口 | `6699` |
| DIFOP 端口 | `7788` |

**设置电脑 IP：**

将电脑有线网卡 IP 设置为 `192.168.1.102`，子网掩码 `255.255.255.0`。

**验证连接：**
```bash
ping 192.168.1.200
```

---

## 2. 驱动安装

### 2.1 下载 SDK

```bash
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
```

### 2.2 安装依赖

```bash
sudo apt-get install -y libpcap-dev
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 2.3 建立工作空间并编译

```bash
mkdir -p ~/robosense_ws/src
cp -r rslidar_sdk ~/robosense_ws/src/
cd ~/robosense_ws
catkin_make
source devel/setup.bash
```

---

## 3. 配置文件修改

编辑 `src/rslidar_sdk/config/config.yaml`：

```yaml
common:
  msg_source: 1                  # 1=在线雷达, 2=ROS包回放, 3=PCAP文件
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RSHELIOS       # Helios 32 对应型号
      msop_port: 6699
      difop_port: 7788
    ros:
      ros_frame_id: rslidar
      ros_send_point_cloud_topic: /rslidar_points
```

> **注意**：RS-Helios-16P 使用 `RSHELIOS_16P`，RS-Helios 32 使用 `RSHELIOS`。

---

## 4. 启动与录制

### 4.1 启动雷达（带 RViz）

```bash
roslaunch rslidar_sdk start.launch
```

### 4.2 启动雷达（不带 RViz，用于录制）

```bash
roslaunch rslidar_sdk run.launch
```

### 4.3 录制点云数据

```bash
rosbag record /rslidar_points /rslidar_packets -o 250108.bag
```

### 4.4 回放数据

修改 `config.yaml` 中 `msg_source` 为 `2`，然后：

```bash
rosbag play 250108.bag
```

---

## 5. 常见错误

| 错误 | 原因 | 解决 |
|------|------|------|
| `ERRCODE_MSOPTIMEOUT` | 雷达未连接或未通电 | 检查电源和网线，`ping 192.168.1.200`，重新 `roslaunch` |
| `ERRCODE_WRONGMSOPID` | `config.yaml` 中 `lidar_type` 写错 | 确认型号为 `RSHELIOS`（32线）或 `RSHELIOS_16P`（16线） |

---

## 6. Helios 32 相关参数

```cpp
// RS-Helios 32
const int N_SCAN = 32;              // 激光雷达线数
const int Horizon_SCAN = 1800;      // 每条线水平点数
const float ang_res_x = 0.2;        // 水平分辨率（度）
const float ang_res_y = 1.5;        // 垂直分辨率（度）
const float ang_bottom = 55;        // 最下方激光与水平面夹角（度）
const int groundScanInd = 10;       // 地面线数
```

---

## 7. 建图算法集成

### 7.1 速腾雷达转 Velodyne 格式

部分算法（如 LeGO-LOAM、Fast-LIO2）默认接收 Velodyne 格式点云，需要转换：

参考：https://blog.csdn.net/weixin_44023934/article/details/123845089

**常见问题：`Failed to find match for field 'intensity'`**

修改 `rs_to_velodyne/src/rs_to_velodyne.cpp`，将 `RsPointXYZIRT` 中 `intensity` 的数据类型改为 `float`。

### 7.2 使用 Fast-LIO2 建图

参考：https://blog.csdn.net/Sangfno1/article/details/125678997

### 7.3 使用 LeGO-LOAM 建图

**常见问题：`Failed to transform from frame [/camera] to frame [map]`**

原因：tf2 不再在 `frame_id` 前面加 `/`。解决方法：将代码中所有 `frame_id` 相关的 `/` 前缀去掉。

**编译错误：`PCL requires C++14 or above`**

在 `CMakeLists.txt` 中修改：
```cmake
# 将
add_compile_options(-std=c++11)
# 改为
add_compile_options(-std=c++14)
# 或
set(CMAKE_CXX_STANDARD 14)
```

### 7.4 OpenCV 版本冲突

如果系统中有两个 OpenCV 版本（如 3.4.15 和 4.2.0），需要删除旧版本：

参考：https://blog.csdn.net/whitephantom1/article/details/136406214

**`opencv/cv.h: No such file or directory`**

参考：https://blog.csdn.net/weixin_42990464/article/details/137771966

---

## 8. 外参标定（LiDAR-IMU）

### 8.1 使用浙大 lidar_IMU_calib

需要用到 `rslidar_packets` 或 `velodyne_packets` 话题：

```bash
rosbag record -o calibration /imu/data /rslidar_points
```

参考：
- https://blog.csdn.net/weixin_45205745/article/details/129462125
- https://blog.csdn.net/weixin_46416017/article/details/120316198

### 8.2 使用 hku-mars LiDAR_IMU_Init

GitHub：https://github.com/hku-mars/LiDAR_IMU_Init

参考：https://www.betaflare.com/biancheng/1730756094a1512576.html

### 8.3 安装 Pangolin（可视化依赖）

参考：https://blog.csdn.net/wuicer/article/details/145716854

测试：https://blog.csdn.net/chenxinyun921/article/details/123374158

---

## 9. 参考链接

| 主题 | 链接 |
|------|------|
| Ubuntu 20.04 使用 RSView 查看点云 | https://blog.csdn.net/weixin_49353445/article/details/133278920 |
| 录制与播放 ROS bag | https://blog.51cto.com/u_15316847/3433288 |
| RS-Helios 32 + Fast-LIO2 建图 | https://blog.csdn.net/Sangfno1/article/details/125678997 |
| RS-Helios 32 转 Velodyne 格式 | https://blog.csdn.net/weixin_44023934/article/details/123845089 |
| LeGO-LOAM 建图 | https://blog.csdn.net/Sangfno1/article/details/125678997 |
| 浙大 LiDAR-IMU 标定 | https://blog.csdn.net/weixin_45205745/article/details/129462125 |
| 完整标定流程 | https://www.betaflare.com/biancheng/1730756094a1512576.html |
