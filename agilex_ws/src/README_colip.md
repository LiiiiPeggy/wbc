# 项目简介

本项目基于ROS，支持激光雷达建图、地图保存、导航与定点巡航等功能。各模块分布于`src`文件夹下的不同子文件夹，以下为各子文件夹的作用及其典型启用方式。

## src文件夹结构与说明

### 1. agilexpro
- **作用**：包含激光雷达驱动、建图（gmapping）、导航等主要功能包。
- **典型启用方式**：
  - 启动激光雷达：
    ```
    roslaunch agilexpro open_lidar.launch
    ```
  - 启动gmapping建图：
    ```
    roslaunch agilexpro gmapping.launch
    ```
  - 启动导航：
    ```
    roslaunch agilexpro navigation_4wd.launch
    ```

### 2. map_server
- **作用**：地图保存与加载相关功能包。
- **典型启用方式**：
    ```
    rosrun map_server map_saver -f map
    ```

### 3. bunker_bringup
- **作用**：底盘CAN口驱动与初始化脚本。
- **典型启用方式**：
    ```
    rosrun bunker_bringup bringup_can2usb.bash
    ```

## 使用流程简述

1. 启动CAN口（仅首次开机需要）：
    ```
    rosrun bunker_bringup bringup_can2usb.bash
    ```
2. 启动激光雷达：
    ```
    roslaunch agilexpro open_lidar.launch
    ```
3. 启动建图或导航：
    - 建图：
      ```
      roslaunch agilexpro gmapping.launch
      ```
    - 导航：
      ```
      roslaunch agilexpro navigation_4wd.launch
      ```
4. 保存地图：
    ```
    rosrun map_server map_saver -f map
    ```

## 其他说明

- 地图保存后，需关闭雷达和建图程序。
- 导航时可通过rviz进行定位校正和定点巡航操作。

> 如需详细操作说明，请参考原始README.md。