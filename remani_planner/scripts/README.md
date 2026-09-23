# 笔记本脚本（Laptop）— `remani_planner/scripts`

只在笔记本 / `remani_planner` 工作空间使用。
实机驱动脚本在：`agilex_ws/scripts/`（车上为 `~/agilex_ws/scripts/`）。

## 控制面

| 脚本 | 作用 |
|------|------|
| `run_real_dry_run_control_plane.sh` | 启动 dry-run 控制面（State Bridge / Gate / Executor / planner / RViz） |
| `publish_cr10_status_stub.sh` | 临时发布 `/remani/cr10_status`（真机反馈缺口） |
| `watch_execution_state.sh` | 只读 `/remani/execution_state` |
| `watch_ranger_cmd.sh` | 只读硬件 `cmd_vel` + dry-run preview |

## 传感器监控（驱动在实机）

| 脚本 | 作用 |
|------|------|
| `watch_lidar.sh` | 看实机 `/rslidar_points` 频率 |
| `watch_camera.sh` | 看实机 `/camera/color` / depth 频率 |

## 笔记本一次会话

先保证实机已跑：

- `agilex_ws/scripts/run_real_dry_run.sh`
- （可选）`run_lidar.sh` / `run_camera.sh`

且两边：

```bash
export ROS_IP=<laptop_ip>
export ROS_MASTER_URI=http://<master>:11311   # 与实机相同
```

```bash
cd remani_planner
source devel/setup.bash

./scripts/run_real_dry_run_control_plane.sh
./scripts/publish_cr10_status_stub.sh
./scripts/watch_execution_state.sh
./scripts/watch_ranger_cmd.sh
./scripts/watch_lidar.sh
./scripts/watch_camera.sh
```

`dry_run:=true` 时笔记本不得编译/source/启动 `agilex_ws/`，也不得把
`start_hardware_drivers:=true` 设在笔记本上。
