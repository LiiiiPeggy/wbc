# Upright 仿真编通 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在自包含 `upright_ws` 编通并跑通 `mpc_sim.py` + `thing_demo.yaml`。

**Architecture:** 独立 catkin 工作空间：MMC + OCS2(`upright`) + pinocchio/hpp-fcl/assets + upright；不碰现有 `ocs2_ws`。

**Tech Stack:** ROS Noetic, catkin_tools, OCS2 upright fork, Pinocchio, HPP-FCL, PyBullet, Python 3.8.

---

### Task 1: Clone 依赖到 upright_ws/src

- [ ] Clone `mobile_manipulation_central`
- [ ] Clone `ocs2` branch `upright`
- [ ] Clone `pinocchio`、`hpp-fcl`、`ocs2_robotic_assets`（版本参考现有 ocs2_ws 或 upright/MMC 文档）

### Task 2: 初始化 catkin 并配置跳过包

- [ ] `catkin init` + extend noetic + RelWithDebInfo
- [ ] 应用 MMC `catkin/config.yaml`

### Task 3: Python 依赖

- [ ] `pip3 install -r upright/requirements.txt`（用户环境，必要时 `--user`）

### Task 4: 编译

- [ ] `catkin build`（或先 build upright 相关包）
- [ ] 修复编译错误直至成功

### Task 5: 跑通仿真

- [ ] `source devel/setup.bash`
- [ ] 运行 `mpc_sim.py --config .../thing_demo.yaml`
- [ ] 记录结果；必要时修运行时问题
