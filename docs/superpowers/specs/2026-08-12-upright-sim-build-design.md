# Upright 仿真编通设计

日期：2026-08-12  
状态：已批准（方案 A）

## 目标

在独立 catkin 工作空间 `upright_ws` 中编通并跑通官方 PyBullet 仿真：

```bash
./mpc_sim.py --config $(rospack find upright_cmd)/config/demos/thing_demo.yaml
```

成功标准：`catkin build` 通过；仿真启动；IPython `exit` 后能步进/跑完一段。

## 非目标

- Ranger + CR10 适配
- 真机 / `mobile_manipulation_central` 硬件 launch
- 修改现有 `ocs2_ws`（保持 Ridgeback demo 可用）

## 布局

`upright_ws/src/`：

| 包 | 来源 |
|----|------|
| upright | 已有 |
| mobile_manipulation_central | utiasDSL/mobile_manipulation_central |
| ocs2 | utiasDSL/ocs2 `-b upright` |
| pinocchio / hpp-fcl / ocs2_robotic_assets | 与 OCS2 upright 文档一致的独立 clone |

## 构建步骤

1. catkin init，extend `/opt/ros/noetic`，RelWithDebInfo
2. 应用 MMC `catkin/config.yaml` 跳过无关包
3. `pip install -r upright/requirements.txt`（系统 Python 3.8）
4. `catkin build`
5. source 后跑 `thing_demo.yaml`

## 风险

- upright 分支 OCS2 与 pinocchio/hpp-fcl 版本需对齐
- 首次 AD/CppAD 代码生成编译较慢
