# AgileX Ranger 移动机械臂 Sim2Real 调研报告

> 目标设备：AgileX Ranger 轮式底盘 + 单机械臂 + 双目相机 + 避障激光雷达
> 应用场景：主动三维重建补测（Active 3D Reconstruction / Next-Best-View Scanning）
> 调研时间：2026-05
> 注意：由于网络访问限制，部分 GitHub 链接和星标数据基于训练知识整理，标注 `⚠️ 需验证` 的条目请自行确认可用性。

---

## 一、项目速查表（按开源可复现程度与关联度排序）

| 项目名称 | 开源可复现 | 关联度 | 移动底盘支持 | 基于 Isaac Lab | 最新活跃 | 论文/代码链接 | 一句话亮点 |
|:---|:---:|:---:|:---:|:---:|:---:|:---|:---|
| **SplaTAM** | ✅ | ★★★★☆ | ✅ (任意平台) | ❌ | 2024 | [GitHub](https://github.com/spla-tam/SplaTAM) | 实时 3DGS SLAM，主动扫描的重建后端 |
| **OpenVLA** | ✅ | ★★★☆☆ | 可扩展 | ❌ | 2024 | [GitHub](https://github.com/openvla/openvla) | 7B 参数开源 VLA 模型，支持 LoRA 微调 |
| **Octo** | ✅ | ★★★☆☆ | 可扩展 | ❌ | 2024 | [GitHub](https://github.com/octo-models/octo) | 通用机器人策略，扩散动作头，多 embodiment |
| **OK-Robot** | ✅ | ★★★☆☆ | ✅ (Fetch) | ❌ | 2024 | [项目页](https://ok-robot.github.io) | 零样本移动操作流水线，开箱即用 |
| **Diffusion Policy** | ✅ | ★★★☆☆ | 可扩展 | ❌ | 2023 | [GitHub](https://github.com/real-stanford/diffusion_policy) | 扩散模型生成动作轨迹，可适配扫描策略 |
| **RoboCasa** | ✅ | ★★★☆☆ | ✅ | ❌ (Robosuite) | 2024 | [GitHub](https://github.com/robocasa/robocasa) | 大规模家庭移动操作仿真环境 |
| **ManiSkill3** | ✅ | ★★☆☆☆ | ✅ | ❌ (SAPIEN) | 2024 | [GitHub](https://github.com/haosulab/ManiSkill) | 操作基准环境，GPU 并行仿真 |
| **Mobile ALOHA** | ✅ | ★★☆☆☆ | ✅ (Tracer) | ❌ | 2024 | [GitHub](https://github.com/MarkFzp/mobile-aloha) | 低成本双臂移动操作系统+模仿学习 |
| **LeRobot** | ✅ | ★★☆☆☆ | 可扩展 | ❌ | 2025 | [GitHub](https://github.com/huggingface/lerobot) | HuggingFace 机器人学习框架 |
| **RSL-RL** | ✅ | ★★★☆☆ | ✅ | ✅ | 2025 | [GitHub](https://github.com/leggedrobotics/rsl_rl) | ETH 出品的轻量 RL 库，Isaac Lab 默认集成 |
| **legged_gym** | ✅ | ★★★☆☆ | ✅ | ✅ | 2024 | [GitHub](https://github.com/leggedrobotics/legged_gym) | ETH 腿式/轮式运动 RL 训练环境 |
| **ActiveGAMER** | ⚠️ 需验证 | ★★★★★ | ✅ (探索 agent) | ❌ | 2024 | ⚠️ GitHub 搜索验证 | 主动 3DGS 重建 + 视点规划，最直接相关 |
| **GNB-NBV** | ⚠️ 需验证 | ★★★★★ | 可扩展 | ❌ | 2024 | ⚠️ arXiv 搜索验证 | 3DGS 驱动的 NBV 规划 |
| **MoManipVLA** | ⚠️ 需验证 | ★★★★☆ | ✅ | ❌ | 2024 | ⚠️ arXiv 搜索验证 | 专门为移动操作设计的 VLA 模型 |
| **DovSG** | ⚠️ 需验证 | ★★★★☆ | ✅ (室内) | ❌ | 2024 | ⚠️ arXiv 搜索验证 | 场景图引导的操作+3D 理解 |
| **SpatialVLA** | ⚠️ 需验证 | ★★★☆☆ | 未知 | ❌ | 2025 | ⚠️ 搜索验证 | 空间感知 VLA，3D 推理能力强 |
| **CogACT** | ⚠️ 需验证 | ★★☆☆☆ | 未知 | ❌ | 2024 | ⚠️ 搜索验证 | 认知-动作统一基础模型 |
| **Robo-GS** | ⚠️ 需验证 | ★★★★☆ | 未知 | ❌ | 2024 | ⚠️ 搜索验证 | 机器人场景 3DGS 主动重建 |

---

## 二、项目详细分析

---

### 1. SplaTAM

- **是否开源**：是 — [https://github.com/spla-tam/SplaTAM](https://github.com/spla-tam/SplaTAM)
- **最新版本/更新时间**：CVPR 2024，代码持续维护
- **关联度评分**：★★★★☆ — 实时 3DGS SLAM，是主动扫描系统的理想重建后端
- **移动底盘支持**：✅ 平台无关 — 接受任意 RGB-D 输入流，可直接配合移动机器人
- **仿真平台**：❌ 不含仿真，纯算法
- **能否直接导入你的 URDF**：不适用（SLAM 算法不依赖 URDF）
- **核心方法**：
  - 使用 3D Gaussian Splatting 作为场景表示
  - 联合优化相机位姿跟踪和 3DGS 地图构建
  - 实时渲染和稠密重建
  - 支持 RGB-D 输入
- **适配性分析**：
  - 可作为主动扫描系统的"重建模块"
  - 输入你的双目相机（转为 RGB-D）数据即可
  - 可以计算信息增益来指导"下一个最佳视点"
  - 与 RL 训练结合：将 SplaTAM 的覆盖率作为 reward signal
- **可直接尝试的复现步骤**：
  ```bash
  git clone https://github.com/spla-tam/SplaTAM.git
  cd SplaTAM
  pip install -r requirements.txt
  # 运行示例（需要 RGB-D 数据集）
  python scripts/splatam.py --config configs/replica/room0.yaml
  ```
- **关键缺失与注意事项**：
  - 需要自行集成到主动扫描 pipeline
  - 不含 NBV 规划逻辑，需要额外实现
  - GPU 显存需求较大（16GB+ 建议）

---

### 2. RSL-RL（Robot Systems Lab RL）

- **是否开源**：是 — [https://github.com/leggedrobotics/rsl_rl](https://github.com/leggedrobotics/rsl_rl)
- **最新版本/更新时间**：v2.0+，2025 年活跃
- **关联度评分**：★★★☆☆ — 轻量级 PPO 实现，Isaac Lab 默认集成，适合作为 RL 训练的算法后端
- **移动底盘支持**：✅ 算法层支持任意 action space，配合 Isaac Lab 可控制底盘
- **仿真平台**：Isaac Lab / Isaac Sim（深度集成）
- **能否直接导入你的 URDF**：通过 Isaac Lab 间接支持
- **核心方法**：PPO（Proximal Policy Optimization），支持 on-policy 训练、GAE、正交初始化等
- **适配性分析**：适合作为 Isaac Lab 环境的 RL 算法后端，无需额外适配
- **可直接尝试的复现步骤**：
  ```bash
  pip install rsl_rl
  ```
- **关键缺失与注意事项**：仅是算法库，需要配合 Isaac Lab 环境使用

---

### 3. OpenVLA

- **是否开源**：是 — [https://github.com/openvla/openvla](https://github.com/openvla/openvla)
- **最新版本/更新时间**：v0.1+，2024 年发布，2025 年有社区更新
- **关联度评分**：★★★☆☆ — 开源 VLA 基础模型，可通过微调适配你的移动扫描任务
- **移动底盘支持**：⚠️ 原始训练数据主要来自固定臂，但 action space 可通过微调扩展为包含底盘控制
- **仿真平台**：❌ 不含仿真，纯模型推理/微调框架
- **能否直接导入你的 URDF**：不适用（VLA 模型不直接使用 URDF）
- **核心方法**：
  - 7B 参数 VLA（基于 Prismatic VLMs + Llama 2 7B）
  - 输入：RGB 图像 + 语言指令
  - 输出：tokenized 离散动作（可配置 action 维度）
  - 支持 SFT 全参数微调和 LoRA 高效微调
  - 预训练于 Open X-Embodiment 数据集（970k episodes）
- **适配性分析**：
  - 微调时可将 action space 扩展为 `[base_vx, base_vy, base_wz, arm_j1...j6, gripper]`
  - 需要自行采集或仿真生成移动扫描的 demonstration 数据
  - 语言指令可设计为"扫描物体的背面"/"移动到左侧观察"等
- **可直接尝试的复现步骤**：
  ```bash
  git clone https://github.com/openvla/openvla.git
  cd openvla
  pip install -e .
  
  # 使用预训练模型推理
  python experiments/robot/libero/run_openvla_libero.py
  
  # 微调到自定义 action space
  # 参考 experiments/robot/ 下的微调脚本
  ```
- **关键缺失与注意事项**：
  - 需要自行准备移动扫描的 demonstration 数据集
  - 7B 模型推理需要较大显存（建议 24GB+）
  - 不含仿真环境，需与 Isaac Lab 或真实机器人配合

---

### 4. Octo

- **是否开源**：是 — [https://github.com/octo-models/octo](https://github.com/octo-models/octo)
- **最新版本/更新时间**：v0.1，2024 年发布
- **关联度评分**：★★★☆☆ — 通用机器人策略模型，支持多种 embodiment，扩散动作头适合连续控制
- **移动底盘支持**：⚠️ 设计支持多 embodiment，可通过微调适配移动底盘+臂
- **仿真平台**：❌ 不含仿真
- **能否直接导入你的 URDF**：不适用
- **核心方法**：
  - Transformer 架构 + 扩散动作头（Diffusion action head）
  - 预训练于 Open X-Embodiment 数据集
  - 支持语言目标、图像目标、本体感觉等多种输入
  - 动作输出支持 diffusion-based 和 tokenized discrete 两种模式
- **适配性分析**：
  - 扩散动作头天然适合连续控制（底盘速度+关节角）
  - 多 embodiment 设计意味着架构已考虑不同机器人形态
  - 需要微调数据集
- **可直接尝试的复现步骤**：
  ```bash
  git clone https://github.com/octo-models/octo.git
  cd octo
  pip install -e .
  
  # 加载预训练模型
  from octo.model.octo_model import OctoModel
  model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small")
  
  # 微调到自定义 embodiment
  # 参考 examples/ 下的微调教程
  ```
- **关键缺失与注意事项**：
  - 需要自行准备 demonstration 数据
  - 模型较小（相比 OpenVLA），推理更快
  - 不含仿真环境

---

### 5. OK-Robot

- **是否开源**：是 — [https://ok-robot.github.io](https://ok-robot.github.io)（含代码链接）
- **最新版本/更新时间**：2024 年 ICRA/RSS
- **关联度评分**：★★★☆☆ — 零样本移动操作流水线，展示了如何集成感知模块用于移动机器人
- **移动底盘支持**：✅ 使用 Fetch 移动机器人（轮式底盘+臂）
- **仿真平台**：❌ 纯真实世界部署
- **能否直接导入你的 URDF**：不适用（使用 Fetch 的 URDF）
- **核心方法**：
  - DINOv2 感知 + LangSAM 分割 + LLM 规划
  - 零样本：无需任务特定训练
  - 模块化流水线：导航→抓取→放置
  - 关键发现：流水线集成设计比单模型性能更重要
- **适配性分析**：
  - 感知模块（DINOv2、LangSAM）可直接复用
  - LLM 规划模块可改造为扫描策略规划
  - 需要将 Fetch 的控制接口适配到 Ranger
  - 不含 RL 训练，是纯推理流水线
- **可直接尝试的复现步骤**：
  ```bash
  # 从项目页获取代码
  # https://ok-robot.github.io
  
  # 需要 Fetch 机器人或自行适配到 Ranger
  # 核心依赖: DINOv2, LangSAM, OpenAI API
  ```
- **关键缺失与注意事项**：
  - 需要真实机器人硬件
  - 不含仿真训练
  - 适配到 Ranger 需要修改底层控制接口

---

### 6. Diffusion Policy

- **是否开源**：是 — [https://github.com/real-stanford/diffusion_policy](https://github.com/real-stanford/diffusion_policy)
- **最新版本/更新时间**：RSS 2023，持续维护
- **关联度评分**：★★★☆☆ — 扩散模型生成动作轨迹，可适配为扫描轨迹生成器
- **移动底盘支持**：⚠️ 需要自行扩展 action space
- **仿真平台**：❌ 纯策略学习框架
- **能否直接导入你的 URDF**：不适用
- **核心方法**：
  - 去噪扩散概率模型生成动作序列
  - 支持 CNN、Transformer、U-Net backbone
  - 输入：视觉观测序列 → 输出：未来动作序列
  - 支持多模态动作分布
- **适配性分析**：
  - 可将扫描轨迹建模为扩散过程
  - action space 可扩展为底盘+臂联合控制
  - 需要 demonstration 数据
- **可直接尝试的复现步骤**：
  ```bash
  git clone https://github.com/real-stanford/diffusion_policy.git
  cd diffusion_policy
  pip install -e .
  
  # 运行示例任务
  python train.py --config-dir=./diffusion_policy/config --config-name=train_robomimic_low_dim.yaml
  ```
- **关键缺失与注意事项**：需要 demonstration 数据和仿真环境配合

---

### 7. RoboCasa

- **是否开源**：是 — [https://github.com/robocasa/robocasa](https://github.com/robocasa/robocasa)
- **最新版本/更新时间**：2024 年发布，持续更新
- **关联度评分**：★★★☆☆ — 大规模家庭移动操作仿真，可作为 RL 训练环境参考
- **移动底盘支持**：✅ 明确支持移动操作（臂+轮式底座）
- **仿真平台**：Robosuite（基于 MuJoCo）
- **能否直接导入你的 URDF**：⚠️ 需要适配 — Robosuite 使用自己的 MJCF 格式
- **核心方法**：
  - ~150 个家庭物体，100+ 任务
  - 多样化室内场景
  - 域随机化支持 sim2real
  - 基于 MuJoCo 物理引擎
- **适配性分析**：
  - 可参考其移动操作任务设计
  - 与 Isaac Lab 不同仿真引擎，直接复用有限
  - 任务设计思路可迁移到 Isaac Lab
- **可直接尝试的复现步骤**：
  ```bash
  git clone https://github.com/robocasa/robocasa.git
  cd robocasa
  pip install -e .
  
  # 运行示例
  python -m robocasa.demos.demo_kitchen_domain
  ```
- **关键缺失与注意事项**：基于 MuJoCo，与 Isaac Lab 技术栈不同

---

### 8. ManiSkill3

- **是否开源**：是 — [https://github.com/haosulab/ManiSkill](https://github.com/haosulab/ManiSkill)
- **最新版本/更新时间**：2024 年（ManiSkill3）
- **关联度评分**：★★☆☆☆ — 操作基准环境，GPU 并行但与你的主动扫描场景关联有限
- **移动底盘支持**：✅ ManiSkill3 包含移动操作任务
- **仿真平台**：SAPIEN（非 Isaac）
- **能否直接导入你的 URDF**：✅ 支持 URDF 导入
- **核心方法**：GPU 并行物理仿真，标准化评估，支持密集/稀疏奖励
- **适配性分析**：可作为快速原型验证环境，但与 Isaac Lab 技术栈不同
- **可直接尝试的复现步骤**：
  ```bash
  pip install mani_skill
  python -m mani_skill.examples.demo_random_action
  ```
- **关键缺失与注意事项**：基于 SAPIEN，与 Isaac Lab 不兼容

---

### 9. Mobile ALOHA

- **是否开源**：是 — [https://github.com/MarkFzp/mobile-aloha](https://github.com/MarkFzp/mobile-aloha)
- **最新版本/更新时间**：2024 年
- **关联度评分**：★★☆☆☆ — 移动双臂系统+模仿学习，但硬件平台差异大
- **移动底盘支持**：✅ Tracer 移动底盘
- **仿真平台**：❌ 主要面向真实硬件
- **能否直接导入你的 URDF**：不适用（使用自己的硬件 URDF）
- **核心方法**：ACT（Action Chunking with Transformers）模仿学习，~50 个 demonstration
- **适配性分析**：可参考其模仿学习 pipeline，但硬件和仿真部分需要大量改造
- **关键缺失与注意事项**：面向真实硬件，无仿真训练流程

---

### 10. LeRobot

- **是否开源**：是 — [https://github.com/huggingface/lerobot](https://github.com/huggingface/lerobot)
- **最新版本/更新时间**：2025 年持续活跃
- **关联度评分**：★★☆☆☆ — 通用机器人学习框架，可集成多种策略
- **移动底盘支持**：⚠️ 可扩展
- **仿真平台**：支持 Gymnasium 环境接口
- **能否直接导入你的 URDF**：⚠️ 需要自行适配
- **核心方法**：集成 ACT、Diffusion Policy、SAC、TDMPC 等多种算法
- **适配性分析**：框架通用性强，可作为算法集成平台
- **可直接尝试的复现步骤**：
  ```bash
  git clone https://github.com/huggingface/lerobot.git
  pip install -e .
  ```
- **关键缺失与注意事项**：需要自行创建自定义环境

---

### 11. legged_gym

- **是否开源**：是 — [https://github.com/leggedrobotics/legged_gym](https://github.com/leggedrobotics/legged_gym)
- **最新版本/更新时间**：2024 年
- **关联度评分**：★★★☆☆ — 轮式/腿式运动 RL 训练，可作为底盘控制的参考
- **移动底盘支持**：✅ 专注于运动控制
- **仿真平台**：Isaac Gym（Isaac Lab 前身）
- **能否直接导入你的 URDF**：✅ 支持 URDF
- **核心方法**：PPO + Isaac Gym 并行训练，支持各种运动任务
- **适配性分析**：
  - 可参考其轮式底盘控制的 RL 训练方法
  - 与 Isaac Lab 深度兼容
  - 需要将运动控制与操作任务结合
- **可直接尝试的复现步骤**：
  ```bash
  git clone https://github.com/leggedrobotics/legged_gym.git
  cd legged_gym
  pip install -e .
  python legged_gym/scripts/train.py --task=anymal_c_flat
  ```
- **关键缺失与注意事项**：仅运动控制，不含操作和扫描

---

### 12. ActiveGAMER

- **是否开源**：⚠️ 需验证 — 在 GitHub 搜索 `ActiveGAMER` 或 `JingwenWang95/ActiveGAMER`
- **最新版本/更新时间**：2024 年
- **关联度评分**：★★★★★ — 最直接相关：主动 3DGS 重建 + 视点规划
- **移动底盘支持**：✅ 适用于探索 agent（可扩展到移动机器人）
- **仿真平台**：未确认（可能是自定义仿真或真实机器人）
- **能否直接导入你的 URDF**：未提及
- **核心方法**：
  - Active Gaussian Splatting for Efficient Mapping and Exploration with Ray sampling
  - 结合主动视点规划和 3DGS 高效场景重建
  - agent 决定下一步观察位置以构建完整 3D 模型
  - 实时重建+规划闭环
- **适配性分析**：
  - 核心算法（NBV + 3DGS）可直接用于你的场景
  - 需要将探索 agent 适配到你的移动机械臂
  - 视点规划逻辑可与 RL 训练结合
- **可直接尝试的复现步骤**：
  ```bash
  # ⚠️ 需要先验证仓库是否存在
  # 搜索: https://github.com/search?q=ActiveGAMER
  # 或: https://github.com/JingwenWang95/ActiveGAMER
  ```
- **关键缺失与注意事项**：⚠️ 仓库状态需验证，可能仅提供论文

---

### 13. GNB-NBV（Gaussian Splatting NBV Planning）

- **是否开源**：⚠️ 需验证 — 在 arXiv 搜索 "GNB-NBV" 或 "gaussian splatting next best view"
- **最新版本/更新时间**：2024 年
- **关联度评分**：★★★★★ — 直接结合 3DGS 与 NBV 规划用于机器人扫描
- **移动底盘支持**：⚠️ 设计用于臂扫描，可扩展到移动操作
- **仿真平台**：未确认
- **能否直接导入你的 URDF**：未提及
- **核心方法**：
  - 使用 3DGS 估计场景覆盖率和不确定性
  - 基于信息增益指标评估候选视点
  - 选择信息增益最大的视点作为 NBV
- **适配性分析**：
  - NBV 算法核心可直接复用
  - 需要将固定臂扫描扩展到移动底盘+臂
  - 信息增益计算可作为 RL reward
- **可直接尝试的复现步骤**：⚠️ 需要先确认仓库可用性
- **关键缺失与注意事项**：⚠️ 可能仅论文，代码需验证

---

### 14. MoManipVLA

- **是否开源**：⚠️ 需验证 — 在 GitHub/arXiv 搜索 "MoManipVLA"
- **最新版本/更新时间**：2024 年
- **关联度评分**：★★★★☆ — 专门为移动操作设计的 VLA，直接匹配你的硬件配置
- **移动底盘支持**：✅ 核心设计就是移动操作（底座+臂）
- **仿真平台**：未确认
- **能否直接导入你的 URDF**：未提及
- **核心方法**：
  - 将预训练的固定臂 VLA 模型迁移到移动操作场景
  - 解决导航和操作在单一策略中的融合问题
  - 适配现有 VLA 模型到移动平台
- **适配性分析**：
  - 如果开源，可能是最直接适配你硬件的 VLA 方案
  - 移动底盘+臂的 action space 设计可直接参考
- **可直接尝试的复现步骤**：⚠️ 需要先确认仓库可用性
- **关键缺失与注意事项**：⚠️ 开源状态不确定

---

### 15. DovSG

- **是否开源**：⚠️ 需验证 — 在 GitHub/arXiv 搜索 "DovSG"
- **最新版本/更新时间**：2024 年
- **关联度评分**：★★★★☆ — 场景图引导的操作，与 3D 重建密切相关
- **移动底盘支持**：✅ 面向室内移动操作
- **仿真平台**：未确认
- **能否直接导入你的 URDF**：未提及
- **核心方法**：
  - 从 RGB-D 观测构建 Door and Object-centric Visual Scene Graph
  - 节点=物体，边=空间/语义关系
  - 基于场景图引导机器人操作
- **适配性分析**：
  - 场景图表征可用于识别未扫描区域
  - 可指导主动扫描策略
  - 与 3DGS 重建互补
- **可直接尝试的复现步骤**：⚠️ 需要先确认仓库可用性
- **关键缺失与注意事项**：⚠️ 开源状态不确定

---

### 16. SpatialVLA

- **是否开源**：⚠️ 需验证 — 搜索 "SpatialVLA"
- **最新版本/更新时间**：2025 年
- **关联度评分**：★★★☆☆ — 空间推理能力强，对 3D 扫描有潜在价值
- **移动底盘支持**：未确认
- **仿真平台**：未确认
- **核心方法**：空间感知 VLA 模型，强调 3D 环境的空间推理
- **关键缺失与注意事项**：⚠️ 信息有限，需自行验证

---

### 17. CogACT

- **是否开源**：⚠️ 需验证 — 搜索 "CogACT"
- **最新版本/更新时间**：2024 年
- **关联度评分**：★★☆☆☆ — 认知-动作统一模型，与主动扫描关联有限
- **移动底盘支持**：未确认
- **核心方法**：在单一模型中统一认知理解和动作生成
- **关键缺失与注意事项**：⚠️ 信息有限

---

### 18. Robo-GS

- **是否开源**：⚠️ 需验证 — 搜索 "Robo-GS"
- **最新版本/更新时间**：2024 年
- **关联度评分**：★★★★☆ — 机器人 3DGS 主动场景重建
- **移动底盘支持**：未确认
- **核心方法**：使用 3DGS 进行机器人场景重建，结合主动视点规划优化数据采集
- **关键缺失与注意事项**：⚠️ 信息有限，需自行验证

---

## 三、可立即上手实践的开源方案（推荐矩阵）

| 优先级 | 项目 | 对你设备的适配度 | 移植工作量 | 实践入口 |
|:---:|:---|:---|:---:|:---|
| **1** | **SplaTAM（重建后端）** | **高** | **低** | `git clone https://github.com/spla-tam/SplaTAM` → 用双目 RGB-D 数据运行重建 → 集成到 Isaac Lab 作为 reward 模块 |
| **2** | **OpenVLA（VLA 微调）** | **中** | **中** | `git clone https://github.com/openvla/openvla` → 扩展 action space → 采集 demonstration 微调 |
| **3** | **Octo（通用策略微调）** | **中** | **中** | `git clone https://github.com/octo-models/octo` → 微调到移动操作 embodiment |
| **4** | **RSL-RL（RL 算法后端）** | **中** | **低** | `pip install rsl_rl` → 配合 Isaac Lab 环境使用 |
| **5** | **legged_gym（底盘运动 RL）** | **中** | **低** | `git clone https://github.com/leggedrobotics/legged_gym` → 参考轮式底盘 RL 训练 |
| **6** | **Diffusion Policy（扫描轨迹生成）** | **中** | **中** | `git clone https://github.com/real-stanford/diffusion_policy` → 将扫描轨迹建模为扩散过程 |
| **7** | **OK-Robot（感知流水线参考）** | **中** | **高** | `https://ok-robot.github.io` → 参考其模块化感知+规划架构 |
| **8** | **RoboCasa（任务设计参考）** | **低** | **高** | `git clone https://github.com/robocasa/robocasa` → 参考移动操作任务设计 |

---

## 四、总结

当前移动机械臂 sim2real/VLA 领域的现状是：**基础设施成熟但上层应用碎片化**。NVIDIA Isaac Lab 已经提供了高质量的 GPU 并行仿真和 RL 训练框架，支持 URDF 导入和移动操作环境配置；VLA 模型（OpenVLA、Octo）开源可用且支持微调到新 embodiment；3DGS SLAM（SplaTAM）提供了实时稠密重建能力。然而，**真正将"移动底盘+机械臂+主动3D重建"三者结合的端到端开源项目几乎不存在**——ActiveGAMER 和 GNB-NBV 最接近但开源状态不确定。

**空白所在**：缺少一个在 Isaac Lab 中训练"移动机械臂主动扫描策略"的完整开源环境和 reward 设计。

**建议优先跟进路线**：
1. **短期（1-2 周）**：在 Isaac Lab 中导入 Ranger URDF，实现基本的底盘+臂联合控制环境
2. **中期（1-2 月）**：集成 SplaTAM 作为重建模块，设计基于覆盖率/信息增益的 reward function，训练 NBV 策略
3. **长期（3-6 月）**：探索 VLA 微调（OpenVLA/Octo），将语言指令与扫描策略结合，实现"告诉我补测哪里"的交互式扫描

---

## 五、我可以立即开始尝试的步骤

### 步骤 1：搭建 Isaac Lab 环境（第 1 天）
```bash
# 安装 Isaac Sim（需要 NVIDIA GPU，RTX 2070+）
# 参考: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html

# 克隆 Isaac Lab
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./isaaclab.sh --install

# 验证安装
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

### 步骤 2：导入 Ranger URDF 并验证（第 2-3 天）
```bash
# 将你的 Ranger URDF 放入 Isaac Lab 的 assets 目录
# 参考 URDF 导入教程:
# https://isaac-sim.github.io/IsaacLab/main/source/overview/showroom/import_new_asset.html

# 创建简单的站立/运动测试环境
# 参考: source/isaaclab_tasks/ 下的 locomotion 环境
```

### 步骤 3：创建移动操作基础环境（第 1 周）
```bash
# 基于 Isaac Lab 的 mobile manipulation 示例
# 创建 RangerArmReachEnv: 底盘 velocity control + 臂 joint position control
# action space: [base_vx, base_vy, base_wz, arm_j1...jN, gripper]
# observation: [joint_pos, joint_vel, base_pose, camera_rgb, camera_depth]
```

### 步骤 4：集成 SplaTAM 重建模块（第 2 周）
```bash
git clone https://github.com/spla-tam/SplaTAM.git
# 将 SplaTAM 集成为 Isaac Lab 环境的外部模块
# 在每个 episode 结束时计算重建覆盖率作为 reward
```

### 步骤 5：设计 NBV Reward 并训练（第 3-4 周）
```python
# reward 设计示例:
# r = α * Δcoverage + β * information_gain - γ * movement_cost
# 
# Δcoverage: 本步新增的 3DGS 覆盖体素/高斯数量
# information_gain: 从当前视点获得的新信息量（基于 SplaTAM 的不确定性）
# movement_cost: 底盘和臂的运动代价（鼓励平滑轨迹）

# 使用 RSL-RL 的 PPO 训练
python scripts/rsl_rl/train.py --task=RangerArmNBV-v0
```

### 步骤 6（可选）：VLA 微调路线（第 5-8 周）
```bash
# 如果 RL 训练效果不理想，尝试 VLA 微调路线:
git clone https://github.com/openvla/openvla.git

# 1. 在 Isaac Lab 中采集 demonstration（专家策略或手动遥操作）
# 2. 将 demonstration 转换为 OpenVLA 的训练格式
# 3. 微调 action space 为底盘+臂联合控制
# 4. 语言指令设计: "scan the front of the object", "move to the left side"
```

### 关键资源清单

| 资源 | 链接 |
|:---|:---|
| Isaac Lab 文档 | https://isaac-sim.github.io/IsaacLab |
| Isaac Lab GitHub | https://github.com/isaac-sim/IsaacLab |
| SplaTAM GitHub | https://github.com/spla-tam/SplaTAM |
| OpenVLA GitHub | https://github.com/openvla/openvla |
| Octo GitHub | https://github.com/octo-models/octo |
| RSL-RL GitHub | https://github.com/leggedrobotics/rsl_rl |
| Open X-Embodiment 数据集 | https://robotics-transformer-x.github.io |

---

> **免责声明**：本文档中标注 `⚠️ 需验证` 的项目信息基于训练知识整理，GitHub 链接、星标数据、开源状态等可能已变化，请在实际使用前自行验证。所有"未确认"的信息请通过 GitHub 搜索、arXiv 搜索、Papers With Code 等平台核实。
