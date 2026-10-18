# 四足机器人建图定位、导航、运动控制接口与 3D Gaussian Splatting 技术调研

**资料核查日期：2026-10-10**  
**范围：** Unitree Go1/Go2/B2、Boston Dynamics Spot、ANYbotics ANYmal、DEEP Robotics Lite3/M20 Pro/X30 Pro、小米 CyberDog、Ghost Robotics Vision 60，以及可移植的通用开源算法。

> **证据口径（非常重要）**
>
> - **A｜厂商商用系统**：厂商官方明确说明的原生定位/导航功能；不意味着所有内部算法开源。
> - **B｜厂商官方开源/适配**：厂商发布算法代码或机器人部署教程；**不等于该算法是出厂固件的内部实现**。
> - **C｜论文实机验证**：高校/研究机构在对应四足机型上实际部署或采集；不等于厂商原装。
> - **D｜第三方工程**：开发者公开的机型适配、ROS 导航配置；不等于厂家采用。
> - **E｜通用候选**：代码和论文存在，但尚未在所讨论四足机型上找到可核实的公开实机证据。
>
> “主流”指公开工程生态与研究中反复使用的路线，不是各厂家量产装机率排名。对闭源厂商，不推测其内部使用 FAST-LIO、A*、MPC 等具体实现。

---

# 1. 四足机器人的主流建图、定位与导航算法

## 1.1 先看核心结论

| 使用条件 | 建图/定位首选候选 | 导航与规划首选候选 | 典型参考平台 | 工程注意 |
|---|---|---|---|---|
| 室内平地、走廊、厂房 | RTAB-Map（RGB-D/3D LiDAR）、SLAM Toolbox（2D）；或 LIO + 全局定位 | Nav2 + NavFn（Dijkstra/A*）+ DWB/MPPI | Go2 第三方、CyberDog 官方开源栈 | 2D 导航不自动解决台阶与落脚问题 |
| 室外园区/工业巡检 | FAST-LIO2、Point-LIO、LIO-SAM | 2D/2.5D Costmap + Nav2，局部 DWB/MPPI | Unitree 雷达生态、Vision 60 研究 | 里程计与“在已知地图重定位”必须区分 |
| 长期重复巡检 | LIO + 回环/地图管理/重定位；或厂商原生系统 | 拓扑路线/任务点 + 局部避障 | Spot GraphNav、X30 Pro | 需要地图更新、失锁恢复、任务级 API |
| 草地、坡地、碎石、野外 | CompSLAM、LIO + 腿部状态估计 | Elevation Mapping + Traversability + ArtPlanner | ANYmal 科研 | 必须考虑坡度、可落脚性、机身稳定性 |
| 激光 + 图像联合精细建图 | FAST-LIVO2、Gaussian-LIC2（研究） | 几何安全地图与导航器分离 | Lite3 Venture 官方适配 | 时间同步、相机外参、GPU 与重定位 |
| 语义高斯地图导航 | 传统 SLAM/里程计提供位姿 + 3DGS | GS 语义可通行性 → ESDF/Costmap → MPPI | Ghost Vision 60（Splatblox） | GS 不能直接等同于可靠占据/碰撞地图 |

**推荐优先级：**
1. **最常见的通用激光里程计/建图候选**：FAST-LIO2、Point-LIO、LIO-SAM。
2. **视觉或多传感器融合**：RTAB-Map、FAST-LIVO2、CompSLAM；相机路线另有 ORB-SLAM3、MIVINS/MILOC。
3. **普通自主导航**：ROS 2 Nav2 框架；全局规划 NavFn/Smac，局部控制 DWB/MPPI/RPP。
4. **四足复杂地形特有重点**：高程图、可通行性估计、机身/足端约束；可参考 ANYmal 的 ArtPlanner。
5. **商业完整产品**：Spot GraphNav 是已明确命名的原生定位导航系统；其他厂家很多底层算法未公开。

## 1.2 常见 SLAM/定位算法对照

| 算法/系统 | 核心传感器 | 核心方法与输出 | 回环/重定位能力 | 可核实平台与证据 | 原始链接 |
|---|---|---|---|---|---|
| **FAST-LIO2** | 3D LiDAR + IMU | 迭代 EKF（iEKF）、原始点云直接匹配、ikd-Tree 增量地图；输出局部连续里程计与点云地图 | **原版不等同于完整全局 SLAM**；通常另接回环、地图匹配、重定位 | 通用开源；可作为 Go2/B2 的二次开发候选，不能称为宇树出厂算法（E） | [官方仓库](https://github.com/hku-mars/FAST_LIO) |
| **Point-LIO** | 3D LiDAR + IMU | 高带宽点级激光惯性状态估计，面向强振动、激烈运动；局部建图 | 需要视项目另补长期地图定位/回环 | **宇树官方适配 L1/L2 LiDAR**（B；是雷达生态适配，不是出厂 SLAM 证明） | [算法](https://github.com/hku-mars/Point-LIO) · [宇树适配](https://github.com/unitreerobotics/point_lio_unilidar) |
| **LIO-SAM** | 3D LiDAR + IMU（可扩展 GPS） | 因子图平滑优化、激光与惯性融合、回环、全局点云地图 | 有回环与图优化；已有图地图上的在线重定位仍需具体部署 | **Ghost Vision 60：Splatblox 论文明确使用**（C） | [代码](https://github.com/TixiaoShan/LIO-SAM) · [实机论文](https://arxiv.org/html/2511.18525) |
| **FAST-LIVO2** | 3D LiDAR + IMU + RGB Camera | 紧耦合激光-惯性-视觉直接里程计；增强带纹理场景的位姿和重建 | 重点是 LIVO 前端，并非天然包含完善长期重定位 | **云深处 Lite3 Venture 官方硬件扩展+实机复现**（B） | [原算法](https://github.com/hku-mars/FAST-LIVO2) · [Lite3 工程](https://github.com/DeepRoboticsLab/fast-livo2-deep-robotics) |
| **RTAB-Map** | RGB-D/双目/3D LiDAR，可配 IMU | 图优化、回环检测、视觉/激光地图、ROS 2 集成 | 支持地图回环和定位模式（具体取决于配置） | **Unitree Go2 + Nav2** 的第三方公开工程（D） | [ROS 代码](https://github.com/introlab/rtabmap_ros) · [Go2 工程](https://github.com/h-naderi/unitree-go2-slam-nav2) |
| **SLAM Toolbox** | 2D LiDAR + 里程计 | 2D 激光位姿图 SLAM、回环、序列化地图与定位模式 | 支持 2D 地图定位及继续建图 | Go2 的第三方 ROS 2 适配；**仅限满足 2D 地面假设的区域**（D） | [代码](https://github.com/SteveMacenski/slam_toolbox) |
| **ORB-SLAM3** | 单目/双目/RGB-D，可结合 IMU | 特征点视觉/视觉惯性 SLAM，多地图与回环 | 有视觉重定位和回环 | 通用视觉候选；**不能据此声称某厂家原生采用**（E） | [代码](https://github.com/UZ-SLAMLab/ORB_SLAM3) |
| **CompSLAM** | LiDAR + IMU + 可见光/热成像 | 激光分支基于 LOAM 并融合 IMU；视觉-热成像分支基于 ROVIO，多模态互补 | 面向恶劣、退化地下环境，含多层状态估计与建图 | **ETH/CERBERUS 的 ANYmal 等机器人研究**（C） | [官方科研代码](https://github.com/leggedrobotics/compslam_subt) |
| **MIVINS + MILOC** | 视觉相机 + IMU（三目视觉地图） | MIVINS 提供视觉惯性位姿；MILOC 用图像与位姿建图，导航阶段对已建地图重定位 | MILOC 明确提供视觉地图定位 | **小米 CyberDog 官方开源栈**（B） | [MIVINS](https://github.com/MiRoboticsLab/cyberdog_mivins) · [MILOC](https://github.com/MiRoboticsLab/cyberdog_miloc) |
| **Lightning-LM** | 3D LiDAR + IMU | 激光建图、回环、保存点云、已知地图定位 | 仓库明确区分 Mapping 与 Localization | **云深处 M20 Pro / Lite3 EDU 官方部署工程**（B） | [代码与硬件说明](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics) |
| **GraphNav** | Spot 机载里程计、视觉/几何特征；可选增强 LiDAR | waypoint/edge 拓扑地图、相对 waypoint 位姿定位、记录与重放路线 | 支持在已有地图内定位；特征贫乏时可能失锁 | **Spot 原生商业系统**（A）；未公开等价于哪一个学术 SLAM 的完整实现 | [技术说明](https://dev.bostondynamics.com/docs/concepts/autonomy/graphnav_tech_summary.html) · [定位说明](https://dev.bostondynamics.com/docs/concepts/autonomy/localization.html) |

### 1.2.1 必须分清三种“定位”

| 术语 | 典型输出 | 是否长期不漂移 | 代表组件 |
|---|---|---|---|
| 局部里程计（Odometry/LIO/VIO） | 连续、平滑的相对运动 `odom -> base_link` | 否，时间长了可能漂移 | FAST-LIO2、Point-LIO、FAST-LIVO2 前端 |
| 建图 SLAM（Mapping + Loop Closure） | 地图、轨迹、回环校正 | 可减少全局漂移，但校正会造成全局坐标跳变 | LIO-SAM、RTAB-Map、CompSLAM 的相应部分 |
| 已知地图定位（Localization/Relocalization） | 机器人当前相对保存地图的位姿 `map -> odom` 或相应地图坐标 | 依赖地图与观测质量 | Spot GraphNav、MILOC、RTAB-Map 定位模式、Lightning-LM 定位模式 |

- **不要**把 FAST-LIO2 的实时 `Odometry` 直接视为长周期巡检可用的全球坐标。
- 导航控制需要平滑低延迟 `odom`；长期任务需要独立的 `map` 对齐/回环/重定位。
- 四足还可加入**腿部里程计**（关节编码器 + 足端接触 + IMU）。脚滑、软地面、抬腿时接触错误会造成误差；它一般是状态估计融合源，而不是单独替代 LiDAR/视觉 SLAM。可参考 [ETH Holistic Fusion](https://github.com/leggedrobotics/holistic_fusion)。

## 1.3 具体厂家/型号：能证明什么、不能证明什么

| 厂家/平台 | 建图定位：有证据的算法或系统 | 导航：有证据的算法或系统 | 证据级别 | 限制 |
|---|---|---|---|---|
| **Unitree Go2** | 官方 L1/L2 雷达 **Point-LIO 适配**；第三方 **RTAB-Map** | 第三方 Go2 Nav2 配置明确是 **NavFn（`use_astar: false` = Dijkstra）+ DWB** | B + D | 并不能推导 Go2 固件内部采用 Point-LIO、Dijkstra、DWB |
| **Unitree B2** | 官方 ROS 2/SDK2 支持状态与控制；未找到足以确认原生 SLAM 算法名称的资料 | 厂家功能与第三方开发均可能；具体原生全局/局部规划器未证实 | A（接口）/ 未公开（算法） | 不能把 Go2 第三方配置当 B2 原厂配置 |
| **Spot** | **GraphNav** 官方建图与 waypoint 定位 | **GraphNav** 记录路线、导航到 waypoint、局部绕障 | A | 官方未公开完整内部搜索/优化器名称 |
| **ANYmal** | **CompSLAM**（ETH 研究）、腿部/IMU 状态估计 | **ArtPlanner** + Elevation Mapping + Traversability（ETH 研究） | C | 学术代码不等于所有商用 ANYmal 的固件 |
| **Lite3 Venture** | **FAST-LIVO2** 厂商官方复现 | 厂商运动能力；该复现工程本身不能证明已集成完整 Nav2 | B | 官方适配文档指明 **Lite3 Venture** 和额外传感器/计算单元 |
| **Lite3 EDU / M20 Pro** | **Lightning-LM** 厂商仓库注明支持两款平台 | 可在定位地图基础上开发导航，具体原厂导航规划器不在该仓库披露 | B | 只证明官方项目的部署范围 |
| **X30 Pro** | 具备官方建图/导航产品功能，底层 SLAM 名称未核实 | **RobotServer SDK** 提供地图导航任务启动、取消、状态查询 | A（产品接口） | SDK 不披露 A*、DWB、MPC 等内部算法 |
| **CyberDog** | 官方 **MIVINS、MILOC、LaserSLAM** | 官方 ROS 2 Nav2 配置中 **NavFn（Dijkstra）+ DWB** 等多个插件 | B | 指开源 CyberDog 软件分支；CyberDog 2 的具体量产配置不可一概而论 |
| **Ghost Vision 60** | **LIO-SAM**（Splatblox 研究） | **Nav2 + MPPI + 语义高斯可通行性 ESDF**（Splatblox） | C | 不是 Ghost 原厂固件公布的算法 |

**直接核对算法配置的两处证据：**
- [Go2 第三方 `nav2_params.yaml`](https://github.com/h-naderi/unitree-go2-slam-nav2/blob/main/go2_slam_nav/config/nav2_params.yaml)：`NavfnPlanner`、`use_astar: false`、`dwb_core::DWBLocalPlanner`。
- [CyberDog `nav2_params.yaml`](https://github.com/MiRoboticsLab/cyberdog_tracking_base/blob/rolling/mcr_bringup/params/nav2_params.yaml)：`NavFn`/`GridBased` 的 `use_astar: false`，以及 `FollowPath` 的 DWB；文件还配置了额外自定义规划/导航插件。

## 1.4 主流导航算法：按层拆开看

**Nav2 是 ROS 2 导航框架，不是单个算法；Costmap、Planner、Controller、Behavior Tree 是不同模块。**

| 层级 | 具体算法/框架 | 工作内容 | 适用四足条件 | 可核实平台/资料 |
|---|---|---|---|---|
| 全局路径规划 | **Dijkstra（NavFn）** | 二维代价地图中求低成本路径 | 平面为主的巡检 | [Go2 配置](https://github.com/h-naderi/unitree-go2-slam-nav2/blob/main/go2_slam_nav/config/nav2_params.yaml)、[CyberDog 配置](https://github.com/MiRoboticsLab/cyberdog_tracking_base/blob/rolling/mcr_bringup/params/nav2_params.yaml) |
| 全局路径规划 | **A* / Smac 2D A*** | 代价感知的栅格图搜索 | 平地、全向/近全向四足 | [Nav2 Smac 官方文档](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/) |
| 全局路径规划 | **Hybrid A* / State Lattice** | 将机身朝向、轨迹可执行性纳入搜索 | 狭窄通道、朝向受限、需要机身形状规划 | [Nav2 Smac 官方文档](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/) |
| 全局路径规划 | **Theta*** | 网格视线捷径、任意角度路径 | 普通二维导航替代方案 | [Nav2 配置总览](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/) |
| 局部跟踪/避障 | **DWB（Dynamic Window Based）** | 采样候选速度，前向模拟并按障碍/路径/终点代价评分 | 基线方案、对调参依赖明显 | Go2、CyberDog 上述开源配置；[官方文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/) |
| 局部跟踪/避障 | **MPPI（Model Predictive Path Integral）** | 预测时域内采样速度序列、优化多项代价 | 全向机身运动、较复杂动态避障 | **Vision 60 + Splatblox**；[Nav2 MPPI](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/) |
| 路径跟踪 | **RPP（Regulated Pure Pursuit）** | 前视点追踪，并按曲率/障碍距离调速 | 一般规则道路和相对简单的路径跟踪 | [Nav2 RPP](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)；不是已核实的特定厂家出厂算法 |
| 复杂地形地图 | **Elevation Mapping / TSDF / ESDF** | 高程、坡度、障碍空间和碰撞距离 | 碎石、斜坡、楼梯、复杂地形 | [ANYmal Elevation Mapping CuPy](https://github.com/leggedrobotics/elevation_mapping_cupy)、[Nvblox](https://github.com/nvidia-isaac/nvblox) |
| 复杂地形评估 | **Traversability Estimation** | 根据高程、粗糙度、局部几何或学习代价评估可通行性 | 野外、非结构化地形 | [ETH 代码](https://github.com/leggedrobotics/traversability_estimation) |
| 复杂地形路线 | **ArtPlanner（PRM 类采样规划 + 学习式运动代价）** | 2.5D 高程图中规划能由腿式机器人执行的路线 | ANYmal 野外、起伏地形 | [ETH 官方科研代码](https://github.com/leggedrobotics/art_planner) |
| 商业任务级导航 | **GraphNav** | 拓扑 waypoint/edge 地图、初始化定位、路线重放和障碍处理 | 工业巡检与固定路线 | [Spot 官方技术说明](https://dev.bostondynamics.com/docs/concepts/autonomy/graphnav_tech_summary.html) |

### 1.4.1 “能避障”不等于“能走复杂地形”

四足从平地走到岩石、台阶、泥地时，需要在普通 2D `Costmap` 之外考虑：

1. **地形几何**：高度差、坡度、粗糙度、狭窄空间、头部/腹部离地间隙。
2. **可通行性**：是否可踩、是否会沉陷/打滑；视觉语义可辅助判断草丛与硬障碍物。
3. **动力学与落脚**：机身姿态、支持多边形、可落脚区域、足端接触稳定性。
4. **运动控制**：路径级输出不等于每条腿的关节角。实际落脚点/接触力可能由 MPC、WBC、优化或强化学习步态策略完成。

因此推荐将**二维路线导航**和**复杂地形运动控制**分层设计。普通 `Nav2 + cmd_vel` 不能保证机器狗安全上台阶。

## 1.5 数据与坐标系接口

推荐 ROS 2 中至少具备：

| 数据 | 常见类型/TF | 谁提供 | 给谁用 |
|---|---|---|---|
| LiDAR 点云 | `sensor_msgs/msg/PointCloud2` | 雷达驱动 | LIO、建图、障碍检测 |
| RGB/深度图 | `sensor_msgs/msg/Image` + `CameraInfo` | 相机驱动 | 视觉 SLAM、3DGS、语义 |
| IMU | `sensor_msgs/msg/Imu` | IMU 驱动 | LIO/VIO/状态估计 |
| 连续机身里程计 | `nav_msgs/msg/Odometry`、`odom -> base_link` | LIO/融合估计器 | 控制器、Nav2 |
| 全局地图定位 | `map -> odom` | 地图定位/回环系统 | Nav2 全局坐标 |
| 静态外参 | `base_link -> lidar/camera/imu` | URDF/TF 静态发布器 | 所有多传感器模块 |
| 二维地图 | `nav_msgs/msg/OccupancyGrid` | SLAM/地图服务器 | Nav2 Costmap |
| 高程图/三维距离场 | `grid_map_msgs/GridMap` 或 TSDF/ESDF 自定义表示 | 地形/碰撞建图 | 复杂地形规划 |
| 导航目标 | `NavigateToPose` Action | 任务模块 | Nav2 |
| 机身速度控制 | `geometry_msgs/msg/Twist` 或 `TwistStamped` | Nav2/上位机 | 机器人速度桥接器 |

`map -> odom -> base_link` 的语义必须一致；不能多个节点抢发同一 TF。IMU 与 LiDAR 的时间戳、点云逐点时间、相机触发时刻需要使用一致时间基准。

---

# 2. 四足机器人一般使用什么运动控制接口？

## 2.1 控制层级：绝大多数导航二次开发只需要“机身速度”

| 层级 | 上层给机器人的指令 | 典型接口字段 | 具体用途 | 是否要自己实现四足步态 |
|---|---|---|---|---|
| **L0：任务级** | 到达目标点、巡检路线、取消任务 | 目标 `Pose`、waypoint ID、route ID、task ID | 商业导航/任务调度 | 否 |
| **L1：机身速度级（最常用）** | 前进、横移、旋转 | `vx`、`vy`、`wz` | ROS Nav2 对接、遥控、路径跟踪 | 否：厂商内部 gait controller 负责 |
| **L2：机身姿态/步态级** | 站起、趴下、改变姿态/步态 | roll、pitch、yaw、身高、gait mode | 上坡/特殊动作/状态切换 | 一般不用 |
| **L3：关节级** | 每个关节位置/速度/力矩目标 | `q_des`、`dq_des`、`tau_ff`、`Kp`、`Kd` | 自研 WBC/MPC/RL 步态、力控、落脚 | **是** |
| **L4：电机/驱动级** | 更底层电流/总线命令 | 厂商驱动协议 | 电机与控制器研发 | 是；安全风险最高 |

机身标准速度通常按 ROS 身体坐标约定：`x` 前、`y` 左、`z` 上；`angular.z` 为偏航角速度。**是否允许侧移**取决于机型和当前步态，不能因为 SDK 有 `vy` 字段就断定任何工况都能横移。

### 2.1.1 典型 Nav2 → 四足控制闭环

```text
Goal / Patrol Task
       │
       ▼
Global Planner（Dijkstra/A* / ArtPlanner / 厂商路线）
       │  nav_msgs/Path
       ▼
Local Controller（DWB / MPPI / RPP）
       │  /cmd_vel：vx, vy, wz
       ▼
Robot Adapter / Velocity Bridge
       ├── 速度与加速度限幅
       ├── Twist/TwistStamped 类型处理
       ├── 控制权限 / 步态状态检查
       ├── 看门狗、失联停机、急停
       └── frame 与时间戳检查
       │  厂商 SDK / DDS / RPC
       ▼
Onboard Locomotion Controller
       │  机身运动、步态、关节/接触控制
       ▼
Quadruped Motors
       │
       └── IMU + joints + contact + odom 反馈上游
```

**关键：`/cmd_vel` 是 ROS 侧常用抽象接口，不是所有厂商原生接口。** 常见做法是用桥接节点转换为厂家 SDK 命令。

## 2.2 各厂商接口实证

| 平台 | 任务级接口 | 速度/姿态接口 | 关节级接口 | 官方资料 |
|---|---|---|---|---|
| **Unitree Go2** | 需自行集成 Nav2，或使用该版本提供的上层服务 | SDK2 **`SportClient.Move(vx, vy, vyaw)`**、`StopMove()`、`StandUp()`、`StandDown()`、`Euler()` 等 | ROS 2/DDS `/lowcmd`：`LowCmd` 含 `MotorCmd` | [Go2 SportClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp) · [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) |
| **Unitree B2** | 可按 SDK/任务模块开发 | B2 另有对应 `SportClient` 实现与示例，**不要直接假设与 Go2 固件行为完全一致** | 低层接口是否启用依机型和模式 | [SDK2 Python B2 示例](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/b2/high_level/b2_sport_client.py) |
| **Spot** | **GraphNav** `NavigateTo` / `NavigateRoute`，任务与地图 API | `RobotCommandBuilder.synchro_velocity_command(v_x, v_y, v_rot)` | 不等于开放任意关节力矩接口；按官方支持范围使用 | [RobotCommand API](https://dev.bostondynamics.com/python/bosdyn-client/src/bosdyn/client/robot_command.html) · [GraphNav Service](https://github.com/boston-dynamics/spot-sdk/blob/master/docs/concepts/autonomy/graphnav_service.md) |
| **云深处 X30 Pro** | **RobotServer SDK**：实时状态、导航任务发起/取消/结果查询 | 速度接口需按具体控制 SDK/版本确认，不能用任务接口替代 | 不从导航任务 SDK 推定可访问关节级 | [RobotServer 中文说明](https://github.com/DeepRoboticsLab/robotserver_sdk/blob/main/README.zh-CN.md) |
| **ANYmal（科研）** | ROS 路径/目标经规划层、任务层 | 研究平台有机身轨迹/控制器接口，具体软件栈视项目 | 存在 WBC、MPC、学习式控制科研系统，但不是统一公开商品 API | [ArtPlanner ROS 输出](https://github.com/leggedrobotics/art_planner) |
| **CyberDog** | Navigation2 任务栈 | ROS 2 控制与速度适配节点 | 官方还公开了运动控制工程，具体设备/分支适配需核对 | [cyberdog_nav2](https://github.com/MiRoboticsLab/cyberdog_nav2) · [cyberdog_locomotion](https://github.com/MiRoboticsLab/cyberdog_locomotion) |

## 2.3 宇树 Go2：可直接落地的 ROS 2 / SDK 接口

### 高层 SDK2

Go2 C++ 官方接口包含：

```cpp
int32_t Move(float vx, float vy, float vyaw);
int32_t StopMove();
int32_t StandUp();
int32_t StandDown();
int32_t BalanceStand();
int32_t Euler(float roll, float pitch, float yaw);
```

Python 也有 `SportClient.Move(vx, vy, vyaw)`，并通过内部 RPC/DDS 协议向机器人发送请求。

- [C++ 头文件](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp)
- [Python 接口实现](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/sport/sport_client.py)
- [Python 高层控制示例](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/go2/high_level/go2_sport_client.py)

### ROS 2 / DDS 主题

| Topic | 消息类型 | 作用 | 说明 |
|---|---|---|---|
| `/api/sport/request` | `unitree_api/msg/Request` | 发送站立、运动、姿态等高层请求 | 不是直接发布 `Twist`；可用官方 `SportClient` 组装 Request |
| `/sportmodestate` 或 `lf/sportmodestate` | `unitree_go/msg/SportModeState` | 机身速度/姿态、步态、足端状态 | 命名/频率依产品与配置 |
| `/lowcmd` | `unitree_go/msg/LowCmd` | 关节低层控制指令 | 高低层权限需明确管理 |
| `/lowstate` 或 `lf/lowstate` | `unitree_go/msg/LowState` | 关节状态、IMU、足端力、电源信息 | 用于监控与闭环 |
| `/utlidar/cloud` | 点云话题 | 原配 LiDAR 数据（相关配置） | Topic 是否存在取决于雷达、驱动、产品版本 |

官方 `MotorCmd` 的主要字段为：

```text
mode   电机模式
q      目标关节位置（rad）
dq     目标关节速度（rad/s）
tau    前馈目标力矩（N·m）
kp     位置增益
kd     速度增益
```

常见关节 PD+前馈力矩解释为：`tau_cmd ≈ kp*(q_des-q) + kd*(dq_des-dq) + tau_ff`。**此式仅用于理解接口含义，具体内环由固件实现，不能凭此假设所有产品完全相同。**

官方 `LowCmd` 中有 `MotorCmd[20]` 的消息数组定义，但**数组容量不等于机器狗实际有效驱动关节数**，应按具体机型 ID 映射和安全规范设置。

源：[Unitree ROS 2 官方 README：状态、Sport、LowCmd](https://github.com/unitreerobotics/unitree_ros2)。

### Nav2 接 Go2 的实际桥接要点

1. Nav2 订阅 `Odometry`、TF、Costmap，输出 `/cmd_vel`。
2. `go2_cmd_bridge` 接收 `Twist` 或 `TwistStamped`，将 `linear.x`/`linear.y`/`angular.z` 转为 `SportClient.Move(vx, vy, vyaw)`，或者构造 `/api/sport/request`。
3. 先完成**控制权移交**，避免厂家运动服务与自定义低层控制同时下发冲突指令。
4. 桥接器必须有**消息超时归零/停车、速度限幅、急停、通讯丢失处理**。
5. 要区分 `map`（全局跳变可接受）与 `odom`（控制必须平滑）的 TF 语义。
6. **ROS 2 版本差异**：Nav2 Kilted 及更新版默认倾向 `TwistStamped`；Jazzy 及更旧版本默认是 `Twist`，最终以 `enable_stamped_cmd_vel` 和所用版本为准。[官方迁移说明](https://docs.nav2.org/rolling/configuration_and_development/migration_guides/jazzy/Jazzy/)

## 2.4 Spot 和 X30：为什么任务级 API 与速度级不同？

**Spot：**
- `RobotCommand` 负责运动指令，`synchro_velocity_command` 使用机身坐标速度；命令需要有效结束时刻，并遵循租约（Lease）、急停与状态限制。
- `GraphNav` 已经负责地图中的 waypoint/route 导航。调用 `NavigateTo` 与“自己写 Nav2 每周期发送速度”是不同开发路线。
- 不能因其支持 RobotCommand，就假定其低层每条腿的任意关节力矩控制向外部开放。

**云深处 X30 Pro：**
- `RobotServer SDK` 公开了 `1002` 状态查询、`1003` 下发导航任务、`1004` 取消任务、`1007` 查询任务状态等服务。
- 这是**任务层**接口，不是公开了内部 LIO/SLAM/全局规划器算法的证据。
- 源：[X30 Pro RobotServer 官方中文说明](https://github.com/DeepRoboticsLab/robotserver_sdk/blob/main/README.zh-CN.md)。

---

# 3. 四足机器人上的 3D Gaussian Splatting（3DGS）与 SLAM 结合案例

## 3.1 先辨别五种组合方式

| 类型 | 3DGS 在系统中的角色 | SLAM 的角色 | 是否意味着“3DGS 直接定位” |
|---|---|---|---|
| **A. SLAM → 3DGS 重建** | 用已有精确相机位姿重建可视化/数字孪生 | 提供轨迹和尺度 | 否 |
| **B. SLAM → 语义 3DGS → 导航代价** | 显式高斯点表示语义、可通行性，转换 ESDF/Costmap | 稳定估计位姿 | 否；这是“SLAM 与 GS 地图协同” |
| **C. SLAM → 在线语义 3DGS → 探索** | 高斯地图提供对象语义、未探索区域信息增益 | 提供定位，支持路线执行 | 否 |
| **D. 高斯地图 + ICP / 直接高斯碰撞规划** | Gaussian 均值配准、重叠积分计算轨迹碰撞 | 单独定位/对齐并执行轨迹 | 不等于紧耦合 GS-SLAM |
| **E. LiDAR/Camera/IMU + GS 联合 SLAM** | 与位姿估计共同优化、实时输出高斯地图 | 与地图重建耦合 | 是研究中的联合方案，但不代表已被四足商用采用 |

## 3.2 已有四足实机/采集的研究对照表

| 项目 | 平台 | 时间/方向 | SLAM、3DGS、规划具体组合 | 实机证据与代码状态 | 论文/主页/源码 |
|---|---|---|---|---|---|
| **Splatblox** | **Ghost Robotics Vision 60** | 2025 预印本，2026 更新；户外语义可通行性导航 | **LIO-SAM** 位姿 → LiDAR + 语义 RGB 增量 Gaussians → GS/激光 **ESDF 融合（Nvblox）** → **Nav2 + MPPI** | **四足真实户外导航**；论文明确说明外接 RTX 3070 笔记本；未找到可确认的算法主仓库 | [论文全文](https://arxiv.org/html/2511.18525) · [项目](https://splatblox.github.io/) |
| **VISTA** | **Boston Dynamics Spot** | 2025 论文；在线语义 GS + 自主探索/目标搜索 | **Spot 自身定位/SLAM** → 前部 RGB-D → 在线 Semantic 3DGS → 语义/几何信息增益 → 视点/waypoint → Spot SDK | **有 Spot 实机演示**，GS/规划使用外接 RTX 4090；开源 README 主要给出无人机实验运行说明，不能视作开箱即用 Spot 部署包 | [论文](https://arxiv.org/html/2507.01125) · [主页](https://stanfordmsl.github.io/VISTA/) · [源码](https://github.com/StanfordMSL/VISTA) |
| **FOCI** | **ANYmal** | 2025；Gaussian 上直接做轨迹优化 | 预建 3DGS 场景 → **LiDAR ICP 对齐 Gaussian 均值** → 以机器人/环境 Gaussian 重叠积分作为可微碰撞代价 → 朝向感知轨迹优化 | **有 ANYmal 实机走障碍通道**；实机高斯地图来自无人机图像，**不是机器人在线紧耦合 GS-SLAM**；代码公开但依赖 MA27 求解器许可 | [论文](https://arxiv.org/html/2505.08510) · [项目](https://rffr.leggedrobotics.com/works/foci/) · [源码](https://github.com/leggedrobotics/foci) |
| **SIREN** | **Unitree Go1** | CoRL 2025；多机器人/多子图高斯地图配准 | Go1 采集 RGB → 不同区域分别训练 GS 子地图 → 语义匹配 → Gaussian 间几何对齐 → 光度精配准 → 子地图融合 | **Go1 真实场景数据采集（厨房/工作间）**；是高斯地图注册/融合研究，**不是 Go1 实时闭环导航**；主页仍标代码 “Coming Soon” | [CoRL 论文](https://proceedings.mlr.press/v305/shorinwa25a.html) · [arXiv 全文](https://arxiv.org/html/2502.06519) · [项目](https://siren-robot.github.io/) |
| **Gaussian-LIC2** | **通用 LiDAR/Camera/IMU 机器人与数据集** | ICRA 2025 / IJRR 2026；GS 联合 SLAM | LiDAR-IMU-Camera 联合位姿估计 + 实时高斯重建；近似类别 E | **项目已开源**；本次未找到可核实的 Go2/B2/Spot/ANYmal 四足公开实机部署证据，**只能列为移植候选** | [源码](https://github.com/APRIL-ZJU/Gaussian-LIC) · [论文](https://arxiv.org/abs/2507.04004) |

### 3.2.1 案例一：Splatblox（最接近“LiDAR-SLAM + 3DGS + 四足自主导航”）

论文明确列出硬件与算法：
- 机器人：Ghost Vision 60；另在 Clearpath Jackal 轮式机器人上测试。
- 传感器：宽视角相机、Ouster OS1-32-U LiDAR、IMU。
- **定位**：LIO-SAM。
- **地图**：对 RGB 图像分割得到的可通行性语义赋予成本；与 LiDAR 的三维几何共同构造、更新 3D Gaussians。
- **碰撞/导航地图**：从高斯构造前方语义可通行 ESDF，与 Nvblox 的 LiDAR ESDF 结合；供 Nav2 使用。
- **局部控制**：MPPI；由机器人运动系统执行轨迹。
- **算力**：论文使用通过以太网连接机器人、搭载 RTX 3070 的笔记本。

```text
OS1 LiDAR ─┬─> LIO-SAM ────────────────> Robot Pose / TF
           ├─> Nvblox LiDAR ESDF ──┐
RGB Camera └─> Semantic Segmentation → Incremental 3DGS → GS ESDF
                                                         │
                                              ESDF / Cost Fusion
                                                         │
                                                    Nav2 + MPPI
                                                         │
                                                Quadruped Motion API
```

**适合复现的部分**：直接研究“高斯语义成本如何投影/栅格化到几何代价地图，再影响局部规划”，而不需要一开始重写 SLAM。

原始来源：[Splatblox 论文 IV-A Implementation Details](https://arxiv.org/html/2511.18525)、[项目](https://splatblox.github.io/)。

### 3.2.2 案例二：VISTA（Spot 上在线语义高斯地图 + 任务驱动探索）

- Spot 提供机载定位/SLAM 与 RGB-D 图像；实验只用机身前部两路相机训练 3DGS。
- 3DGS 不取代 Spot 原生 SLAM：它提供增量场景语义与几何信息。
- 对查询目标（如 “wagon”）计算语义相关性，结合信息增益决定探索视点。
- 用 Spot SDK 下发 waypoint；外部工作站运行高斯优化和规划。
- 论文明确指出系统依赖机器人自己的低层定位/控制；开源仓库的 ROS 2 演示主要针对无人机。

```text
Spot RGB-D + Onboard Pose/SLAM
               │
               ▼
     Semantic-SplatBridge
               │
               ▼
          Semantic 3DGS
               │
               ▼
VISTA-Map：Voxel + Semantic/Unknown/Free
               │
               ▼
VISTA-Plan：Frontier + Task-aware Info Gain
               │
               ▼
           Spot SDK Waypoint
```

**适合研究**：语义巡检、语言目标搜索、探索中在线重建。源：[论文 Spot hardware section](https://arxiv.org/html/2507.01125)、[官方代码](https://github.com/StanfordMSL/VISTA)。

### 3.2.3 案例三：FOCI（ANYmal 在 Gaussian 地图上直接规划轨迹）

- 环境预先由图像重建为 Gaussian 地图；实机场景来源于无人机图像。
- 机器人用 LiDAR 累积点云与 Gaussian 的 surface means 做 **ICP 对齐**，解决坐标配准。
- 机器人自身也用多个 Gaussian 建模，以 Gaussian 重叠积分近似连续可微的碰撞代价。
- 优化机身**位置、朝向**及通行轨迹，实机在需要调整姿态的狭窄障碍通道运行。
- 依赖 IPOPT/CasADi 等优化；仓库注明安装需要 **MA27** 求解器许可。

**创新点**：不必始终将高斯栅格化为传统占据图，而是直接利用 Gaussian 的形状和协方差来做碰撞代价。

**边界**：该实机例子不能写成“ANYmal 在线 3DGS-SLAM 已成熟”。源：[FOCI 论文 IV-A Hardware Experiments](https://arxiv.org/html/2505.08510)、[代码](https://github.com/leggedrobotics/foci)。

### 3.2.4 案例四：SIREN（Go1 高斯子地图配准与多次巡检融合）

- Unitree Go1 等移动平台采集多个真实场景 RGB 图像，训练局部高斯子地图。
- 不要求输入不同子地图的精确初始相对位姿。
- 通过高斯语义特征找到候选匹配，执行几何粗配准与光度精配准，最终融合地图。
- 目标是解决**多机、跨次采集的 3DGS 地图拼接**，不是让 Go1 直接基于该系统避障。

**适合研究**：一个场所多次巡检后进行高保真地图更新，或多机协同建图。源：[SIREN 全文实机设备说明](https://arxiv.org/html/2502.06519)、[正式论文](https://proceedings.mlr.press/v305/shorinwa25a.html)。

### 3.2.5 相关非四足算法：Gaussian-LIC2

- 适合研究 **LiDAR + IMU + Camera 与 Gaussian 地图紧耦合**。
- 比“从外部 SLAM 导入固定相机位姿后离线训练 GS”更接近联合建图定位。
- 但要移植至 Go2/ANYmal，仍需对齐 LiDAR 的逐点时间、相机触发、IMU 外参与时间偏移、ROS 版本/依赖和显存预算。
- 仓库有 2026 年发布和 IJRR 2026 接收信息，但**不能把其通用数据集实验写成四足实机部署**。

源：[Gaussian-LIC2 源码与说明](https://github.com/APRIL-ZJU/Gaussian-LIC)。

## 3.3 如何把 SLAM 与 3DGS 组合成可部署系统

### 路线 A：稳定工程方案——双地图架构

```text
                    LiDAR + IMU
                        │
                        ▼
              LIO / Pose-Graph SLAM
                        │
             ┌──────────┴────────────┐
             ▼                       ▼
       Geometry Mapping         Camera Pose + RGB
       Pointcloud / ESDF               │
             │                         ▼
             │                3D Gaussian Splatting
             │                         │
             │                  Semantic/Traversability
             │                         │
             └───────── Fusion ────────┘
                        │
                  Safe Costmap / ESDF
                        │
                Nav2 / Terrain Planner
                        │
                    Velocity Bridge
                        │
                   Quadruped SDK
```

**优点**：激光几何层独立提供安全碰撞约束；3DGS 出错、训练延迟或显存不足时仍能保持基础避障能力。  
**缺点**：两套地图需要时间、空间对齐，回环后高斯子图需要相应更新。

### 路线 B：研究型紧耦合方案

```text
LiDAR + IMU + Camera
        │
        ▼
Shared Pose Estimation / Joint Optimization
        │                          │
        ▼                          ▼
Continuous Odometry          Gaussian Map Update
        │                          │
        └──────── Map Alignment ───┘
                     │
            Geometry / Semantic Map
                     │
             Traversability Planner
```

可参考 **Gaussian-LIC2** 做融合架构，参考 **Splatblox/FOCI** 做导航部分。但两者不应未经实验就视为“开箱即用的一套四足实时系统”。

### 3.3.1 必须解决的接口问题

1. **统一尺度与坐标**：`T_map_camera = T_map_body × T_body_camera`；RGB 图像与 LiDAR 需共享可追溯的时间戳与外参。
2. **SLAM 回环后重对齐**：优化后的关键帧位姿可能改变；高斯点需关联关键帧或子地图以更新变换，不能继续盲用旧轨迹。
3. **动态物体处理**：行人、车辆、摆动物体不应被永久固化为静态障碍或静态高斯。
4. **3DGS 与物理障碍区别**：高斯密度、渲染颜色和碰撞体积不是同一概念。规划前通常要得到可信的 Occupancy、TSDF、ESDF、Mesh 或经过标定的碰撞代价。
5. **安全降级**：GS 模块掉帧、过期、失锁时，退回 LiDAR 的几何避障或安全停车。
6. **算力边界**：上述 Splatblox/VISTA 的部分实验使用外接 GPU 电脑；“实机运行”不一定意味着全部计算部署在机器人机载 SOC 上。
7. **腿式地形语义**：草丛可通行、台阶能否踩、地形是否打滑是不同判断，语义仅提供附加证据，不能替代足端动力学安全评估。

---

# 4. 面向 Go2 / Lite3 的最小可行工程路线

| 阶段 | 技术模块 | 交付结果 | 核验标准 |
|---|---|---|---|
| **P0 基础联通** | LiDAR、IMU、Camera、TF、SDK2/厂商 API | 全部话题、时间戳、外参和坐标方向正确 | 静止/转弯/起伏动作不出现明显时间错位 |
| **P1 可靠 SLAM** | 先 RTAB-Map + Nav2 快速跑通；或选 LIO-SAM/FAST-LIO2 + 回环/重定位 | 点云/地图与 `odom`、`map` TF | 回到起点、重启加载地图、丢定位恢复 |
| **P2 普通导航** | 2D Costmap + NavFn/DWB 基线，后比较 MPPI | 接目标点、自动避障、停机 | 连续目标测试、动态障碍测试、低速急停 |
| **P3 3DGS 重建** | 同步采集 RGB/LiDAR/IMU/位姿；离线 3DGS | 可视化三维 Gaussian 地图 | 尺度一致、相机轨迹对齐、地面/墙壁不大面积漂浮 |
| **P4 3DGS 参与导航** | 语义 GS → 栅格/ESDF → Nav2，参考 Splatblox | 语义可通行区域影响路径 | 与“仅 LiDAR Costmap”基线对比成功率、卡死、路径长度 |
| **P5 紧耦合科研** | Gaussian-LIC2/高斯对齐、FOCI 高斯轨迹规划、SIREN 地图融合 | 联合位姿/地图/规划创新 | ATE/RPE、重建、定位鲁棒性、规划时间与安全指标 |

**机型适配建议：**
- **已有 Go2 + ROS 2**：先复现 [Go2 RTAB-Map/Nav2 工程](https://github.com/h-naderi/unitree-go2-slam-nav2)，对照 [Go2 SportClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp) 建立安全速度桥接。
- **已有宇树 L1/L2 LiDAR**：可参考 [宇树 Point-LIO 适配](https://github.com/unitreerobotics/point_lio_unilidar)；**该官方仓库 README 的主要测试环境是 ROS Noetic（ROS 1）**，ROS 2 Humble/Jazzy 需确认移植/桥接。
- **已有 Lite3 Venture + 相机/LiDAR/Jetson**：优先看 [云深处 FAST-LIVO2 复现](https://github.com/DeepRoboticsLab/fast-livo2-deep-robotics)，此工程明确提醒时间同步可能导致 SLAM 失败。
- **已有 M20 Pro / Lite3 EDU**：看 [Lightning-LM](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics) 的在线建图、保存地图与已知地图定位。
- **追求 3DGS 语义导航论文**：重点读 **Splatblox → VISTA → FOCI → Gaussian-LIC2**；如果偏长期多机建图再读 SIREN。

## 4.1 建议记录的实验评价指标

| 方向 | 指标 | 目的 |
|---|---|---|
| SLAM | ATE、RPE、回环后轨迹一致性、重定位成功率/时间 | 衡量局部和全局定位 |
| 运行稳定性 | 定位丢失次数、传感器不同步次数、CPU/GPU/显存峰值 | 判断能否上实机 |
| 导航 | 到达率、碰撞/急停次数、卡死次数、路径长度、耗时 | 测试任务级可靠性 |
| 复杂地形 | 可通行性误判、打滑、绊倒、足端接触异常、姿态偏差 | 区分四足与轮式性能 |
| 3DGS | 新视角渲染质量（PSNR/SSIM/LPIPS）、几何重建误差、增量更新延迟 | 判断 3DGS 地图质量 |
| GS + 导航联合 | 相比仅 LiDAR 的成功率、可通行性分类正确率、碰撞代价有效性 | 证明加入 GS 的实际收益 |

---

# 5. 原始资料和源码索引

## 5.1 SLAM 与四足平台

1. [FAST-LIO2（HKU MARS）](https://github.com/hku-mars/FAST_LIO)
2. [Point-LIO（HKU MARS）](https://github.com/hku-mars/Point-LIO)
3. [Point-LIO 宇树 L1/L2 适配（Unitree 官方）](https://github.com/unitreerobotics/point_lio_unilidar)
4. [LIO-SAM（原作者仓库）](https://github.com/TixiaoShan/LIO-SAM)
5. [FAST-LIVO2（HKU MARS）](https://github.com/hku-mars/FAST-LIVO2)
6. [FAST-LIVO2 云深处 Lite3 工程（厂商仓库）](https://github.com/DeepRoboticsLab/fast-livo2-deep-robotics)
7. [RTAB-Map ROS（原项目）](https://github.com/introlab/rtabmap_ros)
8. [Go2 + RTAB-Map + Nav2（第三方）](https://github.com/h-naderi/unitree-go2-slam-nav2)
9. [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
10. [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
11. [ANYmal 研究中的 CompSLAM](https://github.com/leggedrobotics/compslam_subt)
12. [ETH Holistic Fusion](https://github.com/leggedrobotics/holistic_fusion)
13. [CyberDog MIVINS](https://github.com/MiRoboticsLab/cyberdog_mivins)
14. [CyberDog MILOC](https://github.com/MiRoboticsLab/cyberdog_miloc)
15. [CyberDog LaserSLAM](https://github.com/MiRoboticsLab/cyberdog_laserslam)
16. [云深处 M20 Pro/Lite3 EDU Lightning-LM](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics)
17. [Spot GraphNav 官方技术说明](https://dev.bostondynamics.com/docs/concepts/autonomy/graphnav_tech_summary.html)
18. [Spot GraphNav 官方定位说明](https://dev.bostondynamics.com/docs/concepts/autonomy/localization.html)

## 5.2 导航与地形规划

19. [ROS 2 Nav2 官方文档](https://docs.nav2.org/)
20. [Nav2 Smac 2D / Hybrid A* / State Lattice](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/)
21. [Nav2 DWB Controller](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/)
22. [Nav2 MPPI Controller](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)
23. [Nav2 Regulated Pure Pursuit](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
24. [ANYmal ArtPlanner](https://github.com/leggedrobotics/art_planner)
25. [Elevation Mapping CuPy（ETH）](https://github.com/leggedrobotics/elevation_mapping_cupy)
26. [Traversability Estimation（ETH）](https://github.com/leggedrobotics/traversability_estimation)
27. [NVIDIA Nvblox](https://github.com/nvidia-isaac/nvblox)
28. [Go2 公开 Nav2 配置（NavFn/DWB）](https://github.com/h-naderi/unitree-go2-slam-nav2/blob/main/go2_slam_nav/config/nav2_params.yaml)
29. [CyberDog 公开 Nav2 配置（NavFn/DWB）](https://github.com/MiRoboticsLab/cyberdog_tracking_base/blob/rolling/mcr_bringup/params/nav2_params.yaml)

## 5.3 运动接口

30. [Unitree SDK2 C++ Go2 SportClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp)
31. [Unitree SDK2 Python Go2 SportClient](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/sport/sport_client.py)
32. [Unitree 官方 ROS 2 接口、Topic 与 MotorCmd](https://github.com/unitreerobotics/unitree_ros2)
33. [Unitree B2 Python 高层控制示例](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/b2/high_level/b2_sport_client.py)
34. [Spot RobotCommand 官方 API](https://dev.bostondynamics.com/python/bosdyn-client/src/bosdyn/client/robot_command.html)
35. [Spot GraphNav Service 官方说明](https://github.com/boston-dynamics/spot-sdk/blob/master/docs/concepts/autonomy/graphnav_service.md)
36. [云深处 X30 Pro RobotServer SDK](https://github.com/DeepRoboticsLab/robotserver_sdk/blob/main/README.zh-CN.md)
37. [CyberDog Nav2 官方仓库](https://github.com/MiRoboticsLab/cyberdog_nav2)
38. [CyberDog Locomotion 官方仓库](https://github.com/MiRoboticsLab/cyberdog_locomotion)
39. [Nav2 Kilted TwistStamped 迁移说明](https://docs.nav2.org/rolling/configuration_and_development/migration_guides/jazzy/Jazzy/)

## 5.4 3DGS + SLAM + 四足案例

40. [Splatblox 论文全文（Vision 60）](https://arxiv.org/html/2511.18525)
41. [Splatblox 项目主页](https://splatblox.github.io/)
42. [VISTA 论文全文（Spot）](https://arxiv.org/html/2507.01125)
43. [VISTA 项目主页](https://stanfordmsl.github.io/VISTA/)
44. [VISTA 开源仓库](https://github.com/StanfordMSL/VISTA)
45. [FOCI 论文全文（ANYmal）](https://arxiv.org/html/2505.08510)
46. [FOCI 代码](https://github.com/leggedrobotics/foci)
47. [FOCI 项目主页](https://rffr.leggedrobotics.com/works/foci/)
48. [SIREN CoRL 2025 论文（Go1）](https://proceedings.mlr.press/v305/shorinwa25a.html)
49. [SIREN arXiv 全文](https://arxiv.org/html/2502.06519)
50. [SIREN 项目主页](https://siren-robot.github.io/)
51. [Gaussian-LIC2 代码（**通用方案，未证实四足实机**）](https://github.com/APRIL-ZJU/Gaussian-LIC)
52. [Gaussian-LIC2 论文](https://arxiv.org/abs/2507.04004)

**核查范围说明**：以上链接优先选取厂家官方文档、算法作者仓库、论文原文。没有确切来源的闭源算法名称、未经证明的“某型号原装采用算法”、未经证明的四足实机移植，均未列为事实。
