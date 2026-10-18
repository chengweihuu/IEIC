# 应急搜救 P2 和 P4 功能模块

本目录提供两个可供上层程序调用的 Python 模块：**P2 疑似人员热源检测**和 **P4 气体异常记录与警告**。调用方在机器人停稳后传入检查点、位置和朝向，模块完成一次检测、证据保存及可选语音播报。机器人行走、转向、导航和整场状态机由集成方负责。

**当前状态：P2 的 30 项、P4 的 24 项离线测试已通过，新版本实机联测尚未完成。P4 按当前约定暂用 CO 联调。** 比赛原始要求见 [比赛须知](比赛须知.md)，场景参考见 [场景图](场景图.png)。

## 文件入口

| 文件 | 用途 |
|---|---|
| [p2_rescue.py](p2_rescue.py) | P2 核心模块，单文件部署 |
| [P2使用说明.txt](P2使用说明.txt) | P2 完整参数、匹配规则和返回字段 |
| [p4_gas_alarm.py](p4_gas_alarm.py) | P4 核心模块，支持日志读取或外部传感器适配 |
| [P4配置.json](P4配置.json) | 当前 CO 联调配置，现场主要修改入口 |
| [P4使用说明.txt](P4使用说明.txt) | P4 数据格式、告警行为和接入方式 |
| [P4离线样例.json](P4离线样例.json) | 模拟 CO 软件值，用于离线验证，不是真实传感器数据 |
| [test_p2_rescue.py](test_p2_rescue.py) | P2 离线测试 |
| [test_p4_gas_alarm.py](test_p4_gas_alarm.py) | P4 离线测试 |

## 部署环境

建议把 `p2_rescue.py`、`p4_gas_alarm.py` 和 `P4配置.json` 放在导航业务主机 `192.168.1.120` 的同一可写目录中。导入模块时，该目录应位于 Python 模块搜索路径内。

| 功能 | 运行条件 |
|---|---|
| P2 温度矩阵检测 | Python 3.8 及以上、NumPy |
| P2 实机采集 | 能访问相机主机 `192.168.1.214:10086`；SSH 客户端及相机主机密钥登录 |
| P2 可见光取证 | FFmpeg；视频地址为 `rtsp://192.168.1.214:8554/visible` |
| P4 当前 CO 采集 | Python 3.8 及以上、标准库；可读取 `.120` 本地 `/home/root/log/other.log` |
| 中文语音警告 | `.120` 已有的 `libespeak-ng`、`aplay` 和 MCP01 声卡 |

P2 需要在运行模块的系统用户下，预先配置到 `root@192.168.1.214` 的 SSH 密钥登录，并核对、记录相机主机身份。下面的只读检查应能无密码提示完成：

```bash
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes root@192.168.1.214 true
```

P4 默认直接读取本机日志，不需要另外建立 SSH 连接。两个模块均不依赖 ROS，不自动读取 `/current_pose` 或 `/odom`。

下文命令以**本 README 所在目录**为工作目录。在项目工作区可先执行：

```bash
cd IEIC-main/应急搜救
```

## P2 热源搜索与观测位置记录

### 功能行为

P2 获取一张红外温度矩阵，在全画面筛选高温区域，将相邻区域合并为候选目标，再与同一检查点、相近位姿下的历史候选匹配。每个候选返回编号、温度、图像区域、机器人观测位置及左前方、正前方或右前方。

可见光只用于保存现场照片，**不需要可见光与热成像融合**。默认输出的是“发现位置＋方向”；需要粗略目标坐标时，可以配置假设距离和水平视场角，具体见 [P2 使用说明](P2使用说明.txt)。

- 同帧相邻区域合并，可以减少脸和手被分成多个候选的情况。
- 同点同视角匹配成功时沿用编号，已成功播报的目标不重复播报。
- 目标分裂、合并导致对应关系不明确时，返回 `ambiguous_local_match`，不直接新增编号。
- 换视角后无法确认是否重复的观测标记为待确认，不累计为新增人员。

这些规则处理的是疑似热源，**候选数量不能直接当作人数**。`unique_person_count` 和 `new_person_count` 均为 `None`。

### Python 调用

```python
from p2_rescue import RescueDetector

# 初始化一次，后续复用这个对象。
detector = RescueDetector(
    output_dir="p2_results",
    threshold_c=32.0,
    min_area_pixels=64,
    merge_gap_pixels=20,
    speak=True,
)

result = detector.detect(
    task_id="search_capture_001",
    checkpoint_id="search_point_A",
    location_label="搜救点A",
    robot_pose={"x": 5.0, "y": 3.0, "yaw": 0.8},
    frame_id="map",
)

if result["status"] == "ok":
    print(result["target_found"], result["targets"])
    print(result["speech_status"])
else:
    print(result.get("error", "模块正在执行其他检测"))
```

同一检查点保持 `checkpoint_id` 不变；`task_id` 可以标识每次不同的调用。省略 `checkpoint_id` 时使用 `task_id`。

### 主要参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `roi` | `(0, 0, 1, 1)` | 全画面检测，归一化范围 |
| `threshold_c` | `32.0` | 高温像素阈值，需用现场热源调整 |
| `min_area_pixels` | `64` | 合并后候选的最小高温像素面积 |
| `merge_gap_pixels` | `20` | 相邻区域框可合并的最大间距 |
| `same_view_distance_m` | `0.25` | 相对该视角初始观测位置的平移容差 |
| `same_view_yaw_deg` | `10` | 同视角朝向容差 |
| `match_max_age_s` | `120` | 历史匹配有效期 |
| `speak` | `False` | 是否实际播报；上方示例已开启 |

合并距离过大会把相邻人员合为一个候选，过小可能把一个人分成多个候选；需要现场用一人和多人样例调整。热图左右方向也需要通过移动热源核对，必要时设置 `flip_horizontal=True`。

### 单次命令与离线回放

实机单次检测，发现目标时实际播报：

```bash
python3 p2_rescue.py --task-id capture_001 --checkpoint-id A --location-label 搜救点A --pose 5 3 0.8 --speak
```

使用仓库中的历史 FIR 离线回放，不访问设备、不播放声音：

```bash
python3 p2_rescue.py --fir ../测试结果/2026-10-12_云台测温接口/红外矩阵/20230619033310_ir.fir --checkpoint-id A --pose 5 3 0.8
```

这份历史样例最高温约 28.2℃，默认阈值下应返回未发现热源。独立命令每次启动新进程；验证跨调用匹配和重复播报抑制时，应在同一 Python 对象上连续调用。

## P4 CO 读数记录与告警

### 当前联调配置

P4 当前通过 `.120` 上的气体日志等待一条新的 CO 读数，按阈值判断，记录机器人观测位置，并在触发时发出中文警告。保存的位置是传感器观察到异常的位置，不是气体泄漏源位置。

[P4配置.json](P4配置.json) 中的主要设置为：

| 配置项 | 当前值 | 含义 |
|---|---|---|
| `source` | `rk_log` | 读取当前已验证的气体日志 |
| `channel` | `CO` | 暂用 CO 联调 |
| `unit` | `software_value` | 厂家软件内部值，物理单位尚未确认 |
| `threshold` | `1.0` | 临时触发阈值 |
| `comparison` | `high` | 读数大于或等于阈值时触发 |
| `rule_purpose` | `integration_test_only` | 结果和播报标记为联调用途 |
| `wait_timeout_s` | `75` | 等待新日志的时间上限 |
| `repeat_warning_s` | `60` | 同点持续告警的重复播报间隔 |
| `speak` | `true` | 默认实际播报 |

**CO 软件值 ≥ 1 仅用于联调，不是浓度安全限值。** 现有真实 CO 样例均为 0，不会触发这条规则；未触发不代表环境安全。比赛 P4 要求 VOC 检测，CO 是当前临时接入通道。

### Python 调用

```python
from p4_gas_alarm import GasAlarmDetector

# 读取同目录 P4配置.json；初始化时不访问传感器。
detector = GasAlarmDetector.from_config()

result = detector.detect(
    task_id="gas_check_001",
    checkpoint_id="gas_point_A",
    location_label="检测点A",
    robot_pose={"x": 5.0, "y": 3.0, "yaw": 0.8},
    frame_id="map",
)

if result["status"] == "ok":
    print(result["anomaly"], result["warning_text"])
    print(result["speech_status"])
else:
    print(result.get("error", "模块正在执行其他检测"))
```

修改配置后重新初始化对象才会生效。也可以临时覆盖，例如 `GasAlarmDetector.from_config(threshold=2, speak=False)`。相对输出路径以配置文件目录为基准。

每次读取从日志当前位置开始等待新数据；已有记录显示同一通道更新可能约一分钟一次，机器人应在等待期间保持停稳。超时、数据过期、单位不匹配、无效或重复读数返回 `anomaly=None`，不会当成正常值或零值。

同点持续异常会沿用告警事件编号并限制重复播报；新的有效读数恢复到未触发状态后，再次异常会建立新事件。播报失败保留异常结果，下一条有效异常读数可重试。

### 单次命令与离线回放

在 `.120` 实机上使用当前 CO 配置，触发时实际发声：

```bash
python3 p4_gas_alarm.py --checkpoint-id A --location-label 检测点A --pose 5 3 0.8
```

添加 `--no-speak` 可关闭声音。使用模拟 CO 软件值 2 验证告警流程，不访问设备、不播放声音：

```bash
python3 p4_gas_alarm.py --sample-json P4离线样例.json --replay --pose 5 3 0.8
```

默认规则下应返回 `anomaly=True`、`speech_status="disabled_for_replay"`。这个结果验证软件流程，不作为真实气体检测证据。

### 现场更换传感器

如果仍使用已验证的日志通道，可调整 `channel`、`threshold` 和比较方向。当前日志适配器支持 `CO`、`O2`、`temperature`、`humidity`，单位均保留为软件值。

如果现场确认使用 VOC 或不同协议，需要接入实际读取函数，并设置匹配的通道、单位和阈值；**不能只把 CO 名称改成 VOC**。配置设为 `source="external"` 后，可通过 `GasAlarmDetector.from_config(reader=read_sensor)` 接入，上层 `detect()` 调用方式保持不变。读数字典格式见 [P4 使用说明](P4使用说明.txt)。

## 对接约定与结果文件

- `robot_pose` 中 `x`、`y` 单位为米，`yaw` 单位为弧度，以地图正 X 方向为 0、逆时针为正。位姿由调用方提供，模块不验收定位质量。
- 检测接口是同步调用，采集和播报会等待完成；调用方应避免阻塞必须持续响应的控制任务。
- `status="ok"` 表示本次检测和基本记录成功，语音是否成功要另看 `speech_status`。`error` 或 `busy` 不能按“未发现”处理。
- 同一对象保留内存历史，新建对象或重启进程会清空。P2 可调用 `reset(checkpoint_id="A")` 或 `reset()`；P4 使用 `reset()` 开始新会话，已有证据文件不删除。
- P2、P4 共用扬声器时应顺序安排播报，避免同时占用声卡。

| 模块 | 每次调用保存的主要证据 |
|---|---|
| P2 | `result.json`、原始 FIR、温度 CSV、编号热图；可用时保存厂家 JPG、可见光照片和逐目标 WAV |
| P4 | `result.json`、`reading.json`、`reading.csv`、`raw_sample.txt`；成功播报时保存 `warning.wav` |

P2 热图中的数字对应本次 `targets[].observation_index`，稳定的局部目标编号见 `target_id`。P4 通过 `alarm_id` 关联同点持续告警。数据均保存在本地独立目录中，需要由集成方接入全程回传和报告。

接收端时间、日志时间与相机曝光时间含义不同。设备时间同步和数据时序仍需实机核对；现有相机文件名中的 2023 年不作为本次取证时间。P2 触发拍照还会生成相机 SD 卡文件，应管理相机和本地存储空间。

## 离线测试与现场验证

在本目录执行：

```bash
python3 -m unittest -v test_p2_rescue.py test_p4_gas_alarm.py
```

当前共有 **54 项离线测试**：P2 为 30 项，P4 为 24 项。覆盖温度矩阵读取、区域合并、目标重复匹配、播报抑制、气体日志解析、阈值判断、数据新鲜度、告警重试和异常路径。测试使用历史文件、合成数据及模拟设备接口，不会控制机器人或播放声音。

实机联测重点如下：

| 模块 | 待验证内容 |
|---|---|
| P2 | 一个与多个热源的检出、左右方向、区域合并阈值、同点重复检测、转向后的待确认标记、实际播报 |
| P4 | `.120` 新 CO 日志的读取、真实更新间隔、位置对应、告警记录及扬声器播放；现场确认实际气体通道、单位和阈值 |
| 集成 | 设备时间同步、调用耗时、语音顺序、证据回传及比赛认可的位置表达 |

当前 P2 提供疑似热源观测功能，P4 提供 CO 临时联调流程；完整赛项是否达标需结合实际传感器、现场效果和专家确认的判定规则。
