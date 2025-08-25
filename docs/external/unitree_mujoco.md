
# unitree_mujoco 包分析

## 1. 功能概述

`unitree_mujoco` 包利用 MuJoCo 物理引擎，为宇树（Unitree）系列机器人（如 Go2）提供了一个高逼真度的仿真环境。其核心功能是**模拟真实机器人的硬件接口**，充当一个“数字孪生”。

这使得为真实机器人开发的、基于 `unitree_sdk2` 的控制程序，可以**无需任何修改**，直接在仿真环境中运行、测试和调试。

该包主要实现了以下功能：

*   加载指定的机器人模型（XML 文件）并运行物理仿真。
*   提供一个可视化的仿真窗口，可以实时观察机器人的状态。
*   建立一个桥梁，将 MuJoCo 仿真内部的状态数据（如传感器读数）转换为 `unitree_sdk2` 的 DDS 消息并发布出去。
*   接收外部通过 `unitree_sdk2` 发送的控制指令（DDS 消息），并将其转换为施加在仿真机器人关节上的力矩。
*   模拟手柄输入，允许通过游戏手柄控制仿真机器人。

## 2. 数据处理和更新的主要流程

`unitree_mujoco` 的运行主要由两个并行的线程驱动：**物理仿真线程**和**SDK 桥接线程**。

**A. 物理仿真线程 (`PhysicsThread` in `main.cc`)**

1.  **加载模型：** 程序启动时，加载指定的机器人 MJCF (XML) 模型文件。
2.  **仿真循环：** 该线程在一个循环中不断调用 `mj_step()` 函数。
3.  **状态更新：** `mj_step()` 根据当前状态和施加的控制力（`mjData->ctrl`），计算并更新下一个时间步的仿真状态（如关节角度 `qpos`、速度 `qvel` 等）。
4.  **传感器模拟：** 在每次状态更新后，MuJoCo 会根据模型中定义的传感器（如关节位置/速度传感器、IMU、接触力传感器），自动更新 `mjData->sensordata` 数组。

**B. SDK 桥接线程 (`UnitreeSdk2BridgeThread` in `main.cc`)**

这个线程运行 `UnitreeSdk2Bridge` 类的实例，负责仿真数据和 DDS 消息之间的双向转换。

*   **数据流：仿真 -> 外部控制器 (数据发布)**
    1.  `UnitreeSdk2Bridge` 在独立的线程中，以固定的频率（例如 500Hz）执行 `PublishLowStateGo` 等函数。
    2.  这些函数从 `mjData->sensordata` 中读取最新的仿真传感器数据。
    3.  将读取到的数据填充到 `unitree_sdk2` 定义的 DDS 消息结构体中（例如 `unitree_go::msg::dds_::LowState_`）。
    4.  通过 `ChannelPublisher` 将这些 DDS 消息发布到预定义的网络主题上（例如 `rt/lowstate`）。
    5.  外部的控制器通过订阅这些主题，就能接收到仿真机器人的状态，如同接收真实机器人的状态一样。

*   **数据流：外部控制器 -> 仿真 (指令接收)**
    1.  `UnitreeSdk2Bridge` 创建了一个 `ChannelSubscriber` 来订阅控制指令主题（例如 `rt/lowcmd`）。
    2.  当外部控制器发布一个 `LowCmd_` 消息时，绑定的回调函数 `LowCmdGoHandler` 会被触发。
    3.  该回调函数解析消息内容，获取期望的关节位置 `q`、速度 `dq`、前馈力矩 `tau` 以及 PD 增益 `kp`, `kd`。
    4.  它根据公式 `torque = tau + kp * (q_des - q_sim) + kd * (dq_des - dq_sim)` 计算出最终要施加到每个关节上的总力矩。其中 `q_sim` 和 `dq_sim` 从 `mjData->sensordata` 中实时获取。
    5.  计算出的总力矩被写入到 `mjData->ctrl` 数组中。
    6.  在下一次物理仿真线程调用 `mj_step()` 时，这些力矩就会被施加到机器人的关节上，从而驱动仿真机器人运动。

## 3. 与 unitree_sdk2 的交互

`unitree_mujoco` 与 `unitree_sdk2` 的交互是**间接**且**完全基于 DDS 通信协议**的。它本身并不直接链接或调用 `unitree_sdk2` 的客户端 API，而是**完美地模拟了机器人硬件的 DDS 接口**。

*   **角色扮演：** `unitree_mujoco` 扮演的是**机器人本身**（服务器端），而使用 `unitree_sdk2` 的控制程序扮演的是**客户端**。
*   **发布状态：** `unitree_mujoco` **发布** `rt/lowstate` 和 `rt/sportmodestate` 等主题，供 `unitree_sdk2` 客户端订阅和接收。
*   **订阅指令：** `unitree_mujoco` **订阅** `rt/lowcmd` 主题，以接收来自 `unitree_sdk2` 客户端的控制指令。

这种设计实现了仿真环境和真实硬件之间的高度解耦和接口统一。任何为真实 Go2 机器人编写的、遵循 `unitree_sdk2` 通信协议的程序，都可以无缝地将 `unitree_mujoco` 作为其控制对象。

## 4. 值得说明的内容

*   **配置文件 (`config.yaml`)：** 该文件提供了高度的灵活性，允许用户轻松切换不同的机器人模型、指定用于 DDS 通信的网络接口（例如，使用 `lo` 本地回环接口进行本机仿真，或使用 `eth0` 与另一台机器上的控制器通信），以及配置手柄等。
*   **传感器依赖：** `UnitreeSdk2Bridge` 的功能强依赖于 MuJoCo 模型（XML 文件）中正确定义的传感器。它通过传感器名称（如 `imu_quat`, `touch_FR`）来识别和读取特定的数据。如果模型中缺少必要的传感器，桥接功能将无法正常工作。
*   **多机器人支持：** 代码中通过 `idl_type_` 区分了 `unitree_go` 和 `unitree_hg` 两种不同的 DDS 消息定义，表明该仿真框架被设计为可以支持包括 Go2、H1 在内的多种宇树机器人。
