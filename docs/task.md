## tips
1. 使用中文完成文档。
2. 使用中文和我对话。
3. 不要进行编译(colcon build)。

## task1
详细阅读 quadruped_ros2_control 项目并进行分析，完成一个总体的分析文档，包含以下内容:
1. gazebo 模拟下数据如何在各个节点(需指出接口)传递。
2. 实际硬件下数据如何在各个节点(需指出接口)传递。
3. 说明每个（mpc/rl/pd）控制器数据交互的流程（例如：指出指令/期望轨迹->MPC->WBC是如何交互和数据传递的）。给出具体公式和流程示例。
4. 将整理好的内容输出到 docs 下的一个新创建的文档中。

## task2
分析并梳理 ocs2_quadruped_controller 包中的代码，完成以下任务:
1. 说明其实现的功能和各个节点间的关系(关系可以用 Mermaid 表示，但也需用文字表述节点的关系)。
2. 说明各个节点中各个方法的功能。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 ocs2_quadruped_controller.md 中。

## task3
分析并梳理 rl_quadruped_controller 包中的代码，完成以下任务:
1. 说明其实现的功能和各个节点间的关系(关系可以用 Mermaid 表示，但也需用文字表述节点的关系)。
2. 说明各个节点中各个方法的功能。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 rl_quadruped_controller.md 中。

## task4
分析并梳理 ocs2_quadruped_controller 包中的代码，完成以下任务:
1. 说明"步态(gait)"是如何在代码里体现，gait 切换又会如何体现。
2. 将整理好的内容输出到 docs 下的 ocs2_quadruped_controller.md 中（新增相关内容即可）。

## task5
分析并梳理 estimator 中的代码，完成以下任务:
1. 说明其实现的功能和各个节点间的关系(关系可以用 Mermaid 表示，但也需用文字表述节点的关系)。
2. 说明各个节点中各个方法的功能。
3. 着重分析 KalmanFilterEstimate 类的内容，并说明其输入输出具体是什么，输入从哪里获取。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs 下的 estimator.md 中。

## task6
分析并梳理 Ocs2QuadrupedController 类中，完成以下任务:
1. 说明该类各个方法的功能(需要说明各个方法什么时候被调用，调用顺序如何。调用顺序和时间点可以用 Mermaid 表示，但也需用文字表述)。
2. 说明如 command_interfaces_、state_interfaces_ 等接口项从哪里读入，其如何更新。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 Ocs2QuadrupedController.md 中。

## task7
有两个启动命令:
```bash
# gazebo
ros2 launch ocs2_quadruped_controller gazebo.launch.py pkg_description:=go2_description

# mujoco
ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=go2_description
```
分析这两个启动命令涉及到的配置文件，完成以下任务:
1. 忽略 .jpg、.dae 等非文本类型文件。分析 launch.py 如何加载各项配置文件(包括各配置的嵌套关系)。
2. 分析其如何配置 ros2_control。
3. mujoco.launch.py 是否可用于真实的硬件，用于真实的硬件需要注意什么。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs 下的 launch_py.md 中。

## task8
分析并梳理 StateOCS2 类中的代码(会涉及其他类)，完成以下任务:
1. 说明该类各个方法的功能(尤其是 run 方法)。
2. 需要分析 evaluatePolicy、 updatePolicy 中 Policy 具体是什么(一个函数、一个变量还是类似神经网络，其接收什么输入，输出什么)。
3. 需要分析"传感器->mpc->wbc->机器狗"每一次数据传输、处理过程中的输入输出具体是哪些。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs 下的 StateOCS2.md 中。

## task9
分析并梳理 CtrlComponent 类中的代码(会涉及其他类)，完成以下任务:
1. 说明该类各个方法的功能(尤其是 updateState 方法)。
2. 需要分析 updateState 过程中发生的数据转换，如: measured_rbd_state_ 如何转换为 observation_.state ， observation_.state 具体包括哪些量（物理意义以及所在数组位数）。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 CtrlComponent.md 中。

## task10
分析并梳理 TargetManager 类中的代码，完成以下任务:
1. 说明该类各个方法的功能(尤其是 update 方法)。
2. 需要分析 update 过程中发生的几次数据转换（物理意义以及所在数组位数）。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 TargetManager.md 中。

## task11
分析并梳理 SwitchedModelReferenceManager 类及其父类 ReferenceManager 中的代码，完成以下任务：
1. 说明类的各个方法的功能(尤其是 "涉及接受外部数据和更新对外数据" 方法)。
2. 需要分析 "涉及接受外部数据和更新对外数据" 方法执行过程中发生的数据转换（物理意义以及所在数组位数）。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs/class 下的 SwitchedModelReferenceManager.md 中。

## task12
分析并梳理 LeggedInterface 类中的代码，完成以下任务：
1. 说明类的各个方法的功能(重点关注其数据处理和更新的流程)。
2. 需要总结 LeggedInterface 控制的组件名单。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs/class 下的 LeggedInterface.md 中。

## task13
分析并梳理 GaitManager 类中的代码，完成以下任务：
1. 说明类的各个方法的功能(重点关注"涉及接受外部数据和更新对外数据"的流程)。
2. 说明你觉得应当说明的内容。
3. 将整理好的内容输出到 docs/class 下的 GaitManager.md 中。

## task14
分析并梳理 unitree_sdk2 包中的代码，完成以下任务：
1. 可以基于 go2 分析以下内容。
2. 说明该包实现的功能，其数据处理和更新的流程。
3. 说明该包如何与 硬件/mujoco 交互，如何调用该包获取数据。
4. 说明该包是否进行了代码封装(即隐藏具体实现)。如果有的话，如何进行替代。
5. 说明你觉得应当说明的内容。
6. 将整理好的内容输出到 docs/external 下的 unitree_sdk2.md 中。

## task15
分析并梳理 unitree_mujoco 包中的代码，完成以下任务：
1. 可以基于 go2 分析以下内容。
2. 说明该包实现的功能，其数据处理和更新的主要流程。
3. 说明该包如何与 unitree_sdk2 交互。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs/external 下的 unitree_mujoco.md 中。

## task16
分析并梳理 hardware_unitree_sdk2 包中的代码，完成以下任务：
1. 说明该包实现的功能，其数据处理和更新的主要流程。
2. 着重并详细说明该包如何与 ros2_control、unitree_sdk2 、unitree_mujoco 进行数据交互(需要说明数据的具体类型，数组位数含义)。它们是什么关系(关系可以用 Mermaid 表示，但也需用文字表述节点的关系)。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 hardware_unitree_sdk2.md 中。

## task17
对比 hardware_unitree_sdk2 包和 unitree_ros2 包。不要摘录过多源码，使用叙述性文字和 Mermaid 图。完成以下任务：
1. 说明这两个包之间的关系。hardware_unitree_sdk2 是基于 unitree_ros2 完成的吗？如果是的话，它添加了什么工作？
2. 说明你觉得应当说明的内容。
3. 将整理好的内容添加到 hardware_unitree_sdk2.md 中（不要修改现有 md 内容，只需要新增）。