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
