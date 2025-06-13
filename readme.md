# DWA for Ship Collision Avoidance (船舶动态窗口避碰算法)

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)

This project provides a Python simulation of the Dynamic Window Approach (DWA) algorithm tailored for ship collision avoidance scenarios. It visually demonstrates how a vessel (own ship) can navigate safely to a target point while avoiding both static and dynamic obstacles (other ships).

该项目使用 Python 实现了动态窗口法 (DWA)，并将其应用于船舶避碰场景。它直观地展示了本船如何在规避静态及动态障碍物（目标船）的同时，安全航行至目标点。

## 演示 (Demo)

*(建议您运行代码后，将生成的GIF动图替换下面的占位符，例如将其命名为 `dwa_ship_avoidance.gif` 并放在仓库根目录)*

![DWA Ship Collision Avoidance GIF](./dwa_ship_avoidance.gif)

*上图展示了在左交叉会遇场景下的避碰效果。*

## 主要特性 (Features)

- **核心算法**: 完整实现了动态窗口避碰算法 (DWA)。
- **动态仿真**: 能够实时模拟本船和目标船的运动轨迹。
- **多场景支持**: 可轻松配置静态障碍物和动态目标船，模拟如对遇、交叉、追越等多种会遇场景。
- **轨迹预测**: 对本船和目标船的未来轨迹进行预测，作为决策依据。
- **可视化**:
    - 使用 `Matplotlib` 实时绘制仿真环境，包括船体、历史轨迹、预测轨迹和最优轨迹。
    - 动态展示所有采样轨迹及其评价值（通过颜色深浅表示）。
    - 可选择性地显示/隐藏所有采样轨迹（按`T`键）。
- **数据导出**:
    - 自动保存仿真过程为独立的PNG图表，如**速度-航向变化图**和**两船距离变化图**。
    - （已注释）支持将仿真动画保存为GIF文件。
- **高度可配置**: 所有仿真参数（如船舶性能、评价函数权重、障碍物信息等）均在 `Config` 类中集中管理，方便调整和实验。

    ```

## 如何使用 (Usage)

直接运行Python脚本即可启动仿真：

```bash
python DWAforshipcollisionavoidance.py
```

在仿真窗口中，可以通过按键盘上的 `T` 键来切换是否显示所有采样的轨迹。

## 配置与自定义 (Configuration)

所有的仿真参数都可以在 `DWAforshipcollisionavoidance.py` 文件中的 `Config` 类里进行修改。

### 1. 修改会遇场景

通过修改 `self.moving_obstacles` 列表，可以轻松定义不同的会遇场景。

```python
class Config:
    def __init__(self):
        # ...
        self.moving_obstacles = [
            # 不同的会遇场景示例 (取消注释即可启用)
            #MovingObstacle(13.0, 6.0, yaw=math.pi, v=0.7, track_length=self.track_length, color='gold'), # 右交叉
            MovingObstacle(0, 10.0, yaw= -math.pi*0.2, v=0.5, track_length=self.track_length, color='gold'), # 左交叉
           # MovingObstacle(9.5, 8.0, yaw=math.pi+math.atan2(8, 10), v=0.6, track_length=self.track_length, color='gold'),# 对遇
            #MovingObstacle(5.0, 4.0, yaw=math.pi/4, v=0.3, track_length=self.track_length, color='orange') # 追越
        ]
        # ...
```

### 2. 调整评价函数权重

DWA算法的核心在于其评价函数 `G`。你可以通过调整以下四个权重来改变船舶的驾驶行为和避碰策略。

`G = alpha * heading + beta * dist + gamma * velocity + delta * dyn_dist`

| 参数    | 描述                     | 作用                                     |
| :------ | :----------------------- | :--------------------------------------- |
| `alpha` | **航向评价权重**         | 控制船舶朝向目标点的倾向。值越大，越倾向于对准目标。 |
| `beta`  | **(静态)障碍物距离评价权重** | 控制船舶与静态障碍物的安全距离。值越大，越倾向于远离障碍物。 |
| `gamma` | **速度评价权重**         | 鼓励船舶保持较快的速度。值越大，越倾向于高速航行。 |
| `delta` | **动态障碍物距离评价权重** | 控制船舶与其它移动船舶的安全距离。值越大，越倾向于主动规避。 |

```python
class Config:
    def __init__(self):
        # ...
        # 轨迹评价函数系数
        self.alpha = 0.15 # 航向评价权重
        self.beta = 1.0 # 距离评价权重
        self.gamma = 0.7 # 速度评价权重
        self.delta = 0.55  # 动态障碍物预测轨迹权重
        # ...
```

### 3. 修改船舶性能参数

你也可以修改船舶的物理限制，例如最大/最小速度、最大角速度和加速度等。

```python
class Config:
    def __init__(self):
        # robot parameter
        # 线速度边界
        self.v_max = 0.8 # [m/s]
        self.v_min = 0.2  # [m/s]
        # 角速度边界
        self.w_max = 40.0 * math.pi / 180.0  # [rad/s]
        self.w_min = -40.0 * math.pi / 180.0  # [rad/s]
        # 线加速度和角加速度最大值  
        self.a_vmax = 0.2  # [m/ss]
        self.a_wmax = 40.0 * math.pi / 180.0  # [rad/ss]
        # ...
```

## 输出文件 (Outputs)

运行脚本后，除了显示实时仿真窗口，还会在当前目录下生成以下分析图表：

1.  `ships_heading_speed_plot.png`: 本船与目标船的航向和速度随时间变化的组合图。
2.  `ships_distance_plot.png`: 两船之间的距离随时间变化的图表。
