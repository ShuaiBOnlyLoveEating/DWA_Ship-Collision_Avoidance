import numpy as np
import matplotlib.pyplot as plt
import copy
from celluloid import Camera  # 保存动图时用，pip install celluloid
import math
from matplotlib.font_manager import FontProperties
import os
import tempfile

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体
plt.rcParams['axes.unicode_minus'] = False    # 解决负号显示问题

class MovingObstacle:
    """
    移动障碍物类
    """
    def __init__(self, x, y, yaw=0.0, v=0.0, track_length=50, color='r'):
        """初始化移动障碍物

        Args:
            x (float): 初始x坐标
            y (float): 初始y坐标
            yaw (float, optional): 初始朝向角度(弧度). 默认为0.0.
            v (float, optional): 初始速度. 默认为0.0.
            track_length (int, optional): 轨迹记录的最大长度. 默认为50.
            color (str, optional): 障碍物和轨迹的颜色. 默认为'r'.
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.color = color
        self.track_length = track_length
        # 记录历史轨迹
        self.track = [[x, y]]
        # 停止移动标志
        self.stopped = False
    
    def update(self, dt, bounds=None):
        """更新障碍物位置

        Args:
            dt (float): 时间步长
            bounds (tuple, optional): 边界 (x_min, x_max, y_min, y_max). 默认为None.
        """
        # 如果已停止，不再更新位置
        if self.stopped:
            return
            
        # 计算新位置
        new_x = self.x + self.v * math.cos(self.yaw) * dt
        new_y = self.y + self.v * math.sin(self.yaw) * dt
        
        # 检查是否超出边界
        if bounds is not None:
            x_min, x_max, y_min, y_max = bounds
            if new_x < x_min or new_x > x_max or new_y < y_min or new_y > y_max:
                self.stopped = True
                return
                
        # 更新位置
        self.x = new_x
        self.y = new_y
        
        # 记录当前位置到轨迹中
        self.track.append([self.x, self.y])
        
        # 限制轨迹长度，防止内存占用过大
        if len(self.track) > self.track_length:
            self.track.pop(0)
    
    def get_position(self):
        """获取障碍物当前位置

        Returns:
            np.array: [x, y]位置坐标
        """
        return np.array([self.x, self.y])
    
    def get_track(self):
        """获取障碍物轨迹

        Returns:
            np.array: 轨迹坐标点数组
        """
        return np.array(self.track)
    
    def predict_trajectory(self, dt, predict_time):
        """预测障碍物未来轨迹

        Args:
            dt (float): 时间步长
            predict_time (float): 预测时间长度

        Returns:
            np.array: 预测的轨迹点数组，形状为(n, 2)，每行是[x, y]
        """
        # 初始化预测轨迹数组
        predicted_trajectory = [[self.x, self.y]]
        
        # 预测位置
        x = self.x
        y = self.y
        t = 0
        
        # 在预测时间内按照当前速度和方向移动
        while t < predict_time:
            x += self.v * math.cos(self.yaw) * dt
            y += self.v * math.sin(self.yaw) * dt
            t += dt
            predicted_trajectory.append([x, y])
        
        return np.array(predicted_trajectory)

class Config:
    """
    simulation parameter class
    """
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
        # 采样分辨率 
        self.v_sample = 0.01  # [m/s]
        self.w_sample = 0.1 * math.pi / 180.0  # [rad/s]
        # 离散时间
        self.dt = 0.1  # [s] Time tick for motion prediction
        # 轨迹推算时间长度
        self.predict_time = 3.0  # [s]
        # 轨迹评价函数系数
        self.alpha = 0.15 # 航向评价权重
        self.beta = 1.0 # 距离评价权重
        self.gamma = 0.7 # 速度评价权重
        self.delta = 0.55  # 动态障碍物预测轨迹权重

        # Also used to check if goal is reached in both types
        self.robot_radius = 0.5  # [m] for collision check
        
        self.judge_distance = 5 # 若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值

        # 静态障碍物位置 [x(m) y(m), ....]
        self.static_ob = np.array([
                    [2.5, 4],
                    #[7.0, 13.0],
                    [10.0, 10.0]
                    ])
                    
        # 轨迹记录长度
        self.track_length = 300  # 增加轨迹长度
        
        # 屏幕边界设置 (x_min, x_max, y_min, y_max)
        self.bounds = (-5, 25, -5, 25)
                    
        # 动态障碍物列表
       
        self.moving_obstacles = [
            #MovingObstacle(13.0, 6.0, yaw=math.pi, v=0.7, track_length=self.track_length, color='gold'), #右交叉
            MovingObstacle(0, 10.0, yaw= -math.pi*0.2, v=0.5, track_length=self.track_length, color='gold'), #左交叉
           # MovingObstacle(9.5, 8.0, yaw=math.pi+math.atan2(8, 10), v=0.6, track_length=self.track_length, color='gold'),#对遇
            #MovingObstacle(5.0, 4.0, yaw=math.pi/4, v=0.3, track_length=self.track_length, color='orange') #追越
        ]
        
        # 初始化合并后的障碍物位置矩阵
        self.ob = self.update_obstacle_positions()
        
        # 目标点位置
        self.target = np.array([14,14])
        
        # 显示轨迹配置
        self.show_obstacle_track = True
        self.track_alpha = 0.5  # 轨迹透明度
        
        # 显示预测轨迹配置
        self.show_prediction = True
        self.prediction_alpha = 0.3  # 预测轨迹透明度
    
    def update_obstacle_positions(self):
        """更新并合并障碍物位置
        
        Returns:
            np.array: 所有障碍物的位置矩阵
        """
        # 获取所有移动障碍物的当前位置
        moving_ob_positions = np.array([obs.get_position() for obs in self.moving_obstacles])
        
        # 如果没有移动障碍物，直接返回静态障碍物位置
        if len(moving_ob_positions) == 0:
            return self.static_ob
            
        # 合并静态和移动障碍物位置
        return np.vstack((self.static_ob, moving_ob_positions))
        
    def update_moving_obstacles(self):
        """更新所有移动障碍物的位置"""
        for obstacle in self.moving_obstacles:
            obstacle.update(self.dt, self.bounds)
        
        # 更新合并后的障碍物位置矩阵
        self.ob = self.update_obstacle_positions()
    
    def predict_obstacles_trajectories(self):
        """预测所有移动障碍物的未来轨迹
        
        Returns:
            list: 每个障碍物的预测轨迹列表
        """
        predicted_trajectories = []
        for obstacle in self.moving_obstacles:
            predicted_trajectory = obstacle.predict_trajectory(self.dt, self.predict_time)
            predicted_trajectories.append(predicted_trajectory)
        return predicted_trajectories

class DWA:
    def __init__(self,config) -> None:
        """初始化

        Args:
            config (_type_): 参数类
        """
        self.dt=config.dt
        self.v_min=config.v_min
        self.w_min=config.w_min
        self.v_max=config.v_max
        self.w_max=config.w_max
        self.predict_time = config.predict_time
        self.a_vmax = config.a_vmax
        self.a_wmax = config.a_wmax
        self.v_sample = config.v_sample # 线速度采样分辨率
        self.w_sample = config.w_sample # 角速度采样分辨率
        self.alpha = config.alpha
        self.beta = config.beta
        self.gamma = config.gamma
        self.delta = config.delta  # 动态障碍物权重
        self.radius = config.robot_radius
        self.judge_distance = config.judge_distance

    def dwa_control(self, state, goal, obstacle, moving_obstacles=None):
        """滚动窗口算法入口

        Args:
            state (_type_): 机器人当前状态--[x,y,yaw,v,w]
            goal (_type_): 目标点位置，[x,y]
            obstacle (_type_): 障碍物位置，dim:[num_ob,2]
            moving_obstacles (list, optional): 移动障碍物对象列表. Defaults to None.

        Returns:
            _type_: 控制量、最优轨迹、所有采样轨迹、控制量列表、评分列表
        """
        control, trajectory, all_trajectories, all_controls, all_scores = self.trajectory_evaluation(state, goal, obstacle, moving_obstacles)
        return control, trajectory, all_trajectories, all_controls, all_scores


    def cal_dynamic_window_vel(self,v,w,state,obstacle):
        """速度采样,得到速度空间窗口

        Args:
            v (_type_): 当前时刻线速度
            w (_type_): 当前时刻角速度
            state (_type_): 当前机器人状态
            obstacle (_type_): 障碍物位置
        Returns:
            [v_low,v_high,w_low,w_high]: 最终采样后的速度空间
        """
        Vm = self.__cal_vel_limit()
        Vd = self.__cal_accel_limit(v,w)
        Va = self.__cal_obstacle_limit(state,obstacle)
        a = max([Vm[0],Vd[0],Va[0]])
        b = min([Vm[1],Vd[1],Va[1]])
        c = max([Vm[2], Vd[2],Va[2]])
        d = min([Vm[3], Vd[3],Va[3]])
        return [a,b,c,d]

    def __cal_vel_limit(self):
        """计算速度边界限制Vm

        Returns:
            _type_: 速度边界限制后的速度空间Vm
        """
        return [self.v_min,self.v_max,self.w_min,self.w_max]
    
    def __cal_accel_limit(self,v,w):
        """计算加速度限制Vd

        Args:
            v (_type_): 当前时刻线速度
            w (_type_): 当前时刻角速度
        Returns: 
            _type_:考虑加速度时的速度空间Vd
        """
        v_low = v-self.a_vmax*self.dt
        v_high = v+self.a_vmax*self.dt
        w_low = w-self.a_wmax*self.dt
        w_high = w+self.a_wmax*self.dt
        return [v_low, v_high,w_low, w_high]
    
    def __cal_obstacle_limit(self,state,obstacle):
        """环境障碍物限制Va

        Args:
            state (_type_): 当前机器人状态
            obstacle (_type_): 障碍物位置

        Returns:
            _type_: 某一时刻移动机器人不与周围障碍物发生碰撞的速度空间Va
        """
        v_low=self.v_min
        v_high = np.sqrt(2*self._dist(state,obstacle)*self.a_vmax)
        w_low =self.w_min
        w_high = np.sqrt(2*self._dist(state,obstacle)*self.a_wmax)
        return [v_low,v_high,w_low,w_high]

    def trajectory_predict(self,state_init, v,w):
        """轨迹推算

        Args:
            state_init (_type_): 当前状态---x,y,yaw,v,w
            v (_type_): 当前时刻线速度
            w (_type_): 当前时刻线速度

        Returns:
            _type_: _description_
        """
        state = np.array(state_init)
        trajectory = state.reshape(1, -1)  # 确保是二维的
        time = 0
        # 在预测时间段内
        while time <= self.predict_time:
            x = KinematicModel(state, [v,w], self.dt) # 运动学模型
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory

    def trajectory_evaluation(self, state, goal, obstacle, moving_obstacles=None):
        """轨迹评价函数,评价越高，轨迹越优

        Args:
            state (_type_): 当前状态---x,y,yaw,v,w
            goal (_type_): 目标点位置，[x,y]
            obstacle (_type_): 障碍物位置，dim:[num_ob,2]
            moving_obstacles (list, optional): 移动障碍物对象列表. Defaults to None.

        Returns:
            _type_: 最优控制量、最优轨迹、所有采样轨迹列表
        """
        G_max = -float('inf') # 最优评价
        trajectory_opt = state.reshape(1, -1) # 最优轨迹，确保二维
        control_opt = [0.,0.] # 最优控制
        dynamic_window_vel = self.cal_dynamic_window_vel(state[3], state[4], state, obstacle) # 第1步--计算速度空间
        
        # 预测移动障碍物轨迹
        predicted_obstacle_trajectories = []
        if moving_obstacles is not None:
            for obs in moving_obstacles:
                predicted_trajectory = obs.predict_trajectory(self.dt, self.predict_time)
                predicted_obstacle_trajectories.append(predicted_trajectory)
        
        # 收集所有采样轨迹
        all_trajectories = []
        all_controls = []
        all_scores = []
        
        # 在速度空间中按照预先设定的分辨率采样
        sum_heading, sum_dist, sum_vel, sum_dyn_dist = 1, 1, 1, 1 # 不进行归一化
        for v in np.arange(dynamic_window_vel[0], dynamic_window_vel[1], self.v_sample):
            for w in np.arange(dynamic_window_vel[2], dynamic_window_vel[3], self.w_sample):

                trajectory = self.trajectory_predict(state, v, w)  # 第2步--轨迹推算

                # 基本评价函数
                heading_eval = self.alpha * self.__heading(trajectory, goal) / sum_heading
                dist_eval = self.beta * self.__dist(trajectory, obstacle) / sum_dist
                vel_eval = self.gamma * self.__velocity(trajectory) / sum_vel
                
                # 添加对动态障碍物的评估
                dyn_dist_eval = 0
                if moving_obstacles is not None and len(predicted_obstacle_trajectories) > 0:
                    dyn_dist_eval = self.delta * self.__dyn_obstacle_dist(
                        trajectory, predicted_obstacle_trajectories) / sum_dyn_dist
                
                # 总评价
                G = heading_eval + dist_eval + vel_eval + dyn_dist_eval
                
                # 保存当前轨迹和评分
                all_trajectories.append(trajectory)
                all_controls.append([v, w])
                all_scores.append(G)
                
                if G_max <= G:
                    G_max = G
                    trajectory_opt = trajectory
                    control_opt = [v, w]

        return control_opt, trajectory_opt, all_trajectories, all_controls, all_scores

    def _dist(self, state, obstacle):
        """计算当前移动机器人距离障碍物最近的几何距离

        Args:
            state (_type_): 当前机器人状态
            obstacle (_type_): 障碍物位置

        Returns:
            _type_: 移动机器人距离障碍物最近的几何距离
        """
        ox = obstacle[:, 0]
        oy = obstacle[:, 1]
        dx = state[0,None] - ox[:, None]
        dy = state[1,None] - oy[:, None]
        r = np.hypot(dx, dy)
        return np.min(r)

    def __dist(self, trajectory, obstacle):
        """距离评价函数
        表示当前速度下对应模拟轨迹与障碍物之间的最近距离；
        如果没有障碍物或者最近距离大于设定的阈值，那么就将其值设为一个较大的常数值。
        Args:
            trajectory (_type_): 轨迹，dim:[n,5]
            
            obstacle (_type_): 障碍物位置，dim:[num_ob,2]

        Returns:
            _type_: _description_
        """
        ox = obstacle[:, 0]
        oy = obstacle[:, 1]
        dx = trajectory[:, 0, None] - ox[None, :]
        dy = trajectory[:, 1, None] - oy[None, :]
        r = np.hypot(dx, dy)
        return np.min(r) if np.array(r < self.radius + 0.2).any() else self.judge_distance
    
    def __dyn_obstacle_dist(self, trajectory, predicted_obstacle_trajectories):
        """动态障碍物距离评价函数
        计算机器人预测轨迹与移动障碍物预测轨迹之间的最小距离
        
        Args:
            trajectory: 机器人轨迹, dim:[n,5]
            predicted_obstacle_trajectories: 列表，包含每个移动障碍物的预测轨迹
            
        Returns:
            float: 距离评价值
        """
        min_dist = float('inf')
        
        # 对轨迹中的每个时间点
        for t in range(min(len(trajectory), len(predicted_obstacle_trajectories[0]))):
            robot_pos = trajectory[t, 0:2]  # 机器人在t时刻的位置
            
            # 检查每个障碍物在t时刻的位置
            for obs_trajectory in predicted_obstacle_trajectories:
                if t < len(obs_trajectory):
                    obs_pos = obs_trajectory[t]  # 障碍物在t时刻的位置
                    dist = np.hypot(robot_pos[0] - obs_pos[0], robot_pos[1] - obs_pos[1])
                    min_dist = min(min_dist, dist)
        
        # 如果距离小于安全距离，返回一个较小的评价值；否则返回常数
        return min_dist if min_dist < self.radius + 1.0 else self.judge_distance

    def __heading(self, trajectory, goal):
        """方位角评价函数
        评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差

        Args:
            trajectory (_type_): 轨迹，dim:[n,5]
            goal (_type_): 目标点位置[x,y]

        Returns:
            _type_: 方位角评价数值
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = math.pi-abs(cost_angle)

        return cost

    def __velocity(self, trajectory):
        """速度评价函数， 表示当前的速度大小，可以用模拟轨迹末端位置的线速度的大小来表示

        Args:
            trajectory (_type_): 轨迹，dim:[n,5]

        Returns:
            _type_: 速度评价
        """
        return trajectory[-1, 3]


def KinematicModel(state,control,dt):
  """机器人运动学模型

  Args:
      state (_type_): 状态量---x,y,yaw,v,w
      control (_type_): 控制量---v,w,线速度和角速度
      dt (_type_): 离散时间

  Returns:
      _type_: 下一步的状态
  """
  state[0] += control[0] * math.cos(state[2]) * dt
  state[1] += control[0] * math.sin(state[2]) * dt
  state[2] += control[1] * dt
  state[3] = control[0]
  state[4] = control[1]

  return state


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
        # 创建一个细长五边形的小船形状而不是圆形
        ship_length = config.robot_radius * 2.2  # 船长
        ship_width = config.robot_radius * 1.0   # 船宽
        
        # 定义船的形状：五个顶点相对于中心点(x,y)的坐标
        # 船头在前，船尾在后
        points = np.array([
            [ship_length/2, 0],              # 船头
            [ship_length/6, ship_width/2],   # 右舷前部
            [-ship_length/2, ship_width/3],  # 右舷尾部
            [-ship_length/2, -ship_width/3], # 左舷尾部
            [ship_length/6, -ship_width/2],  # 左舷前部
        ])
        
        # 根据航向角旋转形状
        rot_mat = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])
        
        # 旋转并平移所有点
        rotated_points = np.dot(points, rot_mat.T) + np.array([x, y])
        
        # 创建并绘制多边形
        ship_polygon = plt.Polygon(rotated_points, closed=True, fill=True, color='blue', alpha=0.7)
        plt.gcf().gca().add_patch(ship_polygon)
        
        # 船头方向指示线
        # 不需要额外的方向线，因为船的形状本身就指示了方向


def plot_boat_obstacle(x, y, yaw, size=0.5, color='r'):
    """绘制船形障碍物
    
    Args:
        x (float): x坐标
        y (float): y坐标
        yaw (float): 航向角度
        size (float): 船大小比例系数
        color (str): 船颜色
    """
    # 创建一个细长五边形的船形状
    ship_length = size * 2.2
    ship_width = size * 1.0
    
    # 定义船的形状：五个顶点相对于中心点(x,y)的坐标
    points = np.array([
        [ship_length/2, 0],              # 船头
        [ship_length/6, ship_width/2],   # 右舷前部
        [-ship_length/2, ship_width/3],  # 右舷尾部
        [-ship_length/2, -ship_width/3], # 左舷尾部
        [ship_length/6, -ship_width/2],  # 左舷前部
    ])
    
    # 根据航向角旋转形状
    rot_mat = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])
    
    # 旋转并平移所有点
    rotated_points = np.dot(points, rot_mat.T) + np.array([x, y])
    
    # 创建并绘制多边形
    ship_polygon = plt.Polygon(rotated_points, closed=True, fill=True, color=color, alpha=0.7)
    plt.gcf().gca().add_patch(ship_polygon)


def main(config):
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.3, 0.0])
    # 保存起点位置用于可视化
    start_point = np.array([x[0], x[1]])
    # goal position [x(m), y(m)]
    goal = config.target

    # input [forward speed, yaw_rate]

    trajectory = np.array(x)
    ob = config.ob
    dwa = DWA(config)
    fig = plt.figure(1, figsize=(10, 8))
    frames = []  # 用于保存帧
    
    # 添加记录速度的列表
    robot_speeds = []  # 本船（机器人）速度
    obstacle_speeds = []  # 目标船（移动障碍物）速度
    iterations = []  # 迭代步数
    iteration_count = 0  # 迭代计数器
    
    # 添加记录航向的列表
    robot_headings = []  # 本船航向（弧度）
    obstacle_headings = []  # 目标船航向（弧度）
    
    # 添加记录两船间距的列表
    ship_distances = []  # 两船间距
    
    # 添加绘制所有采样轨迹的开关
    show_all_trajectories = True  # 设置为True显示所有采样轨迹
    all_traj_alpha = 0.2  # 采样轨迹透明度
    
    # 添加键盘交互功能
    def on_key(event):
        nonlocal show_all_trajectories
        if event.key == 't':  # 按t键切换是否显示所有轨迹
            show_all_trajectories = not show_all_trajectories
            print(f"显示所有采样轨迹: {'开启' if show_all_trajectories else '关闭'}")
        elif event.key == 'escape':
            exit(0)
    
    # 注册键盘事件
    plt.gcf().canvas.mpl_connect('key_release_event', on_key)
    
    # 设置帧率控制和GIF保存
    frame_interval = 3  # 每隔多少帧保存一次，减小GIF文件大小
    frame_counter = 0
    
    while True:
        # 预测障碍物轨迹用于DWA算法
        u, predicted_trajectory, all_trajectories, all_controls, all_scores = dwa.dwa_control(x, goal, ob, config.moving_obstacles)

        x = KinematicModel(x, u, config.dt)  # 更新机器人位置
        trajectory = np.vstack((trajectory, x))  # 存储历史轨迹
        
        # 更新移动障碍物位置
        config.update_moving_obstacles()
        ob = config.ob  # 获取更新后的障碍物位置
        
        # 预测障碍物轨迹用于可视化
        obstacle_predictions = config.predict_obstacles_trajectories()
        
        # 记录速度数据
        iteration_count += 1
        iterations.append(iteration_count)
        robot_speeds.append(x[3]*10+4) # 本船线速度
        
        
        heading_deg = ( 90 - x[2] * 180 / math.pi) % 360
        robot_headings.append(int(heading_deg))  # 本船航向（角度，整数，0-360）
        
        # 记录目标船速度和航向（如果有移动障碍物）
        if config.moving_obstacles:
            obstacle_speeds.append(config.moving_obstacles[0].v*10+1)  # 第一个移动障碍物的速度
            # 将弧度转换为角度，并调整为从正上方为0度，顺时针为正方向，范围0-360度
            heading_deg = ( config.moving_obstacles[0].yaw * 180 / math.pi-180+350) % 360
            obstacle_headings.append(int(heading_deg))  # 目标船航向（角度，整数，0-360）
            
            # 计算两船之间的距离
            dist = math.hypot(x[0] - config.moving_obstacles[0].x, x[1] - config.moving_obstacles[0].y)
            ship_distances.append(dist)
        else:
            obstacle_speeds.append(0.0)  # 如果没有移动障碍物，则速度为0
            obstacle_headings.append(0.0)  # 如果没有移动障碍物，则航向为0
            ship_distances.append(0.0)  # 如果没有移动障碍物，则距离为0
        
        plt.cla()
        
        # 创建图例句柄和标签列表
        legend_handles = []
        legend_labels = []
        
        # 绘制所有采样轨迹
        if show_all_trajectories:
            # 找出最高分和最低分，用于颜色映射
            if all_scores:
                max_score = max(all_scores)
                min_score = min(all_scores)
                score_range = max_score - min_score
                
                # 使用颜色映射显示不同评分的轨迹
                for i, traj in enumerate(all_trajectories):
                    # 如果所有分数相同，使用统一颜色
                    if score_range == 0:
                        color = 'lightblue'
                    else:
                        # 根据分数映射颜色，从浅蓝到深蓝
                        norm_score = (all_scores[i] - min_score) / score_range
                        color = plt.cm.Blues(norm_score * 0.8 + 0.2)  # 0.2-1.0范围的蓝色渐变
                    
                    # 绘制轨迹并保存第一条轨迹的句柄用于图例
                    if i == 0:
                        sample_traj_handle, = plt.plot(traj[:, 0], traj[:, 1], "-", color=color, alpha=all_traj_alpha)
                        legend_handles.append(sample_traj_handle)
                        legend_labels.append('采样轨迹')
                    else:
                        plt.plot(traj[:, 0], traj[:, 1], "-", color=color, alpha=all_traj_alpha)
        
        # 绘制最优轨迹（绿色，更明显）
        opt_traj_handle, = plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g", linewidth=2)
        legend_handles.append(opt_traj_handle)
        legend_labels.append('最优轨迹')
        
        # 绘制机器人当前位置
        robot_pos_handle, = plt.plot(x[0], x[1], "xr")
        # 不再将本船当前位置添加到图例中
        # legend_handles.append(robot_pos_handle)
        # legend_labels.append('本船当前位置')
        
        # 绘制目标点
        goal_handle, = plt.plot(goal[0], goal[1], "xb")
        legend_handles.append(goal_handle)
        legend_labels.append('终点')
        
        # 绘制起点
        start_handle, = plt.plot(start_point[0], start_point[1], "Dg", markersize=5)  # 使用绿色菱形标记起点
        legend_handles.append(start_handle)
        legend_labels.append('本船起点')
        
        # 绘制静态障碍物
        static_ob_handle, = plt.plot(config.static_ob[:, 0], config.static_ob[:, 1], "ok", markersize=8)
        legend_handles.append(static_ob_handle)
        legend_labels.append('静态障碍物')
        ''''''
        # 绘制移动障碍物及其轨迹
        for i, obs in enumerate(config.moving_obstacles):
            # 绘制障碍物当前位置为船形
            if i == 0:  # 只添加第一个移动障碍物到图例
                # 使用新的船形绘制函数
                plot_boat_obstacle(obs.x, obs.y, obs.yaw, size=config.robot_radius * 0.8, color=obs.color)
                
                # 为图例创建一个假的点
                moving_ob_handle, = plt.plot(obs.x, obs.y, "o", color=obs.color, markersize=0)
                legend_handles.append(moving_ob_handle)
                legend_labels.append('目标船')
                
                # 如果障碍物已停止，显示停止状态
                #if obs.stopped:
                 #   plt.text(obs.x, obs.y + 0.5, "已停止", fontsize=10, color=obs.color)
            else:
                # 其他障碍物也使用船形
                plot_boat_obstacle(obs.x, obs.y, obs.yaw, size=config.robot_radius * 0.8, color=obs.color)
                
                #if obs.stopped:
                 #   plt.text(obs.x, obs.y + 0.5, "已停止", fontsize=10, color=obs.color)
            
            # 绘制障碍物历史轨迹
            if config.show_obstacle_track:
                track = obs.get_track()
                if i == 0:  # 只添加第一个轨迹到图例
                    track_handle, = plt.plot(track[:, 0], track[:, 1], "-", color=obs.color, alpha=config.track_alpha)
                    legend_handles.append(track_handle)
                    legend_labels.append('目标船历史轨迹')
                else:
                    plt.plot(track[:, 0], track[:, 1], "-", color=obs.color, alpha=config.track_alpha)
            
            # 绘制障碍物预测轨迹
            if config.show_prediction and i < len(obstacle_predictions):
                pred = obstacle_predictions[i]
                if i == 0:  # 只添加第一个预测轨迹到图例
                    pred_handle, = plt.plot(pred[:, 0], pred[:, 1], "--", color=obs.color, alpha=config.prediction_alpha)
                    legend_handles.append(pred_handle)
                    legend_labels.append('目标船预测轨迹')
                else:
                    plt.plot(pred[:, 0], pred[:, 1], "--", color=obs.color, alpha=config.prediction_alpha)
                
        
        # 绘制机器人轨迹
        robot_track_handle, = plt.plot(trajectory[:, 0], trajectory[:, 1], "-r", alpha=0.7)
        # 将本船轨迹添加到图例中
        legend_handles.append(robot_track_handle)
        legend_labels.append('本船轨迹')
        
        plot_robot(x[0], x[1], x[2], config)
        plt.axis("equal")
        plt.grid(True)
        
        
        # 使用正确的句柄和标签创建图例
        plt.legend(legend_handles, legend_labels, loc='upper left')
        
        plt.pause(0.001)
        
        # 绘制完成后，捕获当前帧
        plt.draw()  # 确保绘图已完成
        frame_counter += 1
        if frame_counter % frame_interval == 0:
            # 保存当前图像为新文件
            buffer = plt.gcf().canvas.buffer_rgba()
            # 创建副本而不是引用
            frame_file = os.path.join(tempfile.gettempdir(), f"frame_{frame_counter:05d}.png")
            plt.savefig(frame_file)
            frames.append(frame_file)
            
        # 检查是否到达目标
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            # 确保最后一帧被捕获
            frame_file = os.path.join(tempfile.gettempdir(), f"frame_{frame_counter:05d}.png")
            plt.savefig(frame_file)
            frames.append(frame_file)
            print("Goal!!")
            break
    
    print("Done")
    
    
    # 单独绘制航向变化图
    # 创建一个包含两个子图的图形，共享x轴
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    # 左侧Y轴：速度
    ax1.plot(iterations, robot_speeds, 'r-', linewidth=2, label='本船速度')
    ax1.plot(iterations, obstacle_speeds, 'gold', linewidth=2, label='目标船速度')
    ax1.set_xlabel('迭代步数')
    ax1.set_ylabel('速度 (kn)', color='b')
    ax1.tick_params(axis='y', labelcolor='b')
    ax1.grid(True)
    
    # 创建共享X轴的第二个Y轴
    ax2 = ax1.twinx()
    
    # 右侧Y轴：航向
    ax2.plot(iterations, robot_headings, 'r--', linewidth=2, label='本船航向')
    ax2.plot(iterations, obstacle_headings, 'gold', linestyle='--', linewidth=2, label='目标船航向')
    ax2.set_ylabel('航向', color='g')
    ax2.tick_params(axis='y', labelcolor='g')
    
    # 合并两个轴的图例
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    # 调整子图之间的间距
    plt.tight_layout()
    
    # 保存航向和速度组合图像为PNG
    heading_speed_plot_filename = 'ships_heading_speed_plot.png'
    plt.savefig(heading_speed_plot_filename, dpi=300, bbox_inches='tight')
    print(f"航向和速度变化组合图已保存为: {heading_speed_plot_filename}")
    
    # 单独绘制两船间距变化图
    plt.figure(figsize=(10, 6))
    plt.plot(iterations, ship_distances, 'b-', linewidth=2)
    plt.xlabel('迭代步数')
    plt.ylabel('距离 (nmile)')
    plt.grid(True)
    
    # 保存距离图像为PNG
    distance_plot_filename = 'ships_distance_plot.png'
    plt.savefig(distance_plot_filename, dpi=300, bbox_inches='tight')
    print(f"距离变化图已保存为: {distance_plot_filename}")
    # 保存速度图像为PNG
    speed_plot_filename = 'ships_speed_plot.png'
    plt.savefig(speed_plot_filename, dpi=300, bbox_inches='tight')
    print(f"速度变化图已保存为: {speed_plot_filename}")
    
    # 保存动画为GIF
    '''
    if frames:
        print("正在保存GIF动画，请稍候...")
        try:
            import imageio.v2 as imageio  # 使用v2版本避免警告
            # 将捕获的帧转换为图像并保存为GIF
            # fps参数控制GIF动画的播放速度，值越小动画播放越慢
            # 这里将fps从5降低到2，使动画播放速度变慢
            imageio.mimsave('追越避碰.gif', [imageio.imread(f) for f in frames], fps=5)
            print(f"动画已保存为: 追越避碰.gif")
            
            # 删除临时文件
            for f in frames:
                if os.path.exists(f):
                    try:
                        os.remove(f)
                    except:
                        pass
        except ImportError:
            print("保存GIF需要安装imageio库: pip install imageio")
        except Exception as e:
            print(f"保存GIF时出错: {e}")
    '''
    plt.show()

if __name__=="__main__":
    main(Config())
