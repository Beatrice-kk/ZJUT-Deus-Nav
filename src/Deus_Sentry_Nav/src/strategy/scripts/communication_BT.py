#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
机器人策略控制节点 - 行为树版本

本文件使用行为树 (Behavior Tree) 的思想重构了机器人的决策逻辑。
核心优势:
- 模块化: 每个行为和判断都是一个独立的类，易于理解和修改。
- 可扩展性: 添加新行为（如“吃能量符”）只需增加新的行为节点，并插入到树中即可，不会破坏原有逻辑。
- 可读性: 决策逻辑由树的结构清晰地定义，比复杂的 if/else 嵌套更直观。

依赖库:
- py_trees: 核心行为树框架。
- py_trees_ros: py_trees 与 ROS 的集成库。
"""

'''
(Selector: MainStrategy)
    |
    +-- (Sequence: EscapeLogic)  <-- 优先级 1: 逃跑
    |      |
    |      +-- (Condition: IsHealthLow?)
    |      +-- (Action: GoHome)
    |
    +-- (Sequence: AttackLogic)  <-- 优先级 2: 进攻
    |      |
    |      +-- (Condition: IsHealthFull?)
    |      +-- (Action: GoCenter)
    |
    +-- (Sequence: IdlePatrol)   <-- 优先级 3: 巡逻
           |
           +-- (Condition: NOT IsHealthLow?)
           +-- (Condition: NOT IsHealthFull?)
           +-- (Action: PatrolCenter)

'''
import rospy
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import math
from threading import Thread, Lock
import threading

# PyQt 用于调试 UI
import sys
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer

# --- 行为树核心库 ---
import py_trees
import py_trees_ros


from Deus_Sentry_Nav.msg import ChaseCommand

##########################################################################################
# UI 调试类 (与之前版本相同, 用于数据显示)
##########################################################################################

class TrajectoryDebugger(QMainWindow):
    # ... (UI部分代码未变，此处省略以保持简洁) ...
    def __init__(self):
        super().__init__()
        self.setWindowTitle("轨迹调试器 (Trajectory Debugger)")
        self.resize(800, 600)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        self.plot_velocity = pg.PlotWidget(title="速度 (Velocity - vx, vy)")
        self.vx_line = self.plot_velocity.plot(pen='r', name="vx")
        self.vy_line = self.plot_velocity.plot(pen='b', name="vy")
        layout.addWidget(self.plot_velocity)
        self.plot_acc = pg.PlotWidget(title="加速度 (Acceleration - ax, ay)")
        self.ax_line = self.plot_acc.plot(pen='g', name="ax")
        self.ay_line = self.plot_acc.plot(pen='y', name="ay")
        layout.addWidget(self.plot_acc)
        self.label_position = QLabel("位置: (0.0, 0.0)")
        self.label_position.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(self.label_position)
        self.data_vx, self.data_vy, self.data_ax, self.data_ay = [], [], [], []
        self.max_points = 200
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.new_vx, self.new_vy, self.new_ax, self.new_ay, self.new_posx, self.new_posy = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    def update_data(self, vx, vy, ax, ay, posx, posy):
        self.new_vx, self.new_vy, self.new_ax, self.new_ay, self.new_posx, self.new_posy = vx, vy, ax, ay, posx, posy

    def refresh(self):
        self.data_vx.append(self.new_vx); self.data_vy.append(self.new_vy)
        self.data_ax.append(self.new_ax); self.data_ay.append(self.new_ay)
        if len(self.data_vx) > self.max_points:
            self.data_vx.pop(0); self.data_vy.pop(0); self.data_ax.pop(0); self.data_ay.pop(0)
        self.vx_line.setData(self.data_vx); self.vy_line.setData(self.data_vy)
        self.ax_line.setData(self.data_ax); self.ay_line.setData(self.data_ay)
        self.label_position.setText(f"位置: ({self.new_posx:.2f}, {self.new_posy:.2f})")

def build_ui():
    app = QApplication(sys.argv)
    dbg = TrajectoryDebugger()
    dbg.show()
    return app, dbg


##########################################################################################
# 辅助类和ROS相关类
##########################################################################################

class Point:
    """一个简单的二维点，用于存储目标位置。"""
    def __init__(self, x=0.0, y=0.0):
        self.x, self.y, self.z = x, y, 0.05

    def to_ros_pose(self, frame_id="world"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.x, self.y, self.z
        return pose

##########################################################################################
# 行为树节点 (Behavior Tree Nodes)
# 每个类代表一个独立的“行为”或“条件判断”
##########################################################################################

class IsGameRunning(py_trees.behaviour.Behaviour):
    """
    [条件节点] 检查比赛是否正在进行 (game_progress == 4)。
    """
    def __init__(self, name, controller):
        super(IsGameRunning, self).__init__(name)
        self.controller = controller

    def update(self):
        """每当行为树 tick 时，此方法被调用。"""
        if self.controller.navigator.game_progress == 4:
            self.feedback_message = "比赛进行中"
            return py_trees.common.Status.SUCCESS  # 条件满足，返回成功
        else:
            self.feedback_message = "等待比赛开始..."
            return py_trees.common.Status.FAILURE  # 条件不满足，返回失败

class IsHealthLow(py_trees.behaviour.Behaviour):
    """
    [条件节点] 检查机器人血量是否低于阈值。
    """
    def __init__(self, name, controller):
        super(IsHealthLow, self).__init__(name)
        self.controller = controller

    def update(self):
        threshold = self.controller.max_health * self.controller.low_health_threshold
        if self.controller.navigator.health < threshold:
            self.feedback_message = f"血量低 ({self.controller.navigator.health})!"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"血量健康 ({self.controller.navigator.health})"
            return py_trees.common.Status.FAILURE

class IsHealthFull(py_trees.behaviour.Behaviour):
    """
    [条件节点] 检查机器人血量是否已满。
    """
    def __init__(self, name, controller):
        super(IsHealthFull, self).__init__(name)
        self.controller = controller

    def update(self):
        if self.controller.navigator.health >= self.controller.max_health:
            self.feedback_message = "血量已满"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "血量未满"
            return py_trees.common.Status.FAILURE

class GoToPoint(py_trees.behaviour.Behaviour):
    """
    [动作节点] 控制机器人移动到一个指定的目标点。
    这是一个通用的移动行为。
    """
    def __init__(self, name, controller, point, speed):
        super(GoToPoint, self).__init__(name)
        self.controller = controller
        self.point = point
        self.speed = speed

    def initialise(self):
        """当此动作第一次被激活时调用，用于初始化。"""
        self.controller.navigator.CmdID = 1  # 设置导航命令ID
        self.controller.nav_speed = self.speed
        self.controller.go_to(self.point)
        rospy.loginfo(f"行为 '{self.name}' 启动: 前往目标点 ({self.point.x:.2f}, {self.point.y:.2f})")

    def update(self):
        """
        在动作执行期间，此方法被反复调用。
        我们通过检查 control_active 标志位来判断高层路径规划器是否还在运行。
        """
        if self.controller.control_active:
            # 只要还在导航控制中，就保持 RUNNING 状态，让行为树知道任务正在进行。
            self.feedback_message = "正在移动中..."
            return py_trees.common.Status.RUNNING
        else:
            # 如果 control_active 变为 False，说明导航已完成或被中断。
            self.feedback_message = "已到达目标点或导航结束"
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """当行为被中断或完成时调用，用于清理。"""
        rospy.loginfo(f"行为 '{self.name}' 终止，状态: {new_status}")

class PatrolCenter(py_trees.behaviour.Behaviour):
    """
    [动作节点] 在中心区域的几个点之间来回巡逻（游龙）。
    此行为使用底层控制，直接计算速度指令。
    """
    def __init__(self, name, controller):
        super(PatrolCenter, self).__init__(name)
        self.controller = controller

    def initialise(self):
        """初始化巡逻行为。"""
        rospy.loginfo("行为 '巡逻中心' 启动")
        self.controller.navigator.CmdID = 2
        self.controller.nav_speed = self.controller.loong_nav_speed
        self.controller.no_nav_mode = True  # 切换到不依赖高层导航的模式
        self.controller.control_active = True # 激活底层控制
        self.last_switch_time = time.time()
        self.center_index = 0
        # 设置第一个巡逻点
        self.controller.desire_placex = self.controller.center_point[self.center_index].x
        self.controller.desire_placey = self.controller.center_point[self.center_index].y

    def update(self):
        """在巡逻期间，定时切换目标点。"""
        if time.time() - self.last_switch_time > self.controller.loong_switch_time:
            self.center_index = (self.center_index + 1) % len(self.controller.center_point)
            self.controller.desire_placex = self.controller.center_point[self.center_index].x
            self.controller.desire_placey = self.controller.center_point[self.center_index].y
            self.last_switch_time = time.time()
            rospy.loginfo(f"巡逻中，切换到中心点: 索引 {self.center_index}")
        
        # 此行为会一直运行，直到被更高优先级的行为（如“回家”）中断。
        self.feedback_message = f"正在巡逻，目标点 {self.center_index}"
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """结束巡逻，清理状态。"""
        rospy.loginfo("行为 '巡逻中心' 终止")
        self.controller.no_nav_mode = False
        self.controller.control_active = False


######### 追击逻辑的行为树节点 #########
class IsChaseModeActive(py_trees.behaviour.Behaviour):
    """
    [条件节点] 检查是否收到了追击指令。
    """
    def __init__(self, name, controller):
        super(IsChaseModeActive, self).__init__(name)
        self.controller = controller

    def update(self):
        if self.controller.chase_active:
            self.feedback_message = "追击指令已激活！"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "无追击指令"
            return py_trees.common.Status.FAILURE

class ChaseEnemy(py_trees.behaviour.Behaviour):
    """
    [动作节点] 执行追击敌人的行为。
    """
    def __init__(self, name, controller):
        super(ChaseEnemy, self).__init__(name)
        self.controller = controller
        self.chase_target_point = Point()

    def initialise(self):
        """当追击行为开始时调用。"""
        rospy.loginfo("行为 '追击敌人' 启动")
        self.controller.navigator.CmdID = 3  # 可以为追击设置一个独特的命令ID
        self.controller.nav_speed = self.controller.default_nav_speed # 使用默认速度追击

        # --- 核心追击点计算逻辑 ---
        # 策略：选择朝向敌人方向，距离自身一定比例（如50%）的一个点作为目标。
        # 这样做可以避免目标点过远或直接撞向敌人，为后续微调留出空间。
        chase_distance_ratio = 0.5  # 追击距离比例，可以设为参数
        
        # 获取敌人相对位置
        rel_x = self.controller.relative_enemy_pos.x
        rel_y = self.controller.relative_enemy_pos.y

        # 计算追击目标点（在世界坐标系下）
        # 注意：这里的实现假设相对坐标是基于机器人自身坐标系的。
        # 如果相对坐标是基于世界坐标系的，计算方式会更简单。
        # 假设基于机器人坐标系，需要先转换到世界坐标系。
        # 为了简化，我们暂时假设一个简单的模型：直接在当前位置上加上部分相对位移。
        # 一个更鲁棒的实现需要坐标系转换。
        target_x = self.controller.current_placex + rel_x * chase_distance_ratio
        target_y = self.controller.current_placey + rel_y * chase_distance_ratio

        self.chase_target_point.x = target_x
        self.chase_target_point.y = target_y
        
        rospy.loginfo(f"计算追击点: ({target_x:.2f}, {target_y:.2f})")
        self.controller.go_to(self.chase_target_point)

    def update(self):
        """在追击过程中，保持RUNNING状态，直到被更高优先级的行为打断。"""
        # 只要追击标志位仍然为True，并且上层导航还在工作，就认为任务在进行中
        if self.controller.chase_active and self.controller.control_active:
            self.feedback_message = "正在追击敌人..."
            return py_trees.common.Status.RUNNING
        else:
            # 如果 chase_active 变为 False，说明外部指令取消了追击，任务完成。
            self.feedback_message = "追击指令已取消"
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """当追击行为被中断或完成时调用。"""
        rospy.loginfo(f"行为 '追击敌人' 终止，状态: {new_status}")

##########################################################################################
# 主控制器类
##########################################################################################

class VelocityController:
    def __init__(self, debugger):
        # 初始化ROS节点和发布者/订阅者
        rospy.init_node('communication_car', anonymous=True)
        self.debugger = debugger
        self.publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        # 加载所有参数
        self.load_parameters()
        

        self.chase_active = False
        self.relative_enemy_pos = Point() # 使用Point对象来存储相对位置




        # 初始化串口通信和各种状态变量
        self.navigator = NvigateData_t(self.uart_port, self.uart_bps, self.uart_timeout)
        self.last_update_time = time.time()
        self.current_vx, self.current_vy = 0.0, 0.0
        self.control_active = False # 标志位，表示底层控制是否激活
        self.no_nav_mode = False # 标志位，表示是否处于不依赖高层规划的底层控制模式
        
        # 机器人当前和期望的状态
        self.current_placex, self.current_placey, self.current_yaw = 0.0, 0.0, 0.0
        self.desire_placex, self.desire_placey = 0.0, 0.0
        self.desire_vx, self.desire_vy, self.desire_ax, self.desire_ay = 0.0, 0.0, 0.0, 0.0
        self.run_degree, self.now_dis, self.change_angle = 0.0, 0.0, 0.0
        self.nav_speed = self.default_nav_speed
        
        # 定义策略点
        self.home_point = Point(self.home_x, self.home_y)
        self.center_point = [
            Point(self.center_point1_x, self.center_point1_y),
            Point(self.center_point2_x, self.center_point2_y),
            Point(self.center_point3_x, self.center_point3_y)
        ]
        
        # 订阅话题
        rospy.Subscriber("/position_cmd", PositionCommand, self.velocity_callback)
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)

        # 追击话题
        rospy.Subscriber("/chase_cmd", ChaseCommand, self.chase_callback) 

        # 定时器，用于底层控制循环
        rospy.Timer(rospy.Duration(1.0 / self.control_loop_rate), self.control_loop)
        
        # --- 行为树核心 ---
        # 1. 创建行为树
        self.bt = self.create_behavior_tree()
        # 2. 创建一个独立的线程来运行行为树的 "tick"
        self.bt_thread = threading.Thread(target=self.strategy_loop, daemon=True)

    def load_parameters(self):
        """从ROS参数服务器加载所有参数。"""
        self.uart_port = rospy.get_param('~uart_port', -1)
        self.uart_bps = rospy.get_param('~uart_bps', 115200)
        self.uart_timeout = rospy.get_param('~uart_timeout', 5)
        self.default_nav_speed = rospy.get_param('~default_nav_speed', 0.5)
        self.home_nav_speed = rospy.get_param('~home_nav_speed', 0.6)
        self.loong_nav_speed = rospy.get_param('~loong_nav_speed', 0.42)
        self.timeout_threshold = rospy.get_param('~timeout_threshold', 0.5)
        self.home_x, self.home_y, self.home_z = rospy.get_param('~home_x',-0.1), rospy.get_param('~home_y',-0.1), rospy.get_param('~home_z',0.05)
        self.center_point1_x, self.center_point1_y = rospy.get_param('~center_point1_x', 4.6), rospy.get_param('~center_point1_y', 2.47)
        self.center_point2_x, self.center_point2_y = rospy.get_param('~center_point2_x', 4.66), rospy.get_param('~center_point2_y', 1.7)
        self.center_point3_x, self.center_point3_y = rospy.get_param('~center_point3_x', 3.95), rospy.get_param('~center_point3_y', 2.3)
        self.max_health = rospy.get_param('~max_health', 400)
        self.low_health_threshold = rospy.get_param('~low_health_threshold', 0.4)
        self.loong_switch_time = rospy.get_param('~loong_switch_time', 3.5)
        self.strategy_loop_rate = rospy.get_param('~strategy_loop_rate', 10) # 行为树更新频率 (10 Hz)
        self.control_loop_rate = rospy.get_param('~control_loop_rate', 33.33)
        rospy.loginfo("所有参数加载完成。")

    def create_behavior_tree(self):
        """
        核心函数: 创建并组织行为树。
        树的结构定义了机器人的决策逻辑和优先级。
        """
        
        # --- 1. 创建所有行为和条件的实例 ---
        # 动作节点
        go_home = GoToPoint("回家", self, self.home_point, self.home_nav_speed)
        go_center = GoToPoint("前往中心点", self, self.center_point[0], self.default_nav_speed)
        patrol_center = PatrolCenter("巡逻中心", self)

        # 条件节点
        is_game_running = IsGameRunning("比赛是否进行中", self)
        is_health_low = IsHealthLow("血量是否过低", self)
        is_health_full = IsHealthFull("血量是否已满", self)
        
        # --- 2. 构建树的层级结构 ---
        
        # 逃跑逻辑: 这是一个序列(Sequence)，意味着必须“血量低” AND “执行回家”
        escape_logic = py_trees.composites.Sequence("逃跑逻辑", memory=True)
        escape_logic.add_children([is_health_low, go_home])

        # 进攻逻辑: 这是一个序列(Sequence)，意味着必须“血量满” AND “执行前往中心”
        attack_logic = py_trees.composites.Sequence("进攻逻辑", memory=True)
        attack_logic.add_children([is_health_full, go_center])

        # 巡逻逻辑: 这是一个序列(Sequence)，意味着必须“血量不是低的” AND “血量不是满的” AND "执行巡逻"
        # Inverter 是一个装饰器，它会将子节点的 SUCCESS 变为 FAILURE，反之亦然。
        idle_patrol_logic = py_trees.composites.Sequence("空闲巡逻逻辑", memory=True)
        idle_patrol_logic.add_children([py_trees.decorators.Inverter(is_health_low), 
                                        py_trees.decorators.Inverter(is_health_full),
                                        patrol_center])

        # 主策略: 这是一个选择器(Selector)，它定义了所有行为的优先级。
        # 它会从左到右尝试每个子节点，优先执行第一个可以运行的逻辑。
        # 优先级: 逃跑 > 进攻 > 巡逻
        main_strategy = py_trees.composites.Selector("主策略", memory=False)
        main_strategy.add_children([escape_logic, attack_logic, idle_patrol_logic])
        
        # 游戏分支: 只有当游戏正在进行时，主策略才会被激活。
        game_active_branch = py_trees.composites.Sequence("游戏激活分支", memory=True)
        game_active_branch.add_children([is_game_running, main_strategy])

        # 根节点: 整个行为树的入口。
        root = py_trees.composites.Selector("机器人总根节点", memory=False)
        root.add_children([game_active_branch])

        rospy.loginfo("行为树创建完成。")
        # 在控制台打印出树的结构，非常便于调试和理解
        py_trees.display.print_ascii_tree(root)
        
        # 将 py_trees 包装成 ROS 兼容的形式
        return py_trees_ros.trees.BehaviourTree(root)

    def strategy_loop(self):
        """
        行为树的“心跳”。这个循环以固定频率 "tick" 行为树，驱动决策的执行。
        """
        rate = rospy.Rate(self.strategy_loop_rate)
        while not rospy.is_shutdown():
            self.bt.tick()  # 每一次 tick 都会从根节点开始重新评估整个树
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        rospy.loginfo("策略决策循环已停止。")

    def go_to(self, point):
        """向导航系统发布一个目标点。"""
        self.publisher.publish(point.to_ros_pose())

    def odom_callback(self, msg):
        """里程计回调，更新机器人当前位姿。"""
        self.current_placex = msg.pose.pose.position.x
        self.current_placey = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y**2 + o.z**2))
        self.current_yaw = math.degrees(yaw)

    def velocity_callback(self, msg):
        """高层规划器（如 move_base）的速度指令回调。"""
        self.desire_placex, self.desire_placey = msg.position.x, msg.position.y
        self.desire_vx, self.desire_vy = msg.velocity.x, msg.velocity.y
        self.desire_ax, self.desire_ay = msg.acceleration.x, msg.acceleration.y
        self.last_update_time = time.time()
        # 收到高层指令，意味着导航正在进行，激活底层控制
        self.control_active = True
        # 更新UI
        self.debugger.update_data(self.desire_vx, self.desire_vy, self.desire_ax, self.desire_ay, self.desire_placex, self.desire_placey)


    def chase_callback(self, msg):
        """
        回调函数，用于接收和处理来自 /chase_cmd 话题的追击指令。
        """
        self.chase_active = msg.chase_active
        if self.chase_active:
            self.relative_enemy_pos.x = msg.relative_position.x
            self.relative_enemy_pos.y = msg.relative_position.y
            # 可以在此处添加日志，方便调试
            # rospy.loginfo_throttle(1.0, f"收到追击指令: active={self.chase_active}, rel_pos=({self.relative_enemy_pos.x:.2f}, {self.relative_enemy_pos.y:.2f})")

    def create_behavior_tree(self):
        """
        【修改】核心函数: 创建并组织行为树。
        在这里加入新的“追击”逻辑。
        """
        
        # --- 1. 创建所有行为和条件的实例 (新增追击相关的节点) ---
        go_home = GoToPoint("回家", self, self.home_point, self.home_nav_speed)
        go_center = GoToPoint("前往中心点", self, self.center_point[0], self.default_nav_speed)
        patrol_center = PatrolCenter("巡逻中心", self)
        is_game_running = IsGameRunning("比赛是否进行中", self)
        is_health_low = IsHealthLow("血量是否过低", self)
        is_health_full = IsHealthFull("血量是否已满", self)
        
        # 新增的追击节点
        is_chase_mode_active = IsChaseModeActive("是否处于追击模式", self)
        chase_enemy = ChaseEnemy("追击敌人", self)

        # --- 2. 构建树的层级结构 (调整优先级) ---
        
        # 逃跑逻辑 (优先级最高: 1)
        escape_logic = py_trees.composites.Sequence("逃跑逻辑", memory=True)
        escape_logic.add_children([is_health_low, go_home])

        # 【新增】追击逻辑 (优先级: 2)
        # 序列(Sequence)节点: 必须“追击模式激活” AND “执行追击行为”
        chase_logic = py_trees.composites.Sequence("追击逻辑", memory=True)
        chase_logic.add_children([is_chase_mode_active, chase_enemy])

        # 进攻逻辑 (优先级调整为: 3)
        attack_logic = py_trees.composites.Sequence("进攻逻辑", memory=True)
        attack_logic.add_children([is_health_full, go_center])

        # 巡逻逻辑 (优先级最低: 4)
        idle_patrol_logic = py_trees.composites.Sequence("空闲巡逻逻辑", memory=True)
        idle_patrol_logic.add_children([py_trees.decorators.Inverter(is_health_low), 
                                        py_trees.decorators.Inverter(is_health_full),
                                        py_trees.decorators.Inverter(is_chase_mode_active), # 【修改】确保巡逻前没有追击任务
                                        patrol_center])

        # 主策略选择器 (Selector)，定义了所有行为的优先级
        # 【修改】将 chase_logic 插入到正确的位置
        main_strategy = py_trees.composites.Selector("主策略", memory=False)
        main_strategy.add_children([
            escape_logic,         # 1. 优先逃跑
            chase_logic,          # 2. 其次追击
            attack_logic,         # 3. 然后进攻
            idle_patrol_logic     # 4. 最后巡逻
        ])
        
        # ... (根节点的构建保持不变) ...
        game_active_branch = py_trees.composites.Sequence("游戏激活分支", memory=True)
        game_active_branch.add_children([is_game_running, main_strategy])
        root = py_trees.composites.Selector("机器人总根节点", memory=False)
        root.add_children([game_active_branch])

        rospy.loginfo("行为树已更新，包含追击逻辑。")
        py_trees.display.print_ascii_tree(root, show_status=True) # 打印带状态的树结构
        
        return py_trees_ros.trees.BehaviourTree(root)



    def control_loop(self, event):
        """底层控制循环，以较高频率运行，直接计算并发送给串口。"""
        # 超时检查：如果长时间没收到高层指令，并且不是在底层巡逻模式，就停止运动。
        if time.time() - self.last_update_time > self.timeout_threshold and not self.no_nav_mode:
            self.current_vx, self.current_vy, self.control_active = 0.0, 0.0, False

        if self.control_active:
            # --- 这部分是底层的位置+速度+加速度前馈控制器，与之前版本逻辑相同 ---
            theta_rad = math.atan2(self.desire_placey - self.current_placey, self.desire_placex - self.current_placex)
            self.run_degree = self.current_yaw - math.degrees(theta_rad)
            self.now_dis = min(math.sqrt((self.current_placex - self.desire_placex) ** 2 + (self.current_placey - self.desire_placey) ** 2), 1.2)
            
            v_pos_y = self.nav_speed * self.now_dis * math.sin(math.radians(self.run_degree))
            v_pos_x = self.nav_speed * self.now_dis * math.cos(math.radians(self.run_degree))
            
            cmd_vx = v_pos_x + self.desire_vx
            cmd_vy = v_pos_y + self.desire_vy
            cmd_vx += 0.2 * self.desire_ax
            cmd_vy += 0.2 * self.desire_ay
            
            speed_norm = math.sqrt(cmd_vx ** 2 + cmd_vy ** 2)
            if speed_norm > 1.4:
                scale = 1.4 / speed_norm
                cmd_vx *= scale; cmd_vy *= scale
            
            self.current_vx, self.current_vy = cmd_vx, cmd_vy
            self.change_angle = self.run_degree if self.run_degree > 90 or self.run_degree < -90 else 0.0
        
        try:
            # 发送指令到串口
            self.navigator.set_data(vx=self.current_vx, vy=self.current_vy, vz=self.change_angle, control=self.control_active)
            self.change_angle = 0.0
            self.navigator.send()
        except Exception as e:
            rospy.logerr(f"串口通信失败: {str(e)}")

    def start_threads(self):
        """启动所有后台线程。"""
        # 启动串口读取线程
        self.navigator_thread = threading.Thread(target=self.navigator.start, daemon=True)
        self.navigator_thread.start()
        # 启动行为树决策线程
        self.bt_thread.start()
        rospy.loginfo("所有核心线程已启动。")

    def shutdown_hook(self):
        """在ROS节点关闭时被调用，用于安全地停止机器人。"""
        rospy.loginfo("关闭指令已接收。")
        if hasattr(self, 'navigator'):
            self.navigator.set_data(0, 0, 0, False) # 发送停止指令
            self.navigator.send()
            self.navigator.close_uart()
        rospy.loginfo("机器人已停止，串口已关闭。")

if __name__ == '__main__':
    try:
        print("启动 PyQtGraph 监视器...")
        app, debugger = build_ui()

        # 创建主控制器实例
        controller = VelocityController(debugger)
        # 注册关闭时的回调函数
        rospy.on_shutdown(controller.shutdown_hook)
        
        # 启动所有线程
        controller.start_threads()

        # 将 ROS 的 spin() 放在一个单独的线程中，以防它阻塞主线程（GUI线程）
        spin_thread = threading.Thread(target=rospy.spin, daemon=True)
        spin_thread.start()
        
        # 将 Qt 的退出信号连接到 ROS 的关闭流程
        app.aboutToQuit.connect(lambda: rospy.signal_shutdown('GUI已关闭'))

        # 启动 Qt 的事件循环（这将阻塞主线程，直到GUI关闭）
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序发生未处理的异常: {e}")
