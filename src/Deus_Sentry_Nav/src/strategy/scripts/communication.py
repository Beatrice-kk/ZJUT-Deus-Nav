#!/usr/bin/env python
import rospy
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from navigate_UART import NvigateData_t  # 替换为实际串口模块路径
from threading import Thread
import threading


import sys
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer



class TrajectoryDebugger(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Trajectory Debugger")
        self.resize(800, 600)

        # 中央控件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 速度曲线
        self.plot_velocity = pg.PlotWidget(title="Velocity (vx, vy)")
        self.vx_line = self.plot_velocity.plot(pen='r', name="vx")
        self.vy_line = self.plot_velocity.plot(pen='b', name="vy")
        layout.addWidget(self.plot_velocity)

        # 加速度曲线
        self.plot_acc = pg.PlotWidget(title="Acceleration (ax, ay)")
        self.ax_line = self.plot_acc.plot(pen='g', name="ax")
        self.ay_line = self.plot_acc.plot(pen='y', name="ay")
        layout.addWidget(self.plot_acc)

        # 显示位置
        self.label_position = QLabel("Position: (0.0, 0.0)")
        self.label_position.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(self.label_position)

        # 数据存储
        self.data_vx, self.data_vy = [], []
        self.data_ax, self.data_ay = [], []
        self.max_points = 200

        # 定时刷新 UI
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)  # 每 100ms 刷新

        # 外部更新的变量
        self.new_vx = 0.0
        self.new_vy = 0.0
        self.new_ax = 0.0
        self.new_ay = 0.0
        self.new_posx = 0.0
        self.new_posy = 0.0

    def update_data(self, vx, vy, ax, ay, posx, posy):
        """外部调用，更新最新数据"""
        self.new_vx = vx
        self.new_vy = vy
        self.new_ax = ax
        self.new_ay = ay
        self.new_posx = posx
        self.new_posy = posy

    def refresh(self):
        """刷新绘图和标签"""
        # 追加数据
        self.data_vx.append(self.new_vx)
        self.data_vy.append(self.new_vy)
        self.data_ax.append(self.new_ax)
        self.data_ay.append(self.new_ay)

        # 限制最大点数
        if len(self.data_vx) > self.max_points:
            self.data_vx.pop(0)
            self.data_vy.pop(0)
            self.data_ax.pop(0)
            self.data_ay.pop(0)

        # 更新曲线
        self.vx_line.setData(self.data_vx)
        self.vy_line.setData(self.data_vy)
        self.ax_line.setData(self.data_ax)
        self.ay_line.setData(self.data_ay)

        # 更新位置显示
        self.label_position.setText(f"Position: ({self.new_posx:.2f}, {self.new_posy:.2f})")

def run_ui():
    """启动调试 UI"""
    app = QApplication(sys.argv)
    debugger = TrajectoryDebugger()
    debugger.show()
    sys.exit(app.exec_())
# 原 run_ui 改成：
def build_ui():
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    dbg = TrajectoryDebugger()   # 你之前写的窗口类
    dbg.show()
    return app, dbg


##########################################################################################


class Point:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y
        self.z = 0.05

    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get_position(self):
        return self.x, self.y, self.z

    def to_ros_pose(self, frame_id="world"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        return pose
    
def send_position_message(publisher, goal, num_messages=5, interval=0.2):
    for i in range(num_messages):
        publisher.publish(goal)
        rospy.loginfo(f"Sent message {i + 1} of {num_messages}!")
        time.sleep(interval)

def send_position_one(publisher, goal):
    publisher.publish(goal)
    rospy.loginfo(f"Sent message to ego!")


class VelocityController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('communication_car', anonymous=True)

        self.publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        # 从ROS参数服务器读取参数
        self.load_parameters()

        # 初始化串口通信
        self.navigator = NvigateData_t(self.uart_port, self.uart_bps, self.uart_timeout)
        
        # 速度控制状态
        self.last_update_time = time.time()
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.control_active = False
        
        self.last_health = None
	
        self.current_placex = 0.0
        self.current_placey = 0.0
        self.current_yaw = 0.0
        self.run_degree = 0.0
        self.now_dis = 0.0
        
        self.desire_placex = 0.0
        self.desire_placey = 0.0
        
        self.desire_vx=0.0
        self.desire_vy=0.0

        self.desire_ax=0.0
        self.desire_ay=0.0


        
        self.change_angle = 0.0
        
        # 使用参数化的导航速度
        self.nav_speed = self.default_nav_speed

        # 中心游龙模式
        self.loong_mode = False
        self.last_loong_time = None

        self.last_control_active = False
        self.now_publish = False
        self.go_home_state = None

        self.gamestart = True
        self.no_nav_mode = False

        # 策略点 - 使用参数化的位置
        self.home_point = Point(self.home_x, self.home_y)
        self.home_point.z = self.home_z
        
        self.center_point = [
            Point(self.center_point1_x, self.center_point1_y),
            Point(self.center_point2_x, self.center_point2_y),
            Point(self.center_point3_x, self.center_point3_y)
        ]
        
        self.center_index = 0
	
        # 订阅速度指令
        rospy.Subscriber("/position_cmd", PositionCommand, self.velocity_callback)
        
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        
        # 设置定时器（使用参数化的频率）
        control_interval = 1.0 / self.control_loop_rate
        rospy.Timer(rospy.Duration(control_interval), self.control_loop)

    def load_parameters(self):
        """从ROS参数服务器加载所有参数"""
        # 串口参数
        self.uart_port = rospy.get_param('~uart_port', -1)
        self.uart_bps = rospy.get_param('~uart_bps', 115200)
        self.uart_timeout = rospy.get_param('~uart_timeout', 5)
        
        # 导航速度参数
        self.default_nav_speed = rospy.get_param('~default_nav_speed', 0.5)
        self.home_nav_speed = rospy.get_param('~home_nav_speed', 0.6)
        self.loong_nav_speed = rospy.get_param('~loong_nav_speed', 0.42)
        self.timeout_threshold = rospy.get_param('~timeout_threshold', 0.5)
        
        # 策略点位置参数
        self.home_x = rospy.get_param('~home_x', -0.1)
        self.home_y = rospy.get_param('~home_y', -0.1)
        self.home_z = rospy.get_param('~home_z', 0.05)
        
        self.center_point1_x = rospy.get_param('~center_point1_x', 4.6)
        self.center_point1_y = rospy.get_param('~center_point1_y', 2.47)
        self.center_point2_x = rospy.get_param('~center_point2_x', 4.66)
        self.center_point2_y = rospy.get_param('~center_point2_y', 1.7)
        self.center_point3_x = rospy.get_param('~center_point3_x', 3.95)
        self.center_point3_y = rospy.get_param('~center_point3_y', 2.3)
        
        # 策略阈值参数
        self.max_health = rospy.get_param('~max_health', 400)
        self.low_health_threshold = rospy.get_param('~low_health_threshold', 0.4)
        self.home_area_threshold = rospy.get_param('~home_area_threshold', 1.5)
        self.loong_switch_time = rospy.get_param('~loong_switch_time', 3.5)
        self.blood_check_time = rospy.get_param('~blood_check_time', 1.5)
        self.strategy_loop_time = rospy.get_param('~strategy_loop_time', 0.05)
        self.game_check_time = rospy.get_param('~game_check_time', 0.5)
        
        # 控制循环参数
        self.control_loop_rate = rospy.get_param('~control_loop_rate', 33.33)
        
        # 打印加载的参数
        rospy.loginfo("参数加载完成:")
        rospy.loginfo(f"串口: port={self.uart_port}, bps={self.uart_bps}, timeout={self.uart_timeout}")
        rospy.loginfo(f"导航速度: 默认={self.default_nav_speed}, 回家={self.home_nav_speed}, 游龙={self.loong_nav_speed}")
        rospy.loginfo(f"基地点: ({self.home_x}, {self.home_y}, {self.home_z})")
        rospy.loginfo(f"中心点: ({self.center_point1_x}, {self.center_point1_y}), ({self.center_point2_x}, {self.center_point2_y}), ({self.center_point3_x}, {self.center_point3_y})")

    def go_to(self, point):# 前往某个点
        send_position_one(self.publisher, point.to_ros_pose())
        pass

    def stay(self):# 待在原地
        time.sleep(0.3)
        pass
    
    def pub_home(self):
        self.loong_mode = False
        self.nav_speed = self.home_nav_speed  # 使用参数化的回家速度
        self.no_nav_mode = False
        self.go_to(self.home_point)
        self.go_home_state = True
        self.now_publish = True
        print("go home!!")

    def pub_center(self):
        self.nav_speed = self.default_nav_speed  # 使用参数化的默认速度
        self.go_to(self.center_point[0])
        self.go_home_state = False
        self.now_publish = True
        print("go center!!")

    def no_nav_go_home(self):
        self.no_nav_mode = True

        self.desire_placex = self.home_point.x
        self.desire_placey = self.home_point.y
        self.control_active = True
        # 至少运行0.5s
        time.sleep(0.5)
        while not (self.navigator.health > self.last_health or self.now_dis < 0.1):
            time.sleep(0.05)
        self.no_nav_mode = False
        pass

    def no_nav_loong(self):
        if time.time() - self.last_loong_time > self.loong_switch_time:  # 使用参数化的切换时间
            self.center_index = (self.center_index + 1) % 3
            #self.center_index = (self.center_index + 1) % 1
            self.desire_placex = self.center_point[self.center_index].x
            self.desire_placey = self.center_point[self.center_index].y
            self.last_loong_time = time.time()
        pass

    def strategy(self):
        while True:
            # 如果是比赛状态，开启策略，每0.5秒检测一次比赛状态
            if self.navigator.game_progress == 4:
                if not self.gamestart:
                    self.gamestart = True

                # 比赛状态循环检测
                while True:
                    # 如果比赛状态改变就推出循环检测
                    if self.navigator.game_progress != 4:
                        # 不再控制运动
                        print("game stop!")
                        self.gamestart = False
                        break
                    
                    # 检测是否自动复活
                    if self.last_health:
                        # 检测到复活
                        if self.last_health == 0 and self.navigator.health > 0:
                            print("alive")
                            self.pub_home()
                    # 更新血量
                    self.last_health = self.navigator.health
                    

                    # 导航状态改变
                    if self.control_active != self.last_control_active:
                        # 开始导航
                        if self.control_active:
                            pass
                        # 结束导航
                        else:
                            self.now_publish = False
                            if self.go_home_state == True:
                                # 如果1.5秒后没有补到血，做位置闭环
                                time.sleep(self.blood_check_time)  # 使用参数化的检查时间
                                print("not add blood! go no nav!")
                                if self.navigator.health <= self.last_health and self.navigator.health != self.max_health:  # 使用参数化的最大血量
                                    self.no_nav_go_home()
                            # 如果导航到中心
                            elif self.go_home_state == False:
                                print("start lonng!")
                                if not (self.current_placex < self.home_area_threshold and self.current_placey < self.home_area_threshold):  # 使用参数化的区域阈值
                                    print("start lonng!")
                                    self.navigator.CmdID = 2
                                    self.nav_speed = self.loong_nav_speed  # 使用参数化的游龙速度
                                    # self.nav_speed = 0.2

                                    self.loong_mode = True
                                    self.no_nav_mode = True
                                    self.control_active = True
                                    self.last_loong_time = time.time()
                                    
                        # 更新状态
                        self.last_control_active = self.control_active
                    
                    if self.loong_mode:
                        self.no_nav_loong()

                    # 如果没有发布过点位
                    if not self.now_publish:
                        # 残血逃逸
                        if self.navigator.health < self.max_health * self.low_health_threshold and (not self.go_home_state):  # 使用参数化的阈值
                            self.navigator.CmdID = 1
                            self.pub_home()
                        
                        # 满血复活
                        elif self.navigator.health == self.max_health and (self.go_home_state or self.go_home_state == None):
                            self.navigator.CmdID = 1
                            self.pub_center()
                    
                    # 决策周期，使用参数化的循环时间
                    time.sleep(self.strategy_loop_time)

            else:
                print("wait game start!", self.navigator.game_progress, self.navigator.health, self.navigator.big_yaw)
            time.sleep(self.game_check_time)  # 使用参数化的检查时间

    def odom_callback(self, msg):
        self.current_placex = msg.pose.pose.position.x
        self.current_placey = msg.pose.pose.position.y
        # 定义四元数 (w, x, y, z)
        w, x, y, z = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]  # 示例四元数
        # 计算 Yaw 角度
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        self.current_yaw = math.degrees(yaw)
        # if self.current_yaw < 0:
        #     self.current_yaw = 360 + self.current_yaw


    def velocity_callback(self, msg):
        """ 速度指令回调函数 """
        # self.current_vx = msg.velocity.x
        # self.current_vy = -msg.velocity.y



        self.desire_placex = msg.position.x
        self.desire_placey = msg.position.y

        self.desire_vx=msg.velocity.x
        self.desire_vy=msg.velocity.y

        self.desire_ax=msg.acceleration.x
        self.desire_ay=msg.acceleration.y



        self.last_update_time = time.time()
        self.control_active = self.gamestart

        debugger.update_data(self.desire_vx, self.desire_vy,
                        self.desire_ax, self.desire_ay,
                        self.desire_placex, self.desire_placey)


    def control_loop(self, event):

        # 位置误差
            # ex = self.desire_placex - self.current_placex
            # ey = self.desire_placey - self.current_placey

            # # 位置环输出
            # Kp = 1.0  # 位置环比例系数，可调
            # v_pos_x = Kp * ex
            # v_pos_y = Kp * ey

            # # 速度前馈
            # v_ff_x = self.desire_vx
            # v_ff_y = self.desire_vy

            # # 合成总速度指令
            # cmd_vx = v_pos_x + v_ff_x
            # cmd_vy = v_pos_y + v_ff_y

            # # 可选：加速度前馈（如果有速度环或需要更快响应）
            # Ka = 0.2  # 加速度前馈系数
            # cmd_vx += Ka * self.desire_ax
            # cmd_vy += Ka * self.desire_ay

            # # 限制最大速度
            # max_speed = 0.8
            # speed_norm = math.sqrt(cmd_vx ** 2 + cmd_vy ** 2)
            # if speed_norm > max_speed:
            #     scale = max_speed / speed_norm
            #     cmd_vx *= scale
            #     cmd_vy *= scale

            # # 控制指令输出
            # self.current_vx = cmd_vx
            # self.current_vy = cmd_vy



        """ 控制循环（使用参数化的频率定时执行） """
        # 检查超时
        time_since_update = time.time() - self.last_update_time
        if time_since_update > self.timeout_threshold and not self.no_nav_mode:
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.control_active = False
        # 发送控制指令
        try:
            if self.control_active == True:
                # 计算角度（弧度）
                theta_rad = math.atan2(self.desire_placey - self.current_placey, self.desire_placex - self.current_placex)

                self.run_degree = self.current_yaw - math.degrees(theta_rad)

                self.now_dis = math.sqrt((self.current_placex - self.desire_placex) ** 2 + (self.current_placey - self.desire_placey) ** 2)
                self.now_dis = min(self.now_dis, 1.2)
                              
                v_pos_y = self.nav_speed * self.now_dis * math.sin(math.radians(self.run_degree))
                v_pos_x = self.nav_speed * self.now_dis * math.cos(math.radians(self.run_degree))

                v_ff_x = self.desire_vx
                v_ff_y = self.desire_vy


                cmd_vx = v_pos_x + v_ff_x
                cmd_vy = v_pos_y + v_ff_y



                Ka = 0.2  # 加速度前馈系数
                cmd_vx += Ka * self.desire_ax
                cmd_vy += Ka * self.desire_ay


                max_speed = 1.4
                speed_norm = math.sqrt(cmd_vx ** 2 + cmd_vy ** 2)
                if speed_norm > max_speed:
                    scale = max_speed / speed_norm
                    cmd_vx *= scale
                    cmd_vy *= scale



                self.current_vx = cmd_vx
                self.current_vy = cmd_vy




                if self.run_degree > 90 or self.run_degree < -90:
                    self.change_angle = self.run_degree
            self.navigator.set_data(
                vx=self.current_vx,
                vy=self.current_vy,
                vz=self.change_angle,
                control=self.control_active
            )
            self.change_angle=0.0
            if self.control_active:
                # print("now_dis:", self.now_dis, "vx:", self.current_vx, " vy:",self.current_vy, self.control_active)
                pass
            self.navigator.send()
        except Exception as e:
            rospy.logerr(f"串口通信失败: {str(e)}")

    def shutdown_hook(self):
        """ 关闭时发送停止指令 """
        self.navigator.set_data(0, 0, 0, False)
        self.navigator.send()
        if self.navigator.uart.is_open:
            self.navigator.close_uart(self.navigator.uart)

# if __name__ == '__main__':
#     try:
#         print("py  qt    graph    monitorrrrrrrrrrrrrrrrrrrrrrrrrr")
#         run_ui()


#         controller = VelocityController()

#         t1 = threading.Thread(target=controller.navigator.start)

#         t2 = threading.Thread(target=controller.strategy)

#         t1.start()

#         t2.start()

#         rospy.on_shutdown(controller.shutdown_hook)

#         rospy.spin()

#         t2.join()
#         t1.join()
#     except rospy.ROSInterruptException:
#         pass

if __name__ == '__main__':
    try:
        print("py qt graph monitor...................................")



        app, debugger = build_ui()

        controller = VelocityController()


        t1 = threading.Thread(target=controller.navigator.start, daemon=True)
        t2 = threading.Thread(target=controller.strategy, daemon=True)
        t1.start()
        t2.start()

        spin_thread = threading.Thread(target=rospy.spin, daemon=True)
        spin_thread.start()

        from PyQt5.QtCore import QCoreApplication
        app.aboutToQuit.connect(lambda: (controller.shutdown_hook(),
                                         rospy.signal_shutdown('GUI closed')))

        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass
