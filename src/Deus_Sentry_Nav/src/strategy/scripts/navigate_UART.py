import serial  # 串口
import struct  # 处理二进制数据打包为C语言中的结构体
import time  # 时间
import numpy as np
def short(value):
    half_float = np.float16(value)
    return half_float.tobytes()
class NvigateData_t:
    def __init__(self, PORT, BPS, TIMEOUT):
        self.PORT = PORT  # 串口号
        self.BPS = BPS  # 波特率
        self.TIMEOUT = TIMEOUT  # 超时时间
        self._callback = None  # 私有回调引用
        self.uart = self.open_uart(self.PORT, self.BPS, self.TIMEOUT)  # 打开串口
        self.start_flag = False  # 开始标志
        # 头
        self.BEGIN = 0xAA  # 开始字节
        self.CmdID = 2  # 命令ID
        # 发送的数据
        self.vx = 0 # 两字节float
        self.vy = 0 # 两字节float
        self.vz = 0 # 两字节float
        self.control = False  # 是否控制车辆，只有当True时导航才控制
        # 接收的数据
        self.health = None  # 两字节 当前血量
        self.max_health = 400  # 两字节 最大血量
        self.bullet_count = None  # 两字节 剩余子弹数
        self.max_bullet_count = None  # 两字节 最大子弹数
        self.alive_status = None  # 一字节 生存状态
        self.game_progress = None # 一字节 比赛阶段
        self.remain_times = None  # 两字节 比赛剩余时间
        self.big_yaw = None

        # 尾
        self.END = 0xFF

    def send(self):
        if self.uart and self.uart.is_open:
            # 打包起始字节和命令ID
            data = struct.pack('BB', self.BEGIN, self.CmdID)

            # 打包 vx, vy, vz（每个字段为 2 字节 float）
            data += short(self.vx)
            data += short(self.vy)
            data += short(self.vz)

            # 打包 control（1 字节布尔值）
            control_byte = 1 if self.control else 0
            data += struct.pack('B', control_byte)

            # 打包结束字节
            data += struct.pack('B', self.END)

            # 发送数据
            self.uart.write(data)

            
            print("发送数据:", data)

        else:
            time.sleep(0.005)
            print("UART未打开或已关闭")

    def get(self):
        try:
            if not self.uart or not self.uart.is_open:
                print("UART未打开或已关闭")
                time.sleep(0.005)
                return
            max_attempts = 10  # 设置最大尝试次数
            attempts = 0
            while attempts < max_attempts:
                rdata = self.uart.read(1)  # 读取起始字节
                if len(rdata) == 0 or rdata[0] != self.BEGIN:
                    print("无效的start字节----------------")
                    attempts += 1
                    continue
                # 读取剩余数据（7字节）
                data = self.uart.read(8)
                if len(data) != 8:
                    print("接收到的数据不完整---------")
                    attempts += 1
                    continue
                if data[-1] != self.END:
                    print("无效的结束字节---------")
                    attempts += 1
                    continue
                # self.health = None  # 两字节 当前血量
                # self.max_health = None  # 两字节 最大血量
                # self.bullet_count = None  # 两字节 剩余子弹数
                # self.max_bullet_count = None  # 两字节 最大子弹数
                # self.alive_status = None  # 一字节 生存状态
                # self.game_progress = None # 一字节 比赛阶段
                # self.remain_times = None  # 两字节 比赛剩余时间

                # 解析数据
                # self.CmdID = struct.unpack('B', data[0:1])[0]  # 1字节 CmdID
                # self.health = struct.unpack('H', data[1:3])[0]  # 2字节 当前血量
                # self.max_health = struct.unpack('H', data[3:5])[0]  # 2字节 最大血量
                # self.bullet_count = struct.unpack('H', data[5:7])[0]  # 2字节 剩余子弹数
                # self.max_bullet_count = struct.unpack('H', data[7:9])[0]  # 2字节 最大子弹数
                # self.alive_status = struct.unpack('B', data[9:10])[0]  # 1字节 生存状态
                # self.game_progress = struct.unpack('B', data[10:11])[0]  # 1字节 比赛阶段
                # self.remain_times = struct.unpack('H', data[11:12])[0]  # 2字节 比赛剩余时间
                
                self.health = struct.unpack('H', data[0:2])[0]  # 2字节 当前血量
                self.big_yaw = struct.unpack('f', data[2:6])[0] # 4字节 大yaw
                self.game_progress = struct.unpack('B', data[6:7])[0]  # 1字节 比赛阶段
                # self.game_progress=4
                # 调用回调函数
                if self._callback:
                    self._callback(
                        health=self.health,
                        max_health = self.max_health,
                        bullets=self.bullet_count,
                        max_bullets = self.max_bullet_count,
                        alive=bool(self.alive_status),
                        game_progress=self.game_progress,
                        remain_time=self.remain_times
                    )
                break

        except Exception as e:

            print(f"发生异常: {e}")
            self.close_uart()
            if not self.uart:
                time.sleep(1)
                self.uart = self.open_uart(self.PORT, self.BPS, self.TIMEOUT)


    def set_data(self, vx, vy, vz, control):
        """
        设置发送的数据。

        参数:
            vx (float): x 方向速度（2 字节 float）
            vy (float): y 方向速度（2 字节 float）
            vz (float): z 方向速度（2 字节 float）
            control (bool): 是否控制车辆（1 字节布尔值）
        """
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.control = control

    def start(self):
        print("start")
        self.start_flag = True
        while self.start_flag:
            self.get()

    def stop(self):
        self.start_flag = False

    def callback(self):
        # 处理函数
        # print("接收到的数据:", self.CmdID, self.speed, self.yaw, self.pitch, self.roll)
        self.send()
        # 可以在这里添加其他处理逻辑
        pass

    # 打开端口
    def open_uart(self, port, bps, timeout):
        if int(port) < 0:
            for i in range(1000):
                try:
                    uart = serial.Serial(
                        port="/dev/ttyUSB" + str(i),
                        baudrate=bps,
                        timeout=timeout,
                        parity=serial.PARITY_NONE,
                        stopbits=1
                    )
                    opened = i
                    return uart
                except:
                    pass
        else:
            uart = serial.Serial(
                port="/dev/ttyUSB",
                baudrate=bps,
                timeout=timeout,
                parity=serial.PARITY_NONE,
                stopbits=1
            )
            return uart
        print("open uart failed")

    def close_uart(self, uart):
        try:
            uart.close()
        except Exception as e:
            print("关闭UART时发生错误")
