# HTD-35H 总线舵机上位机控制器（vendor 原样拷贝，未改逻辑）
# 来源: https://gitee.com/zangmy04/servo_control_new  hostcomputer/htd35h_controller.py
# 链路: CH340 串口(115200) -> STM32F407 扩展板 -> 总线舵机 ID1(头部左右) / ID2(点头, 限位650)
# 注意: 打开串口自动清 DTR/RTS 并等 1.5s（STM32 开机动画）；调试输出默认开启
import time
import queue
import struct
import serial
import threading

# STM32 硬件拓展板固定 CRC8 校验表
crc8_table = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]

def checksum_crc8(data):
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0x00FF

class HTD35HController:
    # 依据协议，总线舵机功能的 Function ID 根据拓展板设定为全区 0x05
    FUNC_BUS_SERVO = 0x05

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        """
        初始化通讯句柄
        port: 根据实际接入的 USB 转串口设备名填写，如 /dev/ttyUSB0 或 COM3
        """
        self.port = serial.Serial(port, baudrate, timeout=timeout)
        # 禁用 DTR 和 RTS，防止 CH340 的一键下载电路把 STM32 一直按在复位状态
        self.port.setDTR(False)
        self.port.setRTS(False)
        
        # 释放复位后 STM32 可能会重启，等待其开机动画（LED闪烁）完成
        time.sleep(1.5)
        
        # 默认开启 STM32 调试输出
        self._send_debug_toggle(True)
        
        self.running = True
        self.servo_read_lock = threading.Lock()
        
        # 定义获取回读数据的双缓冲队列 (线程安全)
        self.bus_servo_queue = queue.Queue(maxsize=128)
        
        # 启动后台监听和解包线程
        self.recv_thread = threading.Thread(target=self._recv_task, daemon=True)
        self.recv_thread.start()
        
    def _recv_task(self):
        """
        常驻内存的串口读取线程，包含提取完整协议数据包的状态机。
        支持两种 Function ID：
          0x05 - 总线舵机数据包，放入 bus_servo_queue
          0xFE - STM32 调试字符串，直接打印到控制台
        """
        FUNC_DEBUG = 0xFE
        state = 0
        func_id = 0
        frame = []
        dbg_buf = []
        recv_count = 0

        while self.running:
            if self.port.in_waiting:
                dat = self.port.read(1)[0]
                if state == 0:          # 等待第一帧头 0xAA
                    if dat == 0xAA:
                        state = 1
                elif state == 1:        # 等待第二帧头 0x55
                    if dat == 0x55:
                        state = 2
                    else:
                        state = 0
                elif state == 2:        # 识别 Function ID
                    if dat == self.FUNC_BUS_SERVO:
                        func_id = dat
                        frame = [dat, 0]
                        state = 3
                    elif dat == FUNC_DEBUG:
                        func_id = dat
                        dbg_buf = []
                        state = 6       # 进入调试帧长度读取状态
                    else:
                        state = 0       # 未知功能码，丢弃
                elif state == 3:        # 读取舵机包 Length
                    frame[1] = dat
                    recv_count = 0
                    state = 4 if dat > 0 else 5
                elif state == 4:        # 接收舵机包 Payload
                    frame.append(dat)
                    recv_count += 1
                    if recv_count >= frame[1]:
                        state = 5
                elif state == 5:        # CRC8 校验并派发舵机包
                    crc8 = checksum_crc8(bytes(frame))
                    if crc8 == dat:
                        payload = bytes(frame[2:])
                        try:
                            self.bus_servo_queue.put_nowait(payload)
                        except queue.Full:
                            pass
                    state = 0
                elif state == 6:        # 读取调试帧 Length
                    recv_count = int(dat)
                    dbg_buf = []
                    state = 7 if recv_count > 0 else 0
                elif state == 7:        # 接收调试字符串字节
                    dbg_buf.append(dat)
                    if len(dbg_buf) >= recv_count:
                        # 打印 STM32 发来的调试信息（过滤心跳包避免刷屏）
                        msg = bytes(dbg_buf).decode('utf-8', errors='replace').rstrip('\n')
                        if msg != "BEAT":
                            print(f"[STM32] {msg}")
                        state = 0
            else:
                time.sleep(0.001)

    def close(self):
        """
        关闭端口前，停止后台线程
        """
        self.running = False
        self.recv_thread.join(timeout=1.0)
        self.port.close()

    def buf_write(self, func, data):
        """
        构建底层链路封包并发送
        [0xAA, 0x55, Function_ID, Length, Data Payload, CRC8]
        """
        buf = [0xAA, 0x55, int(func)]
        buf.append(len(data))
        buf.extend(data)
        buf.append(checksum_crc8(bytes(buf[2:])))
        self.port.write(bytes(buf))

    def _send_debug_toggle(self, enable):
        """直接发送调试开关包（可在 __init__ 中使用）"""
        data = [0xF0, 1 if enable else 0]
        buf = [0xAA, 0x55, self.FUNC_BUS_SERVO, len(data)] + data
        buf.append(checksum_crc8(bytes(buf[2:])))
        self.port.write(bytes(buf))

    # ==========================
    # 核心控制指令 (Core Actions)
    # ==========================
    def set_position(self, duration_sec, servo_positions):
        """
        设置一个或多个舵机转动到指定角度脉冲
        duration_sec: 运动耗时(秒)，最大支持 30.0 秒
        servo_positions: [(ID, 新角度位置脉冲), (ID2, 新位置), ...] (示例：[(1, 500)])
        """
        duration_ms = int(duration_sec * 1000)
        duration_ms = max(0, min(duration_ms, 30000))
        
        # SubCmd(0x01) + Time_L + Time_H + Servo_Num + (ID + Pos_L + Pos_H)...
        data = [0x01, duration_ms & 0xFF, (duration_ms >> 8) & 0xFF, len(servo_positions)]
        for sid, pos in servo_positions:
            data.extend(struct.pack("<BH", sid, int(pos)))
            
        self.buf_write(self.FUNC_BUS_SERVO, data)

    def stop_action(self, servo_ids):
        """
        立即停止某个或几个总线舵机的动作
        """
        # SubCmd(0x03) + Servo_Num + ID1 + ID2...
        data = [0x03, len(servo_ids)]
        data.extend(struct.pack("<" + "B"*len(servo_ids), *servo_ids))
        self.buf_write(self.FUNC_BUS_SERVO, data)

    def enable_torque(self, servo_id, enable=True):
        """
        使能 / 断开 扭矩锁定
        enable: True 为锁定， False 为掉电释放
        """
        sub_cmd = 0x0B if enable else 0x0C
        data = struct.pack("<BB", sub_cmd, servo_id)
        self.buf_write(self.FUNC_BUS_SERVO, data)

    def set_debug(self, enable=True):
        """开启/关闭 STM32 调试日志输出"""
        self._send_debug_toggle(enable)

    # ==========================
    # 配置与限制修改 (Config & Limits)
    # ==========================
    def set_id(self, current_id, new_id):
        # 0x10 + Current ID + Target ID
        data = struct.pack("<BBB", 0x10, current_id, new_id)
        self.buf_write(self.FUNC_BUS_SERVO, data)

    def set_offset(self, servo_id, offset):
        """临时微调舵机中位"""
        data = struct.pack("<BBb", 0x20, servo_id, int(offset))
        self.buf_write(self.FUNC_BUS_SERVO, data)

    def save_offset(self, servo_id):
        """长久固化舵机的微调中位至 Flash"""
        data = struct.pack("<BB", 0x24, servo_id)
        self.buf_write(self.FUNC_BUS_SERVO, data)

    def set_angle_limit(self, servo_id, limit_min, limit_max):
        data = struct.pack("<BBHH", 0x30, servo_id, int(limit_min), int(limit_max))
        self.buf_write(self.FUNC_BUS_SERVO, data)

    def set_vin_limit(self, servo_id, vin_min_mv, vin_max_mv):
        data = struct.pack("<BBHH", 0x34, servo_id, int(vin_min_mv), int(vin_max_mv))
        self.buf_write(self.FUNC_BUS_SERVO, data)

    def set_temperature_limit(self, servo_id, max_temp):
        data = struct.pack("<BBb", 0x38, servo_id, int(max_temp))
        self.buf_write(self.FUNC_BUS_SERVO, data)

    # ==========================
    # 查询状态与回读 (Telemetry readback)
    # ==========================
    def _read_and_unpack(self, servo_id, cmd, unpack_format):
        """
        发起读指令，并自动阻塞等待接受结构体组装结果。
        unpack_format 需要匹配返回 Payload 的总结构（C-type format）。通常包含:
            ID (1 Bytes) + EchoSubCmd (1 Bytes) + SuccessFlag (1 Bytes) + ReturnDataPayload (... Bytes)
        其中 `<` 代表使用小端解包(Little Endian)。
        """
        # 放空可能的缓冲脏数据
        while not self.bus_servo_queue.empty():
            self.bus_servo_queue.get()
            
        with self.servo_read_lock:
            # 所有的查询皆只发送 [子命令, 对应的 ID] 两字节
            self.buf_write(self.FUNC_BUS_SERVO, [cmd, servo_id])
            try:
                # 阻塞直到收包线程放入符合的包。设置 0.5s 超时时间避免完全冻结
                data = self.bus_servo_queue.get(timeout=0.5)
                
                # 数据被返回后通过 Python 原生 struct 系统转换为解包数组
                res = struct.unpack(unpack_format, data)
                res_id, cmd_echo, success = res[0], res[1], res[2]
                
                # 确认：成功运行 (success == 0), ID未冲突, 子指令对应得上
                if success == 0 and res_id == servo_id and cmd_echo == cmd:
                    return res[3:]  # 返还 payload 后面挂着的有用信息
            except queue.Empty:
                print(f"Reading cmd 0x{cmd:02X} timeout for ID {servo_id}")
            except Exception as e:
                print(f"Unpack deserialization error: {e}")
        return None

    def read_id(self, target_id=254):
        """读取指定物理舵机的 ID 号"""
        res = self._read_and_unpack(target_id, 0x12, "<BBbB")
        return res[0] if res else None

    def read_position(self, servo_id):
        """读取被查询目标的当前脉冲位置"""
        res = self._read_and_unpack(servo_id, 0x05, "<BBbh")
        return res[0] if res else None

    def read_voltage(self, servo_id):
        """读取供电电压 (单位：mV毫伏)"""
        res = self._read_and_unpack(servo_id, 0x07, "<BBbH")
        return res[0] if res else None

    def read_temperature(self, servo_id):
        """读取当前芯片温度 (单位：摄氏度°C)"""
        res = self._read_and_unpack(servo_id, 0x09, "<BBbB")
        return res[0] if res else None

    # ==========================
    # 批量操作 (Batch Operations)
    # ==========================
    def enable_torque_all(self, servo_ids, enable=True):
        """批量使能/卸载扭矩"""
        for sid in servo_ids:
            self.enable_torque(sid, enable)
            time.sleep(0.02)

    def read_all_positions(self, servo_ids):
        """
        批量读取多个舵机的位置
        返回: {servo_id: position} 字典，读取失败的舵机不包含在内
        """
        positions = {}
        for sid in servo_ids:
            pos = self.read_position(sid)
            if pos is not None:
                positions[sid] = pos
            time.sleep(0.02)
        return positions

    def read_all_status(self, servo_ids):
        """
        批量读取多个舵机的完整状态 (位置/电压/温度)
        返回: {servo_id: {"pos": int, "voltage_mv": int, "temp_c": int}} 字典
        """
        status = {}
        for sid in servo_ids:
            info = {}
            pos = self.read_position(sid)
            if pos is not None:
                info["pos"] = pos
            vol = self.read_voltage(sid)
            if vol is not None:
                info["voltage_mv"] = vol
            temp = self.read_temperature(sid)
            if temp is not None:
                info["temp_c"] = temp
            if info:
                status[sid] = info
            time.sleep(0.02)
        return status

    # ==========================
    # 零位校准 (Zero Calibration)
    # ==========================
    def calibrate_zero(self, servo_id, interactive=True):
        """
        零位校准流程：
        1. 卸载扭矩，让用户手动将舵盘转到期望的零位姿态
        2. 读取当前位置，计算与 500 (中位) 的偏差
        3. 写入角度偏移并保存到 EEPROM
        4. 重新使能扭矩

        offset 范围 -125~125 (对应约 ±30°)
        如果当前位置与 500 的差值超出 ±125，则校准失败。

        interactive: True 时会阻塞等待用户按回车确认
        返回: 校准成功时返回 offset 值，失败返回 None
        """
        print(f"[校准] 开始校准舵机 ID={servo_id} 的零位")

        # 步骤 1: 卸载扭矩
        print("[校准] 步骤 1/5: 卸载扭矩，舵机可自由转动...")
        self.enable_torque(servo_id, False)
        time.sleep(0.3)

        # 步骤 2: 等待用户手动调整
        if interactive:
            input("[校准] 步骤 2/5: 请手动将舵盘转到期望的零位姿态，完成后按 Enter 继续...")
        else:
            time.sleep(1.0)

        # 步骤 3: 读取当前位置，计算偏移量
        print("[校准] 步骤 3/5: 读取当前位置...")
        pos = self.read_position(servo_id)
        if pos is None:
            print("[校准] 错误: 无法读取舵机位置，校准中止")
            self.enable_torque(servo_id, True)
            return None

        offset = 500 - pos
        print(f"[校准]   当前位置={pos}, 中位=500, 需要偏移={offset}")

        if offset < -125 or offset > 125:
            print(f"[校准] 错误: 偏移量 {offset} 超出允许范围 (-125~125)，校准中止")
            print(f"[校准] 提示: 请将舵盘物理位置调整到更接近中位(500)的位置后重试")
            self.enable_torque(servo_id, True)
            return None

        # 步骤 4: 写入偏移并保存
        print(f"[校准] 步骤 4/5: 写入偏移量 {offset} 到舵机...")
        self.set_offset(servo_id, offset)
        time.sleep(0.1)
        self.save_offset(servo_id)
        time.sleep(0.1)
        print(f"[校准]   偏移已保存到 EEPROM，掉电不丢失")

        # 步骤 5: 重新使能扭矩并验证
        print("[校准] 步骤 5/5: 重新使能扭矩，验证校准结果...")
        self.enable_torque(servo_id, True)
        time.sleep(0.3)

        # 移动到中位验证
        self.set_position(1.0, [(servo_id, 500)])
        time.sleep(1.5)
        verify_pos = self.read_position(servo_id)
        if verify_pos is not None:
            print(f"[校准]   验证: 发送 500 后实际位置={verify_pos}")

        print(f"[校准] 完成! 舵机 ID={servo_id} 零位偏移={offset}")
        return offset
