import time

import serial

import utils


class RS00Driver:
    def __init__(self, port, baudrate=921600, timeout=0.1, motor_id=0x01):
        """
        RS00驱动器初始化
        port: 串口设备
        baudrate: 波特率（建议先用 921600，与上位机一致）
        timeout: 超时时间
        motor_id: 电机ID
        """

        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.header = "4154"  # 帧头“AT”
        self.footer = "0d0a"  # 帧尾“\r\n”
        self.motor_id = motor_id

    def send_request(
        self,
        mode,
        motor_id,
        data,
        master_id=0xFD,
        read_window_s: float = 0.5,
    ):
        """
        封装并发送完整AT指令帧（支持复刻上位机发送习惯）
        参数：
            mode: 通讯模式
            master_id: 主设备ID
            motor_id: 电机ID
            data: hex字符串，格式必须是：DLC(1字节) + payload(DLC字节)
            read_window_s: 发送后持续读回包的时间窗口
        返回：
            响应数据（hex字符串，可能为空）
        """

        can_id = utils.pack_id(mode, master_id, motor_id)
        frame = self.header + can_id + data + self.footer
        print(f"发送帧: {frame}")
        self.ser.reset_input_buffer()
        self.ser.write(bytes.fromhex(frame))

        # 在窗口期内尽量多读一些返回（单次read很容易错过）
        deadline = time.time() + float(read_window_s)
        rx = bytearray()
        while time.time() < deadline:
            n = getattr(self.ser, "in_waiting", 0)
            if n:
                rx += self.ser.read(min(int(n), 4096))
            else:
                time.sleep(0.01)
        return rx.hex()

    def send_raw_hex(self, raw_hex: str, read_window_s: float = 0.5, chunk: int = 4096):
        """
        直接发送一整帧“原始hex”（例如：41540007e84c01000d0a），并在一段时间内把收到的数据原样读出。

        - raw_hex: 允许包含空格/换行，会自动清理
        - read_window_s: 发送后持续读串口的时间窗口
        - chunk: 单次read读取的最大字节数

        返回：收到的原始hex字符串（拼接）
        """

        frame = "".join(raw_hex.strip().split()).lower()
        if len(frame) % 2 != 0:
            raise ValueError("raw_hex 长度必须是偶数（每2个hex为1字节）")
        print(f"发送原始帧: {frame}")
        self.ser.reset_input_buffer()
        self.ser.write(bytes.fromhex(frame))

        deadline = time.time() + float(read_window_s)
        rx = bytearray()
        while time.time() < deadline:
            n = getattr(self.ser, "in_waiting", 0)
            if n:
                rx += self.ser.read(min(int(n), int(chunk)))
            else:
                time.sleep(0.01)

        if rx:
            print(f"收到 {len(rx)} 字节: {rx.hex()}")
        else:
            print("未收到任何返回数据（可能电机未上电/ID不匹配/总线无终端/波特率不对）。")
        return rx.hex()

    def enable_motor(self, motor_id):
        """
        使能电机
        参数：
            motor_id: 电机ID
        返回：
            bool：使能成功为True，使能失败为False
        """
        res = self.send_request(0x03, motor_id, "0100")
        if not res:
            return False

        motor_status = utils.unpack_motor_feedback(res)

        if motor_status['mode'] == "Motor模式(运行)":
            print(f"电机 {motor_id} 使能成功！")
            return True
        else:
            print(f"电机 {motor_id} 使能失败，当前处于 {motor_status['mode']}")
            return False

    def stop_motor(self, motor_id, clear_fault=False):
        """
        停止电机
        参数：
            motor_id: 电机ID
            clear_fault: True=清除故障并停止, False=仅停止
        """
        # 构造 8 字节数据区
        if clear_fault:
            # Byte[0]=1时：清故障
            data_hex = utils.pack_data_8bytes(00,00,00,0x01)
        else:
            # 正常运行时，data区清0
            data_hex = utils.pack_data_8bytes(00,00,00,00)

        res = self.send_request(mode = 0x04, data = data_hex, motor_id = motor_id)
        if not res:
            print(f"电机 {motor_id} 停止指令发送失败（无响应）")
            return False

        motor_status = utils.unpack_motor_feedback(res)

        if motor_status['mode'] == "Reset模式(复位)":
            if clear_fault:
                print(f"电机 {motor_id} 故障已清除并成功停止。")
            else:
                print(f"电机 {motor_id} 已成功停止（进入复位模式）。")
            return True
        else:
            print(f"电机 {motor_id} 停止状态异常，当前模式: {motor_status['mode']}")
            return False

    def get_device_id(self, motor_id=None):
        """
        获取设备ID
        参数：
            motor_id:设备id
        返回：
            设备id
        """

        if motor_id is None:
            motor_id = self.motor_id

        print(f"获取设备ID: motor_id = {motor_id} 的设备信息")
        res = self.send_request(
            0,
            motor_id,
            "0100"
        )
        print(f"rxHex: {res}")

        # 解析响应数据（上位机示例：415400000ff4 08 515a30209c233712 0d0a）
        res_id, res_data, res_dlc = utils.unpack(res)
        mode, motor_id_rx, status = utils.unpack_id(res_id)
        if mode == 0 and status == 0xFE:
            print(f"成功识别电机，ID = {motor_id_rx}")
            print(f"64位MCU唯一标识符(前{res_dlc}字节): {res_data}")
            return motor_id_rx
        else:
            return f"未识别ID为{motor_id_rx}的电机"


    def send_motion_control(self, motor_id, pos_des, vel_des, kp, kd, tau):
        """
        运控模式指令发送
        参数：
            motor_id: 电机id(hex格式)
            pos_des:目标角度(-12.57f~12.57f)
            vel_des:目标角速度(-33rad/s~33rad/s)
            Kp:比例增益(0~500)
            Kd:微分增益(0~5)
            tau:力矩(-14Nm~14Nm)
        返回：
            pos_now: 当前角度
            vel_now: 当前角速度
            tau_now: 当前力矩
            temp: 当前温度
        """

        pos_des = utils.float_to_uint(pos_des, -12.57, 12.57)
        vel_des = utils.float_to_uint(vel_des, -33, 33)
        kp = utils.float_to_uint(kp, 0, 500)
        kd = utils.float_to_uint(kd, 0, 5)
        tau = utils.float_to_uint(tau, -14, 14)

        # 打包数据, 发送命令
        data = utils.pack_data_8bytes(kd, kp, vel_des, pos_des)
        res = self.send_request(mode = 0x01, data = data, motor_id = motor_id, master_id = tau)

        # 解析返回值
        res = utils.unpack_motor_feedback(res)

        return res


