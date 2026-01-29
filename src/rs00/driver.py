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
        self.master_id = 0xfd

    def send_request(
        self,
        mode,
        motor_id,
        data,
        master_id,
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

    def enable_motor(self):
        """
        使能电机
        返回：
            bool：使能成功为True，使能失败为False
        """
        res = self.send_request(mode = 0x03, motor_id = self.motor_id, master_id = self.master_id, data = "0100")
        if not res:
            return False

        motor_status = utils.unpack_motor_feedback(res)

        if motor_status['mode'] == "Motor模式(运行)":
            print(f"电机 {self.motor_id} 使能成功！")
            return True
        else:
            print(f"电机 {self.motor_id} 使能失败，当前处于 {motor_status['mode']}")
            return False

    def stop_motor(self, clear_fault=False):
        """
        停止电机
            clear_fault: True=清除故障并停止, False=仅停止
        """
        # 构造 8 字节数据区
        if clear_fault:
            # Byte[0]=1时：清故障
            data = [1, 0, 0, 0, 0, 0, 0, 0]
            data_hex = utils.pack_datalist_to_hex(data)
        else:
            # 正常运行时，data区清0
            data = [0, 0, 0, 0, 0, 0, 0, 0]
            data_hex = utils.pack_datalist_to_hex(data)

        res = self.send_request(mode = 0x04, data = data_hex, motor_id = self.motor_id, master_id = self.master_id)
        if not res:
            print(f"电机 {self.motor_id} 停止指令发送失败（无响应）")
            return False

        motor_status = utils.unpack_motor_feedback(res)

        if motor_status['mode'] == "Reset模式(复位)":
            if clear_fault:
                print(f"电机 {self.motor_id} 故障已清除并成功停止。")
            else:
                print(f"电机 {self.motor_id} 已成功停止（进入复位模式）。")
            return True
        else:
            print(f"电机 {self.motor_id} 停止状态异常，当前模式: {motor_status['mode']}")
            return False

    def get_device_id(self):
        """
        获取设备ID
        返回：
            设备id
        """
        print(f"获取设备ID: motor_id = {self.motor_id} 的设备信息")
        res = self.send_request(mode = 0, motor_id = self.motor_id, master_id = self.master_id, data = "0100")

        # 解析响应数据（上位机示例：415400000ff4 08 515a30209c233712 0d0a）
        res_id, res_data, res_dlc = utils.unpack(res)
        mode, motor_id_rx, status = utils.unpack_id(res_id)
        if mode == 0 and status == 0xFE:
            print(f"成功识别电机，ID = {motor_id_rx}")
            print(f"64位MCU唯一标识符(前{res_dlc}字节): {res_data}")
            return motor_id_rx
        else:
            return f"未识别ID为{motor_id_rx}的电机"


    def send_motion_control(self, pos_des, vel_des, kp, kd, tau):
        """
        运控模式指令发送:
        t_ref=Kd*(v_set-v_actual)+Kp*(p_set-p_actual)+t_ff
        参数：
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
        data = utils.pack_data_8bytes_big(pos_des, vel_des, kp, kd)
        res = self.send_request(mode = 0x01, data = data, motor_id = self.motor_id, master_id = tau)

        # 解析返回值
        res = utils.unpack_motor_feedback(res)

        return res

    def set_zero(self):
        """
        设置电机机械零位
        返回：
            motor_status:通讯协议2,电机状态字典
        """
        data = [1, 0, 0, 0, 0, 0, 0, 0]
        data_hex = pack_datalist_to_hex(data)
        res = self.send_request(mode = 0x06, motor_id = self.motor_id, master_id = self.master_id, data = data_hex)
        if not res:
            print(f"[错误] 电机设置零位通讯失败：未收到ID为{self.motor_id}的电机应答")
            return False

        motor_status = utils.unpack_motor_feedback(res)

        current_pos = motor_status.get('pos', 0)

        # 这里的判定逻辑可以改为：只要收到了有效的反馈包，就认为指令下发成功
        print(f"指令已发送，电机{self.motor_id}当前反馈位置: {current_pos}")

        if abs(current_pos) < 0.1:
            print("设置零位成功！")
            return motor_status
        else:
            print(f"提醒：电机{self.motor_id}位置尚未清零，当前为 {current_pos}，请稍后再次查询。")
            return motor_status

    def set_canid(self, motor_new_id):
        """
        设置电机ID
        参数：
            motor_new_id: 电机预设ID（示例：0x01）
        返回：

        """
        master_id = ((motor_new_id & 0xFF) << 8) | (self.master_id & 0xFF)

        # 要在复位下完成
        if self.stop_motor():
            res = self.send_request(mode = 0x07, master_id = master_id, motor_id = self.motor_id, data = "0100")
            if not res:
                print(f"[错误] 修改电机ID通讯失败：未收到ID为{self.motor_id}的电机应答")
                return False
            res_id, res_data, res_dlc = utils.unpack(res)
            mode, motor_id_rx, status = utils.unpack_id(res_id)
            if mode == 0 and status == 0xFE:
                print(f"成功修改电机ID，当前电机ID = {motor_id_rx}")
                self.motor_id  = motor_id_rx
                return motor_id_rx
            else:
                return f"修改电机ID失败"
        else:
            return False


    def read_parameter(self, index):
        """
        读取电机单个参数
        参数:
            index:参数在参数表中的索引(数据类型:int 示例:0x0101)
        返回:
            data[0:4]:响应的参数数据,int类型
        """
        data = utils.pack_data_8bytes_little(index, 0, 0, 0)
        res = self.send_request(mode = 0x11, data = data, motor_id = self.motor_id, master_id = self.master_id)
        if not res:
            print(f"[错误] 读参数通讯失败：未收到ID为{self.motor_id}的电机应答")
            return False
        res_id, res_data, res_dlc = utils.unpack(res)
        mode, status, master_id = utils.unpack_id(res_id)
        if (status >> 8) & 0x0F:
            print("读取参数成功!")
            # 将响应数据由bit类型转换成字节, 低字节在前(小端序), 前4个字节是相应的参数数据
            data = bytearray.fromhex(res_data)
            parameter = int.from_bytes(data[4:8], 'little')
            return parameter
        else:
            print("参数读取失败!")
            return False

    def read_runmode(self):
        """
        读取电机的运行模式
        返回:
            result:电机的运行模式,字符串格式
        """
        parameter = self.read_parameter(0x7005)
        runmode = {
            0:"运控模式",
            1:"位置模式(PP)",
            2:"速度模式",
            3:"电流模式",
            5:"位置模式(CSP)"
        }
        result = runmode.get(parameter)
        print(f"当前电机的运行模式是:{result}")
        return result

    def write_parameter(self, index, parameter):
        """
        根据数据手册中参数表的索引值,写入参数数据
        参数:
            index:索引(字节类型)
            parameter:参数数值(字节类型,小端序)
        返回:
            status:电机状态
        """

        data = "08" + (index + bytes(2) + parameter).hex()
        res = self.send_request(mode = 0x12, data = data, motor_id = self.motor_id, master_id = self.master_id)

        if not res:
            print(f"[错误] 写参数通讯失败：未收到ID为{self.motor_id}的电机应答")
            return False
        else:
            status = utils.unpack_motor_feedback(res)
            return status


    def write_runmode(self, parameter):
        """
        改变电机的运行模式
        参数:
            parameter:电机参数
            runmode = {
            "运控模式":0,
            "位置模式(PP)":1,
            "速度模式":2,
            "电流模式":3,
            "位置模式(CSP)":5
        }
        返回:
            status:电机状态
        """
        runmode = {
            "运控模式":0,
            "位置模式(PP)":1,
            "速度模式":2,
            "电流模式":3,
            "位置模式(CSP)":5
        }

        # 将参数和索引转换为字节格式
        parameter = runmode.get(parameter)
        data = parameter.to_bytes(4, byteorder='little')
        index = 0x7005
        index = index.to_bytes(2, byteorder='little')
        status = self.write_parameter(index, data)
        runmode_now = self.read_runmode()

        if  runmode_now == parameter:
            print(f"ID为{self.motor_id}的电机运行模式修改成功!目前为{parameter}")
            return status
        else:
            print(f"ID为{self.motor_id}的电机运行模式修改失败,目前为{runmode_now}")











