def float_to_uint(x, x_min, x_max, bits = 16):
    """
    将浮点数映射为指定位数的无符号整数
    参数：
        x：浮点数
        x_min: 物理量最小值 (例如 -12.57)
        x_max: 物理量最大值 (例如 12.57)
        bits: 映射位数 (协议中默认为 16)
    返回：
        res_int: 无符号整数
    """
    span = x_max - x_min
    if x < x_min: x = x_min
    if x > x_max: x = x_max
    res_int = int((x - x_min) * ((1 << bits) - 1) / span)
    return res_int

def uint_to_float(x_int, x_min, x_max, bits=16):
    """
    将无符号整数映射回浮点数
    参数：
        x_int: 输入的整数 (例如从数据域解析出的 0~65535)
        x_min: 物理量最小值 (例如 -12.57)
        x_max: 物理量最大值 (例如 12.57)
        bits: 映射位数 (协议中默认为 16)
    返回：
        res_float:浮点数
    """
    span = x_max - x_min
    # 直接计算比例并还原物理量
    res_float = round(float(x_int) * span / ((1 << bits) - 1) + x_min, 5)
    return res_float

def unpack(res):
    """
    解析返回值
    参数：
        返回值：hex格式
    返回：
        res_id_hex: can_id
        res_data_hex: 响应数据
        res_dlc:响应数据字节数
    """
    if len(res) > 30 and res.startswith("4154") and res.endswith("0d0a"):
        res_id_hex = res[4:12]
        res_dlc = int(res[12:14], 16)
        res_data_hex = res[14: 14 + res_dlc * 2]
        return res_id_hex, res_data_hex, res_dlc
    else:
        print("未收到响应数据")
        return None

def pack_id(mode, master_id, motor_id):
    """
    根据数据手册构造29位扩展ID（再转为USB-CAN适配器的32位字段）。
    mode：通讯模式（位于bit28～24）
    master_id：主设备ID（位于bit23～8）
    motor_id：电机ID（位于bit7～0）
    返回：适配器32位ID字段（8位hex字符串）
    """
    logic_id = (mode << 24) | (master_id << 8) | motor_id
    serial_id = (logic_id << 3) | 0b100
    return f"{serial_id:08x}"


def unpack_id(can_id):
    """
    解析“USB-CAN适配器的32位ID字段”（即 pack_id 的返回值）。

    can_id:
      - int: 32位字段（最低3位为适配器控制位）
      - str: 8位hex字符串，例如 "0007e80c"

    返回：(mode, master_id, motor_id)
    """
    if isinstance(can_id, str):
        can_id_int = int(can_id, 16)
    else:
        can_id_int = int(can_id)

    # 去掉适配器最低3位控制位，得到真实29位逻辑ID
    logic_id = can_id_int >> 3

    mode = (logic_id >> 24) & 0x1F
    master_id = (logic_id >> 8) & 0xFFFF
    motor_id = logic_id & 0xFF
    return mode, master_id, motor_id

def pack_data_8bytes(x4, x3, x2, x1):
    """
    将数据打包成8字节格式并添加数据长度
    参数：
        int或hex格式数据（示例：0x01,01）
        x4：高位
        x1：低位
    返回：
        data:08 + 八字节的数据（字符串格式）
    """
    data = bytearray([0x08])  # 以长度字节开头

    # 添加4个值的字节
    data.extend(x1.to_bytes(2, 'big'))
    data.extend(x2.to_bytes(2, 'big'))
    data.extend(x3.to_bytes(2, 'big'))
    data.extend(x4.to_bytes(2, 'big'))

    return data.hex()

def unpack_data_8bytes(data_hex):
    """
    将8字节数据转换成4个变量
    参数：
        data_hex:8字节数据,hex格式
    返回：
        x1,x2,x3,x4(int格式)
    """
    data = bytearray.fromhex(data_hex)
    x1 = int.from_bytes(data[:2], 'big')
    x2 = int.from_bytes(data[2:4], 'big')
    x3 = int.from_bytes(data[4:6], 'big')
    x4 = int.from_bytes(data[6:8], 'big')
    return x4, x3, x2, x1

def unpack_motor_feedback(res):
    """
    通讯类型2， 解析电机反馈数据
    参数：
        res: 响应数据
    返回：
        motor_status:字典，包含电机ID，当前电机模式，故障标志，当前角度，当前角速度，当前力矩，当前温度
    """
    res_id_hex, res_data_hex, res_dlc = unpack(res)
    mode, status, master_id = unpack_id(res_id_hex)
    temp, tau_now, vel_now, pos_now = unpack_data_8bytes(res_data_hex)

    temp = float(temp / 10)
    tau_now = uint_to_float(tau_now, -14, 14)
    vel_now = uint_to_float(vel_now, -33, 33)
    pos_now = uint_to_float(pos_now, -12.57, 12.57)

    # 解析故障信息
    motor_id = (status >> 0) & 0xFF  # Bit 8~15: 当前电机 CAN ID
    fault_bits = (status >> 8) & 0x3F  # Bit 16~21: 故障信息 (6位)
    mode_state = (status >> 14) & 0x03  # Bit 22~23: 模式状态

    # 映射故障标志位 (Bit 16~21)
    fault_info = {
        "欠压故障": bool(fault_bits & 0x01),  # bit 16
        "驱动故障": bool(fault_bits & 0x02),  # bit 17
        "过温故障": bool(fault_bits & 0x04),  # bit 18
        "磁编码故障": bool(fault_bits & 0x08),  # bit 19
        "堵转过载故障": bool(fault_bits & 0x10),  # bit 20
        "未标定": bool(fault_bits & 0x20)  # bit 21
    }

    # 映射模式状态 (Bit 22~23)
    mode_map = {0: "Reset模式(复位)", 1: "Cali模式(标定)", 2: "Motor模式(运行)"}
    current_mode = mode_map.get(mode_state, "未知模式")

    # 打印/输出状态
    active_faults = [k for k, v in fault_info.items() if v]
    print(f"--- 电机 {motor_id} 反馈 ---")
    print(f"当前模式: {current_mode}")
    print(f"故障状态: {active_faults if active_faults else '正常'}")
    print(f"当前位置：{pos_now}, 当前角速度：{vel_now}, 当前力矩：{tau_now}, 当前温度：{temp}")

    motor_status = {
        "id": motor_id,
        "mode": current_mode,
        "faults": active_faults,
        "pos": pos_now,
        "vel": vel_now,
        "tau": tau_now,
        "temp": temp
    }

    return motor_status
