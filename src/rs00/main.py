import time

import serial.tools.list_ports

import driver


def find_usb_serial_port():
    """
    自动检测可用的USB串口设备
    返回：USB串口设备路径
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "USB" in port.device or "usb" in port.device:
            return port.device
    return None


def main():
    motor_id = 0x01
    serial_port = find_usb_serial_port()
    if serial_port is None:
        print("未找到USB串口设备，请检查连接。")
        return

    print(f"检测到串口：{serial_port}")
    motor = driver.RS00Driver(serial_port, motor_id = motor_id)
    limit_cur = motor.read_parameter(0x7018, is_float = True)
    print(f"当前limit_cur是{limit_cur}")

    # 复位电机
    # motor.stop_motor()

    # 设置零位
    # motor.set_zero()

    # 获取电机ID
    # motor.get_device_id()
    # 设置电机ID
    # motor.set_canid(0x01)

    # if motor.write_runmode("速度模式"):
        # time.sleep(0.05)
        # 使能电机
        # motor.enable_motor()

        # 运控电机
        # motor_status = motor.send_motion_control(0, 1, 1,1, 0)

        # 速度模式电机控制
        # motor_status = motor.send_speed_control(16, 10, 20)

        # 电流模式电机控制
        # motor_status =motor.send_current_control(0.5)

        # 位置模式(CSP)电机控制
        # motor_status = motor.send_loc_control_pp(3.14)
        # time.sleep(3)

        # 停止电机
        # motor.stop_motor(motor_status['faults'])


if __name__ == "__main__":
    main()

