# 大然电机 CAN 控制库 - C语言版本

## 概述

本库是将大然电机（DrEmpower）Arduino CAN 库转换为纯 C 语言版本，适配 ESP32 的 TWAI 驱动。

## 文件说明

- `main/daran_motor_can.h` - 头文件，包含所有函数声明
- `main/daran_motor_can.c` - 实现文件，包含所有函数实现
- `main/daran_motor_example.c` - 使用示例

## 主要转换内容

### 1. CAN 驱动替换
- **Arduino**: MCP2515 芯片驱动
- **ESP32**: TWAI (Two-Wire Automotive Interface) 驱动

### 2. 延时函数替换
- **Arduino**: `delay(ms)` → **ESP32**: `vTaskDelay(pdMS_TO_TICKS(ms))`

### 3. 内存管理
- **Arduino**: `malloc/free` → **ESP32**: 标准 C `malloc/free`（FreeRTOS兼容）

### 4. 数据格式转换
- 保持原有的小端序（Intel byte order）数据格式
- 支持 float、u16、s16、u32、s32 五种数据类型转换

## 使用方法

### 1. 初始化 TWAI 驱动

在 `app_main()` 中初始化 TWAI 驱动：

```c
#include "driver/twai.h"
#include "daran_motor_can.h"

void app_main(void)
{
    // TWAI 配置
    const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CONFIG_EXAMPLE_TX_GPIO_NUM, 
        CONFIG_EXAMPLE_RX_GPIO_NUM, 
        TWAI_MODE_NORMAL
    );
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // 安装并启动驱动
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    
    // 初始化库
    daran_can_init();
    
    // 创建接收任务（可选）
    xTaskCreate(daran_receive_task, "daran_rx", 4096, NULL, 8, NULL);
}
```

### 2. 基本控制示例

```c
// 清除错误
daran_clear_error(1);

// 设置闭环控制模式
daran_set_mode(1, 2);

// 设置零点
daran_set_zero_position(1);

// T模式：轨迹跟踪模式，转到90度
daran_set_angle(1, 90.0f, 0, 10.0f, 0);

// Q模式：梯形轨迹模式，转到-150度
daran_set_angle(1, -150.0f, 10.0f, 5.0f, 1);

// V模式：速度控制，20r/min
daran_set_speed(1, 20.0f, 1.5f, 1);

// C模式：扭矩控制，1.5Nm
daran_set_torque(1, 1.5f, 0, 1);

// 读取状态
daran_servo_state_t state = daran_get_state(1);
ESP_LOGI("MOTOR", "位置: %.2f°, 速度: %.2f r/min", state.angle, state.speed);
```

### 3. 多电机控制示例

```c
uint8_t id_list[3] = {1, 2, 3};
float angle_list[3] = {90.0f, -90.0f, 180.0f};

// 同时控制多个电机
daran_set_angles(id_list, angle_list, 10.0f, 5.0f, 1, 3);

// 等待所有电机到达目标位置
daran_positions_done(id_list, 3);
```

## 主要函数说明

### 运动控制函数

| 函数名 | 功能 | 参数说明 |
|--------|------|----------|
| `daran_set_angle()` | 单个电机角度控制（绝对角度） | id_num, angle, speed, param, mode |
| `daran_set_angles()` | 多个电机角度控制 | id_list, angle_list, speed, param, mode, n |
| `daran_step_angle()` | 单个电机相对角度控制 | id_num, angle, speed, param, mode |
| `daran_set_speed()` | 单个电机速度控制 | id_num, speed, param, mode |
| `daran_set_torque()` | 单个电机扭矩控制 | id_num, torque, param, mode |
| `daran_impedance_control()` | 阻抗控制 | id_num, pos, vel, tff, kp, kd |

### 系统控制函数

| 函数名 | 功能 |
|--------|------|
| `daran_estop()` | 急停 |
| `daran_clear_error()` | 清除错误 |
| `daran_set_zero_position()` | 设置零点 |
| `daran_set_mode()` | 设置电机模式（1=IDLE, 2=闭环） |
| `daran_save_config()` | 保存配置 |
| `daran_reboot()` | 电机重启 |

### 参数设置函数

| 函数名 | 功能 |
|--------|------|
| `daran_set_id()` | 设置电机ID |
| `daran_set_can_baud_rate()` | 设置CAN波特率 |
| `daran_set_angle_range()` | 设置角度范围（软件限位） |
| `daran_write_property()` | 写入属性参数 |

### 参数读取函数

| 函数名 | 功能 | 返回值 |
|--------|------|--------|
| `daran_get_state()` | 读取位置和速度 | `daran_servo_state_t` |
| `daran_get_volcur()` | 读取电压和电流 | `daran_servo_volcur_t` |
| `daran_read_property()` | 读取属性参数 | `float` |
| `daran_dump_error()` | 读取错误信息 | `int8_t` (0=无错误) |

## 控制模式说明

### 角度控制模式（mode参数）

- **mode = 0**: 轨迹跟踪模式
  - `speed`: 最大速度限制（r/min）
  - `param`: 角度输入滤波带宽（<300）

- **mode = 1**: 梯形轨迹模式
  - `speed`: 目标速度（r/min）
  - `param`: 加速度（(r/min)/s）

- **mode = 2**: 前馈控制模式
  - `speed`: 前馈速度（r/min）
  - `param`: 前馈扭矩（Nm）

### 速度控制模式（mode参数）

- **mode = 1**: 速度前馈模式
  - `param`: 前馈扭矩（Nm）

- **mode != 1**: 速度爬升模式
  - `param`: 速度爬升速率（(r/min)/s）

### 扭矩控制模式（mode参数）

- **mode = 1**: 直接控制模式
  - `param`: 无意义

- **mode != 1**: 扭矩爬升模式
  - `param`: 扭矩爬升速率（Nm/s）

## 全局变量

- `daran_read_flag`: 读取结果标志位（1=成功, 0=超时, -1=失败）
- `daran_rx_buffer[8]`: 接收缓冲区
- `daran_traj_mode`: 轨迹模式（1=梯形, 2=S形）
- `daran_motor_state[MOTOR_NUM][5]`: 电机状态数组
  - `[0]`: 位置（度）
  - `[1]`: 速度（r/min）
  - `[2]`: 扭矩（Nm）
  - `[3]`: 目标到达标志
  - `[4]`: 错误标志

## 注意事项

1. **波特率**: 默认使用 1Mbps，可通过 `TWAI_TIMING_CONFIG_*` 修改
2. **电机ID**: 根据实际设置修改，范围 1-63
3. **数据格式**: 小端序（Intel byte order）
4. **数量因子**: 速度/扭矩等参数需要乘以100（0.01因子）
5. **接收任务**: 建议创建独立的接收任务处理状态反馈

## 编译配置

在 `main/CMakeLists.txt` 中已包含所有源文件：

```cmake
idf_component_register(SRCS "twai_self_test_example_main.c"
                              "daran_motor_can.c"
                              "daran_motor_example.c"
                    INCLUDE_DIRS ".")
```

## 与 Arduino 库的对应关系

| Arduino函数 | C语言函数 | 说明 |
|------------|----------|------|
| `set_angle()` | `daran_set_angle()` | 角度控制 |
| `set_speed()` | `daran_set_speed()` | 速度控制 |
| `set_torque()` | `daran_set_torque()` | 扭矩控制 |
| `get_state()` | `daran_get_state()` | 读取状态 |
| `clear_error()` | `daran_clear_error()` | 清除错误 |
| `estop()` | `daran_estop()` | 急停 |

## 示例代码

完整示例请参考 `main/daran_motor_example.c`
