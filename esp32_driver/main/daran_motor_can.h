#ifndef __DREMPOWER_CAN_H__
#define __DREMPOWER_CAN_H__

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机状态（快速读取接口返回）
 * @note 单位：angle=deg（输出轴角度），speed=r/min（输出轴速度）
 */
struct servo_state
{
  float angle;
  float speed;
};

/**
 * @brief 电机电压/电流（辅助读取接口返回）
 * @note 单位：vol=V，cur=A（q轴电流）
 */
struct servo_volcur
{
  float vol;
  float cur;
};

/**
 * @brief 最近一次“读返回”的结果标志位
 * - 1：成功收到帧并完成解析
 * - 0：尚未收到（或刚发出请求）
 * - -1：超时/失败
 *
 * @note 该库为“同步收发”风格（模仿 Arduino 官方库），调用 get_state/read_property 等会更新该标志位。
 */
extern int8_t READ_FLAG;

/**
 * @brief 最近一次接收到的 8 字节数据区（原始 CAN payload）
 * @note 主要供 format_data(..., "decode") 解码使用。
 */
extern uint8_t rx_buffer[8];

/**
 * @brief 轨迹模式选择（仅用于多电机轨迹相关函数；当前移植版多电机接口为占位）
 * - 1：梯形轨迹
 * - 2：S形轨迹
 */
extern int8_t TRAJ_MODE;

/**
 * @brief 最近一次接收到的 CAN 标准帧 ID（11-bit）
 * @note 该库中电机帧格式：identifier = (id_num << 5) + cmd
 */
extern uint16_t can_id;

/**
 * @brief ESP32-S3 适配：保持与 Arduino MCP2515 API 同名
 * @note 在 ESP-IDF 版本中：TWAI 的 install/start 由 `app_main()` 完成，这里不做实际初始化。
 */
void MCP2515_CAN_Init(void);

/**
 * @brief ESP32-S3 适配：读取一帧 CAN 数据（非阻塞）
 * @note 成功时会更新 READ_FLAG/rx_buffer/can_id。
 */
void MCP2515_CAN_readMessage(void);

/**
 * @brief 发送一帧指令到电机
 * @param id_num 电机ID（0 可广播）
 * @param cmd    指令码（官方协议中的 cmd）
 * @param data   8 字节 payload（可为 NULL，视为全 0）
 * @param rt     预留（与官方库保持一致，当前未使用）
 *
 * @note 使用标准帧（11-bit ID），ID 计算方式：(id_num << 5) + cmd
 */
void send_command(uint8_t id_num, char cmd, unsigned char *data, uint8_t rt);

/**
 * @brief 同步等待接收（轮询 + 延时），模仿 Arduino 版 receive_data()
 * @note 超时行为：SERVO_POLL_MS * SERVO_MAX_POLLS（目前约 10s）
 */
void receive_data(void);

/**
 * @brief CAN payload 编码/解码（与官方库一致）
 *
 * @param value_data 输入/输出的“可读值”数组（最多 3 个）
 * @param type_data  每个值的类型标识（与官方库一致：0=float,1=u16,2=s16,3=u32,4=s32）
 * @param length     个数（1~3）
 * @param str        "encode" 将 value_data 编码到内部 byte_data；"decode" 从 rx_buffer 解码到 value_data
 */
void format_data(float *value_data, int *type_data, int length, char *str);

/**
 * @brief 运动指令实时状态返回解析（需要 enable_replay_state 开启才会生效）
 * @note 当前移植版默认关闭 enable_replay_state（与示例一致：通过 get_state/read_property 主动查询）。
 */
void reply_state(uint8_t id_num);

/** @brief 预设角度（后续可用多电机同步命令触发） */
void preset_angle(uint8_t id_num, float angle, float t, float param, int mode);
/** @brief 预设速度（后续可用多电机同步命令触发） */
void preset_speed(uint8_t id_num, float speed, float param, int mode);
/** @brief 预设扭矩（后续可用多电机同步命令触发） */
void preset_torque(uint8_t id_num, float torque, float param, int mode);

/**
 * @brief 绝对角度控制（输出轴角度）
 * @param mode 0:轨迹跟踪 1:梯形轨迹 2:前馈
 */
void set_angle(uint8_t id_num, float angle, float speed, float param, int mode);
/** @brief 多电机绝对角度控制（当前为占位实现） */
void set_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n);
/** @brief 相对角度控制（相对当前位置） */
void step_angle(uint8_t id_num, float angle, float speed, float param, int mode);
/** @brief 多电机相对角度控制（当前为占位实现） */
void step_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n);

/**
 * @brief 速度控制（连续转动）
 * @param mode 1:直通速度（INPUT_MODE_PASSTHROUGH） 其他：速度爬升（INPUT_MODE_VEL_RAMP）
 */
void set_speed(uint8_t id_num, float speed, float param, int mode);
/** @brief 多电机速度控制（当前为占位实现） */
void set_speeds(uint8_t *id_list, float *speed_list, float param, float mode, size_t n);

/** @brief 扭矩（电流）闭环控制 */
void set_torque(uint8_t id_num, float torque, float param, int mode);
/** @brief 多电机扭矩控制（当前为占位实现） */
void set_torques(uint8_t *id_list, float *torque_list, float param, int mode, size_t n);

/** @brief 阻抗控制（MIT 模式风格） */
void impedance_control(uint8_t id_num, float pos, float vel, float tff, float kp, float kd);
/** @brief 急停：电机会进入 IDLE 并产生 estop 错误标志 */
void estop(uint8_t id_num);

/** @brief 设置电机 CAN 节点 ID（掉电保存） */
void set_id(uint8_t id_num, int new_id);
/** @brief 设置 UART 波特率（掉电保存） */
void set_uart_baud_rate(uint8_t id_num, int baud_rate);
/** @brief 设置 CAN 波特率（掉电保存） */
void set_can_baud_rate(uint8_t id_num, int baud_rate);

/**
 * @brief 设置电机模式
 * @param mode 1:IDLE 2:CLOSED_LOOP_CONTROL（闭环）
 */
void set_mode(uint8_t id_num, int mode);
/** @brief 将当前位置设为 0 度（零点） */
void set_zero_position(uint8_t id_num);

/** @brief 设置 GPIO 接口模式（当前为占位实现） */
void set_GPIO_mode(uint8_t id_num, uint8_t mode, uint32_t param);
/** @brief 设置软件限位范围（当前为占位实现） */
int8_t set_angle_range(uint8_t id_num, float angle_min, float angle_max);
/** @brief 设置轨迹模式（梯形/S形），主要用于多电机轨迹 */
void set_traj_mode(uint8_t id_num, int mode);

/**
 * @brief 写入电机属性参数（控制参数）
 * @param param_address 属性地址（见协议/枚举）
 * @param param_type    类型（0/1/2/3/4 对应 float/u16/s16/u32/s32）
 * @param value         目标值
 */
void write_property(uint8_t id_num, unsigned short param_address, int8_t param_type, float value);
/** @brief 读取电机 ID（property 31001） */
uint8_t get_id(uint8_t id_num);
/** @brief 快速读取当前位置/速度（同时会更新 READ_FLAG） */
struct servo_state get_state(uint8_t id_num);
/** @brief 读取电压/电流（内部通过 read_property） */
struct servo_volcur get_volcur(uint8_t id_num);
/** @brief 读取 GPIO 模式（内部通过 read_property） */
int8_t get_GPIO_mode(uint8_t id_num, uint8_t *enable_uart, uint8_t *enable_step_dir, uint32_t *n);

/**
 * @brief 读取电机属性参数
 * @return 读取到的值；失败返回 0 且 READ_FLAG=-1
 */
float read_property(uint8_t id_num, int param_address, int param_type);

/** @brief 清除错误标志（出错后恢复控制通常需要先 clear_error 再 set_mode(2)） */
void clear_error(uint8_t id_num);

/**
 * @brief 查询错误汇总（按官方库逻辑）
 * @return 0 无错误；1~7 表示不同错误域非 0；负数表示读取该域时通信失败
 */
int8_t dump_error(uint8_t id_num);

/** @brief 保存配置到 flash（掉电不丢失） */
void save_config(uint8_t id_num);
/** @brief 软件重启电机（效果类似重新上电） */
void reboot(uint8_t id_num);

/**
 * @brief 阻塞等待到位（官方库原样移植）
 * @note 注意：该函数可能阻塞较久；示例代码建议使用带超时的 wait_traj_done 替代。
 */
void position_done(uint8_t id_num);
/** @brief 阻塞等待多个电机到位（当前只是逐个 position_done） */
void positions_done(uint8_t *id_list, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* __DREMPOWER_CAN_H__*/
