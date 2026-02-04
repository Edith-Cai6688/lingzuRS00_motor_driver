/*
 * DrEmpower CAN library (ESP32-S3 / ESP-IDF TWAI port)
 *
 * API mirrors the official Arduino library:
 *   DaRan/04-库函数及其说明书/arduino/Arduino_can/DrEmpower_can.cpp/.h
 *
 * Notes:
 * - TWAI driver must be installed and started by the application.
 * - Uses standard 11-bit ID frames.
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "daran_motor_can.h"

#define TAG "DREMPOWER_CAN"

#define INPUT_MODE_PASSTHROUGH 1
#define INPUT_MODE_VEL_RAMP 2
#define INPUT_MODE_TORQUE_RAMP 6

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

// official receive_data(): delay(100) * 100 loops => 10s max
#define SERVO_POLL_MS 100
#define SERVO_MAX_POLLS 100

// ===== Global vars (same names as official) =====
int8_t READ_FLAG = 0;
uint8_t rx_buffer[8] = {0};
uint16_t can_id = 0x00;
int8_t TRAJ_MODE = 1;

// ===== internal state (as in official) =====
static int8_t enable_replay_state = 0;
#define MOTOR_NUM 64
static float motor_state[MOTOR_NUM][5];
static uint8_t reply_state_error = 0;

// ===== format_data internal struct (as in official) =====
struct format_data_struct {
    float value_data[3];
    unsigned char byte_data[8];
    int type_data[3];
    int length;
};
static struct format_data_struct data_list;

// note: intel little endian
static inline void uint16_to_data(uint16_t val, uint8_t *data)
{
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}

static inline uint16_t data_to_uint16(uint8_t *data)
{
    return (uint16_t)(((uint32_t)data[1] << 8) + ((uint32_t)data[0]));
}

static inline void uint_to_data(uint32_t val, uint8_t *data)
{
    data[3] = (uint8_t)(val >> 24);
    data[2] = (uint8_t)(val >> 16);
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}

static inline uint32_t data_to_uint(uint8_t *data)
{
    return (((uint32_t)data[3] << 24) + ((uint32_t)data[2] << 16) +
            ((uint32_t)data[1] << 8) + ((uint32_t)data[0]));
}

static inline void int16_to_data(int16_t val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
}

static inline int16_t data_to_int16(uint8_t *data)
{
    int16_t tmp;
    *(((uint8_t *)(&tmp)) + 0) = data[0];
    *(((uint8_t *)(&tmp)) + 1) = data[1];
    return tmp;
}

static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
    data[2] = *(((uint8_t *)(&val)) + 2);
    data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
    int tmp;
    *(((uint8_t *)(&tmp)) + 0) = data[0];
    *(((uint8_t *)(&tmp)) + 1) = data[1];
    *(((uint8_t *)(&tmp)) + 2) = data[2];
    *(((uint8_t *)(&tmp)) + 3) = data[3];
    return tmp;
}

static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
    data[2] = *(((uint8_t *)(&val)) + 2);
    data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline float data_to_float(uint8_t *data)
{
    float tmp;
    *(((uint8_t *)(&tmp)) + 0) = data[0];
    *(((uint8_t *)(&tmp)) + 1) = data[1];
    *(((uint8_t *)(&tmp)) + 2) = data[2];
    *(((uint8_t *)(&tmp)) + 3) = data[3];
    return tmp;
}

/**
 * @brief 将 data_list.byte_data 按 data_list.type_data 解码到 data_list.value_data
 * @note 与 Arduino 官方库 byte2value() 逻辑一致（小端序）。
 */
static void byte2value(void)
{
    int value_index = 0;
    int byte_index = 0;
    while (1) {
        switch (data_list.type_data[value_index]) {
        case 0:
            data_list.value_data[value_index] = (float)data_to_float(&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index++;
            break;
        case 1:
            data_list.value_data[value_index] = (float)data_to_uint16(&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index++;
            break;
        case 2:
            data_list.value_data[value_index] = (float)data_to_int16(&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index++;
            break;
        case 3:
            data_list.value_data[value_index] = (float)data_to_uint(&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index++;
            break;
        case 4:
            data_list.value_data[value_index] = (float)data_to_int(&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index++;
            break;
        default:
            value_index++;
            break;
        }
        if ((byte_index >= 8) || (value_index >= data_list.length)) return;
    }
}

/**
 * @brief 将 data_list.value_data 按 data_list.type_data 编码到 data_list.byte_data
 * @note 与 Arduino 官方库 value2byte() 逻辑一致（小端序）。
 */
static void value2byte(void)
{
    int byte_index = 0;
    int value_index = 0;
    while (1) {
        if (data_list.type_data[value_index] == 0) {
            float_to_data(data_list.value_data[value_index], &data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        switch (data_list.type_data[value_index]) {
        case 0:
            float_to_data(data_list.value_data[value_index], &data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
            break;
        case 1:
            uint16_to_data((uint16_t)data_list.value_data[value_index], &data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
            break;
        case 2:
            int16_to_data((int16_t)data_list.value_data[value_index], &data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
            break;
        case 3:
            uint_to_data((uint32_t)data_list.value_data[value_index], &data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
            break;
        case 4:
            int_to_data((int)data_list.value_data[value_index], &data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
            break;
        default:
            value_index++;
            break;
        }
        if ((byte_index >= 8) || (value_index >= data_list.length)) return;
    }
}

// ===== TWAI adapt (names kept as official) =====
void MCP2515_CAN_Init(void)
{
    // TWAI install/start is done in app_main.
}

/**
 * @brief ESP32-S3 适配：读取一帧 CAN（非阻塞）
 *
 * @details
 * 成功时会更新全局：
 * - READ_FLAG = 1
 * - rx_buffer[0..7]（payload）
 * - can_id（11-bit 标准帧 ID）
 */
void MCP2515_CAN_readMessage(void)
{
    twai_message_t msg;
    esp_err_t ret = twai_receive(&msg, 0); // non-blocking
    if (ret == ESP_OK) {
        READ_FLAG = 1;
        for (int i = 0; i < msg.data_length_code && i < 8; i++) rx_buffer[i] = msg.data[i];
        can_id = (uint16_t)(msg.identifier & 0x7FF);
    }
}

/**
 * @brief 发送一帧电机指令（标准帧 11-bit ID）
 *
 * @details
 * 帧 ID 格式与官方库一致：
 * identifier = (id_num << 5) + cmd
 *
 * @param id_num 电机ID（0 可广播）
 * @param cmd    指令码（协议中的 cmd）
 * @param data   8字节 payload（NULL 则填 0）
 * @param rt     预留参数（与官方库保持一致；当前未使用）
 */
void send_command(uint8_t id_num, char cmd, unsigned char *data, uint8_t rt)
{
    (void)rt;
    twai_message_t msg = {0};
    msg.identifier = (uint32_t)((id_num << 5) + (uint8_t)cmd);
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 8;
    for (int i = 0; i < 8; i++) msg.data[i] = data ? data[i] : 0;
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) ESP_LOGW(TAG, "twai_transmit failed: %s", esp_err_to_name(ret));
}

/**
 * @brief 同步等待接收（轮询 + 延时），模仿 Arduino 官方库 receive_data()
 *
 * @note 最多等待：SERVO_POLL_MS * SERVO_MAX_POLLS（默认约 10s）。
 *       超时后 READ_FLAG 可能仍为 0，调用方应结合 READ_FLAG 判断是否成功。
 */
void receive_data(void)
{
    uint8_t OutTime = 0, OutTime_mark = 0;
    do {
        MCP2515_CAN_readMessage();
        if (READ_FLAG == 1) OutTime_mark = 1;
        vTaskDelay(pdMS_TO_TICKS(SERVO_POLL_MS));
        OutTime++;
        if (OutTime == SERVO_MAX_POLLS) OutTime_mark = 1;
    } while (OutTime_mark == 0);
}

/**
 * @brief CAN payload 编码/解码（与 Arduino 官方库一致）
 *
 * - "encode": 将 value_data/type_data 编码到 data_list.byte_data（供 send_command 使用）
 * - "decode": 从 rx_buffer 拷贝到 data_list.byte_data，再解码到 data_list.value_data
 *
 * @note type_data 取值含义见 `daran_motor_can.h`。
 */
void format_data(float *value_data, int *type_data, int length, char *str)
{
    data_list.length = length;
    for (int i = 0; i < length; i++) {
        data_list.value_data[i] = value_data[i];
        data_list.type_data[i] = type_data[i];
    }
    if (strcmp(str, "encode") == 0) value2byte();
    if (strcmp(str, "decode") == 0) {
        for (int i = 0; i < 8; i++) data_list.byte_data[i] = rx_buffer[i];
        byte2value();
    }
}

/**
 * @brief 解析“运动控制指令实时状态返回”
 *
 * @note 需要 enable_replay_state=1 才会生效（当前移植版默认关闭）。
 *       本工程示例主要通过 get_state()/read_property() 主动查询状态。
 */
void reply_state(uint8_t id_num)
{
    if (enable_replay_state && id_num <= MOTOR_NUM) {
        READ_FLAG = 0;
        receive_data();
        if (READ_FLAG == 1) {
            if (id_num == 0) id_num = (uint8_t)((can_id & 0x07E0) >> 5) & 0xFF;
            float factor = 0.01f;
            float v[3] = {0, 0, 0};
            int t[3] = {0, 2, 2};
            format_data(v, t, 3, "decode");
            motor_state[id_num - 1][0] = data_list.value_data[0];
            motor_state[id_num - 1][1] = data_list.value_data[1] * factor;
            motor_state[id_num - 1][2] = data_list.value_data[2] * factor;
            motor_state[id_num - 1][3] = (int)((can_id & 0x02) >> 1);
            motor_state[id_num - 1][4] = (int)((can_id & 0x04) >> 2);
        } else {
            READ_FLAG = -1;
            reply_state_error += 1;
        }
    }
}

/**
 * @brief 预设角度（配合多电机同步触发使用）
 *
 * @param mode
 * - 0：轨迹跟踪（t=速度限制，param=输入滤波带宽）
 * - 1：梯形轨迹（t=运动时间(s)，param=目标加速度）
 * - 2：前馈（t=前馈速度，param=前馈扭矩）
 */
void preset_angle(uint8_t id_num, float angle, float t, float param, int mode)
{
    float factor = 0.01f;
    if (mode == 0) {
        int s16_time = (int)(fabsf(t) / factor);
        if (param > 300) param = 300;
        int s16_width = (int)(fabsf(param / factor));
        float v[3] = {angle, (float)s16_time, (float)s16_width};
        int tp[3] = {0, 2, 2};
        format_data(v, tp, 3, "encode");
        send_command(id_num, 0x0C, data_list.byte_data, 0);
    } else if (mode == 1) {
        int s16_time = (int)(fabsf(t) / factor);
        int s16_accel = (int)(fabsf(param) / factor);
        float v[3] = {angle, (float)s16_time, (float)s16_accel};
        int tp[3] = {0, 2, 2};
        format_data(v, tp, 3, "encode");
        send_command(id_num, 0x0C, data_list.byte_data, 0);
    } else if (mode == 2) {
        int s16_speed_ff = (int)(t / factor);
        int s16_torque_ff = (int)(param / factor);
        float v[3] = {angle, (float)s16_speed_ff, (float)s16_torque_ff};
        int tp[3] = {0, 2, 2};
        format_data(v, tp, 3, "encode");
        send_command(id_num, 0x0C, data_list.byte_data, 0);
    }
    reply_state(id_num);
}

/**
 * @brief 预设速度（配合多电机同步触发使用）
 * @param mode 1:直通速度（INPUT_MODE_PASSTHROUGH），否则：速度爬升（INPUT_MODE_VEL_RAMP）
 */
void preset_speed(uint8_t id_num, float speed, float param, int mode)
{
    float factor = 0.01f;
    if (mode == 1) {
        int s16_torque = (int)(param / factor);
        if (speed == 0) s16_torque = 0;
        int s16_input_mode = (int)(INPUT_MODE_PASSTHROUGH / factor);
        float v[3] = {speed, (float)s16_torque, (float)s16_input_mode};
        int tp[3] = {0, 2, 2};
        format_data(v, tp, 3, "encode");
    } else {
        int s16_ramp_rate = (int)(param / factor);
        int s16_input_mode = (int)(INPUT_MODE_VEL_RAMP / factor);
        float v[3] = {speed, (float)s16_ramp_rate, (float)s16_input_mode};
        int tp[3] = {0, 2, 2};
        format_data(v, tp, 3, "encode");
    }
    send_command(id_num, 0x0C, data_list.byte_data, 0);
    reply_state(id_num);
}

/**
 * @brief 预设扭矩（配合多电机同步触发使用）
 * @param mode 1:直通扭矩（INPUT_MODE_PASSTHROUGH），否则：扭矩爬升（INPUT_MODE_TORQUE_RAMP）
 */
void preset_torque(uint8_t id_num, float torque, float param, int mode)
{
    float factor = 0.01f;
    int s16_ramp_rate, s16_input_mode;
    if (mode == 1) {
        s16_input_mode = (int)(INPUT_MODE_PASSTHROUGH / factor);
        s16_ramp_rate = 0;
    } else {
        s16_input_mode = (int)(INPUT_MODE_TORQUE_RAMP / factor);
        s16_ramp_rate = (int)(param / factor);
    }
    float v[3] = {torque, (float)s16_ramp_rate, (float)s16_input_mode};
    int tp[3] = {0, 2, 2};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x0C, data_list.byte_data, 0);
    reply_state(id_num);
}

/**
 * @brief 绝对角度控制（输出轴角度）
 *
 * @param mode
 * - 0：轨迹跟踪（speed=速度限制，param=输入滤波带宽）
 * - 1：梯形轨迹（speed>0 且 param>0 才会发送）
 * - 2：前馈（speed=前馈速度，param=前馈扭矩）
 */
void set_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    float factor = 0.01f;
    if (mode == 0) {
        int s16_speed = (int)(fabsf(speed) / factor);
        if (param > 300) param = 300;
        int s16_width = (int)(fabsf(param / factor));
        float v[3] = {angle, (float)s16_speed, (float)s16_width};
        int tp[3] = {0, 2, 2};
        format_data(v, tp, 3, "encode");
        send_command(id_num, 0x19, data_list.byte_data, 0);
    } else if (mode == 1) {
        if (speed > 0 && param > 0) {
            int s16_speed = (int)(fabsf(speed) / factor);
            int s16_accel = (int)(fabsf(param) / factor);
            float v[3] = {angle, (float)s16_speed, (float)s16_accel};
            int tp[3] = {0, 2, 2};
            format_data(v, tp, 3, "encode");
            send_command(id_num, 0x1A, data_list.byte_data, 0);
        }
    } else if (mode == 2) {
        int s16_speed_ff = (int)(speed / factor);
        int s16_torque_ff = (int)(param / factor);
        float v[3] = {angle, (float)s16_speed_ff, (float)s16_torque_ff};
        int tp[3] = {0, 2, 2};
        format_data(v, tp, 3, "encode");
        send_command(id_num, 0x1B, data_list.byte_data, 0);
    }
    reply_state(id_num);
}

/** @brief 相对角度控制（相对当前位置） */
void step_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    step_angles(&id_num, &angle, speed, param, mode, 1);
}

/**
 * @brief 多电机绝对角度控制（当前为占位实现）
 * @note 如需完整多电机同步行为，可参考官方 Arduino 库实现补全。
 */
void set_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{
    (void)id_list; (void)angle_list; (void)speed; (void)param; (void)mode; (void)n;
}
/**
 * @brief 多电机相对角度控制（当前为占位实现）
 */
void step_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{
    (void)id_list; (void)angle_list; (void)speed; (void)param; (void)mode; (void)n;
}

/**
 * @brief 速度控制（连续转动）
 * @param mode 1:直通速度（INPUT_MODE_PASSTHROUGH），否则：速度爬升（INPUT_MODE_VEL_RAMP）
 */
void set_speed(uint8_t id_num, float speed, float param, int mode)
{
    float factor = 0.01f;
    if (mode == 1) {
        int s16_torque = (int)(param / factor);
        if (speed == 0) s16_torque = 0;
        unsigned short u16_input_mode = INPUT_MODE_PASSTHROUGH;
        float v[3] = {speed, (float)s16_torque, (float)u16_input_mode};
        int tp[3] = {0, 2, 1};
        format_data(v, tp, 3, "encode");
    } else {
        int s16_ramp_rate = (int)(param / factor);
        unsigned short u16_input_mode = INPUT_MODE_VEL_RAMP;
        float v[3] = {speed, (float)s16_ramp_rate, (float)u16_input_mode};
        int tp[3] = {0, 2, 1};
        format_data(v, tp, 3, "encode");
    }
    send_command(id_num, 0x1C, data_list.byte_data, 0);
    reply_state(id_num);
}

/**
 * @brief 多电机速度控制（当前为占位实现）
 */
void set_speeds(uint8_t *id_list, float *speed_list, float param, float mode, size_t n)
{
    (void)id_list; (void)speed_list; (void)param; (void)mode; (void)n;
}

/**
 * @brief 单电机扭矩控制
 * @param mode 1:直通扭矩；否则：扭矩爬升
 */
void set_torque(uint8_t id_num, float torque, float param, int mode)
{
    float factor = 0.01f;
    int u16_input_mode, s16_ramp_rate;
    if (mode == 1) {
        u16_input_mode = INPUT_MODE_PASSTHROUGH;
        s16_ramp_rate = 0;
    } else {
        u16_input_mode = INPUT_MODE_TORQUE_RAMP;
        s16_ramp_rate = (int)(param / factor);
    }
    float v[3] = {torque, (float)s16_ramp_rate, (float)u16_input_mode};
    int tp[3] = {0, 2, 1};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x1D, data_list.byte_data, 0);
    reply_state(id_num);
}

/**
 * @brief 多电机扭矩控制（当前为占位实现）
 */
void set_torques(uint8_t *id_list, float *torque_list, float param, int mode, size_t n)
{
    (void)id_list; (void)torque_list; (void)param; (void)mode; (void)n;
}

/**
 * @brief 阻抗控制（MIT 风格：pos/vel/tff + kp/kd）
 */
void impedance_control(uint8_t id_num, float pos, float vel, float tff, float kp, float kd)
{
    float factor = 0.01f;
    preset_angle(id_num, pos, vel, tff, 2);
    unsigned char order_num = 0x15;
    float v[3] = {(float)order_num, (float)((int)(kp / factor)), (float)((int)(kd / factor))};
    int tp[3] = {3, 2, 2};
    format_data(v, tp, 3, "encode");
    send_command(0, 0x08, data_list.byte_data, 0);
}

/**
 * @brief 急停：切换到 IDLE 并产生 estop 错误标志
 */
void estop(uint8_t id_num)
{
    unsigned char order_num = 0x06;
    float v[3] = {(float)order_num, 0, 0};
    int tp[3] = {3, 1, 1};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x08, data_list.byte_data, 0);
}

/**
 * @brief 设置电机 CAN ID（掉电保存）
 * @note 内部通过 write_property + save_config 实现。
 */
void set_id(uint8_t id_num, int new_id)
{
    write_property(id_num, 31001, 3, (float)new_id);
    save_config((uint8_t)new_id);
}

/** @brief 设置 UART 波特率（掉电保存） */
void set_uart_baud_rate(uint8_t id_num, int baud_rate)
{
    write_property(id_num, 10001, 3, (float)baud_rate);
    save_config(id_num);
}

/** @brief 设置 CAN 波特率（掉电保存） */
void set_can_baud_rate(uint8_t id_num, int baud_rate)
{
    write_property(id_num, 21001, 3, (float)baud_rate);
    save_config(id_num);
}

/**
 * @brief 设置电机运行模式
 * @param mode 1:IDLE  2:CLOSED_LOOP_CONTROL
 */
void set_mode(uint8_t id_num, int mode)
{
    if (mode == 1) write_property(id_num, 30003, 3, (float)AXIS_STATE_IDLE);
    else if (mode == 2) write_property(id_num, 30003, 3, (float)AXIS_STATE_CLOSED_LOOP_CONTROL);
}

/** @brief 设置当前位置为零点（角度清零） */
void set_zero_position(uint8_t id_num)
{
    unsigned char order_num = 0x05;
    float v[3] = {(float)order_num, 0, 0};
    int tp[3] = {3, 1, 1};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x08, data_list.byte_data, 0);
}

/**
 * @brief 设置 GPIO 控制接口模式（当前为占位实现）
 * @note 如需使用 UART/StepDir 模式切换，可参考官方 Arduino 库补全。
 */
void set_GPIO_mode(uint8_t id_num, uint8_t mode, uint32_t param)
{
    (void)id_num; (void)mode; (void)param;
}

/**
 * @brief 设置软件限位范围（当前为占位实现）
 * @return 0：未实现；如按官方库补全可返回 <0/0/>0 代表失败/成功等。
 */
int8_t set_angle_range(uint8_t id_num, float angle_min, float angle_max)
{
    (void)id_num; (void)angle_min; (void)angle_max;
    return 0;
}

/**
 * @brief 设置速度轨迹模式（梯形/S 形），影响多电机轨迹时间计算
 */
void set_traj_mode(uint8_t id_num, int mode)
{
    write_property(id_num, 35104, 3, (float)mode);
    TRAJ_MODE = (int8_t)mode;
}

/**
 * @brief 写入电机属性参数（控制参数）
 * @note 若需要掉电保存，写入后调用 save_config()。
 */
void write_property(uint8_t id_num, unsigned short param_address, int8_t param_type, float value)
{
    float v[3] = {(float)param_address, (float)param_type, value};
    int tp[3] = {1, 1, param_type};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x1F, data_list.byte_data, 0);
}

/**
 * @brief 读取属性参数（状态量/控制参数）
 * @return 成功返回读取值；失败返回 0 且 READ_FLAG=-1
 */
float read_property(uint8_t id_num, int param_address, int param_type)
{
    float v[3] = {(float)param_address, (float)param_type, 0};
    int tp[3] = {1, 1, 3};
    format_data(v, tp, 3, "encode");
    READ_FLAG = 0;
    send_command(id_num, 0x1E, data_list.byte_data, 0);
    receive_data();
    if (READ_FLAG == 1) {
        float vv[3] = {0, 0, 0};
        int tt[3] = {1, 1, param_type};
        format_data(vv, tt, 3, "decode");
        return data_list.value_data[2];
    }
    READ_FLAG = -1;
    return 0;
}

/** @brief 读取电机 CAN ID（property 31001） */
uint8_t get_id(uint8_t id_num)
{
    return (uint8_t)read_property(id_num, 31001, 3);
}

/**
 * @brief 快速读取电机位置/速度
 * @note 同时会刷新内部 `motor_state` 二维数组。
 */
struct servo_state get_state(uint8_t id_num)
{
    struct servo_state state = {0, 0};
    float v[3] = {0x00, 0x00, 0};
    int tp[3] = {1, 1, 3};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x1E, data_list.byte_data, 0);
    READ_FLAG = 0;
    receive_data();
    if (id_num <= MOTOR_NUM) {
        if (READ_FLAG == 1) {
            if (id_num == 0) id_num = (uint8_t)((can_id & 0x07E0) >> 5) & 0xFF;
            float factor = 0.01f;
            float vv[3] = {0, 0, 0};
            int tt[3] = {0, 2, 2};
            format_data(vv, tt, 3, "decode");
            motor_state[id_num - 1][0] = data_list.value_data[0];
            motor_state[id_num - 1][1] = data_list.value_data[1] * factor;
            motor_state[id_num - 1][2] = data_list.value_data[2] * factor;
            motor_state[id_num - 1][3] = (int)((can_id & 0x02) >> 1);
            motor_state[id_num - 1][4] = (int)((can_id & 0x04) >> 2);
            state.angle = data_list.value_data[0];
            state.speed = data_list.value_data[1] * factor;
        } else {
            READ_FLAG = -1;
            reply_state_error++;
        }
    }
    return state;
}

/**
 * @brief 读取电机电压和电流
 * @note 内部通过两次 read_property 完成。
 */
struct servo_volcur get_volcur(uint8_t id_num)
{
    struct servo_volcur volcur = {0, 0};
    volcur.vol = read_property(id_num, 1, 0);
    if (READ_FLAG == 1) volcur.cur = read_property(id_num, 33206, 0);
    else READ_FLAG = -1;
    return volcur;
}

/**
 * @brief 读取 GPIO 控制接口模式
 * @param enable_uart     输出：是否为 UART 模式
 * @param enable_step_dir 输出：是否为 Step/Dir 模式
 * @param n               输出：当前串口波特率或每圈脉冲数（视模式而定）
 * @return 1 成功；0 通信失败；-1 组合状态非法
 */
int8_t get_GPIO_mode(uint8_t id_num, uint8_t *enable_uart, uint8_t *enable_step_dir, uint32_t *n)
{
    if (enable_uart != NULL) *enable_uart = read_property(id_num, 10008, 1) == 0 ? 0 : 1;
    if (enable_step_dir != NULL) *enable_step_dir = read_property(id_num, 31006, 1) == 0 ? 0 : 1;
    if (READ_FLAG == 1) {
        if (enable_uart != NULL && enable_step_dir != NULL && *enable_uart && !*enable_step_dir) {
            if (n != NULL) *n = (uint32_t)read_property(id_num, 10001, 3);
            return 1;
        } else if (enable_uart != NULL && enable_step_dir != NULL && !*enable_uart && *enable_step_dir) {
            if (n != NULL) *n = (uint32_t)read_property(id_num, 38019, 0);
            return 1;
        }
        return -1;
    }
    return 0;
}

/** @brief 清除错误标志（clear_errors 命令） */
void clear_error(uint8_t id_num)
{
    unsigned char order_num = 0x04;
    float v[3] = {(float)order_num, 0, 0};
    int tp[3] = {3, 1, 1};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x08, data_list.byte_data, 0);
}

/**
 * @brief 汇总错误域（按官方库顺序读取多个错误地址）
 *
 * @return 0 无错误；1~7 表示某类错误非 0；负数表示读该域时通信失败
 */
int8_t dump_error(uint8_t id_num)
{
    unsigned int axis_error = (unsigned int)read_property(id_num, 30001, 3);
    if (READ_FLAG == 1 && axis_error != 0) return 1;
    else if (READ_FLAG != 1) return -1;
    unsigned int motor_error = (unsigned int)read_property(id_num, 33001, 3);
    if (READ_FLAG == 1 && motor_error != 0) return 2;
    else if (READ_FLAG != 1) return -2;
    unsigned int controller_error = (unsigned int)read_property(id_num, 32001, 3);
    if (READ_FLAG == 1 && controller_error != 0) return 3;
    else if (READ_FLAG != 1) return -3;
    unsigned int encoder_error = (unsigned int)read_property(id_num, 34001, 3);
    if (READ_FLAG == 1 && encoder_error != 0) return 4;
    else if (READ_FLAG != 1) return -4;
    unsigned int can_error = (unsigned int)read_property(id_num, 20001, 3);
    if (READ_FLAG == 1 && can_error != 0) return 5;
    else if (READ_FLAG != 1) return -5;
    unsigned int fet_thermistor_error = (unsigned int)read_property(id_num, 36001, 3);
    if (READ_FLAG == 1 && fet_thermistor_error != 0) return 6;
    else if (READ_FLAG != 1) return -6;
    unsigned int motor_thermistor_error = (unsigned int)read_property(id_num, 37001, 3);
    if (READ_FLAG == 1 && motor_thermistor_error != 0) return 7;
    else if (READ_FLAG != 1) return -7;
    return 0;
}

void save_config(uint8_t id_num)
{
    unsigned char order_num = 0x01;
    float v[3] = {(float)order_num, 0, 0};
    int tp[3] = {3, 1, 1};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x08, data_list.byte_data, 0);
}

/** @brief 软件重启电机，效果类似重新上电 */
void reboot(uint8_t id_num)
{
    unsigned char order_num = 0x03;
    float v[3] = {(float)order_num, 0, 0};
    int tp[3] = {3, 1, 1};
    format_data(v, tp, 3, "encode");
    send_command(id_num, 0x08, data_list.byte_data, 0);
}

/**
 * @brief 阻塞等待到位（traj_done）
 *
 * @note 该实现与官方库一致：可能会阻塞较久。
 *       如果你不希望卡住，建议在示例侧使用带超时的等待函数。
 */
void position_done(uint8_t id_num)
{
    int traj_done = 0;
    while (traj_done == 0 || READ_FLAG == -1) traj_done = (int)read_property(id_num, 32008, 3);
}

void positions_done(uint8_t *id_list, size_t n)
{
    for (size_t i = 0; i < n; i++) position_done(id_list[i]);
}

