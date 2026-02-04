#include "utils.h"
#include <string.h>

/**
 * @brief 打包 29 位逻辑 ID 
 */
uint32_t pack_id(uint8_t mode, uint16_t master_id, uint8_t motor_id) {
    return ((uint32_t)(mode & 0x1F) << 24) | 
           ((uint32_t)(master_id & 0xFFFF) << 8) | 
           ((uint32_t)motor_id);
}

/**
 * @brief 解析 29 位逻辑 ID 
 * 注意：输入的 can_id 已经是去掉 USB-CAN 适配器那 3 位控制位的原始 29 位 ID
 */
void unpack_id(uint32_t logic_id, uint8_t *mode, uint16_t *master_id, uint8_t *motor_id) {
    *mode = (logic_id >> 24) & 0x1F;
    *master_id = (logic_id >> 8) & 0xFFFF;
    *motor_id = logic_id & 0xFF;
}

/**
 * @brief 将浮点数映射为无符号整数
 */
uint16_t float_to_uint(float v, float vmin, float vmax) {
    if (v < vmin) v = vmin;
    if (v > vmax) v = vmax;
    float span = vmax - vmin;
    return (uint16_t)((v - vmin) * 65535.0f / span);
}

/**
 * @brief 将无符号整数映射回浮点数 
 */
float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float res = (float)x_int * span / ((1 << bits) - 1) + x_min;
    return res; 
}

void pack_data_8bytes_big(uint8_t *dst, uint16_t p, uint16_t v, uint16_t kp, uint16_t kd) {
    dst[0] = (p >> 8) & 0xFF;  dst[1] = p & 0xFF;
    dst[2] = (v >> 8) & 0xFF;  dst[3] = v & 0xFF;
    dst[4] = (kp >> 8) & 0xFF; dst[5] = kp & 0xFF;
    dst[6] = (kd >> 8) & 0xFF; dst[7] = kd & 0xFF;
}

void pack_write_param_little(uint8_t *dst, uint16_t index, float value) {
    dst[0] = index & 0xFF;         // Index 低位
    dst[1] = (index >> 8) & 0xFF;  // Index 高位
    dst[2] = 0x00; dst[3] = 0x00;  // 占位
    memcpy(&dst[4], &value, 4);    // Float 小端直接拷贝
}

/**
 * @brief 解析电机反馈数据包 
 * @param can_id 接收到的扩展帧 ID
 * @param data 接收到的 8 字节数据
 */
motor_status_t decode_motor_feedback(uint32_t can_id, uint8_t* data) {
    motor_status_t status;
    uint8_t mode_type;
    uint16_t master_id;
    uint8_t motor_id_from_id;

    // 1. 解析 ID 字段
    unpack_id(can_id, &mode_type, &master_id, &motor_id_from_id);
    
    // 根据协议：status 字段隐藏在 master_id 部分
    // Bit 8~15: ID, Bit 16~21: Fault, Bit 22~23: Mode
    uint16_t status_reg = master_id; 
    status.id = status_reg & 0xFF;
    uint8_t fault_bits = (status_reg >> 8) & 0x3F;
    uint8_t mode_state = (status_reg >> 14) & 0x03;

    status.mode_num = mode_state;

    // 2. 解析 8 字节数据 (大端序)
    uint16_t t_int   = (data[0] << 8) | data[1];
    uint16_t tau_int = (data[2] << 8) | data[3];
    uint16_t vel_int = (data[4] << 8) | data[5];
    uint16_t pos_int = (data[6] << 8) | data[7];

    // 3. 转换物理量
    status.temp = (float)t_int / 10.0f;
    status.tau  = uint_to_float(tau_int, T_MIN, T_MAX, 16);
    status.vel  = uint_to_float(vel_int, V_MIN, V_MAX, 16);
    status.pos  = uint_to_float(pos_int, P_MIN, P_MAX, 16);

    // 4. 故障位映射
    for(int i=0; i<6; i++) {
        status.faults[i] = (fault_bits >> i) & 0x01;
    }

    return status;
}