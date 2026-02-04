#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdbool.h>


#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -33.0f
#define V_MAX 33.0f
#define T_MIN -14.0f
#define T_MAX 14.0f
#define Kp_MIN 0.0f
#define Kp_MAX 500.0f
#define Kd_MIN 0.0f
#define Kd_MAX 5.0f

typedef struct {
    uint8_t id;
    uint8_t mode_num;
    bool faults[6]; // 欠压, 驱动, 过温, 磁编码, 堵转, 未标定
    float pos;
    float vel;
    float tau;
    float temp;
} motor_status_t;


// 映射 Python 的 pack_id: (mode << 24) | (master_id << 8) | motor_id
uint32_t pack_id(uint8_t mode, uint16_t master_id, uint8_t motor_id);
void unpack_id(uint32_t can_id, uint8_t *mode, uint16_t *master_id, uint8_t *motor_id);

// 映射 Python 的 float_to_uint (16-bit 缩放)
uint16_t float_to_uint(float v, float vmin, float vmax);
float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits);

// 大端序打包 8 字节 (用于运控模式)
void pack_data_8bytes_big(uint8_t *dst, uint16_t p, uint16_t v, uint16_t kp, uint16_t kd);

// 小端序打包 (用于写参数)
void pack_write_param_little(uint8_t *dst, uint16_t index, float value);

// 解析电机反馈数据包
motor_status_t decode_motor_feedback(uint32_t can_id, uint8_t* data);

#endif