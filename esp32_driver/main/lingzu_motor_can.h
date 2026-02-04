/**
 * @file lingzu_motor_can.h
 * @brief 灵足/RS00 电机 CAN 控制（ESP32 TWAI 适配）。
 *
 * 协议假设（来源 RS00 手册截图，若与实机不符请告知调整）：
 * - CAN 2.0B，1 Mbps，扩展帧 29 位 ID。
 * - ID 结构（假设）：bit28~24 为消息类型，bit23~8 为源 ID（主控），bit7~0 为目标 ID（电机）。
 * - DLC 固定 8 字节。
 *
 * 封装了常用指令：读 UID、控制帧、使能/停止、清故障、机械零位、改 CAN ID、参数读写、
 * 保存参数、改波特率、主动上报开关，以及反馈解析。
 */

#pragma once

#include "driver/twai.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 手册默认波特率 1 Mbps。 */
#define LZ_CAN_BITRATE 1000000

/** 消息类型（ID 的 bit28~24）。 */
typedef enum {
    LZ_MSG_DEVICE_UID_REQ = 0x00,
    LZ_MSG_CONTROL        = 0x01,
    LZ_MSG_FEEDBACK       = 0x02,
    LZ_MSG_ENABLE         = 0x03,
    LZ_MSG_STOP           = 0x04,
    LZ_MSG_CLEAR_FAULT    = 0x05,
    LZ_MSG_SET_ZERO       = 0x06,
    LZ_MSG_SET_CAN_ID     = 0x07,
    LZ_MSG_READ_PARAM     = 0x11,
    LZ_MSG_WRITE_PARAM    = 0x12,
    LZ_MSG_FAULT_FEEDBACK = 0x15,
    LZ_MSG_SAVE_PARAMS    = 0x16,
    LZ_MSG_CHANGE_BITRATE = 0x17,
    LZ_MSG_ACTIVE_REPORT  = 0x18,
} lz_msg_type_t;

/** 反馈帧解析结果（类型 0x2 / 0x15）。 */
typedef struct {
    float angle_rad;
    float speed_rad_s;
    float torque_nm;
    float temperature_c;
    uint16_t fault_bits;
    twai_message_t raw_msg;
    bool valid;
} lz_feedback_t;

/**
 * @brief 组装 29 位扩展帧 ID。
 * @param msg_type 消息类型（bit28~24）。
 * @param src_id   源 ID（主控，bit23~8）。
 * @param dst_id   目标 ID（电机，bit7~0）。
 * @return 29 位 ID（已掩码）。
 */
uint32_t lz_compose_id(uint8_t msg_type, uint16_t src_id, uint8_t dst_id);

/**
 * @brief 发送原始 8 字节数据帧。
 * @param msg_type 消息类型。
 * @param src_id   源 ID。
 * @param dst_id   目标 ID。
 * @param data     8 字节有效载荷。
 * @return ESP_OK 成功；驱动错误返回对应 err。
 * @note 调用前需已安装并启动 TWAI 驱动。
 */
esp_err_t lz_send_cmd(uint8_t msg_type, uint16_t src_id, uint8_t dst_id, const uint8_t data[8]);

/**
 * @brief 发送控制帧（类型 0x1）。
 * @param src_id     源 ID。
 * @param dst_id     电机 ID。
 * @param angle_rad  目标角度（rad）。
 * @param speed_rad_s目标速度（rad/s）。
 * @param torque_nm  目标力矩（Nm）。
 * @param kp         KP（当前未写入帧，如需请确认协议字节）。
 * @param kd         KD。
 * @return ESP_OK 成功；驱动错误返回对应 err。
 * 缩放假设：角度 ±12.57 rad，速度 ±33 rad/s，力矩 ±14 Nm，KP/KD 0~500。
 * 布局：Byte0~1 角度，Byte2~3 速度，Byte4~5 力矩，Byte6~7 KD。
 */
esp_err_t lz_send_control(uint16_t src_id, uint8_t dst_id, float angle_rad, float speed_rad_s,
                          float torque_nm, float kp, float kd);

/** 常用便捷指令。 */
esp_err_t lz_enable_motor(uint16_t src_id, uint8_t dst_id);
esp_err_t lz_stop_motor(uint16_t src_id, uint8_t dst_id);
esp_err_t lz_clear_fault(uint16_t src_id, uint8_t dst_id);
esp_err_t lz_set_zero(uint16_t src_id, uint8_t dst_id);
esp_err_t lz_request_uid(uint16_t src_id, uint8_t dst_id);
esp_err_t lz_set_motor_can_id(uint16_t src_id, uint8_t current_id, uint16_t new_id);
esp_err_t lz_read_param(uint16_t src_id, uint8_t dst_id, uint16_t index);
esp_err_t lz_write_param(uint16_t src_id, uint8_t dst_id, uint16_t index, const uint8_t *payload, uint8_t len);
esp_err_t lz_save_params(uint16_t src_id, uint8_t dst_id);
esp_err_t lz_change_bitrate(uint16_t src_id, uint8_t dst_id, uint8_t bitrate_code);
esp_err_t lz_set_active_report(uint16_t src_id, uint8_t dst_id, bool enable, uint8_t period_code);

/**
 * @brief 解析反馈帧（0x2 或 0x15）。
 * @param msg 输入 TWAI 消息（需扩展帧且 DLC>=8）。
 * @param out 输出解析结果，解析成功 out->valid=true，否则 false。
 * 非对应类型或长度不足会返回 valid=false。
 */
void lz_parse_feedback(const twai_message_t *msg, lz_feedback_t *out);

/**
 * @brief 简单阻塞接收并解析反馈。
 * @param timeout_ticks twai_receive 等待超时（tick）。
 * @param out 解析结果。
 * @return ESP_OK 收到并解析到反馈；ESP_ERR_TIMEOUT 未收到；其他为驱动错误。
 */
esp_err_t lz_wait_feedback(TickType_t timeout_ticks, lz_feedback_t *out);

#ifdef __cplusplus
}
#endif
