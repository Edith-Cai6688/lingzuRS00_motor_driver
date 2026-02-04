/**
 * @file lingzu_motor_can.c
 * @brief 灵足/RS00 电机 CAN 控制（ESP32 TWAI 适配）。
 *
 * 说明：
 * - 29 位扩展帧：bit28~24 消息类型，bit23~8 源 ID，bit7~0 目标 ID（假设，若实机不同请调整）。
 * - 发送前需完成 twai_driver_install + twai_start。
 * - 控制帧字段取值区间依据手册截图，如与实机不符，请按需修改缩放常量。
 */

#include "lingzu_motor_can.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#define TAG "LZ_CAN"

#define MSG_TYPE_SHIFT 24
#define SRC_ID_SHIFT   8
#define MSG_TYPE_MASK  0x1F
#define ID_FIELD_MASK  ((1u << 29) - 1)

/* 缩放范围取自手册截图，若实机不同请调整。 */
#define ANGLE_MIN_RAD   (-12.57f)
#define ANGLE_MAX_RAD   (12.57f)
#define SPEED_MIN_RAD_S (-33.0f)
#define SPEED_MAX_RAD_S (33.0f)
#define TORQUE_MIN_NM   (-14.0f)
#define TORQUE_MAX_NM   (14.0f)
#define KP_MIN          (0.0f)
#define KP_MAX          (500.0f)
#define KD_MIN          (0.0f)
#define KD_MAX          (500.0f)

static float clampf(float v, float vmin, float vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

static uint16_t float_to_u16(float v, float vmin, float vmax)
{
    float clamped = clampf(v, vmin, vmax);
    float span = vmax - vmin;
    if (span <= 0.0f) {
        return 0;
    }
    int32_t scaled = (int32_t)lroundf((clamped - vmin) * 65535.0f / span);
    if (scaled < 0) scaled = 0;
    if (scaled > 0xFFFF) scaled = 0xFFFF;
    return (uint16_t)scaled;
}

static float u16_to_float(uint16_t raw, float vmin, float vmax)
{
    float span = vmax - vmin;
    return vmin + (span * (float)raw / 65535.0f);
}

static void pack_u16(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)((v >> 8) & 0xFF);
    dst[1] = (uint8_t)(v & 0xFF);
}

/**
 * @brief 组装扩展帧 ID（29 位）。
 */
uint32_t lz_compose_id(uint8_t msg_type, uint16_t src_id, uint8_t dst_id)
{
    uint32_t id = ((uint32_t)(msg_type & MSG_TYPE_MASK) << MSG_TYPE_SHIFT) |
                  ((uint32_t)(src_id & 0xFFFF) << SRC_ID_SHIFT) |
                  ((uint32_t)dst_id);
    return id & ID_FIELD_MASK;
}

/* 发送一帧扩展数据帧，DLC 固定 8 字节。 */
static esp_err_t send_frame(uint8_t msg_type, uint16_t src_id, uint8_t dst_id, const uint8_t data[8])
{
    twai_message_t msg = {
        .identifier = lz_compose_id(msg_type, src_id, dst_id),
        .extd = 1,
        .rtr = 0,
        .ss = 0,
        .data_length_code = 8,
    };
    memcpy(msg.data, data, 8);
    return twai_transmit(&msg, pdMS_TO_TICKS(10));
}

/**
 * @brief 发送原始 8 字节指令。
 */
esp_err_t lz_send_cmd(uint8_t msg_type, uint16_t src_id, uint8_t dst_id, const uint8_t data[8])
{
    return send_frame(msg_type, src_id, dst_id, data);
}

/**
 * @brief 发送控制帧（角度/速度/力矩/KD）。
 */
esp_err_t lz_send_control(uint16_t src_id, uint8_t dst_id, float angle_rad, float speed_rad_s,
                          float torque_nm, float kp, float kd)
{
    uint8_t payload[8];
    /* 布局假设：角度、速度、力矩、KD。KP 暂未写入（协议有歧义，如需写入请确认字节分配）。 */
    pack_u16(&payload[0], float_to_u16(angle_rad, ANGLE_MIN_RAD, ANGLE_MAX_RAD));
    pack_u16(&payload[2], float_to_u16(speed_rad_s, SPEED_MIN_RAD_S, SPEED_MAX_RAD_S));
    pack_u16(&payload[4], float_to_u16(torque_nm, TORQUE_MIN_NM, TORQUE_MAX_NM));
    pack_u16(&payload[6], float_to_u16(kd, KD_MIN, KD_MAX));
    return send_frame(LZ_MSG_CONTROL, src_id, dst_id, payload);
}

/**
 * @brief 使能电机。
 */
esp_err_t lz_enable_motor(uint16_t src_id, uint8_t dst_id)
{
    const uint8_t data[8] = {0};
    return send_frame(LZ_MSG_ENABLE, src_id, dst_id, data);
}

/**
 * @brief 停止电机。
 */
esp_err_t lz_stop_motor(uint16_t src_id, uint8_t dst_id)
{
    const uint8_t data[8] = {0};
    return send_frame(LZ_MSG_STOP, src_id, dst_id, data);
}

/**
 * @brief 清除故障。
 */
esp_err_t lz_clear_fault(uint16_t src_id, uint8_t dst_id)
{
    const uint8_t data[8] = {0};
    return send_frame(LZ_MSG_CLEAR_FAULT, src_id, dst_id, data);
}

/**
 * @brief 机械零位设定。
 */
esp_err_t lz_set_zero(uint16_t src_id, uint8_t dst_id)
{
    uint8_t data[8] = {0};
    data[0] = 0x01;  // per manual Byte0=1 to clear/zero
    return send_frame(LZ_MSG_SET_ZERO, src_id, dst_id, data);
}

/**
 * @brief 请求 UID。
 */
esp_err_t lz_request_uid(uint16_t src_id, uint8_t dst_id)
{
    const uint8_t data[8] = {0};
    return send_frame(LZ_MSG_DEVICE_UID_REQ, src_id, dst_id, data);
}

/**
 * @brief 修改电机 CAN ID。
 */
esp_err_t lz_set_motor_can_id(uint16_t src_id, uint8_t current_id, uint16_t new_id)
{
    uint8_t data[8] = {0};
    data[2] = (uint8_t)((new_id >> 8) & 0xFF);
    data[3] = (uint8_t)(new_id & 0xFF);
    return send_frame(LZ_MSG_SET_CAN_ID, src_id, current_id, data);
}

/**
 * @brief 读取参数索引。
 */
esp_err_t lz_read_param(uint16_t src_id, uint8_t dst_id, uint16_t index)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)((index >> 8) & 0xFF);
    data[1] = (uint8_t)(index & 0xFF);
    return send_frame(LZ_MSG_READ_PARAM, src_id, dst_id, data);
}

/**
 * @brief 写入参数索引及数据（最多 6 字节）。
 */
esp_err_t lz_write_param(uint16_t src_id, uint8_t dst_id, uint16_t index, const uint8_t *payload, uint8_t len)
{
    if (len > 6) {
        ESP_LOGE(TAG, "write_param payload too long (%u)", (unsigned)len);
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data[8] = {0};
    data[0] = (uint8_t)((index >> 8) & 0xFF);
    data[1] = (uint8_t)(index & 0xFF);
    if (payload && len > 0) {
        memcpy(&data[2], payload, len);
    }
    return send_frame(LZ_MSG_WRITE_PARAM, src_id, dst_id, data);
}

/**
 * @brief 保存当前参数到闪存。
 */
esp_err_t lz_save_params(uint16_t src_id, uint8_t dst_id)
{
    /* Manual shows arbitrary bytes; use 1..8 sequence as placeholder. */
    const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    return send_frame(LZ_MSG_SAVE_PARAMS, src_id, dst_id, data);
}

/**
 * @brief 修改电机波特率（需确认电机端支持的码表）。
 */
esp_err_t lz_change_bitrate(uint16_t src_id, uint8_t dst_id, uint8_t bitrate_code)
{
    /* F_CMD 在 Byte6；码表：1=1M,2=500k,3=250k,4=125k。 */
    uint8_t data[8] = {0};
    data[6] = bitrate_code;
    return send_frame(LZ_MSG_CHANGE_BITRATE, src_id, dst_id, data);
}

/**
 * @brief 设置主动上报开关与周期。
 */
esp_err_t lz_set_active_report(uint16_t src_id, uint8_t dst_id, bool enable, uint8_t period_code)
{
    /* Byte6 使能标志；Byte7 上报周期/保留，依固件定义。 */
    uint8_t data[8] = {0};
    data[6] = enable ? 0x01 : 0x00;
    data[7] = period_code;
    return send_frame(LZ_MSG_ACTIVE_REPORT, src_id, dst_id, data);
}

/**
 * @brief 解析反馈帧（类型 0x2/0x15）。
 */
void lz_parse_feedback(const twai_message_t *msg, lz_feedback_t *out)
{
    if (!msg || !out) return;
    memset(out, 0, sizeof(*out));
    out->raw_msg = *msg;
    out->valid = false;

    if (!msg->extd || msg->data_length_code < 8) {
        return;
    }
    uint8_t msg_type = (uint8_t)((msg->identifier >> MSG_TYPE_SHIFT) & MSG_TYPE_MASK);
    if (msg_type != LZ_MSG_FEEDBACK && msg_type != LZ_MSG_FAULT_FEEDBACK) {
        return;
    }
    const uint8_t *d = msg->data;
    uint16_t angle_raw = ((uint16_t)d[0] << 8) | d[1];
    uint16_t speed_raw = ((uint16_t)d[2] << 8) | d[3];
    uint16_t torque_raw = ((uint16_t)d[4] << 8) | d[5];
    uint16_t fault = ((uint16_t)d[0] << 8) | d[1];  // 与 angle_raw 同源，便于上层直接判位
    float temp_c = (float)d[6] + ((float)d[7]) / 10.0f;  // 手册：温度*10

    out->fault_bits = fault;
    out->angle_rad = u16_to_float(angle_raw, ANGLE_MIN_RAD, ANGLE_MAX_RAD);
    out->speed_rad_s = u16_to_float(speed_raw, SPEED_MIN_RAD_S, SPEED_MAX_RAD_S);
    out->torque_nm = u16_to_float(torque_raw, TORQUE_MIN_NM, TORQUE_MAX_NM);
    out->temperature_c = temp_c;
    out->valid = true;
}

/**
 * @brief 阻塞等待一帧反馈并解析。
 */
esp_err_t lz_wait_feedback(TickType_t timeout_ticks, lz_feedback_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    twai_message_t msg;
    esp_err_t err = twai_receive(&msg, timeout_ticks);
    if (err != ESP_OK) {
        return err;
    }
    lz_parse_feedback(&msg, out);
    return out->valid ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}
