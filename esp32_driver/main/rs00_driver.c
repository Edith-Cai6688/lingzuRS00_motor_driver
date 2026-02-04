#include "rs00_driver.h"
#include "utils.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "lingzu_test_main.h"
#include <string.h>

static const char *TAG = "RS00_DRV";

motor_status motor;


/* 通用发送函数 */
static esp_err_t send_frame(uint8_t mode, uint16_t master_id, uint8_t motor_id, uint8_t *data, uint8_t dlc) {
    twai_message_t msg = {
        .identifier = pack_id(mode, master_id, motor_id),
        .extd = 1,
        .data_length_code = dlc, 
    };
    
    // 拷贝实际长度的数据
    if (data && dlc > 0) {
        memcpy(msg.data, data, dlc);
    }
    
    return twai_transmit(&msg, pdMS_TO_TICKS(10));
}

/* --------------------- 电机基础管理函数 --------------------- */

/**
 * @brief 使能电机。发送使能指令并等待电机反馈进入 "Motor" 模式。
 * @param motor_id 目标电机 ID
 * @return err: ESP_OK 使能成功并已就绪
 */
esp_err_t rs00_enable_motor(uint8_t motor_id) {
    esp_err_t err;
    ESP_LOGI(TAG, "正在使能电机 ID: %d...", MOTOR_ID);

    //发送指令
    uint8_t data[1] = {0}; 
    err = send_frame(0x03, MASTER_ID_DEFAULT, motor_id, data, 1);

    //判断指令是否发送成功
    if (err != ESP_OK){
        ESP_LOGE(TAG, "使能指令发送失败: %s", esp_err_to_name(err)); 
        return err;
    }else{
        int retry = 20; // 尝试 20 次，每次 10ms，总共 200ms 超时
        while (retry--) {
            if (motor.mode_num == 2) {
                ESP_LOGI(TAG, "使能成功！模式已切换为运行状态！");
                return ESP_OK;
            }
        vTaskDelay(pdMS_TO_TICKS(10)); // 关键：释放 CPU 让接收任务跑
        }
        ESP_LOGE(TAG, "使能超时！电机未响应或状态未改变，当前模式: %d", motor.mode_num);
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief 停止电机。发送停止指令并等待电机反馈进入 "复位" 模式。
 * @param motor_id 目标电机 ID
 * @return err: ESP_OK 停止电机成功
 */
esp_err_t rs00_stop_motor(uint8_t motor_id) {
    esp_err_t err;
    ESP_LOGI(TAG, "正在停止电机 ID: %d...", MOTOR_ID);

    //发送指令
    uint8_t data[8] = {0};
    if (motor.fault) data[0] = 0x01;
    err = send_frame(0x04, MASTER_ID_DEFAULT, motor_id, data, 8);

    //判断指令是否发送成功
    if (err != ESP_OK){
        ESP_LOGE(TAG, "停止指令发送失败: %s", esp_err_to_name(err)); 
        return err;
    }else{
        int retry = 20; // 尝试 20 次，每次 10ms，总共 200ms 超时
        while (retry--) {
            if (motor.mode_num == 0) {
                ESP_LOGI(TAG, "停止成功！模式已切换为复位状态！");
                return ESP_OK;
            }
        vTaskDelay(pdMS_TO_TICKS(10)); // 关键：释放 CPU 让接收任务跑
        }
        ESP_LOGE(TAG, "使能超时！电机未响应或状态未改变，当前模式: %d", motor.mode_num);
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief 获取设备ID,使用的时候can总线上只能接一个电机
 * @param motor_id 目标电机 ID
 * @return err: ESP_OK 停止电机成功
 */
esp_err_t rs00_get_device_id() {
    esp_err_t err;
    ESP_LOGI(TAG, "开始广播搜索电机...");

    //发送命令
    uint8_t data[1] = {0};
    err = send_frame(0x00, MASTER_ID_DEFAULT, 0, data, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); //给can回包时间

    //解析电机ID
    if (motor.id != 0) {
        ESP_LOGI(TAG, "发现电机！真实 ID 为: %d", motor.id);
        return err;
    } else {
        ESP_LOGE(TAG, "广播搜索失败，请检查接线。");
        return err;
    }
}

/**
 * @brief 设置设备ID,使用的时候can总线上只能接一个电机
 * @param old_id 原 ID
 * @param new_id 新 ID
 * @return err: ESP_OK 设置设备ID成功
 */
esp_err_t rs00_set_can_id(uint8_t old_id, uint8_t new_id) {
    esp_err_t err;
    ESP_LOGI(TAG, "开始设置ID: %d电机的新ID为:%d", old_id, new_id);

    //准备数据 bit8～15是master_id， bit16~23是新ID
    uint16_t mid = ((uint16_t)new_id << 8) | (MASTER_ID_DEFAULT & 0xFF);
    uint8_t data[1] = {0};

    //发送命令
    err = send_frame(0x07, mid, old_id, data, 1);

    //判断指令是否发送成功
    if (err != ESP_OK){
        ESP_LOGE(TAG, "设置ID指令发送失败: %s", esp_err_to_name(err)); 
        return err;
    }else{
        int retry = 20; // 尝试 20 次，每次 10ms，总共 200ms 超时
        while (retry--) {
            if (motor.id == new_id) {
                ESP_LOGI(TAG, "设置ID指令发送成功!当前ID为:%d", motor.id);
                return ESP_OK;
            }
        vTaskDelay(pdMS_TO_TICKS(10)); // 关键：释放 CPU 让接收任务跑
        }
        ESP_LOGE(TAG, "命令超时!电机ID更改失败,当前电机ID为: %d", motor.id);
        return ESP_ERR_TIMEOUT;
    }
}

/* 4. 设置零位 */
esp_err_t rs00_set_zero(uint8_t motor_id) {
    uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return send_frame(0x06, MASTER_ID_DEFAULT, motor_id, data, 8);
}



/* 6. 写单个参数 (对应 Python write_parameter) */
esp_err_t rs00_write_parameter(uint8_t motor_id, uint16_t index, float value) {
    uint8_t data[8];
    pack_write_param_little(data, index, value);
    esp_err_t err = send_frame(0x12, MASTER_ID_DEFAULT, motor_id, data, 8);
    if (err == ESP_OK) {
        rs00_save_parameter(motor_id); // 对应 Python 里的 self.save_parameter()
    }
    return err;
}

/* 7. 保存参数到 Flash */
esp_err_t rs00_save_parameter(uint8_t motor_id) {
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    return send_frame(0x16, MASTER_ID_DEFAULT, motor_id, data, 8);
}

/* 8. 修改运行模式 */
esp_err_t rs00_write_runmode(uint8_t motor_id, rs00_runmode_t mode) {
    // 对应 Python write_runmode: 将 mode 转为 4 字节小端
    uint32_t mode_val = (uint32_t)mode;
    uint8_t data[8];
    // 使用 pack_write_param_little，但这里传入的是 int 的内存布局
    pack_write_param_little(data, 0x7005, *(float*)&mode_val);
    return send_frame(0x12, MASTER_ID_DEFAULT, motor_id, data, 8);
}

/* --- 控制函数 --- */

// 运控模式
esp_err_t rs00_send_motion_control(uint8_t motor_id, float pos, float vel, float kp, float kd, float tau) {
    uint8_t data[8];
    uint16_t p = float_to_uint(pos, P_MIN, P_MAX);
    uint16_t v = float_to_uint(vel, V_MIN, V_MAX);
    uint16_t k_p = float_to_uint(kp, Kp_MIN, Kp_MAX);
    uint16_t k_d = float_to_uint(kd, Kd_MIN, Kd_MAX);
    uint16_t t = float_to_uint(tau, T_MIN, T_MAX);

    pack_data_8bytes_big(data, p, v, k_p, k_d);
    return send_frame(0x01, t, motor_id, data, 8);
}

// 速度模式 (对应 Python send_speed_control)
esp_err_t rs00_send_speed_control(uint8_t motor_id, float limit_cur, float speed_ref, float acc_rad) {
    rs00_write_parameter(motor_id, 0x7018, limit_cur);
    rs00_write_parameter(motor_id, 0x7022, acc_rad);
    return rs00_write_parameter(motor_id, 0x700A, speed_ref);
}

// 电流模式 (对应 Python send_current_control)
esp_err_t rs00_send_current_control(uint8_t motor_id, float iq_ref) {
    return rs00_write_parameter(motor_id, 0x7006, iq_ref);
}

// 位置模式 CSP (对应 Python send_loc_control_csp)
esp_err_t rs00_send_loc_control_csp(uint8_t motor_id, float limit_spd, float loc_ref) {
    rs00_write_parameter(motor_id, 0x7017, limit_spd);
    return rs00_write_parameter(motor_id, 0x7016, loc_ref);
}

// 位置模式 PP (对应 Python send_loc_control_pp)
esp_err_t rs00_send_loc_control_pp(uint8_t motor_id, float loc_ref, float vel_max, float acc_set) {
    rs00_write_parameter(motor_id, 0x7024, vel_max);
    rs00_write_parameter(motor_id, 0x7025, acc_set);
    return rs00_write_parameter(motor_id, 0x7016, loc_ref);
}