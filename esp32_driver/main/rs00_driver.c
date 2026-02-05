#include "rs00_driver.h"
#include "utils.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "lingzu_test_main.h"
#include <string.h>
#include <math.h>

static const char *TAG = "RS00_DRV";

const mode_map_t rs00_mode_table[] = {
    {0, "MOTION"},
    {1, "PP"},
    {2, "SPEED"},
    {3, "CUR"},
    {5, "CSP"}
};

volatile motor_status motor;


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

/**
 * @brief 设置机械零位
 * @param motor_id 目标电机 ID
 * @return err: ESP_OK 
 */
esp_err_t rs00_set_zero(uint8_t motor_id) {
    esp_err_t err;
    ESP_LOGI(TAG, "开始设置ID: %d电机的机械零位!", MOTOR_ID);

    //发送命令
    uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    err = send_frame(0x06, MASTER_ID_DEFAULT, motor_id, data, 8);

    //判断指令是否发送成功
    if(err != ESP_OK){
        ESP_LOGE(TAG, "设置机械零位指令发送失败: %s", esp_err_to_name(err)); 
        return err;
    }else{
        int retry = 20; // 尝试 20 次，每次 10ms，总共 200ms 超时
        const float tolerance = 0.1f;// 误差容忍度
        while (retry--) {
            if (fabsf(motor.pos) < tolerance) { // 使用绝对值判断约等于0
            ESP_LOGI(TAG, "电机归零成功!当前位置为:%f", motor.pos);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 关键：释放 CPU 让接收任务跑
        }
        ESP_LOGE(TAG, "命令超时!电机归零失败,当前位置为: %f", motor.pos);
        return ESP_ERR_TIMEOUT;}
}


/**
 * @brief 写入参数
 * @param motor_id 目标电机 ID
 * @param index 参数索引
 * @param value 参数值
 * @return err: ESP_OK
 */
esp_err_t rs00_write_parameter(uint8_t motor_id, uint16_t index, float value) {
    //发送命令
    uint8_t data[8];
    pack_write_param_little(data, index, value);
    esp_err_t err = send_frame(0x12, MASTER_ID_DEFAULT, motor_id, data, 8);   
    if (err == ESP_OK) 
        rs00_save_parameter(motor_id); 
    return err;
}

/**
 * @brief 保存参数
 * @param motor_id 目标电机 ID
 * @return err: ESP_OK 
 */
esp_err_t rs00_save_parameter(uint8_t motor_id) {
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    return send_frame(0x16, MASTER_ID_DEFAULT, motor_id, data, 8);
}

/**
 * @brief 读取参数
 * @param motor_id 目标电机 ID
 * @param index 参数索引
 * @return err: ESP_OK
 */
esp_err_t rs00_read_parameter(uint8_t motor_id, uint16_t index){
    //发送命令
    uint8_t data[8];
    pack_write_param_little(data, index, 0);
    esp_err_t err = send_frame(0x11, MASTER_ID_DEFAULT, motor_id, data, 8);
    vTaskDelay(pdMS_TO_TICKS(50)); //给读数据时间       
    return err;
}


/* --------------------- 电机控制函数 --------------------- */

/**
 * @brief 修改电机运行模式
 * @param motor_id 目标电机 ID
 * @param mode 电机目标运行模式
 * @return err: ESP_OK 
 **/
esp_err_t rs00_write_runmode(uint8_t motor_id, rs00_runmode_t mode) {
    esp_err_t err;
    ESP_LOGI(TAG, "开始修改ID: %d电机的运行模式!", MOTOR_ID);

    //发送命令
    uint32_t mode_val = (uint32_t)mode;
    err = rs00_write_parameter(motor_id, 0x7005, *(float*)&mode_val);

    //判断指令是否发送成功
    if(err != ESP_OK){
        ESP_LOGE(TAG, "修改电机运行模式指令发送失败:%s", esp_err_to_name(err)); 
        return err;
    }else{
        int retry = 20; // 尝试 20 次，每次 10ms，总共 200ms 超时
        while (retry--) {
            rs00_read_runmode(motor_id);
            vTaskDelay(pdMS_TO_TICKS(20));
            if (motor.index == 0x7005 && motor.data == (uint32_t)mode) { 
                motor.runmode = mode;
                ESP_LOGI(TAG, "修改电机运行模式成功!当前模式为:%s", rs00_mode_table[mode].mode_name);
                return ESP_OK;
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // 关键：释放 CPU 让接收任务跑
        }
        ESP_LOGE(TAG, "命令超时!修改电机运行模式失败,当前模式为:%s", rs00_mode_table[motor.runmode].mode_name);
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief 读取当前运行模式
 * @param motor_id 目标电机 ID
 * @param index 参数索引
 * @return err: ESP_OK
 */
esp_err_t rs00_read_runmode(uint8_t motor_id){
    esp_err_t err;
    ESP_LOGI(TAG, "开始读取ID: %d电机当前的运行模式!", MOTOR_ID);

    err = rs00_read_parameter(motor_id, 0x7005);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "读取运行模式指令发送失败!");
        return err;
    }else{
        ESP_LOGI(TAG, "读取运行模式指令发送成功!当前运行模式为%s", rs00_mode_table[motor.runmode].mode_name);
        return err;
    }
}


/**
 * @brief 运控模式指令发送:
          t_ref=Kd*(v_set-v_actual)+Kp*(p_set-p_actual)+t_ff
 * @param motor_id 目标电机 ID
 * @param pos 目标角度(-12.57f~12.57f)
 * @param vel 目标角速度(-33rad/s~33rad/s)
 * @param kp 比例增益(0~500)
 * @param kd 微分增益(0~5)
 * @param tau 力矩(-14Nm~14Nm)
 * @return err: ESP_OK 
 **/
esp_err_t rs00_send_motion_control(uint8_t motor_id, float pos, float vel, float kp, float kd, float tau) {
    esp_err_t err;
    ESP_LOGD(TAG, "开始对ID: %d电机发送运动控制命令!", MOTOR_ID);

    //准备数据
    uint8_t data[8];
    uint16_t p = float_to_uint(pos, P_MIN, P_MAX);
    uint16_t v = float_to_uint(vel, V_MIN, V_MAX);
    uint16_t k_p = float_to_uint(kp, Kp_MIN, Kp_MAX);
    uint16_t k_d = float_to_uint(kd, Kd_MIN, Kd_MAX);
    uint16_t t = float_to_uint(tau, T_MIN, T_MAX);
    pack_data_8bytes_big(data, p, v, k_p, k_d);
    err = send_frame(0x01, t, motor_id, data, 8);
    vTaskDelay(pdMS_TO_TICKS(10));

    //判断是否发送成功
    if(err != ESP_OK){
        ESP_LOGE(TAG, "ID: %d 运控指令发送失败!", motor_id);
    }
    return err;
}

/**
 * @brief 速度模式指令发送:
 * @param motor_id 目标电机 ID
 * @param limit_cur 电流限制,(0~16A),float类型,4个字节
 * @param acc_rad 加速度,默认值20rad/s^2,float类型,4个字节(暂定范围为5~100,可能还需要后续实验测量)
 * @param speed_ref 速度(-33~33rad/s),float类,4个字节
 * @return err: ESP_OK 
 **/
esp_err_t rs00_send_speed_control(uint8_t motor_id, float limit_cur, float speed_ref, float acc_rad) {
    esp_err_t err = ESP_OK;
    ESP_LOGD(TAG, "开始对ID: %d 电机发送速度控制命令", motor_id);

    // 逐个发送参数，并累加错误状态
    err |= rs00_write_parameter(motor_id, 0x7018, limit_cur);
    err |= rs00_write_parameter(motor_id, 0x7022, acc_rad);
    err |= rs00_write_parameter(motor_id, 0x700A, speed_ref);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ID: %d 速度指令发送过程中存在失败!", motor_id);
    } 

    return err; // 返回累加后的状态，0 代表全部成功
}

/**
 * @brief 电流模式指令发送:
 * @param motor_id 目标电机 ID
 * @param iq_ref 目标电流值,-16~16A
 * @return err: ESP_OK 
 **/
esp_err_t rs00_send_current_control(uint8_t motor_id, float iq_ref) {
    esp_err_t err;
    ESP_LOGD(TAG, "开始对ID: %d 电机发送电流控制命令", motor_id);

    err = rs00_write_parameter(motor_id, 0x7006, iq_ref);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ID: %d 电流指令发送过程中存在失败!", motor_id);
    } 

    return err; 
}

/**
 * @brief 位置模式(CSP)指令发送:
 * @param limit_spd 速度最大限制,0~33rad/s
 * @param loc_ref:目标位置角度,0~2pi rad
 * @return err: ESP_OK 
 **/
esp_err_t rs00_send_loc_control_csp(uint8_t motor_id, float limit_spd, float loc_ref) {
    esp_err_t err = ESP_OK;
    ESP_LOGD(TAG, "开始对ID: %d 电机发送位置(CSP)控制命令", motor_id);

    err |= rs00_write_parameter(motor_id, 0x7017, limit_spd);
    err |= rs00_write_parameter(motor_id, 0x7016, loc_ref);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ID: %d 位置指令(CSP)发送过程中存在失败!", motor_id);
    } 

    return err;
}

/**
 * @brief 位置模式(PP)指令发送:
 * @param vel_max 速度最大限制,默认值位10rad/s
 * @param acc_set 预设加速度,默认值为10rad/s^2
 * @param loc_ref 目标位置角度,0~2pi rad
 * @return err: ESP_OK 
 **/
esp_err_t rs00_send_loc_control_pp(uint8_t motor_id, float loc_ref, float vel_max, float acc_set) {
    esp_err_t err = ESP_OK;
    ESP_LOGD(TAG, "开始对ID: %d 电机发送位置(PP)控制命令", motor_id);

    err |= rs00_write_parameter(motor_id, 0x7024, vel_max);
    err |= rs00_write_parameter(motor_id, 0x7025, acc_set);
    err |= rs00_write_parameter(motor_id, 0x7016, loc_ref);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ID: %d 位置指令(PP)发送过程中存在失败!", motor_id);
    } 

    return err;
}