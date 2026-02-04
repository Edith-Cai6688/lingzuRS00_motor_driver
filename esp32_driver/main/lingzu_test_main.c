#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <math.h>

/* FreeRTOS 包含 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* ESP-IDF 包含 */
#include <esp_err.h>
#include <esp_log.h>
#include <driver/twai.h>

/* 自定义驱动包含 */
#include "rs00_driver.h"
#include "utils.h"

/* --------------------- 硬件配置 ------------------ */
#define TX_GPIO_NUM             CONFIG_EXAMPLE_TX_GPIO_NUM
#define RX_GPIO_NUM             CONFIG_EXAMPLE_RX_GPIO_NUM

static const char *TAG = "LINGZU_TEST";
static const char *CAN_TAG = "CAN";

/* CAN (TWAI) 配置参数 */
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); 
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

/* 电机 ID 配置 */
uint8_t MOTOR_ID = 0x01;


/**
 * @brief 电机接受信息并更新状态表
 * 
 */
void can_receive_task(){
    twai_message_t msg;

    while(1){
        if(twai_receive(&msg, portMAX_DELAY) == ESP_OK){ //无数据时任务暂停，释放cpu
            if(msg.extd){
                // 1. 提取 ID 中的关键信息用于分流
                uint8_t mode;
                uint16_t master_id;
                uint8_t motor_id;
                unpack_id(msg.identifier, &mode, &master_id, &motor_id);

                // 2. 根据不同的 Mode 或特征进行解析
                if (mode == 0x00 && motor_id == 0xFE) {
                    // --- 场景 A: ID 查询应答帧 ---
                    // 这里的 master_id 其实是电机的真实 ID
                    motor.id = master_id;            
                } else if (mode == 0x02) {
                    // --- 场景 B: 标准运控/使能反馈帧 ---
                    motor_status_t s = decode_motor_feedback(msg.identifier, msg.data);
                    motor.id = s.id;
                    motor.mode_num = s.mode_num; // 建议用数字存，方便判断
                    motor.pos = s.pos;
                    
                    // 修正你的故障检查逻辑：必须先全部设为 0，发现任一故障再设为 1
                    motor.fault = 0;
                    for (int i = 0; i < 6; i++) {
                        if (s.faults[i]) {
                            motor.fault = 1;
                            break; // 只要发现一个，立刻退出循环
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief 电机控制任务
 * 模拟 Python 驱动的使用流程
 */
void motor_control_task(void *pvParameters) {
    

    // // 1. 切换运行模式 
    // // 0: 运控模式, 1: 位置模式(PP), 2: 速度模式, 3: 电流模式, 5: 位置模式(CSP)
    // ESP_LOGI(TAG, "正在设置电机为运控模式...");
    // err = rs00_write_runmode(MOTOR_ID, 0);
    // if(err != ESP_OK){
    //     ESP_LOGE(TAG, "写入模式失败: %s", esp_err_to_name(err));
    //     goto exit_task; 
    // }
    // vTaskDelay(pdMS_TO_TICKS(100));

    // 2. 使能电机 
    
    rs00_enable_motor(MOTOR_ID);
    

    rs00_stop_motor(MOTOR_ID);

    rs00_get_device_id(MOTOR_ID);


//     float target_pos = 0.0f;
    
    while (1) {
//         /* * 3. 发送运控指令 (对应 Python 的 send_motion_control)
//          * 参数：ID, 目标位置, 目标速度, Kp, Kd, 力矩(tau)
//          */
//         esp_err_t err = rs00_send_motion_control(MOTOR_ID, target_pos, 0.0f, 10.0f, 1.0f, 0.0f);
        
//         if (err != ESP_OK) {
//             ESP_LOGE(TAG, "发送运动控制指令失败: %s", esp_err_to_name(err));
//         }

//         // 4. 这里的反馈解析建议放在一个独立的接收任务里，或者在这里调用接收函数
//         // 如果你还没写接收解析，可以先观察电机是否有转动反应

//         // 产生一个简单的正弦波形：在 -1.0 到 1.0 弧度之间往复
//         target_pos = sinf(xTaskGetTickCount() * 0.01f);

//         // 控制频率：20ms (50Hz)，运控模式通常建议 100Hz-500Hz
//         vTaskDelay(pdMS_TO_TICKS(20)); 
    }

// exit_task:
//     ESP_LOGE(TAG, "任务因错误终止");
//     vTaskDelete(NULL); 
}



void app_main(void) {
    ESP_LOGI(TAG, "------------------- 灵足电机驱动测试 ---------------------");

    // 1. 初始化 TWAI (CAN) 驱动
    ESP_LOGI(TAG, "正在初始化 TWAI 驱动 (TX:%d RX:%d)...", TX_GPIO_NUM, RX_GPIO_NUM);
    esp_err_t install_err = twai_driver_install(&g_config, &t_config, &f_config);
    if (install_err != ESP_OK) {
        ESP_LOGE(TAG, "驱动安装失败: %s", esp_err_to_name(install_err));
        return;
    }

    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "驱动启动失败");
        return;
    }
    ESP_LOGI(TAG, "TWAI 驱动已启动");

    // 2. 创建控制任务
    // 优先级设为 5，堆栈 4096 字节
    xTaskCreate(can_receive_task,"can_receive_task", 4096, NULL, 6, NULL);
    xTaskCreate(motor_control_task, "motor_ctrl_task", 4096, NULL, 5, NULL);
}