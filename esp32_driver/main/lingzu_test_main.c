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
                    
                    // 必须先全部设为 0，发现任一故障再设为 1
                    motor.fault = 0;
                    for (int i = 0; i < 6; i++) {
                        if (s.faults[i]) {
                            motor.fault = 1;
                            break; // 只要发现一个，立刻退出循环
                        }
                    }
                } else if (mode == 0x11){
                    // --- 场景 C: 读取参数应答帧 ---
                    // 提取应答标识符中的状态信息
                    uint8_t status = (master_id >> 8) & 0xFF;
                    
                    // 检查状态位的高4位是否有错误 
                    if (status != 0) {
                        ESP_LOGE(CAN_TAG, "参数读取失败!");
                        continue;
                    }
                    unpack_read_param_little(msg.data);
                    ESP_LOGI(CAN_TAG, "参数读取成功!");

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
    // rs00_stop_motor(MOTOR_ID);
    esp_err_t err;

/*----------------------------位置控制(PP)控制测试代码-----------------------------------*/
// 1. 切换至 PP 模式
    ESP_LOGI(TAG, "正在切换至 PP 位置模式...");
    err = rs00_write_runmode(MOTOR_ID, PP); // 假设 RS00_MODE_PP = 1
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "模式切换失败");
        vTaskDelete(NULL);
    }

    // 2. 使能电机
    rs00_enable_motor(MOTOR_ID);

    // 3. 规划两个测试点：0 弧度和 3.14 弧度 (180度)
    float positions[] = {0.0f, 3.14f};
    int p_idx = 0;

    ESP_LOGI(TAG, "开始 PP 模式点对点测试...");

    while (1) {
        if (motor.fault) {
            ESP_LOGE(TAG, "检测到故障，退出测试");
            break;
        }

        float target = positions[p_idx];
        ESP_LOGI(TAG, "前往目标点: %.2f", target);

        // 4. 发送 PP 指令
        // 参数：ID, 目标位置, 最大速度(5rad/s), 加速度(10rad/s^2)
        rs00_send_loc_control_pp(MOTOR_ID, target, 5.0f, 10.0f);

        // 5. 等待电机到达目标点 (简单的阈值判断)
        int timeout = 500; // 5秒超时
        while (fabsf(motor.pos - target) > 0.5f && timeout--) {
            if (timeout % 50 == 0) { // 每 500ms 打印一次
                ESP_LOGI(TAG, "正在移动... 当前: %.3f, 目标: %.3f, 差值: %.3f", 
                motor.pos, target, fabsf(motor.pos - target));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (timeout <= 0) {
            ESP_LOGW(TAG, "到达目标点超时，可能是限速太慢或被卡住");
        } else {
            ESP_LOGI(TAG, "已到达！当前位置: %.2f", motor.pos);
        }

        // 停顿 2 秒后前往下一个点
        vTaskDelay(pdMS_TO_TICKS(2000));
        p_idx = (p_idx + 1) % 2; 
    }

    rs00_stop_motor(MOTOR_ID);
    vTaskDelete(NULL);

// /*----------------------------位置控制(CSP)控制测试代码-----------------------------------*/
//     float start_pos = 0.0f;
//     float target_pos = 0.0f;

//     // 1. 切换至位置模式 (CSP)
//     ESP_LOGI(TAG, "正在切换至 CSP 位置模式...");
//     err = rs00_write_runmode(MOTOR_ID, CSP);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "模式切换失败");
//         vTaskDelete(NULL);
//     }

//     // 2. 使能电机
//     err = rs00_enable_motor(MOTOR_ID);
//     if (err != ESP_OK) {
//         vTaskDelete(NULL);
//     }

//     // 3. 关键步骤：获取当前位置作为起始点，避免由于目标位置突变导致的冲击
//     // 假设接收任务已经解析了 motor.pos
//     start_pos = motor.pos;
//     ESP_LOGI(TAG, "当前位置: %.2f, 开始 1 弧度范围内的正弦路径追踪...", start_pos);

//     int test_cycles = 10; // 测试约 20 秒 (20ms * 1000)
//     while (test_cycles--) {
//         // 安全检查：故障监测
//         if (motor.fault) {
//             ESP_LOGE(TAG, "监测到电机故障，任务终止！");
//             break; 
//         }

//         // 计算目标位置：以起始点为中心，进行幅度为 1.0 rad 的正弦运动
//         // xTaskGetTickCount() * 0.01f 控制运动周期
//         target_pos = start_pos + 1.0f * sinf(xTaskGetTickCount() * 0.01f);

//         // 4. 发送 CSP 指令
//         // 参数：ID, 速度限幅(5.0 rad/s), 目标位置
//         rs00_send_loc_control_csp(MOTOR_ID, 5.0f, target_pos);

//         // 每 100ms 打印一次反馈，观察追踪情况
//         if (test_cycles % 5 == 0) {
//             ESP_LOGI(TAG, "目标: %.2f | 实际: %.2f | 误差: %.2f", 
//                      target_pos, motor.pos, target_pos - motor.pos);
//         }

//         vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz 更新频率
//     }

//     // 5. 停止电机
//     ESP_LOGI(TAG, "CSP 测试完成，停止电机。");
//     rs00_stop_motor(MOTOR_ID);
    
//     vTaskDelete(NULL);    

// /*----------------------------电流控制测试代码-----------------------------------*/
//     float target_current = 0.0f;
//     bool increasing = true;

//     // 1. 切换至电流模式 (必须在使能前完成)
//     ESP_LOGI(TAG, "正在切换至电流模式...");
//     err = rs00_write_runmode(MOTOR_ID, CURRENT);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "模式切换失败");
//         vTaskDelete(NULL);
//     }

//     // 2. 使能电机
//     err = rs00_enable_motor(MOTOR_ID);
//     if (err != ESP_OK) {
//         vTaskDelete(NULL);
//     }

//     ESP_LOGI(TAG, "开始电流测试：电流将从 0A 缓慢变动至 1.0A...");

//     int test_cycles = 10; // 测试约 20 秒
//     while (test_cycles--) {
//         // 安全检查：故障监测
//             if (motor.fault) {
//             ESP_LOGE(TAG, "监测到电机故障，紧急停止！");
//             break; 
//         }

//         // 安全检查：空载限速 (假设电机反馈速度在 motor.vel)
//         // 注意：电流模式空载极易飞车，建议手动阻碍转动或设置软限速
//         /*
//         if (fabsf(motor.vel) > 20.0f) { 
//             ESP_LOGW(TAG, "速度过高，电流置零保护！");
//             target_current = 0.0f;
//         } else {
//             // 简单的三角波电流测试：0A -> 1.0A -> 0A
//             if (increasing) {
//                 target_current += 0.01f;
//                 if (target_current >= 1.0f) increasing = false;
//             } else {
//                 target_current -= 0.01f;
//                 if (target_current <= 0.0f) increasing = true;
//             }
//         }
//         */
        
//         // 为了演示安全，这里使用正弦微小电流测试 (-0.5A ~ 0.5A)
//         // target_current = 0.5f * sinf(xTaskGetTickCount() * 0.01f);

//         // 3. 发送电流指令
//         // rs00_send_current_control(MOTOR_ID, 1);

//         // 每 100ms 打印一次状态
//         // if (test_cycles % 5 == 0) {
//         //     ESP_LOGI(TAG, "目标电流: %.2f A", target_current);
//         // }

//         // vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz 频率
//     }

//     // 4. 停止电机
//     ESP_LOGI(TAG, "电流测试完成，停止电机。");
//     rs00_stop_motor(MOTOR_ID);
    
//     vTaskDelete(NULL);


// /*----------------------------速度控制测试代码-----------------------------------*/
//     // 1. 切换到速度模式 (必须在使能前完成)
//     ESP_LOGI(TAG, "正在切换至速度模式...");
//     err = rs00_write_runmode(MOTOR_ID, SPEED);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "模式切换失败，任务退出");
//         vTaskDelete(NULL);
//     }

//     // 2. 使能电机
//     err = rs00_enable_motor(MOTOR_ID);
//     if (err != ESP_OK) {
//         vTaskDelete(NULL);
//     }

//     float target_speed = 0.0f;
//     int test_duration = 500; // 测试 500 次，约 10 秒 (20ms * 500)

//     ESP_LOGI(TAG, "开始速度控制测试...");

//     while (test_duration--) {
//         // 安全检查：如果接收任务检测到故障，立即退出
//         if (motor.fault) {
//             ESP_LOGE(TAG, "检测到电机故障，紧急停止！");
//             break; 
//         }

//         // 产生一个正弦速度曲线：在 -5.0 到 5.0 rad/s 之间波动
//         // 使用 sinf 产生变化的速度，0.01f 是变化频率
//         target_speed = 5.0f * sinf(xTaskGetTickCount() * 0.01f);

//         // 发送速度控制指令
//         // 参数：ID, 电流限幅(2A), 目标速度, 加速度(20rad/s^2)
//         rs00_send_speed_control(MOTOR_ID, 2.0f, target_speed, 20.0f);

//         // 打印当前实时速度（假设接收任务在更新 motor.vel）
//         // ESP_LOGD(TAG, "设定速度: %.2f, 实际速度: %.2f", target_speed, motor.vel);

//         vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz 控制频率
//     }

//     // 3. 测试结束，停止电机
//     ESP_LOGI(TAG, "速度测试完成，正在停止电机...");
//     rs00_stop_motor(MOTOR_ID);
    
//     vTaskDelete(NULL);

    


/*-----------------------------运控测试代码----------------------------------*/
    // float target_pos = 0.0f;
    
    // while (1) {

    //     if (motor.fault) {
    //         ESP_LOGE(TAG, "检测到电机故障！");
    //         break; 
    //     }

    //     rs00_send_motion_control(MOTOR_ID, target_pos, 0.0f, 10.0f, 1.0f, 0.0f);//发送运动控制指令
        

    //     // 产生一个简单的正弦波形：在 -1.0 到 1.0 弧度之间往复
    //     target_pos = sinf(xTaskGetTickCount() * 0.01f);

    //     // 控制频率：20ms (50Hz)，运控模式通常建议 100Hz-500Hz
    //     vTaskDelay(pdMS_TO_TICKS(20)); 
    // }

    // rs00_stop_motor(MOTOR_ID);

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