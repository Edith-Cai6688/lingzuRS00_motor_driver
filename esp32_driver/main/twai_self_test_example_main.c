/*
 * 此示例演示如何使用 ESP32-S3 通过 TJA1050 CAN 收发器与 CAN 分析仪通信。
 * 通过代码中的条件宏选择模式：
 *   EXAMPLE_ENABLE_TX = 1 仅启用发送
 *   EXAMPLE_ENABLE_RX = 1 仅启用接收
 *   两者均为 1 时同时启用收发
 *
 * ⚠ 注意（非常重要）：
 * - 大然电机库 `daran_motor_can.c` 内部是“同步收包”风格（调用 get_state/read_property 会自己 twai_receive）。
 * - 如果你同时开启了本文件的 `twai_receive_task`（通用 RX 任务），它可能会把电机返回帧先收走，
 *   导致库侧 READ_FLAG 超时，表现为“电机抽一下/不动/读不到状态”。
 * - 所以跑电机控制示例时，建议把 `EXAMPLE_ENABLE_RX` 保持为 0。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "daran_motor_example.h"

/* --------------------- 定义和静态变量 ------------------ */

//示例配置
#define TX_GPIO_NUM             CONFIG_EXAMPLE_TX_GPIO_NUM
#define RX_GPIO_NUM             CONFIG_EXAMPLE_RX_GPIO_NUM
#define TX_TASK_PRIO            8       //发送任务优先级
#define RX_TASK_PRIO            8       //接收任务优先级
#define MSG_ID                  0x123   //CAN 消息 ID（11 位标准格式）
#define TX_INTERVAL_MS          1       //发送间隔（毫秒）
#define RX_TIMEOUT_MS           1000    //接收超时时间（毫秒）
#define EXAMPLE_TAG             "TWAI CAN"

//运行模式（直接在此修改，简化配置）
#define EXAMPLE_ENABLE_TX   0   //1: 启用发送  0: 禁用发送
#define EXAMPLE_ENABLE_RX   0   //1: 启用接收  0: 禁用接收
// 大然电机示例开关：1=启动示例任务（需要总线上有对应电机）
#define EXAMPLE_ENABLE_DARAN_MOTOR  1
#if EXAMPLE_ENABLE_TX && EXAMPLE_ENABLE_RX
#define EXAMPLE_MODE_STR    "BOTH 收发同时"
#elif EXAMPLE_ENABLE_TX
#define EXAMPLE_MODE_STR    "TX 仅发送"
#elif EXAMPLE_ENABLE_RX
#define EXAMPLE_MODE_STR    "RX 仅接收"
#else
#define EXAMPLE_MODE_STR    "禁用收发"
#endif

//CAN 波特率配置：1Mbps（大然电机默认波特率）
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

//接受所有消息（接收来自 CAN 分析仪的所有消息）
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

//正常模式（需要 CAN 总线上的其他节点提供应答）
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

/* --------------------------- 任务和函数 -------------------------- */

/* ==================== 发送功能（原有代码） ==================== */

/**
 * @brief 填充固定的测试帧数据（便于检查）
 * @param tx_msg 指向要填充的 CAN 消息结构体的指针
 */
static void fill_test_frame(twai_message_t *tx_msg)
{
    //固定帧格式：便于在 CAN 分析仪中检查
    //数据格式：0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08
    tx_msg->data[0] = 0x01;
    tx_msg->data[1] = 0x02;
    tx_msg->data[2] = 0x03;
    tx_msg->data[3] = 0x04;
    tx_msg->data[4] = 0x05;
    tx_msg->data[5] = 0x06;
    tx_msg->data[6] = 0x07;
    tx_msg->data[7] = 0x08;
}

/**
 * @brief 测试函数：发送固定格式的 CAN 消息
 * @param arg 未使用的参数
 */
static void twai_test_task(void *arg)
{
    twai_message_t tx_msg;
    uint32_t send_count = 0;

    //初始化消息结构
    tx_msg.identifier = MSG_ID;
    tx_msg.data_length_code = 8;  //8 字节数据
    tx_msg.flags = TWAI_MSG_FLAG_NONE;
    tx_msg.self = 0;  //不使用自接收请求

    //填充固定测试数据
    fill_test_frame(&tx_msg);

    ESP_LOGI(EXAMPLE_TAG, "[TX] 开始发送 CAN 测试消息...");
    ESP_LOGI(EXAMPLE_TAG, "[TX] 消息 ID: 0x%03X, 波特率: 1000Kbps (1Mbps)", MSG_ID);
    ESP_LOGI(EXAMPLE_TAG, "[TX] 发送间隔: %d ms", TX_INTERVAL_MS);
    ESP_LOGI(EXAMPLE_TAG, "[TX] 固定数据帧: %02X %02X %02X %02X %02X %02X %02X %02X",
             tx_msg.data[0], tx_msg.data[1], tx_msg.data[2], tx_msg.data[3],
             tx_msg.data[4], tx_msg.data[5], tx_msg.data[6], tx_msg.data[7]);

    while (1) {
        //发送固定格式的消息
        esp_err_t ret = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
        if (ret == ESP_OK) {
            send_count++;
            //每发送1000条消息打印一次，避免日志过多
            if (send_count % 1000 == 0) {
                ESP_LOGI(EXAMPLE_TAG, "[TX] 已发送 %lu 条消息", send_count);
            }
        } else {
            ESP_LOGE(EXAMPLE_TAG, "[TX] 发送失败: %s (已发送: %lu)", esp_err_to_name(ret), send_count);
        }

        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
        }
    }

/* ==================== 接收功能（新增代码） ==================== */

/**
 * @brief 打印接收到的 CAN 消息内容
 * @param rx_msg 指向接收到的 CAN 消息结构体的指针
 * @param msg_count 消息计数
 */
static void print_received_message(const twai_message_t *rx_msg, uint32_t msg_count)
{
    //判断消息格式（标准帧或扩展帧）
    const char *frame_type = (rx_msg->flags & TWAI_MSG_FLAG_EXTD) ? "扩展帧" : "标准帧";
    
    //判断是否为远程帧
    const char *frame_kind = (rx_msg->flags & TWAI_MSG_FLAG_RTR) ? "远程帧" : "数据帧";
    
    ESP_LOGI(EXAMPLE_TAG, "[RX] ========== 消息 #%lu ==========", msg_count);
    ESP_LOGI(EXAMPLE_TAG, "[RX] 类型: %s %s", frame_type, frame_kind);
    ESP_LOGI(EXAMPLE_TAG, "[RX] ID: 0x%08lX", rx_msg->identifier);
    ESP_LOGI(EXAMPLE_TAG, "[RX] 数据长度: %d 字节", rx_msg->data_length_code);
    
    //打印数据内容
    ESP_LOGI(EXAMPLE_TAG, "[RX] 数据: ");
    for (int i = 0; i < rx_msg->data_length_code; i++) {
        ESP_LOGI(EXAMPLE_TAG, "[RX]   [%d] = 0x%02X (%3d)", i, rx_msg->data[i], rx_msg->data[i]);
        }
    
    //以十六进制格式打印所有数据（一行）
    char data_str[64] = {0};
    for (int i = 0; i < rx_msg->data_length_code; i++) {
        char temp[4];
        snprintf(temp, sizeof(temp), "%02X ", rx_msg->data[i]);
        strcat(data_str, temp);
    }
    ESP_LOGI(EXAMPLE_TAG, "[RX] 十六进制: %s", data_str);
    ESP_LOGI(EXAMPLE_TAG, "[RX] ==============================");
}

/**
 * @brief 接收任务：持续接收来自 CAN 分析仪的消息
 * @param arg 未使用的参数
 */
static void twai_receive_task(void *arg)
{
    twai_message_t rx_msg;
    uint32_t receive_count = 0;
    uint32_t error_count = 0;

    ESP_LOGI(EXAMPLE_TAG, "[RX] 开始接收 CAN 消息...");
    ESP_LOGI(EXAMPLE_TAG, "[RX] 波特率: 1000Kbps (1Mbps)");
    ESP_LOGI(EXAMPLE_TAG, "[RX] 等待来自 CAN 分析仪的消息...");

    while (1) {
        //接收消息
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(RX_TIMEOUT_MS));
        
        if (ret == ESP_OK) {
            receive_count++;
            //打印接收到的消息
            print_received_message(&rx_msg, receive_count);
        } else if (ret == ESP_ERR_TIMEOUT) {
            //超时是正常的，不打印错误
            //可以在这里添加超时提示（可选）
        } else {
            error_count++;
            ESP_LOGE(EXAMPLE_TAG, "[RX] 接收错误: %s (错误计数: %lu)", esp_err_to_name(ret), error_count);
        }
    }
}

void app_main_backup(void)
{
    ESP_LOGI(EXAMPLE_TAG, "TWAI CAN 通信示例 - 模式: %s", EXAMPLE_MODE_STR);
    ESP_LOGI(EXAMPLE_TAG, "TX GPIO: %d, RX GPIO: %d", TX_GPIO_NUM, RX_GPIO_NUM);
    ESP_LOGI(EXAMPLE_TAG, "CAN波特率: 1Mbps");

    //安装 TWAI 驱动
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "TWAI 驱动已安装");

    //启动 TWAI 驱动
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "TWAI 驱动已启动");

    // ========== 功能选择（由 menuconfig 控制） ==========
#if EXAMPLE_ENABLE_TX
    xTaskCreatePinnedToCore(twai_test_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
#endif

#if EXAMPLE_ENABLE_RX
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
#endif

#if EXAMPLE_ENABLE_DARAN_MOTOR
    // 启动大然电机示例（示例会创建自己的收/控任务）
    daran_motor_example_start();
#endif

    // 按 Ctrl+C 停止程序
}
