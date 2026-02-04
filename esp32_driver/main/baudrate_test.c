/**
 * @file baudrate_test.c
 * @brief 自动测试多个CAN波特率，找到电机实际波特率
 *
 * @note 该工具会反复 stop/uninstall TWAI 并用不同 timing_config 重新 install/start。
 *       运行时请不要同时跑电机控制示例/其他收发任务，避免互相干扰。
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "daran_motor_can.h"

#define TEST_TAG "BAUDRATE_TEST"
#define TEST_MOTOR_ID 2
#define TX_GPIO 4
#define RX_GPIO 5

// 要测试的波特率列表
typedef struct {
    const char *name;
    twai_timing_config_t config;
} baudrate_test_t;

static const baudrate_test_t baudrate_list[] = {
    {"1 Mbps",   TWAI_TIMING_CONFIG_1MBITS()},
    {"800 Kbps", TWAI_TIMING_CONFIG_800KBITS()},
    {"500 Kbps", TWAI_TIMING_CONFIG_500KBITS()},
    {"250 Kbps", TWAI_TIMING_CONFIG_250KBITS()},
    {"125 Kbps", TWAI_TIMING_CONFIG_125KBITS()},
};

static const int baudrate_count = sizeof(baudrate_list) / sizeof(baudrate_list[0]);

/**
 * @brief 测试指定波特率是否能与电机通信
 * @return true: 通信成功, false: 通信失败
 *
 * @note 判定方式：调用 dump_error() / get_state() 是否能收到回包（READ_FLAG==1）。
 */
static bool test_baudrate(const twai_timing_config_t *timing_config, const char *name)
{
    ESP_LOGI(TEST_TAG, "");
    ESP_LOGI(TEST_TAG, "========================================");
    ESP_LOGI(TEST_TAG, "测试波特率: %s", name);
    ESP_LOGI(TEST_TAG, "========================================");

    // 停止并卸载当前驱动
    twai_stop();
    twai_driver_uninstall();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 重新安装驱动
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO, RX_GPIO, TWAI_MODE_NORMAL);
    
    if (twai_driver_install(&g_config, timing_config, &f_config) != ESP_OK) {
        ESP_LOGE(TEST_TAG, "驱动安装失败");
        return false;
    }
    
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TEST_TAG, "驱动启动失败");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    // 说明：DrEmpower_can 风格库不需要额外 init（TWAI 已在上面安装并 start）

    // 尝试读取电机状态（发送3次）
    bool success = false;
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TEST_TAG, "尝试 %d/3: 读取电机ID=%d状态...", i+1, TEST_MOTOR_ID);
        
        int8_t result = dump_error(TEST_MOTOR_ID);
        
        if (result >= 0) {
            ESP_LOGI(TEST_TAG, "✅ 成功! 电机响应正常");
            success = true;
            
            // 读取详细状态
            struct servo_state state = get_state(TEST_MOTOR_ID);
            ESP_LOGI(TEST_TAG, "电机状态: 位置=%.2f°, 速度=%.2f r/min", 
                     state.angle, state.speed);
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    if (!success) {
        ESP_LOGW(TEST_TAG, "❌ 失败: 电机无响应");
    }

    return success;
}

/**
 * @brief 波特率自动检测任务
 *
 * @note 找到可用波特率后会打印结果并退出任务。
 */
void baudrate_auto_detect_task(void *arg)
{
    ESP_LOGI(TEST_TAG, "");
    ESP_LOGI(TEST_TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TEST_TAG, "║   CAN 波特率自动检测工具               ║");
    ESP_LOGI(TEST_TAG, "╚════════════════════════════════════════╝");
    ESP_LOGI(TEST_TAG, "");
    ESP_LOGI(TEST_TAG, "目标电机ID: %d", TEST_MOTOR_ID);
    ESP_LOGI(TEST_TAG, "将依次测试 %d 个常见波特率...", baudrate_count);
    ESP_LOGI(TEST_TAG, "");

    vTaskDelay(pdMS_TO_TICKS(1000));

    bool found = false;
    const char *working_baudrate = NULL;

    // 依次测试每个波特率
    for (int i = 0; i < baudrate_count; i++) {
        if (test_baudrate(&baudrate_list[i].config, baudrate_list[i].name)) {
            found = true;
            working_baudrate = baudrate_list[i].name;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TEST_TAG, "");
    ESP_LOGI(TEST_TAG, "========================================");
    ESP_LOGI(TEST_TAG, "检测结果");
    ESP_LOGI(TEST_TAG, "========================================");

    if (found) {
        ESP_LOGI(TEST_TAG, "");
        ESP_LOGI(TEST_TAG, "🎉 成功找到工作波特率!");
        ESP_LOGI(TEST_TAG, "");
        ESP_LOGI(TEST_TAG, "  电机ID: %d", TEST_MOTOR_ID);
        ESP_LOGI(TEST_TAG, "  波特率: %s", working_baudrate);
        ESP_LOGI(TEST_TAG, "");
        ESP_LOGI(TEST_TAG, "请在代码中修改波特率配置:");
        ESP_LOGI(TEST_TAG, "  static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_XXX();");
        ESP_LOGI(TEST_TAG, "");
    } else {
        ESP_LOGE(TEST_TAG, "");
        ESP_LOGE(TEST_TAG, "❌ 未找到工作波特率!");
        ESP_LOGE(TEST_TAG, "");
        ESP_LOGE(TEST_TAG, "可能的原因:");
        ESP_LOGE(TEST_TAG, "1. 电机未上电");
        ESP_LOGE(TEST_TAG, "2. 电机ID不是%d", TEST_MOTOR_ID);
        ESP_LOGE(TEST_TAG, "3. CAN线连接错误");
        ESP_LOGE(TEST_TAG, "4. TJA1050未正常工作");
        ESP_LOGE(TEST_TAG, "5. 终端电阻配置错误");
        ESP_LOGE(TEST_TAG, "");
        ESP_LOGE(TEST_TAG, "建议:");
        ESP_LOGE(TEST_TAG, "1. 检查硬件连接");
        ESP_LOGE(TEST_TAG, "2. 用CAN分析仪监听总线，查看是否有数据");
        ESP_LOGE(TEST_TAG, "3. 修改 TEST_MOTOR_ID 尝试其他ID");
        ESP_LOGE(TEST_TAG, "");
    }

    ESP_LOGI(TEST_TAG, "========================================");
    ESP_LOGI(TEST_TAG, "");

    vTaskDelete(NULL);
}

/**
 * @brief 启动波特率自动检测
 * @note 会创建一个 FreeRTOS 任务异步执行。
 */
void start_baudrate_auto_detect(void)
{
    xTaskCreatePinnedToCore(
        baudrate_auto_detect_task,
        "baudrate_test",
        8192,
        NULL,
        5,
        NULL,
        tskNO_AFFINITY
    );
}
