/**
 * ESP32-S3 + TJA1050 CAN 收发示例（Arduino 版）
 *
 * 通过宏控制模式（不依赖 menuconfig）：
 *   EXAMPLE_ENABLE_TX = 1 启用发送
 *   EXAMPLE_ENABLE_RX = 1 启用接收
 *   两者均为 1 时同时启用收发
 *
 * 默认参数：
 *   波特率：1 Mbps
 *   TX GPIO：4（按实际接线修改）
 *   RX GPIO：5（按实际接线修改）
 */

#include <Arduino.h>
#include "driver/twai.h"

/* ==================== 配置宏 ==================== */
// 引脚配置（请按实际接线修改）
constexpr gpio_num_t TX_GPIO = static_cast<gpio_num_t>(4);
constexpr gpio_num_t RX_GPIO = static_cast<gpio_num_t>(5);

// 模式开关：直接修改为 0/1
#define EXAMPLE_ENABLE_TX 1  // 1: 启用发送
#define EXAMPLE_ENABLE_RX 1  // 1: 启用接收

// 基本参数
constexpr int TX_TASK_PRIO = 8;
constexpr int RX_TASK_PRIO = 8;
constexpr uint32_t MSG_ID = 0x123;      // 标准帧 11 位 ID
constexpr uint32_t TX_INTERVAL_MS = 1;  // 发送间隔 1ms
constexpr uint32_t RX_TIMEOUT_MS = 1000;
constexpr const char *EXAMPLE_TAG = "TWAI CAN";

// TWAI 配置
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO, RX_GPIO, TWAI_MODE_NORMAL);

/* ==================== 工具函数 ==================== */
static inline void check_err(esp_err_t err, const char *msg) {
  if (err != ESP_OK) {
    Serial.printf("%s failed: %s\n", msg, esp_err_to_name(err));
    while (true) {
      delay(1000);
    }
  }
}

/* ==================== 发送功能 ==================== */
static void fill_test_frame(twai_message_t *tx_msg) {
  // 固定帧便于检查：01 02 03 04 05 06 07 08
  tx_msg->data[0] = 0x01;
  tx_msg->data[1] = 0x02;
  tx_msg->data[2] = 0x03;
  tx_msg->data[3] = 0x04;
  tx_msg->data[4] = 0x05;
  tx_msg->data[5] = 0x06;
  tx_msg->data[6] = 0x07;
  tx_msg->data[7] = 0x08;
}

static void twai_tx_task(void *arg) {
  twai_message_t tx_msg{};
  uint32_t send_count = 0;

  tx_msg.identifier = MSG_ID;
  tx_msg.data_length_code = 8;
  tx_msg.flags = TWAI_MSG_FLAG_NONE;
  tx_msg.self = 0;
  fill_test_frame(&tx_msg);

  Serial.printf("[%s][TX] start, ID: 0x%03lX, interval: %lu ms\n", EXAMPLE_TAG,
                (unsigned long)MSG_ID, (unsigned long)TX_INTERVAL_MS);

  while (true) {
    // 让帧内容随发送计数变化，便于在分析仪观察变化
    tx_msg.data[0] = send_count & 0xFF;
    tx_msg.data[1] = (send_count >> 8) & 0xFF;
    tx_msg.data[2] = (send_count >> 16) & 0xFF;
    tx_msg.data[3] = (send_count >> 24) & 0xFF;
    // 保持后 4 字节固定标记
    tx_msg.data[4] = 0xAA;
    tx_msg.data[5] = 0xBB;
    tx_msg.data[6] = 0xCC;
    tx_msg.data[7] = 0xDD;

    esp_err_t ret = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
      send_count++;
      if (send_count % 1000 == 0) {
        Serial.printf("[%s][TX] sent %lu frames\n", EXAMPLE_TAG,
                      (unsigned long)send_count);
      }
    } else {
      Serial.printf("[%s][TX] send failed: %s (count %lu)\n", EXAMPLE_TAG,
                    esp_err_to_name(ret), (unsigned long)send_count);
    }
    vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
  }
}

/* ==================== 接收功能 ==================== */
static void print_rx_message(const twai_message_t *rx_msg, uint32_t idx) {
  const char *frame_type =
      (rx_msg->flags & TWAI_MSG_FLAG_EXTD) ? "EXT" : "STD";
  const char *frame_kind =
      (rx_msg->flags & TWAI_MSG_FLAG_RTR) ? "RTR" : "DATA";

  Serial.printf("[%s][RX] === Msg #%lu ===\n", EXAMPLE_TAG,
                (unsigned long)idx);
  Serial.printf("[%s][RX] Type: %s %s\n", EXAMPLE_TAG, frame_type, frame_kind);
  Serial.printf("[%s][RX] ID: 0x%08lX\n", EXAMPLE_TAG,
                (unsigned long)rx_msg->identifier);
  Serial.printf("[%s][RX] DLC: %d\n", EXAMPLE_TAG, rx_msg->data_length_code);

  Serial.printf("[%s][RX] Data bytes:\n", EXAMPLE_TAG);
  for (int i = 0; i < rx_msg->data_length_code; ++i) {
    Serial.printf("  [%d] = 0x%02X (%3d)\n", i, rx_msg->data[i],
                  rx_msg->data[i]);
  }

  Serial.printf("[%s][RX] Hex: ", EXAMPLE_TAG);
  for (int i = 0; i < rx_msg->data_length_code; ++i) {
    Serial.printf("%02X ", rx_msg->data[i]);
  }
  Serial.println();
}

static void twai_rx_task(void *arg) {
  twai_message_t rx_msg{};
  uint32_t recv_count = 0;
  uint32_t err_count = 0;

  Serial.printf("[%s][RX] start, baud: 1Mbps, timeout: %lu ms\n", EXAMPLE_TAG,
                (unsigned long)RX_TIMEOUT_MS);

  while (true) {
    esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(RX_TIMEOUT_MS));
    if (ret == ESP_OK) {
      recv_count++;
      print_rx_message(&rx_msg, recv_count);
    } else if (ret == ESP_ERR_TIMEOUT) {
      // 正常超时不打印
    } else {
      err_count++;
      Serial.printf("[%s][RX] recv error: %s (err_count %lu)\n", EXAMPLE_TAG,
                    esp_err_to_name(ret), (unsigned long)err_count);
    }
  }
}

/* ==================== Arduino 入口 ==================== */
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println();
  Serial.printf("[%s] setup begin\n", EXAMPLE_TAG);
  Serial.printf("[%s] Mode: TX:%d RX:%d\n", EXAMPLE_TAG, EXAMPLE_ENABLE_TX,
                EXAMPLE_ENABLE_RX);
  Serial.printf("[%s] TX GPIO: %d, RX GPIO: %d\n", EXAMPLE_TAG, (int)TX_GPIO,
                (int)RX_GPIO);

  check_err(twai_driver_install(&g_config, &t_config, &f_config),
            "twai_driver_install");
  check_err(twai_start(), "twai_start");
  Serial.printf("[%s] TWAI started\n", EXAMPLE_TAG);

#if EXAMPLE_ENABLE_TX
  xTaskCreatePinnedToCore(twai_tx_task, "TWAI_tx", 4096, nullptr, TX_TASK_PRIO,
                          nullptr, tskNO_AFFINITY);
#endif

#if EXAMPLE_ENABLE_RX
  xTaskCreatePinnedToCore(twai_rx_task, "TWAI_rx", 4096, nullptr, RX_TASK_PRIO,
                          nullptr, tskNO_AFFINITY);
#endif
}

void loop() {
  // 任务在 FreeRTOS 中运行，这里保持空闲即可
  delay(1000);
}
