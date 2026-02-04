/**
 * 串口输出诊断（ESP32-S3）
 *
 * 目的：同时尝试往 USB-CDC (Serial) 和 UART0 (Serial0) 打印，
 * 用来确认你应该在 Arduino IDE 里打开哪个 COM 口来观察日志。
 *
 * 使用：
 * - 烧录后，分别打开每一个可能的 COM 口的串口监视器（115200），看哪个口有输出。
 */

#include <Arduino.h>

static void print_all(const char *s) {
  // Serial: 可能是 USB-CDC 或 UART0（取决于 Tools -> USB CDC on Boot 等设置）
  Serial.println(s);

  // Serial0: 通常是 UART0（TX/RX 引脚输出），部分板子接到 USB-UART 芯片
  Serial0.println(s);
}

void setup() {
  // 尽量都初始化
  Serial.begin(115200);
  Serial0.begin(115200);

  delay(1500);
  print_all("===== serial_diag start =====");

  // 打印一些关键信息
  Serial.printf("Serial baud=115200, millis=%lu\n", (unsigned long)millis());
  Serial0.printf("Serial0 baud=115200, millis=%lu\n", (unsigned long)millis());

  print_all("If you can see this line, this COM port is the correct monitor port.");
}

void loop() {
  static uint32_t n = 0;
  char buf[64];
  snprintf(buf, sizeof(buf), "tick %lu", (unsigned long)n++);
  print_all(buf);
  delay(1000);
}

