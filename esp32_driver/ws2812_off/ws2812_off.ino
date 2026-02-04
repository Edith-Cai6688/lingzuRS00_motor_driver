/**
 * 关闭板载 WS2812 LED 的示例。
 * 如需点亮，请修改 setPixelColor 为目标颜色并调用 show()。
 *
 * 请根据你的开发板实际 WS2812 数据引脚修改 WS2812_PIN：
 * - 常见开发板：GPIO 48（ESP32-S3-DevKitC-1 一些版本）
 * - 若无效，请查阅原理图或示例手册填写正确引脚。
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ==== 根据实际接线修改此引脚号 ====
constexpr uint8_t WS2812_PIN = 48;  // TODO: 如不同，请改成实际的 WS2812 数据引脚
// 板载一般只有 1 颗
constexpr uint16_t WS2812_COUNT = 1;

// NeoPixel 对象
Adafruit_NeoPixel pixels(WS2812_COUNT, WS2812_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();
  pixels.clear();        // 所有像素清零（黑）
  pixels.setBrightness(0); // 可选：亮度设为 0，确保完全熄灭
  pixels.show();         // 发送数据，立即熄灭
}

void loop() {
  // 保持熄灭，无需操作
  delay(1000);
}
