/**
 * @file baudrate_test.h
 * @brief CAN波特率自动检测工具
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动波特率自动检测
 * @note 会自动测试常见波特率: 1M, 800K, 500K, 250K, 125K
 */
void start_baudrate_auto_detect(void);

#ifdef __cplusplus
}
#endif
