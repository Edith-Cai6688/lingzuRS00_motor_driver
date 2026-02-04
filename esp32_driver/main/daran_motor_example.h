/**
 * @file daran_motor_example.h
 * @brief 大然电机示例程序头文件
 */

#ifndef DARAN_MOTOR_EXAMPLE_H
#define DARAN_MOTOR_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动大然电机示例
 * @note 需要在TWAI驱动已安装并启动后调用
 */
void daran_motor_example_start(void);

#ifdef __cplusplus
}
#endif

#endif /* DARAN_MOTOR_EXAMPLE_H */
