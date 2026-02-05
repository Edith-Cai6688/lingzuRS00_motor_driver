#ifndef RS00_DRIVER_H
#define RS00_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define MASTER_ID_DEFAULT 0xFD

/* 运行模式定义 (对应 Python runmode 字典) */
typedef enum {
    MOTION  = 0,
    PP      = 1,
    SPEED   = 2,
    CURRENT = 3,
    CSP     = 4
} rs00_runmode_t;

typedef struct{
    uint8_t id;
    uint8_t mode_num; //0:复位 1:标定 2:运行
    uint8_t fault; //0:无故障 1:有故障
    uint8_t runmode; 
    uint16_t index;
    uint32_t data;
    float pos;
} motor_status;

typedef struct {
    int mode_val;
    const char* mode_name;
} mode_map_t;


extern volatile motor_status motor;


/* 基础指令 */
esp_err_t rs00_enable_motor(uint8_t motor_id);
esp_err_t rs00_stop_motor(uint8_t motor_id);
esp_err_t rs00_set_zero(uint8_t motor_id);
esp_err_t rs00_get_device_id(); 
esp_err_t rs00_set_can_id(uint8_t old_id, uint8_t new_id);

/* 参数读写 */
esp_err_t rs00_write_parameter(uint8_t motor_id, uint16_t index, float value);
esp_err_t rs00_save_parameter(uint8_t motor_id);
esp_err_t rs00_write_runmode(uint8_t motor_id, rs00_runmode_t mode);
esp_err_t rs00_read_runmode(uint8_t motor_id);

/* 控制指令 */
// 运控模式
esp_err_t rs00_send_motion_control(uint8_t motor_id, float pos, float vel, float kp, float kd, float tau);
// 速度模式
esp_err_t rs00_send_speed_control(uint8_t motor_id, float limit_cur, float speed_ref, float acc_rad);
// 电流模式
esp_err_t rs00_send_current_control(uint8_t motor_id, float iq_ref);
// 位置模式 CSP
esp_err_t rs00_send_loc_control_csp(uint8_t motor_id, float limit_spd, float loc_ref);
// 位置模式 PP
esp_err_t rs00_send_loc_control_pp(uint8_t motor_id, float loc_ref, float vel_max, float acc_set);

#endif