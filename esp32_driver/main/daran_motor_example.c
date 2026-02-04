/**
 * @file daran_motor_example.c
 * @brief 大然电机示例程序 - ESP32-S3版本
 * 
 * @details 基于官方Arduino库转换，适配ESP32-S3 TWAI驱动
 * @note 电机ID: 2, CAN波特率: 1Mbps
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "daran_motor_can.h"

#define EXAMPLE_TAG "DARAN_EXAMPLE"

// 电机配置（必须与实际电机ID一致）
#define MOTOR_ID 2

// ===== 只用一个宏选择示例（一次只跑一个）=====
// 1: 读状态（不动电机）
// 2: 速度模式（转 3 秒，然后停）
// 3: 位置来回（0 -> +A -> 0 -> -A），带超时等待
#define DARAN_EXAMPLE_SELECT 2

// 是否循环执行（1=循环，0=只跑一次）
#define DARAN_EXAMPLE_LOOP 1

// ===== 小工具：读状态 / 等待到位（都带超时，避免卡死）=====
/**
 * @brief 读取电机当前角度/速度（非阻塞等待：由库内部 receive_data 的超时决定）
 *
 * @param id  电机ID（1~63），0 为广播（不建议在读状态时用 0）
 * @param out 输出结构体指针（angle/speed）
 * @return true 读取成功（READ_FLAG==1）
 * @return false 读取失败/超时（READ_FLAG!=1）
 *
 * @note 失败常见原因：电机ID不对、波特率不对、接线/终端电阻问题、总线上有其他任务“抢收包”。
 */
static bool read_state(uint8_t id, struct servo_state *out)
{
    if (out == NULL) return false;
    *out = get_state(id);
    return (READ_FLAG == 1);
}

/**
 * @brief 等待“位置到达”标志（traj_done），带超时，避免卡死
 *
 * @param id         电机ID
 * @param timeout_ms 超时时间（毫秒）
 * @return true  在超时内检测到 traj_done != 0
 * @return false 超时未检测到（可能是未回包/未到位/位置模式未生效）
 *
 * @note 该函数会周期性调用 read_property(32008, type=3)。
 */
static bool wait_traj_done(uint8_t id, uint32_t timeout_ms)
{
    const uint32_t step_ms = 50;
    for (uint32_t t = 0; t < timeout_ms; t += step_ms) {
        int done = (int)read_property(id, 32008, 3);
        if (READ_FLAG == 1 && done != 0) return true;
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
    return false;
}

/**
 * @brief 确保电机处于可控状态：清除错误 + 进入闭环控制
 *
 * @param id 电机ID
 *
 * @note 按官方库习惯：电机出错会退到 IDLE，此时 set_angle/set_speed/set_torque 往往会“抽一下”或无效。
 *       这里不做复杂重试/打印，保持示例简洁稳定。
 */
static void ensure_closed_loop(uint8_t id)
{
    // 最小化：清错 + 设闭环（不做复杂重试，避免“抽一下抽一下”时越改越乱）
    clear_error(id);
    vTaskDelay(pdMS_TO_TICKS(50));
    set_mode(id, 2);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// ===== 示例 1：只读状态 =====
/**
 * @brief 示例1：只读电机状态（不发送运动指令）
 *
 * @param id 电机ID
 */
static void run_example_read_only(uint8_t id)
{
    struct servo_state s;
    if (read_state(id, &s)) {
        ESP_LOGI(EXAMPLE_TAG, "state: angle=%.2f deg, speed=%.2f r/min", s.angle, s.speed);
    } else {
        ESP_LOGW(EXAMPLE_TAG, "state: no response");
    }
}

// ===== 示例 2：速度模式 =====
/**
 * @brief 示例2：速度模式运行一段时间然后停止
 *
 * @param id 电机ID
 *
 * @note 使用 set_speed(mode=1) 直通速度模式；如果你发现“只抖一下”，优先怀疑错误未清掉/未进闭环/收包超时。
 */
static void run_example_speed(uint8_t id)
{
    const float rpm = 60.0f;
    const uint32_t run_ms = 3000;

    ensure_closed_loop(id);

    // 用直通模式（mode=1），不带扭矩前馈，尽量“干净”
    set_speed(id, rpm, 0.0f, 1);
    vTaskDelay(pdMS_TO_TICKS(run_ms));
    set_speed(id, 0.0f, 0.0f, 1);

    // 只打印一次状态，避免日志太多
    run_example_read_only(id);
}

// ===== 示例 3：位置来回 =====
/**
 * @brief 示例3：绝对位置来回（0 -> +A -> 0 -> -A），每段都等待 traj_done（带超时）
 *
 * @param id 电机ID
 *
 * @note 如果你看到 timeout，说明：没回包 或者 位置模式未执行/未到位（例如负载太大、限位、出错等）。
 */
static void run_example_position(uint8_t id)
{
    const float A = 30.0f;      // 角度幅值（度）
    const float speed = 50.0f;  // r/min
    const float accel = 10.0f;  // (r/min)/s
    const uint32_t wait_ms = 8000;

    ensure_closed_loop(id);

    set_angle(id, 0.0f, speed, accel, 1);
    if (!wait_traj_done(id, wait_ms)) ESP_LOGW(EXAMPLE_TAG, "pos: wait timeout @0");

    set_angle(id, +A, speed, accel, 1);
    if (!wait_traj_done(id, wait_ms)) ESP_LOGW(EXAMPLE_TAG, "pos: wait timeout @+A");

    set_angle(id, 0.0f, speed, accel, 1);
    if (!wait_traj_done(id, wait_ms)) ESP_LOGW(EXAMPLE_TAG, "pos: wait timeout @0");

    set_angle(id, -A, speed, accel, 1);
    if (!wait_traj_done(id, wait_ms)) ESP_LOGW(EXAMPLE_TAG, "pos: wait timeout @-A");
}

/**
 * @brief 控制任务（电机控制示例）
 *
 * @param arg 未使用
 *
 * @note 该任务假设：TWAI 驱动已在别处完成 install/start。
 */
static void daran_control_task(void *arg)
{
    ESP_LOGI(EXAMPLE_TAG, "start motor example, id=%d sel=%d loop=%d", MOTOR_ID, DARAN_EXAMPLE_SELECT, DARAN_EXAMPLE_LOOP);

    // 等待 TWAI 启动完成
    vTaskDelay(pdMS_TO_TICKS(1000));

    do {
#if DARAN_EXAMPLE_SELECT == 1
        run_example_read_only(MOTOR_ID);
#elif DARAN_EXAMPLE_SELECT == 2
        run_example_speed(MOTOR_ID);
#elif DARAN_EXAMPLE_SELECT == 3
        run_example_position(MOTOR_ID);
#else
#error "Invalid DARAN_EXAMPLE_SELECT"
#endif
        vTaskDelay(pdMS_TO_TICKS(500));
    } while (DARAN_EXAMPLE_LOOP);

    ESP_LOGI(EXAMPLE_TAG, "example finished");
    vTaskDelete(NULL);
}

/**
 * @brief 启动大然电机示例
 * @note 需要在TWAI驱动已安装并启动后调用
 *
 * @details 仅创建一个 FreeRTOS 任务来跑示例；示例选择由 DARAN_EXAMPLE_SELECT 宏决定。
 */
void daran_motor_example_start(void)
{
    ESP_LOGI(EXAMPLE_TAG, "启动大然电机示例");
    ESP_LOGI(EXAMPLE_TAG, "电机ID: %d", MOTOR_ID);
    ESP_LOGI(EXAMPLE_TAG, "CAN波特率: 1Mbps");
    
    // 创建控制任务
    xTaskCreatePinnedToCore(
        daran_control_task,
        "daran_ctrl",
        8192,
        NULL,
        5,
        NULL,
        tskNO_AFFINITY
    );
    
    ESP_LOGI(EXAMPLE_TAG, "示例任务已创建");
}
