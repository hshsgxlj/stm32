#include "h_course.h"

#include "led.h"
#include "motor.h"
#include "mpu6050.h"
#include "mpu6050_dmp.h"
#include "oled.h"
#include "track.h"

#include <stdio.h>

typedef struct {
    const uint8_t *nodes;
    uint8_t length;
} HCourse_Route;

typedef enum {
    H_COURSE_TASK2_AB_STRAIGHT = 0,
    H_COURSE_TASK2_BC_LINE,
    H_COURSE_TASK2_CD_STRAIGHT,
    H_COURSE_TASK2_DA_LINE
} HCourse_Task2Phase;

typedef enum {
    H_COURSE_TASK3_START_ALIGN = 0, /* 出发:陀螺仪转向 AC 斜线方向 */
    H_COURSE_TASK3_START_SETTLE,    /* 转向后静止确认 */
    H_COURSE_TASK3_DIAGONAL,        /* 斜线:航向保持 + 距离窗口等线头 */
    H_COURSE_TASK3_VERTEX_BRAKE,    /* 顶点:刹车 + 静止确认 */
    H_COURSE_TASK3_ALIGN,           /* 顶点转向 */
    H_COURSE_TASK3_CAPTURE,         /* 切线低速捕捉线头 */
    H_COURSE_TASK3_ARC,             /* 弧线:巡线 + 距离窗口判结束 */
    H_COURSE_TASK3_FINAL_BRAKE,     /* 终点刹车 */
    H_COURSE_TASK4_EXIT_ALIGN,      /* 任务4:出弯后对准下一段全局航向 */
    H_COURSE_TASK4_EXIT_SETTLE,     /* 任务4:摆正后等车轮停稳 */
    H_COURSE_TASK3_DONE,
    H_COURSE_TASK3_FAULT
} HCourse_Task3Phase;

static const uint8_t route_task1[] = {2U};
static const uint8_t route_task2[] = {2U, 3U, 4U, 1U};
static const uint8_t route_task3[] = {3U, 2U, 4U, 1U};
static const uint8_t route_task4[] = {
    1U, 2U, 3U, 4U, 1U, 2U, 3U, 4U,
    1U, 2U, 3U, 4U, 1U, 2U, 3U, 4U
};

static HCourse_Task course_task = H_COURSE_TASK1_AB;
static HCourse_State course_state = H_COURSE_STATE_IDLE;
static HCourse_Route course_route;
static uint8_t course_node_count;
static uint8_t course_node_armed;
static HCourse_Task2Phase task2_phase;
static uint32_t course_start_tick;
static uint32_t course_last_tick;
static uint32_t course_last_node_tick;
static uint32_t course_node_candidate_tick;
static uint32_t course_lost_line_tick;
static uint32_t course_last_debug_tick;
static uint32_t task2_edge_candidate_tick;
static uint32_t buzzer_off_tick;
static HCourse_Task3Phase task3_phase;
static uint32_t task3_phase_tick;
static uint32_t task3_edge_tick;          /* 通用去抖计时 */
static uint8_t task3_leg;                 /* 0=AC, 1=CB, 2=BD, 3=DA */
static float task3_distance_mm;           /* 编码器累计距离 */
static float task3_leg_start_mm;
static int16_t task3_start_heading_x10;   /* 出发基准(=AB 方向-α,AC 即此值) */
static int16_t task3_turn_target_x10;

static float line_integral;
static float line_previous_error;
static float line_last_error;
static uint8_t task4_lap;   /* 任务4 当前圈数(1..4) */
static uint8_t task3_diag_rescue;   /* 任务4 斜线救援模式:超距未压线,持续直行找线 */
static uint8_t dist_test_active;    /* 距离标定测试模式 */
static uint8_t dist_test_running;
static float dist_test_target_mm;
static int8_t line_recover_dir;   /* 循迹恢复模式方向:+1 线在左(P1),-1 线在右(P4),0 正常 */
static uint32_t line_recover_exit_tick;   /* 中线压稳确认计时 */
static uint32_t line_recover_brake_until; /* 入恢复急刹截止时刻 */
static uint32_t line_recover_white_tick;  /* 恢复中全白持续计时 */
static float course_line_kp = H_COURSE_LINE_KP;
static float course_line_ki = H_COURSE_LINE_KI;
static float course_line_kd = H_COURSE_LINE_KD;

static uint8_t yaw_ready;
static uint8_t yaw_sample_valid;
static uint32_t yaw_last_ok_tick;
static char task4_imu_failure;  /* I=初始化,C=校准,R=读取 */
static int16_t current_yaw_x10;
static int16_t target_yaw_x10;
static int16_t last_yaw_correction;
static float yaw_integral;
static float yaw_previous_error;
static TRACK_StateTypeDef last_track;
static int16_t last_left_pwm;
static int16_t last_right_pwm;

static void buzzer_set(uint8_t on);
static void control_line(uint32_t now, uint8_t blacks,
                         int16_t base_left, int16_t base_right);

static void buzzer_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    buzzer_set(0U);

    /* LED1 (PC2) lights together with the buzzer beeps. */
    LED_Init();
}

static void buzzer_set(uint8_t on)
{
    /* PA15 is BUZ_ON in the C10B schematic; active high through S8050. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,
                      (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void buzzer_beep(uint32_t now)
{
    buzzer_set(1U);
    LED_On(1U);
    buzzer_off_tick = now + H_COURSE_BUZZER_BEEP_MS;
}

static void buzzer_update(uint32_t now)
{
    if ((buzzer_off_tick != 0U) &&
        ((int32_t)(now - buzzer_off_tick) >= 0)) {
        buzzer_off_tick = 0U;
        buzzer_set(0U);
        LED_Off(1U);
    }
}

static int16_t clamp_speed(int32_t speed)
{
    if (speed > H_COURSE_MAX_SPEED) return H_COURSE_MAX_SPEED;
    if (speed < -H_COURSE_MAX_SPEED) return -H_COURSE_MAX_SPEED;
    return (int16_t)speed;
}

static HCourse_Route route_for_task(HCourse_Task task)
{
    HCourse_Route route;

    route.nodes = route_task1;
    route.length = (uint8_t)(sizeof(route_task1) / sizeof(route_task1[0]));
    if (task == H_COURSE_TASK2_ABCD) {
        route.nodes = route_task2;
        route.length = (uint8_t)(sizeof(route_task2) / sizeof(route_task2[0]));
    } else if (task == H_COURSE_TASK3_ACBDA) {
        route.nodes = route_task3;
        route.length = (uint8_t)(sizeof(route_task3) / sizeof(route_task3[0]));
    } else if (task == H_COURSE_TASK4_FOUR_LAPS) {
        route.nodes = route_task4;
        route.length = (uint8_t)(sizeof(route_task4) / sizeof(route_task4[0]));
    }
    return route;
}

static int16_t yaw_error_x10(int16_t target, int16_t current)
{
    int32_t error = (int32_t)target - (int32_t)current;
    while (error > 1800) error -= 3600;
    while (error < -1800) error += 3600;
    return (int16_t)error;
}

static int16_t yaw_relative_x10(void)
{
    /* The DMP yaw is an arbitrary absolute heading.  Only the change from
     * the stationary launch heading is useful for straight-line control. */
    if (yaw_ready == 0U) return 0;
    return yaw_error_x10(current_yaw_x10, target_yaw_x10);
}

static uint8_t black_count(const TRACK_StateTypeDef *track)
{
    uint8_t count = 0U;

    if (track->p1 == 0U) count++;
    if (track->p2 == 0U) count++;
    if (track->p3 == 0U) count++;
    if (track->p4 == 0U) count++;
    return count;
}

static void update_yaw(void)
{
    /* 和 Sunward 参考代码同一方案:不用 DMP 的 yaw,原始陀螺仪积分 +
     * 静止零偏校准(MPU6050_Calibrate),漂移小一个数量级。
     * 校准在开机时阻塞完成,能读到数据即可用。 */
    MPU6050_AttitudeTypeDef attitude;

    yaw_sample_valid = 0U;
    if (MPU6050_ReadAttitude(&attitude) == HAL_OK) {
        current_yaw_x10 = attitude.yaw_deg_x10;
        yaw_ready = 1U;
        yaw_sample_valid = 1U;
        yaw_last_ok_tick = HAL_GetTick();
    }
}

static int16_t yaw_hold_correction(void)
{
    int16_t error_x10 = yaw_error_x10(target_yaw_x10, current_yaw_x10);
    float error = (float)error_x10 / 10.0f;
    float derivative = error - yaw_previous_error;
    float correction;

    if (yaw_ready == 0U) {
        last_yaw_correction = 0;
        return 0;
    }

    yaw_integral += error * ((float)H_COURSE_CONTROL_PERIOD_MS / 1000.0f);
    if (yaw_integral > H_COURSE_YAW_INTEGRAL_LIMIT) yaw_integral = H_COURSE_YAW_INTEGRAL_LIMIT;
    if (yaw_integral < -H_COURSE_YAW_INTEGRAL_LIMIT) yaw_integral = -H_COURSE_YAW_INTEGRAL_LIMIT;
    correction = H_COURSE_YAW_SIGN *
                 ((H_COURSE_YAW_KP * error) +
                  (H_COURSE_YAW_KI * yaw_integral) +
                  (H_COURSE_YAW_KD * derivative));
    yaw_previous_error = error;

    if (correction > H_COURSE_YAW_CORRECTION_LIMIT) correction = H_COURSE_YAW_CORRECTION_LIMIT;
    if (correction < -H_COURSE_YAW_CORRECTION_LIMIT) correction = -H_COURSE_YAW_CORRECTION_LIMIT;
    last_yaw_correction = (int16_t)correction;
    return last_yaw_correction;
}

static void apply_yaw_hold_pair(int16_t left_speed, int16_t right_speed)
{
    int16_t correction = yaw_hold_correction();

    last_left_pwm = clamp_speed((int32_t)left_speed + (int32_t)correction);
    last_right_pwm = clamp_speed((int32_t)right_speed - (int32_t)correction);
    MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
}

/* Re-capture the heading when a new task-2 segment starts: each straight
 * section points a different way (CD runs opposite to AB). */
static void yaw_retarget(void)
{
    target_yaw_x10 = current_yaw_x10;
    yaw_integral = 0.0f;
    yaw_previous_error = 0.0f;
    /* 阶段切换时同步清循迹积分,上一段的弯道转向力不带进下一段 */
    line_integral = 0.0f;
    line_previous_error = 0.0f;
    line_last_error = 0.0f;
}

static uint8_t update_node_detector(uint32_t now, uint8_t blacks)
{
    /* 路口判定:任意一路压黑就算到达路口(横线标记)。 */
    if ((now - course_start_tick) < H_COURSE_START_IGNORE_MS) return 0U;

    if (blacks < H_COURSE_NODE_BLACK_COUNT) {
        course_node_armed = 1U;
        course_node_candidate_tick = 0U;
        return 0U;
    }

    if (course_node_armed == 0U) return 0U;
    if ((now - course_last_node_tick) < H_COURSE_NODE_COOLDOWN_MS) return 0U;

    if (course_node_candidate_tick == 0U) {
        course_node_candidate_tick = now;
        return 0U;
    }

    if ((now - course_node_candidate_tick) < H_COURSE_NODE_CONFIRM_MS) return 0U;

    course_node_candidate_tick = 0U;
    course_node_armed = 0U;
    course_last_node_tick = now;
    if (course_node_count < course_route.length) course_node_count++;

    if ((course_task != H_COURSE_TASK2_ABCD) &&
        (course_node_count >= course_route.length)) {
        course_state = H_COURSE_STATE_FINISHED;
        last_left_pwm = 0;
        last_right_pwm = 0;
        MOTOR_SetSpeeds(0, 0);
    }
    return 1U;
}

static uint8_t task2_all_white_event(uint32_t now, uint8_t blacks)
{
    if (blacks != 0U) {
        task2_edge_candidate_tick = 0U;
        return 0U;
    }
    if (task2_edge_candidate_tick == 0U) {
        task2_edge_candidate_tick = now;
        return 0U;
    }
    if ((now - task2_edge_candidate_tick) < H_COURSE_TASK2_EDGE_CONFIRM_MS) {
        return 0U;
    }
    task2_edge_candidate_tick = 0U;
    return 1U;
}

/* 任务3的节点身份来自阶段顺序,不是一条黑线自身能提供的身份。
 * 斜线靠航向保持,弧线靠巡线;实车需验证 C/B/D/A 的实际位置。 */
static void task3_stop(HCourse_State state, uint32_t now)
{
    MOTOR_SetSpeeds(0, 0);
    MOTOR_SetEnabled(0U);
    last_left_pwm = 0;
    last_right_pwm = 0;
    course_state = state;
    buzzer_beep(now);
}

static int16_t task3_wrap_x10(int32_t a)
{
    while (a > 1800) a -= 3600;
    while (a < -1800) a += 3600;
    return (int16_t)a;
}

/* 1~3 路压黑且连续才算"线有效"(和 Sunward 的 validity 检查一致) */
static uint8_t task3_line_valid(void)
{
    uint8_t black[4];
    int8_t first = -1;
    int8_t last = -1;
    uint8_t count = 0U;
    uint8_t i;

    black[0] = (last_track.p1 == 0U) ? 1U : 0U;
    black[1] = (last_track.p2 == 0U) ? 1U : 0U;
    black[2] = (last_track.p3 == 0U) ? 1U : 0U;
    black[3] = (last_track.p4 == 0U) ? 1U : 0U;
    for (i = 0U; i < 4U; ++i) {
        if (black[i] != 0U) {
            if (first < 0) first = (int8_t)i;
            last = (int8_t)i;
            ++count;
        }
    }
    return ((count > 0U) && (count < 4U) &&
            ((int8_t)(last - first + 1) == (int8_t)count)) ? 1U : 0U;
}

/* 通用去抖:条件持续 DEBOUNCE_MS 返回 1 */
static uint8_t task3_stable(int matches, uint32_t now)
{
    if (matches == 0) {
        task3_edge_tick = 0U;
        return 0U;
    }
    if (task3_edge_tick == 0U) task3_edge_tick = now;
    return ((now - task3_edge_tick) >= H_COURSE_TASK3_DEBOUNCE_MS) ? 1U : 0U;
}

static void task3_enter_phase(HCourse_Task3Phase phase, uint32_t now)
{
    task3_phase = phase;
    task3_phase_tick = now;
    task3_edge_tick = 0U;
    course_lost_line_tick = 0U;
    line_integral = 0.0f;
    line_previous_error = 0.0f;
    line_recover_dir = 0;
    line_recover_exit_tick = 0U;
    line_recover_white_tick = 0U;
    task3_diag_rescue = 0U;
}

/* 几何航向(度×10;正 yaw = 左转,AC 为右转 α):
 * 出发记录 start = 出发yaw − α;东向基准 east = start + α = 出发方向。 */
static int16_t task3_diag_heading_x10(void)
{
    int32_t t = (int32_t)task3_start_heading_x10;
    if (task3_leg == 2U) {
        t += 1800 + 2 * (int32_t)H_COURSE_TASK3_DIAG_ANGLE_X10 +
             (int32_t)H_COURSE_TASK3_BD_TRIM_X10;
    } else {
        t += (int32_t)H_COURSE_TASK3_AC_TRIM_X10;
    }
    return task3_wrap_x10(t);
}

/* A->C 的全局航向。任务4 每圈回到 A 后直接摆正到下一段斜线，
 * 不把上一圈的实际偏差累积到下一圈。 */
static int16_t task3_ac_global_heading_x10(void)
{
    return task3_wrap_x10((int32_t)task3_start_heading_x10 +
                          (int32_t)H_COURSE_TASK3_DIAG_ANGLE_X10 +
                          (int32_t)H_COURSE_TASK3_AC_TRIM_X10);
}

static int16_t task3_tangent_heading_x10(uint8_t end)
{
    int32_t east = (int32_t)task3_start_heading_x10 +
                   (int32_t)H_COURSE_TASK3_DIAG_ANGLE_X10;
    uint8_t add180 = (task3_leg == 1U) ? end : (uint8_t)(!end);
    return task3_wrap_x10(east + (add180 != 0U ? 1800 : 0));
}

static int16_t task3_align_heading_x10(void)
{
    return (task3_leg & 1U) ? task3_tangent_heading_x10(0U)
                            : task3_diag_heading_x10();
}

/* 单轮前进原地转向(不倒车);返回 1 = 已到目标角(去抖后) */
static uint8_t task3_turn_control(uint32_t now)
{
    int16_t error = task3_wrap_x10((int32_t)task3_turn_target_x10 -
                                   (int32_t)current_yaw_x10);
    int16_t turn_pwm = H_COURSE_TASK3_TURN_PWM;

    if (task3_stable((error > -H_COURSE_TASK3_ALIGN_TOL_X10) &&
                     (error <  H_COURSE_TASK3_ALIGN_TOL_X10), now) != 0U) {
        MOTOR_SetSpeeds(0, 0);
        return 1U;
    }
    /* 靠近目标(误差<10°)降速:避免高速扫过停止窗口来回摆;
     * 但不能低于 50(再低会推不动车,直接停转超时) */
    if ((error > -100) && (error < 100)) {
        turn_pwm = 50;
    }
    /* 本车:正 yaw = 左转,MOTOR_A = 物理右轮(交叉接线):
     * 误差>0 → 需左转 → 驱动物理右轮 MOTOR_A;
     * 误差<0 → 需右转 → 驱动物理左轮 MOTOR_B。 */
    last_left_pwm = (error > 0) ? turn_pwm : 0;
    last_right_pwm = (error < 0) ? turn_pwm : 0;
    MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
    return 0U;
}

static uint8_t task3_fault_code;   /* 1总超时 2陀螺仪 3航向 4线头距离 5转向超时 6捕捉超时 */

static void task3_fault(uint32_t now, uint8_t code)
{
    task3_fault_code = code;
    task3_phase = H_COURSE_TASK3_FAULT;
    task3_stop(H_COURSE_STATE_FAULT, now);
}

/* 斜线接近弯道时的预修正:线先出现在最外侧传感器(P1/P4)时,
 * 提前向线侧打方向,避免确认窗口内车继续沿斜线跑而错过线头。
 * 白区无黑线时仍走 yaw 保持直行。 */
/* ============ 直行同步:右轮参照左轮 ============ */

static float sync_integral;
static float sync_previous_error;
static float sync_left_filt;
static float sync_right_filt;
static float sync_correction;
static uint8_t sync_filter_init;

static void sync_reset(void)
{
    sync_integral = 0.0f;
    sync_previous_error = 0.0f;
    sync_left_filt = 0.0f;
    sync_right_filt = 0.0f;
    sync_correction = 0.0f;
    sync_filter_init = 0U;
}

/* 每个控制周期调用:读编码器,更新左右轮滤波转速和右轮修正量 */
static void sync_tick(void)
{
    int16_t delta_l = MOTOR_A_ReadEncoderDelta();
    int16_t delta_r = MOTOR_B_ReadEncoderDelta();
    int32_t mag_l = (delta_l < 0) ? -delta_l : delta_l;
    int32_t mag_r = (delta_r < 0) ? -delta_r : delta_r;
    float raw_l = (float)((int64_t)mag_l * 6000LL) /
                  (float)H_COURSE_TASK1_COUNTS_PER_REV;
    float raw_r = (float)((int64_t)mag_r * 6000LL) /
                  (float)H_COURSE_TASK1_COUNTS_PER_REV;
    float error;

    /* 距离累计(两轮平均),任务3 计距用 */
    task3_distance_mm += 0.5f * (float)(mag_l + mag_r) *
                         H_COURSE_TASK3_WHEEL_CIRC_MM /
                         (float)H_COURSE_TASK1_COUNTS_PER_REV;

    if (sync_filter_init == 0U) {
        sync_left_filt = raw_l;
        sync_right_filt = raw_r;
        sync_filter_init = 1U;
        return;
    }
    sync_left_filt += 0.3f * (raw_l - sync_left_filt);
    sync_right_filt += 0.3f * (raw_r - sync_right_filt);

    /* 误差 = (左轮实际转速 + 轮径补偿) − 右轮实际转速
     * (右轮慢 → 误差为正 → 右轮加力) */
    error = (sync_left_filt + H_COURSE_SYNC_RPM_TRIM) - sync_right_filt;
    sync_integral += error * ((float)H_COURSE_CONTROL_PERIOD_MS / 1000.0f);
    if (sync_integral > H_COURSE_SYNC_INTEGRAL_LIMIT) sync_integral = H_COURSE_SYNC_INTEGRAL_LIMIT;
    if (sync_integral < -H_COURSE_SYNC_INTEGRAL_LIMIT) sync_integral = -H_COURSE_SYNC_INTEGRAL_LIMIT;

    sync_correction = (H_COURSE_SYNC_KP * error) +
                      (H_COURSE_SYNC_KI * sync_integral) +
                      (H_COURSE_SYNC_KD * (error - sync_previous_error));
    sync_previous_error = error;
    if (sync_correction > H_COURSE_SYNC_LIMIT) sync_correction = H_COURSE_SYNC_LIMIT;
    if (sync_correction < -H_COURSE_SYNC_LIMIT) sync_correction = -H_COURSE_SYNC_LIMIT;
}

/* 直行时右轮 PWM:固定基准 + 同步修正 */
static int16_t sync_right_pwm(void)
{
    return clamp_speed((int32_t)H_COURSE_STRAIGHT_PWM_RIGHT + (int32_t)sync_correction);
}

static void task3_diag_steer(uint8_t blacks,
                             int16_t base_left, int16_t base_right)
{
    /* 已由 Sunward 风格的状态机(转向/捕捉/航向保持)取代,保留空壳 */
    (void)blacks;
    (void)base_left;
    (void)base_right;
}

static void control_task3(uint32_t now, uint8_t blacks,
                          int16_t base_left, int16_t base_right)
{
    uint32_t elapsed = now - task3_phase_tick;
    float progress = task3_distance_mm - task3_leg_start_mm;
    int16_t heading_err;
    int16_t target;

    (void)base_left;
    (void)base_right;

    if ((task3_phase == H_COURSE_TASK3_FAULT) ||
        (task3_phase == H_COURSE_TASK3_DONE)) return;
    if ((course_task == H_COURSE_TASK4_FOUR_LAPS) &&
        (yaw_sample_valid == 0U)) {
        /* 不能用上次的 yaw 继续转向。短暂读失败先停车,持续 1 秒才报 E2R。 */
        MOTOR_SetSpeeds(0, 0);
        last_left_pwm = 0;
        last_right_pwm = 0;
        if ((now - yaw_last_ok_tick) >= 1000U) {
            task4_imu_failure = 'R';
            task3_fault(now, 2U);
        }
        return;
    }
    if (yaw_ready == 0U) {
        /* 陀螺仪尚未就绪:给 1 秒宽限期等它恢复(偶发 I2C 读失败不判死),
         * 超时才判故障。宽限期内先停住等待。 */
        if ((now - task3_phase_tick) >= 1000U) {
            task3_fault(now, 2U);
        }
        MOTOR_SetSpeeds(0, 0);
        return;
    }

    course_state = H_COURSE_STATE_STRAIGHT_HOLD;

    switch (task3_phase) {
    case H_COURSE_TASK3_START_ALIGN:
        if (elapsed >= H_COURSE_TASK3_ALIGN_TIMEOUT_MS) { task3_fault(now, 5U); return; }
        if (task3_turn_control(now) != 0U) {
            task3_enter_phase(H_COURSE_TASK3_START_SETTLE, now);
        }
        return;

    case H_COURSE_TASK3_START_SETTLE:
        if ((elapsed >= H_COURSE_TASK3_SETTLE_MS) &&
            ((int32_t)sync_left_filt <= H_COURSE_TASK3_SETTLE_RPM) &&
            ((int32_t)sync_right_filt <= H_COURSE_TASK3_SETTLE_RPM)) {
            task3_leg_start_mm = task3_distance_mm;
            task3_enter_phase(H_COURSE_TASK3_DIAGONAL, now);
        }
        return;

    case H_COURSE_TASK3_DIAGONAL:
        target = task3_diag_heading_x10();
        heading_err = task3_wrap_x10((int32_t)target - (int32_t)current_yaw_x10);
        if ((heading_err > H_COURSE_TASK3_DIAG_TOL_X10) ||
            (heading_err < -H_COURSE_TASK3_DIAG_TOL_X10)) { task3_fault(now, 3U); return; }
        if ((course_task == H_COURSE_TASK4_FOUR_LAPS) &&
            (task3_diag_rescue == 0U) &&
            (progress > H_COURSE_TASK3_DIAG_MM + H_COURSE_TASK3_ENTRY_WINDOW_MM)) {
            /* 任务4 救援:超出预期距离还没压到黑线 → 不判故障,
             * 摆正车身(yaw 保持自动回正),持续直行直到压线 */
            task3_diag_rescue = 1U;
            target_yaw_x10 = target;
            yaw_integral = 0.0f;
            yaw_previous_error = 0.0f;
        } else if ((course_task != H_COURSE_TASK4_FOUR_LAPS) &&
                   (progress > H_COURSE_TASK3_DIAG_MM + H_COURSE_TASK3_ENTRY_WINDOW_MM)) {
            task3_fault(now, 4U); return;
        }
        if ((task3_diag_rescue != 0U) &&
            (progress > 2.0f * H_COURSE_TASK3_DIAG_MM)) {
            /* 救援模式跑了两倍斜线长度还找不到线:真故障 */
            task3_fault(now, 4U); return;
        }
        if ((task3_line_valid() != 0U) && (task3_stable(1, now) != 0U) &&
            ((task3_diag_rescue != 0U) ||
             (progress >= H_COURSE_TASK3_DIAG_MM - H_COURSE_TASK3_ENTRY_WINDOW_MM))) {
            /* 窗口内(或救援中)压到有效黑线 → 进入顶点制动 */
            course_node_count = (task3_leg == 0U) ? 1U : 3U;   /* C / D */
            buzzer_beep(now);
            task3_enter_phase(H_COURSE_TASK3_VERTEX_BRAKE, now);
            MOTOR_SetSpeeds(0, 0);
            return;
        }
        target_yaw_x10 = target;
        apply_yaw_hold_pair(H_COURSE_TASK3_DIAG_PWM, H_COURSE_TASK3_DIAG_PWM);
        return;

    case H_COURSE_TASK3_VERTEX_BRAKE:
        if ((elapsed >= H_COURSE_TASK3_SETTLE_MS) &&
            ((int32_t)sync_left_filt <= H_COURSE_TASK3_SETTLE_RPM) &&
            ((int32_t)sync_right_filt <= H_COURSE_TASK3_SETTLE_RPM)) {
            if ((course_task == H_COURSE_TASK4_FOUR_LAPS) &&
                (task3_leg == 1U)) {
                /* C-B 出弯:直接按全局基准摆正到下一段 B-D 斜线。 */
                ++task3_leg;
                task3_turn_target_x10 = task3_diag_heading_x10();
                sync_reset();
                task3_enter_phase(H_COURSE_TASK4_EXIT_ALIGN, now);
            } else {
                ++task3_leg;
                task3_turn_target_x10 = task3_align_heading_x10();
                sync_reset();
                task3_enter_phase(H_COURSE_TASK3_ALIGN, now);
            }
        }
        return;

    case H_COURSE_TASK3_ALIGN:
        if (elapsed >= H_COURSE_TASK3_ALIGN_TIMEOUT_MS) { task3_fault(now, 5U); return; }
        if (task3_turn_control(now) != 0U) {
            sync_reset();
            if (task3_leg & 1U) {
                task3_enter_phase(H_COURSE_TASK3_CAPTURE, now);
            } else {
                task3_leg_start_mm = task3_distance_mm;
                task3_enter_phase(H_COURSE_TASK3_DIAGONAL, now);
            }
        }
        return;

    case H_COURSE_TASK3_CAPTURE:
        target = task3_tangent_heading_x10(0U);
        heading_err = task3_wrap_x10((int32_t)target - (int32_t)current_yaw_x10);
        if (elapsed >= H_COURSE_TASK3_CAPTURE_TIMEOUT_MS) { task3_fault(now, 6U); return; }
        if ((heading_err > H_COURSE_TASK3_DIAG_TOL_X10) ||
            (heading_err < -H_COURSE_TASK3_DIAG_TOL_X10)) { task3_fault(now, 3U); return; }
        if ((task3_line_valid() != 0U) && (task3_stable(1, now) != 0U)) {
            task3_leg_start_mm = task3_distance_mm;
            task3_enter_phase(H_COURSE_TASK3_ARC, now);
            return;
        }
        target_yaw_x10 = target;
        apply_yaw_hold_pair(H_COURSE_TASK3_CAPTURE_PWM, H_COURSE_TASK3_CAPTURE_PWM);
        return;

    case H_COURSE_TASK3_ARC:
        /* 和任务2 相同的全白判定:全白持续 250ms = 弧线走完 */
        if ((line_recover_dir == 0) &&
            (task2_all_white_event(now, blacks) != 0U)) {
            course_node_count = (task3_leg == 1U) ? 2U : 4U;   /* B / A */
            if (task3_leg == 3U) {
                if (course_task == H_COURSE_TASK4_FOUR_LAPS) buzzer_beep(now);
                task3_enter_phase(H_COURSE_TASK3_FINAL_BRAKE, now);
            } else {
                buzzer_beep(now);
                task3_enter_phase(H_COURSE_TASK3_VERTEX_BRAKE, now);
            }
            MOTOR_SetSpeeds(0, 0);
            return;
        }
        control_line(now, blacks, H_COURSE_TASK3_ARC_PWM, H_COURSE_TASK3_ARC_PWM);
        return;

    case H_COURSE_TASK3_FINAL_BRAKE:
        if ((elapsed >= H_COURSE_TASK3_SETTLE_MS) &&
            ((int32_t)sync_left_filt <= H_COURSE_TASK3_SETTLE_RPM) &&
            ((int32_t)sync_right_filt <= H_COURSE_TASK3_SETTLE_RPM)) {
            if (course_task == H_COURSE_TASK4_FOUR_LAPS) {
                /* D-A 出弯:直接按启动时的全局基准摆正到下一段 A-C。 */
                task3_turn_target_x10 = task3_ac_global_heading_x10();
                sync_reset();
                task3_enter_phase(H_COURSE_TASK4_EXIT_ALIGN, now);
                return;
            }
            task3_phase = H_COURSE_TASK3_DONE;
            task3_stop(H_COURSE_STATE_FINISHED, now);
        }
        return;

    case H_COURSE_TASK4_EXIT_ALIGN:
        if (elapsed >= H_COURSE_TASK3_ALIGN_TIMEOUT_MS) { task3_fault(now, 5U); return; }
        if (task3_turn_control(now) != 0U) {
            sync_reset();
            task3_enter_phase(H_COURSE_TASK4_EXIT_SETTLE, now);
        }
        return;

    case H_COURSE_TASK4_EXIT_SETTLE:
        if ((elapsed >= H_COURSE_TASK3_SETTLE_MS) &&
            ((int32_t)sync_left_filt <= H_COURSE_TASK3_SETTLE_RPM) &&
            ((int32_t)sync_right_filt <= H_COURSE_TASK3_SETTLE_RPM)) {
            if (task3_leg == 3U) {
                ++task4_lap;
                if (task4_lap >= H_COURSE_TASK4_LAPS) {
                    /* 第四圈回到 A 并摆正后停车。 */
                    task3_phase = H_COURSE_TASK3_DONE;
                    MOTOR_SetSpeeds(0, 0);
                    MOTOR_SetEnabled(0U);
                    last_left_pwm = 0;
                    last_right_pwm = 0;
                    course_state = H_COURSE_STATE_FINISHED;
                    return;
                }
                task3_leg = 0U;
                task3_distance_mm = 0.0f;
                task3_leg_start_mm = 0.0f;
                task3_turn_target_x10 = task3_diag_heading_x10();
                sync_reset();
                task3_enter_phase(H_COURSE_TASK3_START_ALIGN, now);
            } else {
                /* C-B 出弯已完成摆正,下一步直接进入 B-D 斜线。 */
                task3_leg_start_mm = task3_distance_mm;
                task3_enter_phase(H_COURSE_TASK3_DIAGONAL, now);
            }
        }
        return;

    default:
        return;
    }
}

static void control_line(uint32_t now, uint8_t blacks,
                         int16_t base_left, int16_t base_right)
{
    float weighted_sum = 0.0f;
    float error;
    float correction;
    int32_t left;
    int32_t right;

    if (blacks == 0U) {
        if (course_lost_line_tick == 0U) course_lost_line_tick = now;
        course_state = H_COURSE_STATE_STRAIGHT_HOLD;
        /* 恢复模式下全白(线在传感器间隙/刚脱离最外侧):
         * 继续按恢复方向缓慢漂移找线,不立刻掰直 */
        if (line_recover_dir != 0) {
            if (line_recover_white_tick == 0U) line_recover_white_tick = now;
            if ((now - line_recover_white_tick) <= H_COURSE_LINE_RECOVER_WHITE_MS) {
                int32_t diff = (int32_t)H_COURSE_LINE_RECOVER_DIFF * line_recover_dir;
                int32_t slow_base = ((int32_t)base_left + (int32_t)base_right) *
                                    (int32_t)H_COURSE_LINE_RECOVER_SPEED_PCT / 200;
                last_left_pwm = clamp_speed(slow_base + diff);
                last_right_pwm = clamp_speed(slow_base - diff);
                MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
                return;
            }
            /* 持续全白超时:放弃恢复,转常规丢线处理 */
            line_recover_dir = 0;
            line_recover_white_tick = 0U;
        }
        /* Narrow-line gap between sensors: keep the last differential so a
         * curve keeps turning instead of snapping to a straight heading. */
        if ((now - course_lost_line_tick) <= H_COURSE_LOST_HOLD_DIFF_MS) {
            MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
            return;
        }
#if H_COURSE_ENABLE_YAW_HOLD
        if ((yaw_ready != 0U) && ((now - course_lost_line_tick) <= H_COURSE_LOST_LINE_HOLD_MS)) {
            apply_yaw_hold_pair(base_left, base_right);
            return;
        }
#endif
        if ((now - course_lost_line_tick) <= H_COURSE_LOST_LINE_HOLD_MS) {
            /* 丢线找线:积分冻结(不更新也不清零)——弯道上短暂全白后
             * 找回线,累积的转向力还在,车能继续压弯 */
            last_left_pwm = clamp_speed(base_left);
            last_right_pwm = clamp_speed(base_right);
            MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
            return;
        }
        last_left_pwm = 0;
        last_right_pwm = 0;
        MOTOR_SetSpeeds(0, 0);
        return;
    }

    course_lost_line_tick = 0U;
    line_recover_white_tick = 0U;
    course_state = H_COURSE_STATE_LINE_FOLLOW;

    /* P1..P4 are physically left-to-right: black=0.  A black P2 means the
     * line is on the left, so correct to the right: left wheel faster and
     * right wheel slower.  A black P3 produces the opposite correction.
     * 中间两路权重 0.5:压线只做微调;外侧 ±3:线跑偏才大力掰。 */
    if (last_track.p1 == 0U) weighted_sum += 3.0f;
    if (last_track.p2 == 0U) weighted_sum += 0.8f;
    if (last_track.p3 == 0U) weighted_sum -= 0.8f;
    if (last_track.p4 == 0U) weighted_sum -= 3.0f;

    if (blacks >= 4U) {
        line_previous_error = line_last_error;
        line_integral = 0.0f;
        last_left_pwm = H_COURSE_ALL_BLACK_SPEED;
        last_right_pwm = H_COURSE_ALL_BLACK_SPEED;
        MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
        return;
    }

    /* 恢复模式:线在最外侧(P1/P4)时,持续向线侧打固定差速,
     * 直到中间两路(P2/P3)任一压黑才恢复正常修正。 */
    if (line_recover_dir != 0) {
        if ((int32_t)(now - line_recover_brake_until) < 0) {
            /* 刚进入恢复:先急刹杀掉横向动量,再缓慢追线 */
            last_left_pwm = 0;
            last_right_pwm = 0;
            MOTOR_SetSpeeds(0, 0);
            return;
        }
        if ((last_track.p2 == 0U) || (last_track.p3 == 0U)) {
            /* 中线压到了:先走正常微调,连续压稳 30ms 才算真正救回,
             * 避免摆幅掠过中线瞬间就退出恢复、线又跑掉。 */
            if (line_recover_exit_tick == 0U) {
                line_recover_exit_tick = now;
            } else if ((now - line_recover_exit_tick) >= H_COURSE_LINE_RECOVER_EXIT_MS) {
                line_recover_dir = 0;
                line_recover_exit_tick = 0U;
            }
        } else {
            line_recover_exit_tick = 0U;
            /* 减速追线:减小动量,避免摆幅太大直接越过中线 */
            int32_t diff = (int32_t)H_COURSE_LINE_RECOVER_DIFF * line_recover_dir;
            int32_t slow_base = ((int32_t)base_left + (int32_t)base_right) *
                                (int32_t)H_COURSE_LINE_RECOVER_SPEED_PCT / 200;
            last_left_pwm = clamp_speed(slow_base + diff);
            last_right_pwm = clamp_speed(slow_base - diff);
            MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
            return;
        }
    }
    if ((last_track.p1 == 0U) && (last_track.p2 != 0U) && (last_track.p3 != 0U)) {
        if (line_recover_dir == 0) {
            line_recover_brake_until = now + H_COURSE_LINE_RECOVER_BRAKE_MS;
        }
        line_recover_dir = 1;
    } else if ((last_track.p4 == 0U) && (last_track.p2 != 0U) && (last_track.p3 != 0U)) {
        if (line_recover_dir == 0) {
            line_recover_brake_until = now + H_COURSE_LINE_RECOVER_BRAKE_MS;
        }
        line_recover_dir = -1;
    }

    error = (float)weighted_sum / (float)blacks;
    line_integral += error * ((float)H_COURSE_CONTROL_PERIOD_MS / 1000.0f);
    if (line_integral > H_COURSE_LINE_INTEGRAL_LIMIT) line_integral = H_COURSE_LINE_INTEGRAL_LIMIT;
    if (line_integral < -H_COURSE_LINE_INTEGRAL_LIMIT) line_integral = -H_COURSE_LINE_INTEGRAL_LIMIT;

    correction = (course_line_kp * error) +
                 (course_line_ki * line_integral) +
                 (course_line_kd * (error - line_previous_error));
    if (correction > H_COURSE_LINE_CORRECTION_LIMIT) correction = H_COURSE_LINE_CORRECTION_LIMIT;
    if (correction < -H_COURSE_LINE_CORRECTION_LIMIT) correction = -H_COURSE_LINE_CORRECTION_LIMIT;
    line_previous_error = error;
    line_last_error = error;

    left = (int32_t)base_left + (int32_t)correction;
    right = (int32_t)base_right - (int32_t)correction;
    last_left_pwm = clamp_speed(left);
    last_right_pwm = clamp_speed(right);
    MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
}

static void debug_output(uint32_t now)
{
#if H_COURSE_DEBUG
    if ((now - course_last_debug_tick) >= 100U) {
        uint8_t expected = 0U;
        if (course_node_count < course_route.length) expected = course_route.nodes[course_node_count];
        if (course_task == H_COURSE_TASK1_AB) {
            printf("H1,P=%u%u%u%u,L=%d,R=%d\r\n",
                   (unsigned)last_track.p1,
                   (unsigned)last_track.p2,
                   (unsigned)last_track.p3,
                   (unsigned)last_track.p4,
                   (int)last_left_pwm,
                   (int)last_right_pwm);
        } else {
            printf("H,T=%u,S=%u,N=%u,E=%u,P=%u%u%u%u,Y=%d,YE=%d,L=%d,R=%d\r\n",
                   (unsigned)course_task,
                   (unsigned)course_state,
                   (unsigned)course_node_count,
                   (unsigned)expected,
                   (unsigned)last_track.p1,
                   (unsigned)last_track.p2,
                   (unsigned)last_track.p3,
                   (unsigned)last_track.p4,
                   (int)yaw_relative_x10(),
                   (int)yaw_error_x10(target_yaw_x10, current_yaw_x10),
                   (int)last_left_pwm,
                   (int)last_right_pwm);
        }
        course_last_debug_tick = now;
    }
#else
    (void)now;
#endif
}

HAL_StatusTypeDef HCourse_Init(HCourse_Task task)
{
    HCourse_SetTask(task);
    TRACK_Init();
    MOTOR_Init();
    MOTOR_SetEnabled(1U);

    yaw_ready = 0U;
    yaw_sample_valid = 0U;
    yaw_last_ok_tick = HAL_GetTick();
    task4_imu_failure = '-';
    yaw_integral = 0.0f;
    yaw_previous_error = 0.0f;
    task2_phase = H_COURSE_TASK2_AB_STRAIGHT;
    task2_edge_candidate_tick = 0U;
    task3_phase = H_COURSE_TASK3_START_ALIGN;
    task3_phase_tick = 0U;
    task3_edge_tick = 0U;
    task3_leg = 0U;
    task3_distance_mm = 0.0f;
    task3_leg_start_mm = 0.0f;
    buzzer_off_tick = 0U;
    buzzer_init();
    /* 陀螺仪:原始模式 + 静止零偏校准。
     * 先预热 3 秒(零偏在升温期变化剧烈,不能边漂边校),
     * 再采 500 样本 ≈ 5 秒。全程车必须放稳不动。 */
    if (MPU6050_Init() == HAL_OK) {
        HAL_Delay(3000U);
        if (MPU6050_Calibrate(500) == HAL_OK) {
        } else {
            task4_imu_failure = 'C';
        }
    } else {
        task4_imu_failure = 'I';
    }

    HCourse_Stop();
    course_state = H_COURSE_STATE_IDLE;
    return HAL_OK;
}

void HCourse_SetTask(HCourse_Task task)
{
    if ((task < H_COURSE_TASK1_AB) || (task > H_COURSE_TASK4_FOUR_LAPS)) {
        task = H_COURSE_TASK1_AB;
    }
    course_task = task;
    course_route = route_for_task(task);
    task2_phase = H_COURSE_TASK2_AB_STRAIGHT;
    task3_phase = H_COURSE_TASK3_START_ALIGN;
}

void HCourse_Start(void)
{
    uint32_t now = HAL_GetTick();

    TRACK_Read(&last_track);
    update_yaw();
    /* 按键启动时允许重新建立一次有效采样，避免恰好一次 I2C 读失败
     * 就立即进入 E2；持续无数据仍会在任务控制中停车报错。 */
    yaw_last_ok_tick = now;
    /* 每次出发都重新记录目标航向:车已摆在出发点,当前朝向就是
     * 本任务的基准方向,上次任务累积的 yaw 误差不带进下一次。 */
    target_yaw_x10 = current_yaw_x10;
    course_node_count = 0U;
    course_node_armed = 1U;
    course_start_tick = now;
    course_last_tick = now;
    course_last_node_tick = now;
    course_node_candidate_tick = 0U;
    course_lost_line_tick = 0U;
    course_last_debug_tick = now;
    line_integral = 0.0f;
    line_previous_error = 0.0f;
    line_last_error = 0.0f;
    line_recover_dir = 0;
    line_recover_exit_tick = 0U;
    yaw_integral = 0.0f;
    yaw_previous_error = 0.0f;
    task2_phase = H_COURSE_TASK2_AB_STRAIGHT;
    task2_edge_candidate_tick = 0U;
    task4_lap = 0U;
    if ((course_task == H_COURSE_TASK3_ACBDA) ||
        (course_task == H_COURSE_TASK4_FOUR_LAPS)) {
        /* 出发朝 AB;AC 方向 = 出发yaw − α(右转)。记录基准并开始转向。 */
        task3_leg = 0U;
        task3_distance_mm = 0.0f;
        task3_leg_start_mm = 0.0f;
        task3_start_heading_x10 = task3_wrap_x10((int32_t)current_yaw_x10 -
                                                  (int32_t)H_COURSE_TASK3_DIAG_ANGLE_X10);
        task3_turn_target_x10 = task3_diag_heading_x10();   /* 含 AC_TRIM */
        task3_phase = H_COURSE_TASK3_START_ALIGN;
        task3_phase_tick = now;
        task3_edge_tick = 0U;
    }
    buzzer_off_tick = 0U;
    buzzer_set(0U);
    last_left_pwm = 0;
    last_right_pwm = 0;
    course_state = (course_task == H_COURSE_TASK2_ABCD)
                 ? H_COURSE_STATE_STRAIGHT_HOLD
                 : H_COURSE_STATE_LINE_FOLLOW;
    MOTOR_ResetEncoders();
    sync_reset();
    MOTOR_SetEnabled(1U);
    if ((course_task == H_COURSE_TASK3_ACBDA) ||
        (course_task == H_COURSE_TASK4_FOUR_LAPS)) {
        /* A 是起点,后续 C/B/D/A 的响铃由各段切换时发出 */
        buzzer_beep(now);
    }
}

void HCourse_PeriodicTask(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t blacks;
    uint8_t node_event;
    int16_t base_left;
    int16_t base_right;

    if ((now - course_last_tick) < H_COURSE_CONTROL_PERIOD_MS) return;
    course_last_tick = now;

    /* Keep the IMU running during the idle wait so the 20 s calibration
     * completes at power-up + 20 s, not after the launch key press. */
    update_yaw();
    buzzer_update(now);

    if ((course_state == H_COURSE_STATE_IDLE) ||
        (course_state == H_COURSE_STATE_FINISHED) ||
        (course_state == H_COURSE_STATE_STOPPED) ||
        (course_state == H_COURSE_STATE_FAULT)) {
        return;
    }

    TRACK_Read(&last_track);
    blacks = black_count(&last_track);
    sync_tick();

    /* H题任务一：开环直行(固定 PWM),任意一路检测到黑线后立即停车。 */
    if (course_task == H_COURSE_TASK1_AB) {
        if (blacks != 0U) {
            last_left_pwm = 0;
            last_right_pwm = 0;
            MOTOR_SetSpeeds(0, 0);
            MOTOR_SetEnabled(0U);
            course_state = H_COURSE_STATE_FINISHED;
            buzzer_beep(now);   /* 任务1 停车:响铃 + LED */
        } else {
            MOTOR_SetEnabled(1U);
            if (yaw_ready != 0U) {
                /* 直行:陀螺仪航向保持(校准后的原始 yaw) */
                apply_yaw_hold_pair(H_COURSE_STRAIGHT_PWM_LEFT,
                                    H_COURSE_STRAIGHT_PWM_RIGHT);
            } else {
                /* 陀螺仪失效时退回右轮同步 */
                last_left_pwm = H_COURSE_STRAIGHT_PWM_LEFT;
                last_right_pwm = sync_right_pwm();
                MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
            }
        }
        debug_output(now);
        return;
    }

    /* 开环基础速度:直行和循迹都用固定 PWM 基准 */
    base_left = H_COURSE_STRAIGHT_PWM_LEFT;
    base_right = H_COURSE_STRAIGHT_PWM_RIGHT;
    node_event = 0U;
    if (course_task == H_COURSE_TASK2_ABCD) {
        if ((task2_phase == H_COURSE_TASK2_AB_STRAIGHT) ||
            (task2_phase == H_COURSE_TASK2_CD_STRAIGHT)) {
            node_event = update_node_detector(now, blacks);
        }
    }
    if (course_state == H_COURSE_STATE_FINISHED) {
        debug_output(now);
        return;
    }

    if (course_task == H_COURSE_TASK2_ABCD) {
        if (node_event != 0U) {
            /* Beep at the end of every segment: B and D are junction
             * crossings; C and the final A are detected below as the
             * guide line ending. */
            if ((course_node_count == 1U) || (course_node_count == 2U)) {
                buzzer_beep(now);
            }
            if (course_node_count == 1U) {
                yaw_retarget();
                task2_phase = H_COURSE_TASK2_BC_LINE;
            } else if (course_node_count == 2U) {
                yaw_retarget();
                task2_phase = H_COURSE_TASK2_CD_STRAIGHT;
            }
        }

        if ((task2_phase == H_COURSE_TASK2_BC_LINE) &&
            (line_recover_dir == 0) &&
            (task2_all_white_event(now, blacks) != 0U)) {
            buzzer_beep(now);
            yaw_retarget();
            /* 第二个直线(C-D)方向补偿:过第一个弯道后右偏 → 往左 +5° */
            target_yaw_x10 = task3_wrap_x10((int32_t)target_yaw_x10 +
                                            (int32_t)H_COURSE_TASK2_CD_TRIM_X10);
            task2_phase = H_COURSE_TASK2_CD_STRAIGHT;
        } else if ((task2_phase == H_COURSE_TASK2_DA_LINE) &&
                   (line_recover_dir == 0) &&
                   (task2_all_white_event(now, blacks) != 0U)) {
            HCourse_Stop();
            course_state = H_COURSE_STATE_FINISHED;
            buzzer_beep(now);
            debug_output(now);
            return;
        }

        if ((task2_phase == H_COURSE_TASK2_CD_STRAIGHT) &&
            (course_node_count >= 2U) && (node_event != 0U)) {
            yaw_retarget();
            task2_phase = H_COURSE_TASK2_DA_LINE;
        }

        if ((task2_phase == H_COURSE_TASK2_AB_STRAIGHT) ||
            (task2_phase == H_COURSE_TASK2_CD_STRAIGHT)) {
            /* 直行:陀螺仪航向保持;失效时退回右轮同步 */
            course_state = H_COURSE_STATE_STRAIGHT_HOLD;
            if (yaw_ready != 0U) {
                apply_yaw_hold_pair(H_COURSE_STRAIGHT_PWM_LEFT,
                                    H_COURSE_STRAIGHT_PWM_RIGHT);
            } else {
                last_left_pwm = H_COURSE_STRAIGHT_PWM_LEFT;
                last_right_pwm = sync_right_pwm();
                MOTOR_SetSpeeds(last_left_pwm, last_right_pwm);
            }
        } else {
            control_line(now, blacks, base_left, base_right);
        }
    } else if ((course_task == H_COURSE_TASK3_ACBDA) ||
               (course_task == H_COURSE_TASK4_FOUR_LAPS)) {
        control_task3(now, blacks, base_left, base_right);
    } else {
        control_line(now, blacks, base_left, base_right);
    }
    debug_output(now);
}

void HCourse_UpdateOLED(void)
{
    static uint32_t last_oled_tick;
    uint32_t now = HAL_GetTick();
    const char *state_text = "WAIT    ";

    if ((now - last_oled_tick) < 100U) return;
    last_oled_tick = now;

    /* 距离标定测试模式:显示编码器计距 */
    if (dist_test_active != 0U) {
        OLED_ShowString(0U, 2U, "D:");
        OLED_ShowInt16(12U, 2U, (int16_t)task3_distance_mm);
        OLED_ShowString(0U, 6U, (dist_test_running != 0U) ? "TST RUN" : "TST END");
        return;
    }

    /* 陀螺仪校准在开机时阻塞完成,状态显示不再需要 CAL WAIT 提示 */
    if (course_state == H_COURSE_STATE_FAULT) {
        if ((course_task == H_COURSE_TASK3_ACBDA) ||
            (course_task == H_COURSE_TASK4_FOUR_LAPS)) {
            static char h3_err_text[9];
            h3_err_text[0] = 'H';
            h3_err_text[1] = (course_task == H_COURSE_TASK4_FOUR_LAPS) ? '4' : '3';
            h3_err_text[2] = ' '; h3_err_text[3] = 'E';
            h3_err_text[4] = (char)('0' + task3_fault_code);
            h3_err_text[5] = (course_task == H_COURSE_TASK4_FOUR_LAPS) ?
                              task4_imu_failure : ' ';
            h3_err_text[6] = ' '; h3_err_text[7] = '\0';
            state_text = h3_err_text;
        } else {
            state_text = "H ERR  ";
        }
    } else if (course_state == H_COURSE_STATE_IDLE) {
        state_text = "H IDLE ";
    } else if (course_task == H_COURSE_TASK1_AB) {
        if (course_state == H_COURSE_STATE_LINE_FOLLOW) state_text = "H1 RUN ";
        else if (course_state == H_COURSE_STATE_FINISHED) state_text = "H1 STOP";
        else if (course_state == H_COURSE_STATE_STOPPED) state_text = "H1 STOP";
        else if (course_state == H_COURSE_STATE_FAULT) state_text = "H1 ERR ";
    } else if (course_task == H_COURSE_TASK2_ABCD) {
        if (course_state == H_COURSE_STATE_FINISHED) {
            state_text = "H2 STOP";
        } else if (course_state == H_COURSE_STATE_STOPPED) {
            state_text = "H2 STOP";
        } else if (course_state == H_COURSE_STATE_FAULT) {
            state_text = "H2 ERR ";
        } else if (task2_phase == H_COURSE_TASK2_AB_STRAIGHT) {
            state_text = "H2 A-B ";
        } else if (task2_phase == H_COURSE_TASK2_BC_LINE) {
            state_text = "H2 B-C ";
        } else if (task2_phase == H_COURSE_TASK2_CD_STRAIGHT) {
            state_text = "H2 C-D ";
        } else {
            state_text = "H2 D-A ";
        }
    } else if (course_task == H_COURSE_TASK3_ACBDA) {
        if ((course_state == H_COURSE_STATE_FINISHED) ||
            (course_state == H_COURSE_STATE_STOPPED)) {
            state_text = "H3 STOP";
        } else if (course_state == H_COURSE_STATE_FAULT) {
            state_text = "H3 ERR ";
        } else if ((task3_phase == H_COURSE_TASK3_START_ALIGN) ||
                   (task3_phase == H_COURSE_TASK3_ALIGN)) {
            state_text = "H3 TURN";
        } else if (task3_phase == H_COURSE_TASK3_DIAGONAL) {
            state_text = (task3_leg == 0U) ? "H3 A-C " : "H3 B-D ";
        } else if (task3_phase == H_COURSE_TASK3_CAPTURE) {
            state_text = "H3 CAP ";
        } else if (task3_phase == H_COURSE_TASK3_ARC) {
            state_text = (task3_leg == 1U) ? "H3 C-B " : "H3 D-A ";
        } else if ((task3_phase == H_COURSE_TASK3_START_SETTLE) ||
                   (task3_phase == H_COURSE_TASK3_VERTEX_BRAKE) ||
                   (task3_phase == H_COURSE_TASK3_FINAL_BRAKE)) {
            state_text = "H3 BRK ";
        } else {
            state_text = "H3 DONE";
        }
    } else if (course_task == H_COURSE_TASK4_FOUR_LAPS) {
        if ((course_state == H_COURSE_STATE_FINISHED) ||
            (course_state == H_COURSE_STATE_STOPPED)) {
            state_text = "H4 STOP";
        } else if (course_state == H_COURSE_STATE_FAULT) {
            state_text = "H4 ERR ";
        } else {
            static char h4_text[8];
            const char *phase = "TN";   /* TURN */
            if (task3_phase == H_COURSE_TASK3_DIAGONAL) {
                phase = (task3_leg == 0U) ? "AC" : "BD";
            } else if (task3_phase == H_COURSE_TASK3_CAPTURE) {
                phase = "CP";
            } else if (task3_phase == H_COURSE_TASK3_ARC) {
                phase = (task3_leg == 1U) ? "CB" : "DA";
            } else if ((task3_phase == H_COURSE_TASK3_START_SETTLE) ||
                       (task3_phase == H_COURSE_TASK3_VERTEX_BRAKE) ||
                       (task3_phase == H_COURSE_TASK3_FINAL_BRAKE)) {
                phase = "BK";
            } else if ((task3_phase == H_COURSE_TASK4_EXIT_ALIGN) ||
                       (task3_phase == H_COURSE_TASK4_EXIT_SETTLE)) {
                phase = "EA";
            }
            h4_text[0] = 'H'; h4_text[1] = '4'; h4_text[2] = 'L';
            h4_text[3] = (char)('0' + task4_lap); h4_text[4] = ' ';
            h4_text[5] = phase[0]; h4_text[6] = phase[1]; h4_text[7] = '\0';
            state_text = h4_text;
        }
    } else if (course_state == H_COURSE_STATE_FINISHED) {
        state_text = "H STOP ";
    } else if (course_state == H_COURSE_STATE_FAULT) {
        state_text = "H ERR  ";
    } else if (course_state == H_COURSE_STATE_LINE_FOLLOW) {
        state_text = "H RUN  ";
    }

    OLED_ShowString(0U, 0U, "P:");
    OLED_ShowChar(12U, 0U, (char)('0' + last_track.p1));
    OLED_ShowChar(18U, 0U, (char)('0' + last_track.p2));
    OLED_ShowChar(24U, 0U, (char)('0' + last_track.p3));
    OLED_ShowChar(30U, 0U, (char)('0' + last_track.p4));

    OLED_ShowString(0U, 2U, "L:");
    OLED_ShowInt16(12U, 2U, (int16_t)sync_left_filt);
    OLED_ShowString(54U, 2U, "R:");
    OLED_ShowInt16(66U, 2U, (int16_t)sync_right_filt);

    OLED_ShowString(0U, 4U, "Y:");
    OLED_ShowAngle(12U, 4U, yaw_relative_x10());
    OLED_ShowString(54U, 4U, "C:");
    OLED_ShowInt16(66U, 4U, last_yaw_correction);

    OLED_ShowString(0U, 6U, state_text);
}

/* ============ 距离标定测试模式 ============ */

void HCourse_DistanceTestStart(void)
{
    task3_distance_mm = 0.0f;
    dist_test_target_mm = H_COURSE_TEST_DIST_MM;
    dist_test_active = 1U;
    dist_test_running = 1U;
    target_yaw_x10 = current_yaw_x10;
    yaw_integral = 0.0f;
    yaw_previous_error = 0.0f;
    MOTOR_ResetEncoders();
    sync_reset();
    MOTOR_SetEnabled(1U);
}

uint8_t HCourse_DistanceTestDone(void)
{
    return (dist_test_running == 0U) ? 1U : 0U;
}

void HCourse_DistanceTestPeriodic(void)
{
    uint32_t now = HAL_GetTick();

    if (dist_test_running == 0U) return;
    update_yaw();
    sync_tick();
    buzzer_update(now);
    if (task3_distance_mm >= dist_test_target_mm) {
        MOTOR_SetSpeeds(0, 0);
        MOTOR_SetEnabled(0U);
        dist_test_running = 0U;
        buzzer_beep(now);
        return;
    }
    apply_yaw_hold_pair(H_COURSE_TASK3_DIAG_PWM, H_COURSE_TASK3_DIAG_PWM);
}

void HCourse_Stop(void)
{
    MOTOR_SetSpeeds(0, 0);
    MOTOR_SetEnabled(0U);
    last_left_pwm = 0;
    last_right_pwm = 0;
    buzzer_off_tick = 0U;
    buzzer_set(0U);
    LED_Off(1U);
    dist_test_active = 0U;
    dist_test_running = 0U;
    course_state = H_COURSE_STATE_STOPPED;
}

uint8_t HCourse_IsFinished(void)
{
    return (course_state == H_COURSE_STATE_FINISHED) ? 1U : 0U;
}

HCourse_State HCourse_GetState(void)
{
    return course_state;
}

uint8_t HCourse_GetNodeCount(void)
{
    return course_node_count;
}
