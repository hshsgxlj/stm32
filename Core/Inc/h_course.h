#ifndef H_COURSE_H
#define H_COURSE_H

#include "stm32f1xx_hal.h"

/*
 * 2024 H题 route controller.
 * All values below are deliberately conservative and can be tuned in this
 * header without changing the control implementation.
 */
/* ================================================================
 * 一、控制周期与启动
 * ================================================================ */

/* 控制周期:所有控制(速度环/循迹/路口检测)每 10ms 执行一次。
 * 一般不用改;调大会让控制变"卡顿",调小加重 CPU 负担。 */
#define H_COURSE_CONTROL_PERIOD_MS       10U

/* 启动忽略期:按键出发后前 800ms 内不检测路口,
 * 防止起步瞬间压到的杂线被误判成路口。
 * 如果 B 点离起点太近导致第一个路口被忽略,把这个值调小。 */
#define H_COURSE_START_IGNORE_MS         800U

/* ================================================================
 * 二、编码器(右轮追左轮同步、任务3 计距用)
 * ================================================================ */

/* 编码器每转一圈的计数值(含 4 倍频):由减速比和编码器线数决定,
 * 是硬件参数,换电机/编码器才需要改,平时别动。 */
#define H_COURSE_TASK1_COUNTS_PER_REV    62400L

/* 距离标定测试:直行跑完此距离(编码器计)自动停车,
 * 用尺量实测距离,按"实测÷此值"修正 WHEEL_CIRC_MM。 */
#define H_COURSE_TEST_DIST_MM             1000.0f

/* ================================================================
 * 三、路口检测(任务2 的 B/D 点、任务3/4 的节点计数)
 * ================================================================ */

/* 路口确认时间:压黑状态要持续 30ms 才确认"到达路口",用于去抖。 */
#define H_COURSE_NODE_CONFIRM_MS         30U

/* 路口冷却时间:确认一个路口后 350ms 内不再检测,
 * 防止车还在同一横线上时重复计数。 */
#define H_COURSE_NODE_COOLDOWN_MS        350U

/* 路口判定的压黑路数:任意 1 路压黑就算到达路口(B/D 横线标记)。 */
#define H_COURSE_NODE_BLACK_COUNT        1U

/* 线尾确认时间(任务2 的 C/A 点、任务3/4 的弧线结束):全白持续
 * 此时间才判定"黑线走完"。传感器间隙的短暂全白不会误触发;
 * 判得太慢可调小。 */
#define H_COURSE_TASK2_EDGE_CONFIRM_MS   450U

/* 弯道至少前进这段距离后才允许用“持续全白”判断 C/A 线尾。
 * 防止刚离开 B/D 横线或传感器短暂跨过线缝时提前切换阶段。 */
#define H_COURSE_TASK2_EDGE_MIN_MM        350.0f

/* 任务2弯道出线时,全白间隙内保持最后一次差速,避免提前切回上一段
 * 直线的航向保持而把车头拉回旧方向。 */
#define H_COURSE_TASK2_EDGE_HOLD_MS      H_COURSE_TASK2_EDGE_CONFIRM_MS

/* 任务4:任务2 的路线连续行驶的圈数。 */
#define H_COURSE_TASK4_LAPS              4U

/* 任务2 第二个直线(C-D)方向补偿(度×10,正 yaw = 左转):
 * 过第一个弯道(B-C)后左偏 → 负值(往右);右偏 → 正值。 */
#define H_COURSE_TASK2_CD_TRIM_X10       10

/* 蜂鸣器(和 LED)每次响铃/点亮的时长。 */
#define H_COURSE_BUZZER_BEEP_MS          100U

/* ================================================================
 * 四、循迹控制(BC/DA 弯道压线走)
 * ================================================================ */

/* 电机 PWM 最大限幅(所有输出共用)。 */
#define H_COURSE_MAX_SPEED               450

/* 四路全黑(压到横线/粗线)时的固定减速 PWM。 */
#define H_COURSE_ALL_BLACK_SPEED         100

/* 任务2 弯道(BC/DA)循迹基准 PWM:与直线速度解耦,
 * 直线段用 STRAIGHT_PWM,弯道用此值。弯道飞出/压不住线优先调它。 */
#define H_COURSE_TASK2_CURVE_PWM         180

/* 丢线保持总时长:全白超过间隙保持期后进入 yaw 直行找线,
 * 超过 1000ms 还找不到线就停车。 */
#define H_COURSE_LOST_LINE_HOLD_MS       1000U

/* 传感器间隙保持:全白后的前 150ms 保持上一次的左右差速
 * (窄线在 P2/P3 之间切换时短暂全白,不能让车被"掰直")。
 * 实测间隙比 150ms 还长就把这个值调大。 */
#define H_COURSE_LOST_HOLD_DIFF_MS       150U

/* 循迹 PID(把车压在线上):
 * KP 62:比例,偏离线的响应强度。车左右摆=太大;贴不住线=太小
 * KI 12:积分,圆弯道上提供"持续转向力"(每偏差 1 每秒涨 12 PWM),
 *       弯道外侧一直偏/飞出去 = 太小;出弯后摆 = 太大
 * KD 8:微分,进弯瞬间提前打方向。进弯慢半拍调大;甩一下调小 */
#define H_COURSE_LINE_KP                 62.0f
#define H_COURSE_LINE_KI                 12.0f
#define H_COURSE_LINE_KD                 8.0f

/* 循迹积分限幅:KI×此值 = 弯道上可持续的最大差速(12×8=96,
 * 实际被修正限幅 75 封顶)。 */
#define H_COURSE_LINE_INTEGRAL_LIMIT     8.0f

/* 循迹修正限幅:左右轮差速最多 ±75 PWM。
 * 中间压线给 62,外侧压线封顶 75——外侧不再猛甩,又保留弯道力度。 */
#define H_COURSE_LINE_CORRECTION_LIMIT   75.0f

/* 循迹恢复模式:线跑到最外侧(P1/P4)时,先急刹 RECOVER_BRAKE_MS
 * 毫秒杀掉横向动量,再减速 + 持续向线侧打方向(左右轮差速固定为
 * RECOVER_DIFF,车速降到正常速度的 RECOVER_SPEED_PCT%),
 * 直到中间两路(P2/P3)连续压黑 RECOVER_EXIT_MS 毫秒才恢复正常修正。
 * 恢复期间状态机不跳段。
 * 偏转太快 = DIFF 调小;救不回来 = 调大。 */
#define H_COURSE_LINE_RECOVER_DIFF        45
#define H_COURSE_LINE_RECOVER_SPEED_PCT   65
#define H_COURSE_LINE_RECOVER_BRAKE_MS    80U
#define H_COURSE_LINE_RECOVER_EXIT_MS     30U

/* 任务3/4弯道入口专用:最外侧 P1/P4 刚压黑时更快打方向。
 * 只影响任务3/4,任务2仍使用上面的通用参数。 */
#define H_COURSE_TASK34_RECOVER_DIFF      85
#define H_COURSE_TASK34_RECOVER_SPEED_PCT 70
#define H_COURSE_TASK34_RECOVER_BRAKE_MS  10U

/* 恢复模式下全白(线在传感器间隙/刚脱离最外侧)时,继续按恢复方向
 * 缓慢漂移找线的最长时间;超过则放弃恢复转常规丢线处理。 */
#define H_COURSE_LINE_RECOVER_WHITE_MS    1500U

/* ================================================================
 * 五、航向保持(直行段防跑偏,靠 MPU6050)
 * ================================================================ */

/* 航向保持总开关:1=启用。直行段用校准后的原始陀螺仪
 * (MPU6050_Calibrate,与 Sunward 参考代码同方案)保持航向;
 * 陀螺仪读数失败时自动退回"右轮追左轮"同步。 */
#define H_COURSE_ENABLE_YAW_HOLD         1U

/* 航向修正 PID(把车头掰回目标航向):
 * KP 3.0:偏 1 度给 3 PWM 差速(之前 1.5 太弱,车带着 1.5° 稳态偏走)。
 *       直行左右摆=太大;走不直=太小
 * KI 0.0:积分(暂时关闭)
 * KD 0.08:微分,抑制摆动 */
#define H_COURSE_YAW_KP                  3.0f
#define H_COURSE_YAW_KI                  0.0f
#define H_COURSE_YAW_KD                  0.08f

/* 航向积分限幅(KI 启用时才有效)。 */
#define H_COURSE_YAW_INTEGRAL_LIMIT      20.0f

/* 航向修正限幅:直行段左右轮差速最多 ±30 PWM(约基准的 20%,
 * 小步慢纠,和 Sunward 的"温和航向保持"一致)。 */
#define H_COURSE_YAW_CORRECTION_LIMIT    30

/* 航向修正方向:本车正 yaw = 左转(标准约定),符号取正。
 * 直行越修越偏(正反馈)就把这个符号反过来。 */
#define H_COURSE_YAW_SIGN                1.0f

/* ================================================================
 * 六、任务3(Sunward 参考方案:陀螺仪转向 + 编码器距离窗口 + 巡线弧线)
 * A->C 斜线(无引导线)→ C->B 弧线(引导线)→ B->D 斜线 → D->A 弧线
 * ================================================================ */

/* 斜线几何:长度和与出发方向(AB)的夹角(实车标定)。 */
#define H_COURSE_TASK3_DIAG_MM            1280.6f   /* sqrt(1000²+800²) */
#define H_COURSE_TASK3_DIAG_ANGLE_X10     350       /* 当前任务标定值:35° */

/* BD 斜线方向补偿(度×10,正 yaw = 左转)。保留原始 -3°微调;
 * B 点的 180°反向由 h_course.c 中的 170°目标项单独调节。 */
#define H_COURSE_TASK3_BD_TRIM_X10       -30

/* AC 斜线方向补偿(度×10,正 yaw = 左转):A-C 转向/斜线左偏 →
 * 往右补(负值);右偏 → 正值。只作用于 AC 转向和斜线保持。 */
#define H_COURSE_TASK3_AC_TRIM_X10       -10

/* 任务4 每圈微调表(度×10;正 yaw = 左转,负值 = 往右补):
 * 行 = 第1~4圈,列 = 斜线段(0 = AC 斜线,1 = BD 斜线)。
 * 叠加在全局 AC_TRIM / BD_TRIM 之上,只作用于任务4,任务3 不受影响。
 * 先全部设 0;按每圈实际跑偏逐格微调,方向约定与全局微调一致。 */
#define H_COURSE_TASK4_LAP_TRIM_X10 \
    { {   0,   0 }, \
      {   -5,   -5 }, \
      {   -10,   -10 }, \
      {   -15,   15 } }

/* 轮子周长(mm/圈),用于编码器计距。
 * 实车标定:编码器计 1000mm 实际跑 1090mm → 204.2 × 1.09 ≈ 222.6 */
#define H_COURSE_TASK3_WHEEL_CIRC_MM      222.6f

/* 各段速度(基准 PWM):斜线、弧线、顶点转向(单轮)、捕捉短驶。
 * 转向单轮速度大 → 惯性大 → 容易转过窗口;角度偏大就先降它。 */
#define H_COURSE_TASK3_DIAG_PWM           200
#define H_COURSE_TASK3_ARC_PWM            145
#define H_COURSE_TASK3_TURN_PWM           110
#define H_COURSE_TASK3_TURN_FINE_PWM      75
#define H_COURSE_TASK3_CAPTURE_PWM        60

/* 任务4 的第二个弯需要更大的单轮启动力;只作用于任务4,不改变任务3。 */
#define H_COURSE_TASK4_TURN_PWM           110
#define H_COURSE_TASK4_ALIGN_TIMEOUT_MS   6000U
#define H_COURSE_TASK4_RESCUE_PWM         80

/* 任务3/4 B点出弯、驶离黑线后的最终车头目标(相对启动方向,度×10)。
 * 只作用于出弯摆正阶段,不改变 B-D 斜线的几何航向目标。 */
#define H_COURSE_TASK34_B_EXIT_TARGET_X10 -1750

/* 距离窗口:斜线末尾 ±ENTRY 内允许捕捉线头;跑出窗口即判故障。 */
#define H_COURSE_TASK3_ENTRY_WINDOW_MM    120.0f

/* 任务4专用:斜线超过预计长度多少毫米后进入救援摆正。
 * 数值越小,越早摆正;建议从 20mm 开始现场调整。 */
#define H_COURSE_TASK4_RESCUE_TRIGGER_MM  -100.0f

/* 角度容差(度×10):转向到位 3°,斜线航向 20°。 */
#define H_COURSE_TASK3_ALIGN_TOL_X10      30
#define H_COURSE_TASK3_DIAG_TOL_X10       200

/* 航向误差必须连续超限才报 E3，过滤单次陀螺仪跳变和阶段切换瞬态。 */
#define H_COURSE_TASK3_HEADING_FAULT_MS   250U

/* 时序:确认去抖、顶点制动后静止判据(轮速 RPM)、各阶段超时。 */
#define H_COURSE_TASK3_DEBOUNCE_MS        30U
#define H_COURSE_TASK3_SETTLE_MS          200U
#define H_COURSE_TASK3_SETTLE_RPM         3
#define H_COURSE_TASK3_ALIGN_TIMEOUT_MS   6000U
#define H_COURSE_TASK3_CAPTURE_TIMEOUT_MS 2000U

/* ================================================================
 * 七、直线控制：陀螺仪航向保持或左右编码器转速同步
 * ================================================================ */

/* 直行段:MOTOR_A(物理右轮)固定 PWM(基准),MOTOR_B(物理左轮)
 * 用 PID 跟踪右轮实际转速。两个 PWM 常量是 B 轮修正的基准值。 */
#define H_COURSE_STRAIGHT_PWM_LEFT       200
#define H_COURSE_STRAIGHT_PWM_RIGHT      200

/* 左轮同步 PID(误差 = 右轮(A)实际转速 + 补偿 − 左轮(B)实际转速,
 * 单位 RPM):KP:每 1 RPM 差给多少 PWM;KI:消除持续偏差(左右轮阻力差异)。
 * 同步后左右轮转速一致,车走直线。 */
#define H_COURSE_SYNC_KP                 2.0f
#define H_COURSE_SYNC_KI                 0.5f
#define H_COURSE_SYNC_KD                 0.0f
#define H_COURSE_SYNC_INTEGRAL_LIMIT     30.0f
#define H_COURSE_SYNC_LIMIT              80

/* 轮径差补偿:左轮(B)的目标转速 = 右轮(A) + 此值(RPM)。
 * 车右偏 → 调大(左轮跑更快);车左偏 → 调小或填负值。 */
#define H_COURSE_SYNC_RPM_TRIM           0.3f

/* 串口遥测开关:1=打开,USART2(PA2-TX)以 115200 8N1 每 100ms 输出一行 CSV。
 * 0=关闭。 */
#ifndef H_COURSE_DEBUG
#define H_COURSE_DEBUG                   1
#endif
#define H_COURSE_DEBUG_PERIOD_MS         100U

typedef enum {
    H_COURSE_TASK1_AB = 1,
    H_COURSE_TASK2_ABCD,
    H_COURSE_TASK3_ACBDA,
    H_COURSE_TASK4_FOUR_LAPS
} HCourse_Task;

typedef enum {
    H_COURSE_STATE_IDLE = 0,
    H_COURSE_STATE_LINE_FOLLOW,
    H_COURSE_STATE_STRAIGHT_HOLD,
    H_COURSE_STATE_FINISHED,
    H_COURSE_STATE_STOPPED,
    H_COURSE_STATE_FAULT
} HCourse_State;

HAL_StatusTypeDef HCourse_Init(HCourse_Task task);
void HCourse_SetTask(HCourse_Task task);
void HCourse_Start(void);
void HCourse_PeriodicTask(void);
void HCourse_UpdateOLED(void);
void HCourse_Stop(void);
uint8_t HCourse_IsFinished(void);
HCourse_State HCourse_GetState(void);
uint8_t HCourse_GetNodeCount(void);

/* 距离标定测试模式 */
void HCourse_DistanceTestStart(void);
void HCourse_DistanceTestPeriodic(void);
uint8_t HCourse_DistanceTestDone(void);

#endif /* H_COURSE_H */
